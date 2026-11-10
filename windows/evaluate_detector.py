"""拿人工标注当标准答案，比较红块检测器的精度与耗时。

支持两种被测对象：
  hsv  —— 现有 `red_blocks` 颜色+形状检测（预填用的那条链路）
  onnx —— 训练好的 `.onnx` 模型

判据用 IoU 一对一匹配：一个预测最多配一个标注，配不上算误检，标注配不上算漏检。

用法：
    python windows\\evaluate_detector.py --frames windows\\test_data\\red_train_20261110\\frames --detector hsv
    python windows\\evaluate_detector.py --frames ... --detector onnx --model windows\\models\\red_block.onnx
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from label_boxes import load_store, read_image                     # noqa: E402
from export_yolo import is_validation                              # noqa: E402
from red_blocks import load_red_config, prelabel_boxes             # noqa: E402

HERE = Path(__file__).resolve().parent


def iou(box_a, box_b) -> float:
    ax, ay, aw, ah = box_a
    bx, by, bw, bh = box_b
    ix = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    if inter <= 0:
        return 0.0
    union = aw * ah + bw * bh - inter
    return float(inter / union) if union > 0 else 0.0


def match_boxes(predicted, truth, iou_thres: float = 0.3):
    """Greedy one-to-one matching; returns (tp, fp, fn, matched_ious)."""
    used = set()
    matched = []
    for prediction in predicted:
        best, best_index = 0.0, None
        for index, target in enumerate(truth):
            if index in used:
                continue
            score = iou(prediction, target)
            if score > best:
                best, best_index = score, index
        if best_index is not None and best >= iou_thres:
            used.add(best_index)
            matched.append(best)
    tp = len(matched)
    return tp, len(predicted) - tp, len(truth) - tp, matched


def run_detector(kind: str, image, config=None, session=None, imgsz=(1280, 960)):
    if kind == "hsv":
        return prelabel_boxes(image, config)
    boxes = session.detect(image)
    return [[box["bbox_px"][0], box["bbox_px"][1], box["bbox_px"][2], box["bbox_px"][3]]
            for box in boxes]


def evaluate(frames: Path, kind: str, model=None, limit=None, iou_thres: float = 0.3):
    store = load_store(frames / "labels.json")
    config = load_red_config(HERE / "red_block_profile.json") \
        if (HERE / "red_block_profile.json").exists() else None
    detector = None
    if kind == "onnx":
        from onnx_blocks import OnnxBlockDetector
        detector = OnnxBlockDetector(model, imgsz=(1280, 960))
    names = sorted(name for name, entry in store["images"].items() if entry.get("reviewed"))
    if limit:
        names = names[:limit]
    rows = {"tp": 0, "fp": 0, "fn": 0}
    times = []
    per_group = {}
    per_split = {}
    for name in names:
        image = read_image(frames / name)
        if image is None:
            continue
        began = time.perf_counter()
        predicted = run_detector(kind, image, config, detector)
        times.append((time.perf_counter() - began) * 1000)
        truth = store["images"][name]["boxes"]
        tp, fp, fn, _ = match_boxes(predicted, truth, iou_thres)
        rows["tp"] += tp
        rows["fp"] += fp
        rows["fn"] += fn
        split = "val" if is_validation(name, 5) else "train"
        group = split + ":" + name.split("_")[0]
        for key in (group, split):
            entry = per_group.setdefault(key, {"tp": 0, "fp": 0, "fn": 0, "images": 0})
            entry["tp"] += tp
            entry["fp"] += fp
            entry["fn"] += fn
            entry["images"] += 1
        per_split[split] = per_split.get(split, 0) + 1
    return rows, per_group, per_split, times


def summary(counts):
    tp, fp, fn = counts["tp"], counts["fp"], counts["fn"]
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return precision, recall, f1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--detector", choices=("hsv", "onnx"), default="hsv")
    parser.add_argument("--model", type=Path)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--iou", type=float, default=0.3)
    parser.add_argument("--json", type=Path, help="把结果写成 JSON")
    args = parser.parse_args()
    if args.detector == "onnx" and not args.model:
        print("onnx 模式需要 --model", file=sys.stderr)
        return 2
    try:
        counts, groups, splits, times = evaluate(args.frames, args.detector, args.model,
                                                 args.limit, args.iou)
    except RuntimeError as error:
        print(f"检测器无法启动：{error}", file=sys.stderr)
        return 2
    precision, recall, f1 = summary(counts)
    print(f"检测器 {args.detector}；图片 {len(times)} 张；IoU 阈值 {args.iou}")
    print(f"  TP {counts['tp']}  FP {counts['fp']}  FN {counts['fn']}")
    print(f"  precision {precision:.3f}  recall {recall:.3f}  F1 {f1:.3f}")
    if times:
        print(f"  单帧 ms：中位 {np.median(times):.1f}  P95 {np.percentile(times, 95):.1f}")
    print("  按训练/验证划分（train 子集是模型的记忆分数，只有 val 可信）：")
    for split in ("val", "train"):
        entry = groups.get(split)
        if not entry:
            continue
        group_precision, group_recall, _ = summary(entry)
        print(f"    [{split}] {entry['images']} 张  P {group_precision:.3f}  "
              f"R {group_recall:.3f}")
    print("  按图片来源：")
    for key in sorted(key for key in groups if ":" in key):
        entry = groups[key]
        group_precision, group_recall, _ = summary(entry)
        print(f"    [{key}] {entry['images']} 张  P {group_precision:.3f}  "
              f"R {group_recall:.3f}")
    if args.json:
        args.json.write_text(json.dumps({"detector": args.detector, "counts": counts,
                                         "precision": precision, "recall": recall,
                                         "f1": f1, "per_group": groups,
                                         "per_split_images": splits,
                                         "ms_median": float(np.median(times)) if times else None},
                                        ensure_ascii=False, indent=2) + "\n",
                             encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
