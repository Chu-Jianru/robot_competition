"""把 `label_boxes.py` 的 JSON 标注导出成 YOLO 数据集结构。

不训练、不联网：只复制图片、写 `labels/*.txt` 和 `data.yaml`，训练在别的
机器/Colab 上进行。按文件名稳定划分 train/val，同一段录像的相邻帧会落在
同一侧，避免用几乎重复的画面虚高验证分数。

用法：
    python windows/export_yolo.py windows\\test_data\\red_train_20261110 ^
        --out windows\\test_data\\red_train_20261110\\yolo --val-every 5
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")


def normalized(box, width: int, height: int):
    """[x, y, w, h] pixels -> (cx, cy, w, h) in 0..1, or None if degenerate."""
    x, y, w, h = (float(value) for value in box)
    if width <= 0 or height <= 0 or w <= 0 or h <= 0:
        return None
    cx = (x + w / 2.0) / width
    cy = (y + h / 2.0) / height
    nw, nh = w / width, h / height
    if not (0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0):
        return None
    return (min(max(cx, 0.0), 1.0), min(max(cy, 0.0), 1.0),
            min(nw, 1.0), min(nh, 1.0))


def is_validation(name: str, val_every: int) -> bool:
    """Stable per-file decision, no random state to reproduce."""
    if val_every <= 1:
        return False
    digits = "".join(character for character in Path(name).stem if character.isdigit())
    number = int(digits) if digits else abs(hash(name))
    return number % val_every == 0


def export(folder: Path, out: Path, *, val_every: int = 5,
           include_unreviewed: bool = False, class_name: str = "red_block") -> dict:
    labels_path = folder / "labels.json"
    if not labels_path.exists() and (folder / "frames" / "labels.json").exists():
        # label_boxes.py writes next to the images, i.e. inside frames/.
        labels_path = folder / "frames" / "labels.json"
    if not labels_path.exists():
        raise ValueError(f"缺少 {labels_path}；先用 label_boxes.py 标注")
    store = json.loads(labels_path.read_text(encoding="utf-8"))
    images = store.get("images", {})
    stats = {"images_train": 0, "images_val": 0, "boxes_train": 0, "boxes_val": 0,
             "skipped_unreviewed": 0, "skipped_missing": 0, "bad_boxes": 0}
    for split in ("train", "val"):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)
    for name in sorted(images):
        entry = images[name]
        if not entry.get("reviewed") and not include_unreviewed:
            stats["skipped_unreviewed"] += 1
            continue
        source = folder / "frames" / name
        if not source.exists():
            source = folder / name
        if not source.exists():
            stats["skipped_missing"] += 1
            continue
        width = int(entry.get("width") or 0)
        height = int(entry.get("height") or 0)
        if width <= 0 or height <= 0:
            stats["skipped_missing"] += 1
            continue
        split = "val" if is_validation(name, val_every) else "train"
        lines = []
        for box in entry.get("boxes", []):
            values = normalized(box, width, height)
            if values is None:
                stats["bad_boxes"] += 1
                continue
            lines.append("0 " + " ".join(f"{value:.6f}" for value in values))
        shutil.copy2(source, out / "images" / split / name)
        (out / "labels" / split / (Path(name).stem + ".txt")).write_text(
            "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        stats[f"images_{split}"] += 1
        stats[f"boxes_{split}"] += len(lines)
    (out / "data.yaml").write_text(
        f"path: {out.resolve().as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        f"  0: {class_name}\n", encoding="utf-8")
    (out / "export.json").write_text(json.dumps(stats, ensure_ascii=False, indent=2) + "\n",
                                     encoding="utf-8")
    return stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", type=Path, help="含 frames/ 与 labels.json 的数据集目录")
    parser.add_argument("--out", type=Path, help="默认 <folder>/yolo")
    parser.add_argument("--val-every", type=int, default=5, help="每 N 张取一张做验证集")
    parser.add_argument("--include-unreviewed", action="store_true")
    parser.add_argument("--class-name", default="red_block")
    args = parser.parse_args()
    out = args.out or (args.folder / "yolo")
    try:
        stats = export(args.folder, out, val_every=args.val_every,
                       include_unreviewed=args.include_unreviewed,
                       class_name=args.class_name)
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    print(f"导出到 {out}")
    print(f"训练集 {stats['images_train']} 张 / {stats['boxes_train']} 框；"
          f"验证集 {stats['images_val']} 张 / {stats['boxes_val']} 框")
    if stats["skipped_unreviewed"]:
        print(f"跳过未按 V 核对的 {stats['skipped_unreviewed']} 张"
              f"（加 --include-unreviewed 可强制导出）")
    if stats["skipped_missing"]:
        print(f"跳过找不到图片或尺寸的 {stats['skipped_missing']} 张")
    if stats["bad_boxes"]:
        print(f"跳过无效框 {stats['bad_boxes']} 个")
    print(f"配置：{out / 'data.yaml'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
