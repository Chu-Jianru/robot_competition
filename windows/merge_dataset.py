"""把另一批图片并进一个训练集目录，标注文件一起合并。

用于把手机直拍的照片并进取流截帧的主数据集，最后只有一个 `frames/` 和一个
`labels.json`，标注、导出都只对一个目录。

规则：
- 图片按 `<prefix>_NNN.jpg` 复制进目标 `frames/`，不删除源目录；
- 目标 `labels.json` 里已有的条目原样保留，新图片写入空框或 HSV 预填框；
- 每张图在 `sources.json` 里记下来源路径和原文件名，便于追溯。

用法：
    python windows\\merge_dataset.py --target windows\\test_data\\red_train_20261110 ^
        --source windows\\test_data\\red_photos_20261110 --prefix photo
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

from label_boxes import load_store, save_store                      # noqa: E402
from red_blocks import load_red_config, prelabel_boxes              # noqa: E402

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")
HERE = Path(__file__).resolve().parent


def image_folder(source: Path) -> Path:
    """Accept either a dataset root (with frames/) or a frames folder."""
    return source / "frames" if (source / "frames").is_dir() else source


def read_image(path: Path):
    encoded = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR) if encoded.size else None


def next_number(frames: Path, prefix: str) -> int:
    highest = 0
    for path in frames.glob(f"{prefix}_*.jpg"):
        digits = path.stem.split("_")[-1]
        if digits.isdigit():
            highest = max(highest, int(digits))
    return highest + 1


def merge(target: Path, source: Path, prefix: str = "photo", prefill: bool = True,
          config=None) -> dict:
    frames = target / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    labels_path = frames / "labels.json"
    store = load_store(labels_path)
    sources_path = target / "sources.json"
    sources = json.loads(sources_path.read_text(encoding="utf-8")) if sources_path.exists() else {}

    start = next_number(frames, prefix)
    added = 0
    skipped = 0
    for position, path in enumerate(sorted(p for p in image_folder(source).iterdir()
                                           if p.suffix.lower() in IMAGE_SUFFIXES)):
        image = read_image(path)
        if image is None:
            skipped += 1
            continue
        name = f"{prefix}_{start + position:03d}.jpg"
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            skipped += 1
            continue
        buffer.tofile(str(frames / name))
        entry = store["images"].setdefault(name, {"boxes": [], "reviewed": False})
        if not entry.get("boxes") and prefill and config is not None:
            entry["boxes"] = prelabel_boxes(image, config)
            entry["auto"] = True
        entry["width"], entry["height"] = int(image.shape[1]), int(image.shape[0])
        sources[name] = {"from": str(path.resolve()), "merged": time.strftime("%Y-%m-%d %H:%M:%S")}
        added += 1
    save_store(labels_path, store)
    sources_path.write_text(json.dumps(sources, ensure_ascii=False, indent=2) + "\n",
                            encoding="utf-8")
    return {"added": added, "skipped": skipped, "labels": str(labels_path),
            "total_images": len(store["images"])}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target", type=Path, required=True, help="主数据集目录")
    parser.add_argument("--source", type=Path, required=True, help="要并进来的目录（含 frames/ 或图片）")
    parser.add_argument("--prefix", default="photo", help="新文件名前缀")
    parser.add_argument("--no-prefill", action="store_true", help="不预填 HSV 框")
    args = parser.parse_args()

    config = None
    if not args.no_prefill:
        profile = HERE / "red_block_profile.json"
        config = load_red_config(profile) if profile.exists() else None
    result = merge(args.target, args.source, args.prefix, not args.no_prefill, config)
    print(f"并入 {result['added']} 张，跳过 {result['skipped']} 张；"
          f"标注文件现在共 {result['total_images']} 条")
    print(f"标注：{result['labels']}")
    print("来源记录：sources.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
