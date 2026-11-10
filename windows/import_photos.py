"""把手机拍的照片收进数据集目录：统一成“黄区在上、蓝区在下”的方向。

训练图必须和实机取流看到的方向一致（`find_seam` 要求黄上蓝下），而手机相册里
同一场地的照片可能是竖拍（蓝左黄右）或横拍（黄上蓝下）。这个脚本只做旋转、
改名、去重和清单，不改亮度、不裁剪、不改 111 里的原图。

用法：
    python windows\\import_photos.py --src 111 ^
        --out windows\\test_data\\red_photos_20261110
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from orientation import apply_rotation, centroids, choose_rotation, field_rotation  # noqa: E402

HERE = Path(__file__).resolve().parent
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")


def canonical(image):
    """Rotate so yellow is above blue; returns (image, degrees)."""
    degrees = field_rotation(image) or 0
    return apply_rotation(image, degrees), degrees


def read_image(path: Path):
    encoded = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR) if encoded.size else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--src", type=Path, default=Path("111"))
    parser.add_argument("--out", type=Path,
                        default=HERE / "test_data" / time.strftime("red_photos_%Y%m%d"))
    parser.add_argument("--prefix", default="photo")
    parser.add_argument("--no-rotate", action="store_true")
    parser.add_argument("--quality", type=int, default=95)
    args = parser.parse_args()

    source = args.src if args.src.is_absolute() else Path.cwd() / args.src
    files = sorted(path for path in source.iterdir()
                   if path.suffix.lower() in IMAGE_SUFFIXES) if source.is_dir() else []
    if not files:
        print(f"{source} 里没有图片", file=sys.stderr)
        return 2
    frames = args.out / "frames"
    frames.mkdir(parents=True, exist_ok=True)

    seen = set()
    entries = []
    rotations = {}
    for path in files:
        raw = path.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if digest in seen:
            print(f"跳过重复：{path.name}")
            continue
        seen.add(digest)
        image = read_image(path)
        if image is None:
            print(f"读不到：{path.name}")
            continue
        degrees = 0
        if not args.no_rotate:
            image, degrees = canonical(image)
        rotations[degrees] = rotations.get(degrees, 0) + 1
        name = f"{args.prefix}_{len(entries) + 1:03d}.jpg"
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
        if not ok:
            print(f"编码失败：{path.name}")
            continue
        buffer.tofile(str(frames / name))
        entries.append({
            "file": name,
            "source": path.name,
            "source_sha256": digest,
            "rotation_ccw": degrees,
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
        })

    manifest = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source_dir": str(source),
        "canonical": "yellow above blue" + (" (disabled)" if args.no_rotate else ""),
        "quality": args.quality,
        "images": entries,
        "rotations": rotations,
    }
    (args.out / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"导入 {len(entries)} 张到 {frames}")
    print("旋转统计：" + "，".join(f"{degrees}° x{count}"
                                  for degrees, count in sorted(rotations.items())))
    print(f"清单：{args.out / 'manifest.json'}")
    print(f"下一步：python windows\\label_boxes.py \"{frames}\"")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
