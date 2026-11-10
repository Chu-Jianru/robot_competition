"""把只有“红块中心点”标注的老照片并进训练集，点转成框。

老批次（`field_photos_20261103`、`field_no_robot_20261103`）标的是可见红块的
**中心点**，不是框。脚本用该图人工四角算出的单应性求局部尺度，再按 20 mm 的
实物边长生成居中的框（留 25% 余量）。点不是框，转换只是起点：合并后仍需人工
逐个确认，所以 `reviewed` 一律留 false。

图片会按蓝黄方向统一成“黄上蓝下”，框随之旋转。

用法：
    python windows\\import_points.py --batch windows\\test_data\\field_no_robot_20261103 ^
        --target windows\\test_data\\red_train_20261110 --prefix norobot
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
from merge_dataset import next_number, read_image                    # noqa: E402
from orientation import apply_rotation, field_rotation               # noqa: E402

FIELD_MM = 1200.0
BLOCK_MM = 20.0
MARGIN = 1.25


def homography(corners):
    quad = np.float32(corners)
    if len(quad) != 4 or not cv2.isContourConvex(quad) or cv2.contourArea(quad) < 100:
        return None
    dst = np.float32([[0, 0], [FIELD_MM, 0], [FIELD_MM, FIELD_MM], [0, FIELD_MM]])
    return cv2.getPerspectiveTransform(quad, dst)


def local_mm_per_px(matrix, point) -> tuple:
    """Jacobian of the field homography at a point: mm per pixel along x and y."""
    if matrix is None:
        return None
    source = np.float32([[point[0], point[1]], [point[0] + 1, point[1]],
                         [point[0], point[1] + 1]]).reshape(-1, 1, 2)
    mapped = cv2.perspectiveTransform(source, matrix).reshape(-1, 2)
    return (float(np.linalg.norm(mapped[1] - mapped[0])),
            float(np.linalg.norm(mapped[2] - mapped[0])))


def box_for_point(point, scales, block_mm: float = BLOCK_MM, margin: float = MARGIN):
    sx, sy = scales
    width = block_mm * margin / max(sx, 1e-6)
    height = block_mm * margin / max(sy, 1e-6)
    return [point[0] - width / 2.0, point[1] - height / 2.0, width, height]


def rotate_point(point, degrees: int, width: int, height: int):
    x, y = point
    if degrees == 90:
        return (y, width - x)
    if degrees == 180:
        return (width - x, height - y)
    if degrees == 270:
        return (height - y, x)
    return (x, y)


def rotate_box(box, degrees: int, width: int, height: int):
    x, y, w, h = box
    corners = [(x, y), (x + w, y), (x, y + h), (x + w, y + h)]
    moved = [rotate_point(corner, degrees, width, height) for corner in corners]
    xs = [p[0] for p in moved]
    ys = [p[1] for p in moved]
    return [min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)]


def clamp(box, width, height, min_side=6):
    x, y, w, h = box
    x0, y0 = max(0, int(round(x))), max(0, int(round(y)))
    x1 = min(width, int(round(x + w)))
    y1 = min(height, int(round(y + h)))
    if x1 - x0 < min_side or y1 - y0 < min_side:
        return None
    return [x0, y0, x1 - x0, y1 - y0]


def batch_median_scale(entries):
    """Fallback scale for images whose corners are unusable."""
    samples = []
    for name, entry in entries.items():
        matrix = homography(entry.get("field_corners", []))
        if matrix is None:
            continue
        for point in entry.get("blocks", [])[:3]:
            scales = local_mm_per_px(matrix, point)
            if scales:
                samples.append((scales[0] + scales[1]) / 2.0)
    return float(np.median(samples)) if samples else None


def import_batch(batch: Path, target: Path, prefix: str) -> dict:
    annotations = json.loads((batch / "annotations.json").read_text(encoding="utf-8"))
    frames = target / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    labels_path = frames / "labels.json"
    store = load_store(labels_path)
    fallback = batch_median_scale(annotations)
    start = next_number(frames, prefix)
    added = boxes_total = 0
    skipped_points = 0
    for offset, (source_name, entry) in enumerate(sorted(annotations.items())):
        source = batch / source_name
        image = read_image(source) if source.exists() else None
        if image is None:
            continue
        height, width = image.shape[:2]
        degrees = field_rotation(image) or 0
        rotated = apply_rotation(image, degrees)
        matrix = homography(entry.get("field_corners", []))
        boxes = []
        for point in entry.get("blocks", []):
            scales = local_mm_per_px(matrix, point)
            if scales is None:
                if fallback is None:
                    skipped_points += 1
                    continue
                scales = (fallback, fallback)
            box = rotate_box(box_for_point(point, scales), degrees, width, height)
            box = clamp(box, rotated.shape[1], rotated.shape[0])
            if box:
                boxes.append(box)
        name = f"{prefix}_{start + offset:03d}.jpg"
        ok, buffer = cv2.imencode(".jpg", rotated, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            continue
        buffer.tofile(str(frames / name))
        store["images"][name] = {"boxes": boxes, "reviewed": False, "auto": True,
                                 "from_points": True,
                                 "width": int(rotated.shape[1]),
                                 "height": int(rotated.shape[0]),
                                 "source": source_name,
                                 "rotation_ccw": degrees}
        added += 1
        boxes_total += len(boxes)
    save_store(labels_path, store)
    return {"images": added, "boxes": boxes_total, "skipped_points": skipped_points,
            "fallback_mm_per_px": fallback}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--batch", type=Path, required=True, help="含 annotations.json 的老批次目录")
    parser.add_argument("--target", type=Path, required=True)
    parser.add_argument("--prefix", required=True)
    args = parser.parse_args()
    result = import_batch(args.batch, args.target, args.prefix)
    print(f"并入 {result['images']} 张、{result['boxes']} 个框"
          + (f"；{result['skipped_points']} 个点因缺几何被跳过"
             if result["skipped_points"] else ""))
    if result["fallback_mm_per_px"]:
        print(f"无有效四角的图使用批次中位尺度 {result['fallback_mm_per_px']:.3f} mm/px")
    print("框由中心点按 20 mm 推算，仍需在标注工具里逐个确认")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
