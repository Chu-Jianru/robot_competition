"""台架自检：没有场地时，检查 tag 与红块能不能被同时识别。

不需要四角标定——用 tag 自身 50 mm 的已知边长反推 mm/px，
所以只要俯视拍一张"tag + 若干红块"的照片就能验收。

用法：
    & 'E:\\scrcpy\\scrcpy-win64-v5.0.1\\.venv\\Scripts\\python.exe' windows\\bench_check.py ^
        windows\\captures_hi\\hi_xxxx.png

输出：控制台一张表 + 同目录 `*_bench.png` 标注图。
判定标准：tag 边长换算后红块等效边长应接近 20 mm；偏大偏小说明俯视角度或标尺有问题。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from red_blocks import detect_red_blocks, filter_by_size_mm, load_red_config   # noqa: E402
from tag_detector import TAG_ID, detect_markers  # noqa: E402

IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".bmp")
DEFAULT_PROFILE = Path(__file__).resolve().with_name("red_block_profile.json")


def find_tag(image: np.ndarray, tag_id: int | None = TAG_ID):
    """返回 (四角像素坐标 或 None, id, 边长px)。"""
    best = None
    for marker in detect_markers(image, tag_id):
        pts, marker_id = marker.corners, marker.tag_id
        edges = [float(np.linalg.norm(pts[i] - pts[(i + 1) % 4])) for i in range(4)]
        side = float(np.mean(edges))
        if tag_id is not None and int(marker_id) != tag_id:
            continue
        if best is None or side > best[2]:
            best = (pts, int(marker_id), side)
    return best if best else (None, None, None)


def annotate(image: np.ndarray, tag_pts, tag_id, side_px, scale, blocks, tag_center,
             rejected=None):
    canvas = image.copy()
    if tag_pts is not None:
        cv2.polylines(canvas, [tag_pts.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        cv2.putText(canvas, f"id={tag_id} side={side_px:.1f}px = {side_px * scale:.1f}mm",
                    (int(tag_pts[:, 0].min()), max(20, int(tag_pts[:, 1].min()) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    for index, block in enumerate(blocks):
        x, y, w, h = block["bbox_px"]
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cx, cy = block["center_px"]
        side_mm = math.sqrt(block["area_px"]) * scale
        label = f"{index}:{side_mm:.1f}mm"
        if tag_center is not None:
            dist = math.hypot(cx - tag_center[0], cy - tag_center[1]) * scale
            label += f" d={dist:.0f}"
        cv2.putText(canvas, label, (x, max(14, y - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 255, 255), 1)
        cv2.circle(canvas, (int(cx), int(cy)), 2, (0, 0, 255), -1)
    for block, reason in rejected or []:
        x, y, w, h = block["bbox_px"]
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (140, 140, 140), 1)
        cv2.putText(canvas, f"x{block['side_mm']:.0f} {reason[:5]}", (x, max(12, y - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (140, 140, 140), 1)
    return canvas


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", type=Path, help="图片或目录")
    parser.add_argument("--tag-mm", type=float, default=50.0)
    parser.add_argument("--tag-id", type=int, default=TAG_ID)
    parser.add_argument("--scale", type=float, help="没有 tag 时直接给 mm/px")
    parser.add_argument("--min-mm", type=float, default=12.0, help="等效边长下限，低于即判为碎块")
    parser.add_argument("--max-mm", type=float, default=32.0, help="等效边长上限，高于即判为粘连")
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--save-dir", type=Path, help="标注图输出目录，默认与输入同目录")
    args = parser.parse_args()

    files = ([args.images] if args.images.is_file()
             else sorted(p for p in args.images.rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES))
    files = [p for p in files if "_bench" not in p.name and "_annotated" not in p.name]
    if not files:
        raise SystemExit("没有找到图片")

    config = load_red_config(args.profile) if args.profile.exists() else None
    print(f"红块配置：{args.profile} (exists={args.profile.exists()})")

    for path in files:
        encoded = np.fromfile(str(path), dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR) if encoded.size else None
        if image is None:
            print(f"{path.name}: 读不到")
            continue
        h, w = image.shape[:2]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        tag_pts, tag_id, side_px = find_tag(gray, args.tag_id)

        if tag_pts is not None:
            scale = args.tag_mm / side_px
            tag_center = tag_pts.mean(axis=0)
            source = f"tag id={tag_id} side={side_px:.1f}px"
        elif args.scale:
            scale = args.scale
            tag_center = None
            source = f"--scale {args.scale:.4f} mm/px"
        else:
            scale = None
            tag_center = None
            source = "无 tag 且未给 --scale"

        blocks = detect_red_blocks(image, config)
        kept, rejected = ([], [])
        if scale:
            kept, rejected = filter_by_size_mm(blocks, scale, args.min_mm, args.max_mm)
        print(f"\n=== {path.name}  {w}x{h} ===")
        print(f"标尺：{source}" + (f" -> {scale:.4f} mm/px" if scale else ""))
        print(f"tag：{'未检出' if tag_pts is None else f'检出 id={tag_id}，边长 {side_px:.1f} px'}")
        print(f"红块候选：{len(blocks)}" + (f"（毫米筛后 {len(kept)}，剔除 {len(rejected)}）" if scale else ""))
        if rejected:
            too_small = sum(1 for _b, reason in rejected if reason == "too_small")
            too_large = len(rejected) - too_small
            detail = "，".join(f"{b['side_mm']:.1f}mm/{reason}" for b, reason in rejected[:6])
            print(f"  剔除：过小 {too_small}、粘连 {too_large}｜样例 {detail}")
        if scale and kept:
            sides = [b["side_mm"] for b in kept]
            print(f"  等效边长 mm：中位 {np.median(sides):.1f}，范围 {min(sides):.1f}–{max(sides):.1f}")
            if tag_center is not None:
                dists = [math.hypot(b["center_px"][0] - tag_center[0],
                                    b["center_px"][1] - tag_center[1]) * scale for b in kept]
                print(f"  到 tag 中心距离 mm：中位 {np.median(dists):.1f}，范围 {min(dists):.1f}–{max(dists):.1f}")

        draw_blocks = kept if scale else blocks
        marked = annotate(image, tag_pts, tag_id, side_px, scale or 0.0, draw_blocks,
                          tag_center, rejected=rejected)
        out_dir = args.save_dir or path.parent
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"{path.stem}_bench.png"
        ok, buf = cv2.imencode(".png", marked)
        if ok:
            buf.tofile(str(out_path))
            print(f"  标注图：{out_path}")


if __name__ == "__main__":
    main()
