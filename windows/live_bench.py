"""实机台架测量：连续抓 N 帧，统计 tag 检出率与红块数量（不需要颜色场地）。

与 `bench_check.py` 的区别：那个吃单张图，这个直接连手机取流，回答
"连续看 60 帧，tag 能认出多少帧、红块数稳不稳、每帧要多久"。

不需要四角标定：用 tag 自身 50 mm 边长反推 mm/px，红块再按毫米筛。

用法：
    & 'E:\\scrcpy\\scrcpy-win64-v5.0.1\\.venv\\Scripts\\python.exe' windows\\live_bench.py ^
        --scrcpy-dir 'E:\\scrcpy\\scrcpy-win64-v5.0.1' --frames 60

输出：控制台摘要 + 输出目录里的 raw/annotated PNG + frames.csv。
本脚本只读画面，不发任何底盘指令。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from bench_check import annotate as annotate_bench, find_tag          # noqa: E402
from red_blocks import detect_red_blocks, filter_by_size_mm, load_red_config  # noqa: E402
from tag_detector import TAG_ID                                      # noqa: E402
from usb_camera import UsbCamera                                     # noqa: E402

ROTATIONS = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def rotate(frame, angle: int):
    return cv2.rotate(frame, ROTATIONS[angle]) if angle else frame


def wait_first_frame(camera, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if camera.error:
            print(f"采集线程报错：{camera.error}")
            return False
        frame, _received_at, sequence = camera.latest.get()
        if sequence > 0 and frame is not None:
            return True
        time.sleep(0.05)
    print(f"{timeout_s:.0f} s 内没有收到任何一帧。服务端最后几行：")
    for line in list(camera.logs)[-8:]:
        print(f"  {line}")
    return False


def circular_spread_deg(angles: list[float]) -> tuple[float, float]:
    """返回 (圆周均值, 圆周标准差°)；角度单位度，0–360。"""
    if not angles:
        return float("nan"), float("nan")
    radians = np.radians(np.asarray(angles, dtype=np.float64))
    mean = math.degrees(math.atan2(float(np.sin(radians).mean()),
                                   float(np.cos(radians).mean()))) % 360.0
    resultant = math.hypot(float(np.sin(radians).mean()), float(np.cos(radians).mean()))
    spread = math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(resultant, 1e-9)))))
    return mean, spread


def save_png(path: Path, image: np.ndarray) -> bool:
    ok, buf = cv2.imencode(".png", image)
    if ok:
        buf.tofile(str(path))
    return bool(ok)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scrcpy-dir", type=Path,
                        default=Path(r"E:\scrcpy\scrcpy-win64-v5.0.1"))
    parser.add_argument("--serial")
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=90)
    parser.add_argument("--max-size", type=int, default=1280)
    parser.add_argument("--fps", type=int, default=30, help="手机摄像头帧率")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--timeout", type=float, default=12.0, help="等首帧的秒数")
    parser.add_argument("--profile", type=Path,
                        default=Path(__file__).resolve().with_name("red_block_profile.json"))
    parser.add_argument("--tag-mm", type=float, default=50.0)
    parser.add_argument("--tag-id", type=int, default=TAG_ID)
    parser.add_argument("--min-mm", type=float, default=12.0)
    parser.add_argument("--max-mm", type=float, default=32.0)
    parser.add_argument("--min-rate", type=float, default=0.95, help="tag 检出率门槛")
    parser.add_argument("--out", type=Path, help="默认 windows/captures_hi/live_<时间>")
    args = parser.parse_args()

    out_dir = args.out or (Path(__file__).resolve().parent / "captures_hi"
                           / time.strftime("live_%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)
    config = load_red_config(args.profile) if args.profile.exists() else None
    print(f"输出目录：{out_dir}")
    print(f"红块配置：{args.profile} (exists={args.profile.exists()})")
    print(f"目标：tag id={args.tag_id} 边长 {args.tag_mm:.0f} mm；"
          f"红块毫米筛 {args.min_mm:.0f}–{args.max_mm:.0f} mm；{args.frames} 帧")

    camera = UsbCamera(args.scrcpy_dir, args.serial, max_size=args.max_size,
                       camera_fps=args.fps)
    try:
        camera.start()
        print("等服务端首帧 …")
        if not wait_first_frame(camera, args.timeout):
            return 2

        rows: list[dict] = []
        tag_sides: list[float] = []
        headings: list[float] = []
        centers_x: list[float] = []
        centers_y: list[float] = []
        kept_counts: list[int] = []
        candidate_counts: list[int] = []
        rejected_counts: list[int] = []
        all_sides_mm: list[float] = []
        dists_mm: list[float] = []
        tag_times: list[float] = []
        block_times: list[float] = []
        ages: list[float] = []
        best_score = -1.0
        best_canvas = None
        best_raw = None
        first_raw = None
        last_sequence = 0
        frames_seen = 0
        started = time.monotonic()

        while len(rows) < args.frames:
            if camera.error:
                raise RuntimeError(camera.error)
            frame, received_at, sequence = camera.latest.get()
            if frame is None or sequence == last_sequence:
                if time.monotonic() - received_at > 3.0:
                    raise RuntimeError("画面停滞超过 3 s；检查摄像头是否被别的 App 占用。")
                time.sleep(0.005)
                continue
            last_sequence = sequence
            frames_seen += 1
            image = rotate(frame.to_ndarray(format="bgr24"), args.rotate)
            height, width = image.shape[:2]
            if first_raw is None:
                first_raw = image.copy()
                print(f"首帧 {width}x{height}（rotate={args.rotate}）")

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            tag_started = time.perf_counter()
            tag_pts, tag_id, side_px = find_tag(gray, args.tag_id)
            tag_ms = (time.perf_counter() - tag_started) * 1000
            block_started = time.perf_counter()
            candidates = detect_red_blocks(image, config)
            block_ms = (time.perf_counter() - block_started) * 1000

            scale = args.tag_mm / side_px if tag_pts is not None else None
            if scale is not None and scale > 0:
                kept, rejected = filter_by_size_mm(candidates, scale,
                                                   args.min_mm, args.max_mm)
            else:
                kept, rejected = [], []
            tag_center = tag_pts.mean(axis=0) if tag_pts is not None else None
            sides_mm = [block["side_mm"] for block in kept]
            frame_dists = ([math.hypot(block["center_px"][0] - tag_center[0],
                                       block["center_px"][1] - tag_center[1]) * scale
                            for block in kept]
                           if scale is not None and tag_center is not None else [])

            heading = float("nan")
            if tag_pts is not None:
                up = tag_pts[0] - tag_pts[3]
                heading = math.degrees(math.atan2(-float(up[1]), float(up[0]))) % 360.0
                tag_sides.append(side_px)
                headings.append(heading)
                centers_x.append(float(tag_center[0]))
                centers_y.append(float(tag_center[1]))
            kept_counts.append(len(kept))
            candidate_counts.append(len(candidates))
            rejected_counts.append(len(rejected))
            all_sides_mm.extend(sides_mm)
            dists_mm.extend(frame_dists)
            tag_times.append(tag_ms)
            block_times.append(block_ms)
            ages.append((time.monotonic() - received_at) * 1000)

            index = len(rows)
            rows.append({
                "frame": index,
                "width": width,
                "height": height,
                "tag": int(tag_pts is not None),
                "tag_id": -1 if tag_id is None else int(tag_id),
                "tag_side_px": round(side_px, 2) if side_px else "",
                "mm_per_px": round(scale, 4) if scale else "",
                "heading_deg": round(heading, 1) if not math.isnan(heading) else "",
                "blocks": len(kept),
                "candidates": len(candidates),
                "rejected": len(rejected),
                "block_side_med_mm": round(float(np.median(sides_mm)), 1) if sides_mm else "",
                "block_dist_med_mm": round(float(np.median(frame_dists)), 1) if frame_dists else "",
                "tag_ms": round(tag_ms, 1),
                "blocks_ms": round(block_ms, 1),
                "pc_age_ms": round(ages[-1], 0),
            })

            canvas = annotate_bench(image, tag_pts, tag_id, side_px, scale or 0.0,
                                    kept, tag_center, rejected=rejected)
            score = (10.0 if tag_pts is not None else 0.0) + len(kept) - 0.001 * len(rejected)
            if score > best_score:
                best_score = score
                best_canvas = canvas
                best_raw = image.copy()

            if (index + 1) % 10 == 0 or index == 0:
                print(f"  帧 {index + 1}/{args.frames}: tag={'Y' if tag_pts is not None else 'N'}"
                      f" blocks={len(kept)}(x{len(rejected)}/{len(candidates)})"
                      f" {tag_ms + block_ms:.0f} ms")

            if image is not None and first_raw is not None and index == 0:
                save_png(out_dir / "frame000_raw.png", image)
                save_png(out_dir / "frame000_annotated.png", canvas)

        elapsed = time.monotonic() - started
        if best_raw is not None:
            save_png(out_dir / "best_raw.png", best_raw)
        if best_canvas is not None:
            save_png(out_dir / "best_annotated.png", best_canvas)

        with (out_dir / "frames.csv").open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        hits = sum(1 for row in rows if row["tag"])
        rate = hits / len(rows) if rows else 0.0

        def pct(values, q):
            return float(np.percentile(values, q)) if values else float("nan")

        print("\n=== 实机台架结果 ===")
        print(f"画面：{width}x{height}，处理 {len(rows)} 帧 / {elapsed:.1f} s"
              f"（{len(rows) / max(elapsed, 1e-6):.1f} 帧/s，收到 {frames_seen} 帧）")
        print(f"tag：{hits}/{len(rows)} = {rate * 100:.1f}% 检出"
              f"（门槛 {args.min_rate * 100:.0f}%）")
        if tag_sides:
            print(f"  tag 边长 px：中位 {np.median(tag_sides):.1f}，"
                  f"范围 {min(tag_sides):.1f}–{max(tag_sides):.1f}")
            print(f"  换算 mm/px：中位 {args.tag_mm / np.median(tag_sides):.4f}")
            mean_heading, spread = circular_spread_deg(headings)
            print(f"  图像朝向：中位 {mean_heading:.1f}°，圆周标准差 {spread:.2f}°")
            print(f"  中心抖动 px：x σ={np.std(centers_x):.2f}，y σ={np.std(centers_y):.2f}")
        print(f"红块候选：每帧中位 {np.median(candidate_counts):.0f}"
              f"（范围 {min(candidate_counts)}–{max(candidate_counts)}）")
        print(f"毫米筛后：每帧中位 {np.median(kept_counts):.0f}"
              f"（范围 {min(kept_counts)}–{max(kept_counts)}），"
              f"每帧剔除中位 {np.median(rejected_counts):.0f}")
        if all_sides_mm:
            print(f"  等效边长 mm：中位 {np.median(all_sides_mm):.1f}，"
                  f"P5–P95 {pct(all_sides_mm, 5):.1f}–{pct(all_sides_mm, 95):.1f}"
                  f"（标称 20）")
        if dists_mm:
            print(f"  到 tag 中心 mm：中位 {np.median(dists_mm):.0f}，"
                  f"范围 {min(dists_mm):.0f}–{max(dists_mm):.0f}")
        print(f"耗时：tag P50 {pct(tag_times, 50):.1f} ms / P95 {pct(tag_times, 95):.1f} ms；"
              f"红块 P50 {pct(block_times, 50):.1f} / P95 {pct(block_times, 95):.1f} ms")
        print(f"PC age：P50 {pct(ages, 50):.0f} ms / P95 {pct(ages, 95):.0f} ms")
        print(f"结果文件：{out_dir}")
        verdict = "通过" if rate >= args.min_rate else "未通过"
        print(f"判定：tag 检出率 {rate * 100:.1f}% -> {verdict}")
        return 0
    finally:
        camera.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
    except Exception as exc:                       # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
