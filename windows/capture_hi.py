"""高分辨率取图 + 实时识别（tag / 红块）。

三种用法：
1) 纯取图：只显示画面，S 存图。
2) 台架自检（--bench）：不需要场地，用 tag 自身 50 mm 反推 mm/px，
   实时统计红块数量与等效边长、到 tag 的距离。
3) 场地模式（--config）：四角校正后测 tag 位姿，另开 "field view" 窗口。

本脚本只读画面，不发任何底盘指令。

用法：
    & 'E:\\scrcpy\\scrcpy-win64-v5.0.1\\.venv\\Scripts\\python.exe' windows\\capture_hi.py ^
        --scrcpy-dir 'E:\\scrcpy\\scrcpy-win64-v5.0.1' --rotate 90 --max-size 1280 --bench

按键：S 存图（台架/场地模式同时存标注图），R 旋转，Q 退出。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from field_view import FieldCalibration, draw_overlay     # noqa: E402
from pose_tag import detect as detect_tag                 # noqa: E402
from tag_detector import TAG_ID                           # noqa: E402
from usb_camera import UsbCamera                          # noqa: E402

ROTATIONS = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def rotate(frame, angle: int):
    return cv2.rotate(frame, ROTATIONS[angle]) if angle else frame


def wait_first_frame(camera, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if camera.error:
            print(f"  采集线程报错：{camera.error}")
            return False
        frame, _received_at, sequence = camera.latest.get()
        if sequence > 0 and frame is not None:
            print("  首帧到达")
            return True
        time.sleep(0.05)
    print(f"  {timeout_s:.0f} s 内没有收到任何一帧。服务端最后几行：")
    for line in list(camera.logs)[-8:]:
        print(f"    {line}")
    return False


def open_camera(scrcpy_dir: Path, serial, sizes, fps: int, timeout_s: float):
    for index, size in enumerate(sizes):
        print(f"[{index + 1}/{len(sizes)}] 尝试 max_size={size} fps={fps} …")
        camera = UsbCamera(scrcpy_dir, serial, max_size=size, camera_fps=fps)
        try:
            camera.start()
        except Exception as exc:                       # noqa: BLE001
            print(f"  启动失败：{exc}")
            camera.close()
            continue
        print("  服务端已就绪，等待首帧 …")
        if wait_first_frame(camera, timeout_s):
            return camera, size
        camera.close()
        print(f"  max_size={size} 无画面，换下一档。")
    return None, None


def draw_pose(view: np.ndarray, pose, mm_per_px: float) -> np.ndarray:
    canvas = view.copy()
    if pose is None or pose.corners_px is None:
        cv2.putText(canvas, "TAG: not found", (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2)
        return canvas
    quad = pose.corners_px.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(canvas, [quad], True, (0, 255, 0), 2)
    cx, cy = int(pose.center_px[0]), int(pose.center_px[1])
    cv2.circle(canvas, (cx, cy), 4, (0, 0, 255), -1)
    dx = int(40 * np.cos(np.radians(pose.heading_deg)))
    dy = int(-40 * np.sin(np.radians(pose.heading_deg)))
    cv2.arrowedLine(canvas, (cx, cy), (cx + dx, cy + dy), (0, 200, 255), 2, tipLength=0.3)
    text = (f"x={pose.x_mm:6.1f} y={pose.y_mm:6.1f} h={pose.heading_deg:5.1f} "
            f"side={pose.side_mm:4.1f}mm")
    cv2.putText(canvas, text, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scrcpy-dir", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--serial")
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=90)
    parser.add_argument("--max-size", type=int, default=1280)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--no-fallback", action="store_true")
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parent / "captures_hi")
    parser.add_argument("--config", type=Path, help="field_view 标定文件；给出时启用场地位姿")
    parser.add_argument("--bench", action="store_true",
                        help="无场地台架自检：tag 边长自换算 mm/px，同时统计红块")
    parser.add_argument("--profile", type=Path,
                        default=Path(__file__).resolve().with_name("red_block_profile.json"))
    parser.add_argument("--tag-mm", type=float, default=50.0)
    parser.add_argument("--tag-id", type=int, default=TAG_ID)
    parser.add_argument("--min-mm", type=float, default=12.0, help="红块等效边长下限")
    parser.add_argument("--max-mm", type=float, default=32.0, help="红块等效边长上限")
    parser.add_argument("--detect-hz", type=float, default=12.0, help="识别频率，预览不降频")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    calib = FieldCalibration.load(args.config) if args.config else None
    if calib:
        print(f"场地模式：{args.config} -> {calib.size}x{calib.size}，{calib.mm_per_px:.3f} mm/px")

    find_tag = None
    annotate_bench = None
    red_config = None
    detect_blocks = None
    if args.bench:
        from bench_check import annotate as annotate_bench, find_tag
        from red_blocks import detect_red_blocks, filter_by_size_mm, load_red_config
        detect_blocks = detect_red_blocks
        size_filter = filter_by_size_mm
        red_config = load_red_config(args.profile) if args.profile.exists() else None
        print(f"台架模式：红块配置 {args.profile} (exists={args.profile.exists()})；"
              f"用 tag {args.tag_mm:.0f} mm 边长自换算 mm/px；"
              f"毫米筛 {args.min_mm:.0f}–{args.max_mm:.0f} mm")

    sizes = [args.max_size]
    if not args.no_fallback:
        for candidate in (1280, 960):
            if candidate < args.max_size and candidate not in sizes:
                sizes.append(candidate)

    print(f"输出目录：{args.out}")
    camera, used_size = open_camera(args.scrcpy_dir, args.serial, sizes, args.fps, args.timeout)
    if camera is None:
        print("所有分辨率档位都没拿到画面。请检查：", file=sys.stderr)
        print("  1) 手机相机是否被其他 App 占用；2) 是否还开着另一个客户端；3) 换 USB 口。",
              file=sys.stderr)
        return 2

    rotation = args.rotate
    window = f"raw | S save | R rotate | Q quit | max_size={used_size}"
    field_window = "field view | calibrated"
    saved = 0
    last_sequence = 0
    last_report = time.monotonic()
    frames_since_report = 0
    last_detect = 0.0
    min_detect_dt = 1.0 / max(1.0, args.detect_hz)
    pose, reason = None, "not_run"
    bench_stats: dict[str, object] = {}
    canvas_extra = None

    try:
        while True:
            frame, received_at, sequence = camera.latest.get()
            now = time.monotonic()
            if camera.error:
                raise RuntimeError(camera.error)
            if sequence == last_sequence:
                if now - received_at > 3.0:
                    raise RuntimeError("画面停滞超过 3 s；重新运行并检查摄像头占用。")
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    break
                time.sleep(0.01)
                continue
            last_sequence = sequence
            frames_since_report += 1

            image = rotate(frame.to_ndarray(format="bgr24"), rotation)
            height, width = image.shape[:2]

            if now - last_detect >= min_detect_dt:
                last_detect = now
                if args.bench:
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    tag_pts, tag_id, side_px = find_tag(gray, args.tag_id)
                    scale = args.tag_mm / side_px if tag_pts is not None else None
                    candidates = detect_blocks(image, red_config)
                    if scale:
                        kept, rejected = size_filter(candidates, scale, args.min_mm, args.max_mm)
                    else:
                        kept, rejected = candidates, []
                    tag_center = tag_pts.mean(axis=0) if tag_pts is not None else None
                    sides = [b["side_mm"] for b in kept] if scale else []
                    dists = ([math.hypot(b["center_px"][0] - tag_center[0],
                                         b["center_px"][1] - tag_center[1]) * scale for b in kept]
                             if scale and tag_center is not None else [])
                    bench_stats = {
                        "side_px": side_px, "scale": scale, "candidates": len(candidates),
                        "blocks": len(kept), "rejected": len(rejected),
                        "median_mm": float(np.median(sides)) if sides else None,
                        "min_mm": float(min(sides)) if sides else None,
                        "max_mm": float(max(sides)) if sides else None,
                        "median_d": float(np.median(dists)) if dists else None,
                    }
                    canvas_extra = annotate_bench(image, tag_pts, tag_id, side_px,
                                                  scale or 0.0, kept, tag_center,
                                                  rejected=rejected)
                if calib is not None:
                    view = calib.warp(image)
                    pose, reason = detect_tag(view, calib, args.tag_mm, args.tag_id)
                    field_canvas = draw_pose(draw_overlay(view, calib), pose, calib.mm_per_px)
                    cv2.putText(field_canvas, reason, (12, calib.size - 16),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0) if pose else (0, 0, 255), 2)
                    cv2.imshow(field_window, field_canvas)

            display = canvas_extra if canvas_extra is not None else image
            dheight, dwidth = display.shape[:2]
            scale_preview = min(1280 / dwidth, 720 / dheight, 1.0)
            preview = display if scale_preview >= 1.0 else cv2.resize(
                display, (int(dwidth * scale_preview), int(dheight * scale_preview)),
                interpolation=cv2.INTER_AREA)
            age_ms = (now - received_at) * 1000
            fps_now = frames_since_report / max(1e-6, now - last_report)
            cv2.putText(preview, f"{width}x{height} | {fps_now:4.1f} fps | age={age_ms:.0f} ms | saved={saved}",
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(window, preview)

            if now - last_report >= 1.0:
                parts = [f"{width}x{height}", f"{fps_now:4.1f} fps",
                         f"PC age={age_ms:5.0f} ms", f"saved={saved}"]
                if args.bench:
                    if bench_stats.get("scale"):
                        parts.append(f"tag {bench_stats['side_px']:.1f}px -> {bench_stats['scale']:.3f} mm/px")
                        if bench_stats.get("median_mm"):
                            parts.append(f"blocks={bench_stats['blocks']}"
                                         f"(x{bench_stats['rejected']} of {bench_stats['candidates']}) "
                                         f"side={bench_stats['median_mm']:.1f}mm "
                                         f"[{bench_stats['min_mm']:.1f}-{bench_stats['max_mm']:.1f}]")
                            if bench_stats.get("median_d"):
                                parts.append(f"d~{bench_stats['median_d']:.0f}mm")
                        else:
                            parts.append(f"blocks=0 (x{bench_stats['rejected']} of {bench_stats['candidates']})")
                    else:
                        parts.append("tag: not found")
                if calib is not None and not args.bench:
                    parts.append(f"tag x={pose.x_mm:.1f} y={pose.y_mm:.1f} h={pose.heading_deg:.1f}"
                                 if pose else f"tag={reason}")
                print(" | ".join(parts))
                last_report = now
                frames_since_report = 0

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord("r"), ord("R")):
                rotation = (rotation + 90) % 360
            elif key in (ord("s"), ord("S")):
                stamp = time.strftime("%H%M%S") + f"_{time.time_ns() % 1000:03d}"
                raw_path = args.out / f"hi_{stamp}_{width}x{height}.png"
                ok, buf = cv2.imencode(".png", image)
                if not ok:
                    print("编码失败，未保存")
                    continue
                buf.tofile(str(raw_path))
                print(f"已保存 {raw_path}")
                if canvas_extra is not None:
                    marked_path = args.out / f"hi_{stamp}_marked.png"
                    ok, buf = cv2.imencode(".png", canvas_extra)
                    if ok:
                        buf.tofile(str(marked_path))
                        print(f"已保存 {marked_path}")
                saved += 1
        return 0
    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
    except Exception as exc:                       # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
