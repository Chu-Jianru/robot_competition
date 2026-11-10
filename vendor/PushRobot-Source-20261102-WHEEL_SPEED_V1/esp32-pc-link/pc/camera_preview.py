"""Standalone camera preview. It has no connection to motion control.

Examples:
    python camera_preview.py --list
    python camera_preview.py --source 0
    python camera_preview.py --source "rtsp://..."
    python camera_preview.py --synthetic
    python camera_preview.py --synthetic --headless --frames 30

The iPhone/Camo device is not auto-detected. Install Camo Camera on the
iPhone and Camo Studio on Windows first, confirm the picture in the Windows
Camera app, then run --list and try the reported indices.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from camera_source import CameraSource, parse_source

CONFIG_PATH = Path(__file__).with_name("camera_config.json")


def load_config() -> dict:
    try:
        data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def config_int(config: dict, key: str, fallback: int) -> int:
    try:
        return int(config.get(key, fallback))
    except (TypeError, ValueError):
        return fallback


def open_capture(source, backend: str):
    if not isinstance(source, int):
        return cv2.VideoCapture(source)
    if backend == "dshow" and hasattr(cv2, "CAP_DSHOW"):
        return cv2.VideoCapture(source, cv2.CAP_DSHOW)
    if backend == "msmf" and hasattr(cv2, "CAP_MSMF"):
        return cv2.VideoCapture(source, cv2.CAP_MSMF)
    return cv2.VideoCapture(source)


def list_device_names() -> None:
    try:
        from pygrabber.dshow_graph import FilterGraph
    except Exception as exc:
        print(f"无法列出设备名（需要 pygrabber）：{exc}")
        print("请先运行 setup.cmd。")
        return
    try:
        names = FilterGraph().get_input_devices()
    except Exception as exc:
        print(f"枚举 DirectShow 设备失败：{exc}")
        return
    print("DirectShow 设备名（编号顺序通常与 MSMF 一致，但不保证）：")
    for index, name in enumerate(names):
        print(f"  {index}: {name}")
    print("请用 --source <编号> --backend msmf 打开，并用画面确认。")


def list_cameras(max_index: int) -> None:
    print("探测摄像头编号（只报告“能打开并读到画面”，不代表已经确认是 Camo）：")
    print("同一编号在不同后端的设备可能不同，请以实际画面为准。")
    found = []
    for index in range(max_index + 1):
        candidates = []
        if hasattr(cv2, "CAP_MSMF"):
            candidates.append(("Media Foundation", cv2.CAP_MSMF))
        if hasattr(cv2, "CAP_DSHOW"):
            candidates.append(("DirectShow", cv2.CAP_DSHOW))
        candidates.append(("默认后端", None))
        for backend_name, backend in candidates:
            if backend is None:
                capture = cv2.VideoCapture(index)
            else:
                capture = cv2.VideoCapture(index, backend)
            if not capture.isOpened():
                capture.release()
                continue
            ok, frame = capture.read()
            if ok and frame is not None:
                height, width = frame.shape[:2]
                fps = capture.get(cv2.CAP_PROP_FPS)
                found.append(f"{index} ({backend_name})")
                print(
                    f"  编号 {index} [{backend_name}]: 可打开，"
                    f"{width}x{height}，上报 FPS={fps:.1f}"
                )
            capture.release()
    if not found:
        print("  没有找到可读画面的摄像头。")
        print("  请在 Windows 相机应用中先确认 Camo Studio 画面正常。")
    else:
        print(f"可用组合：{found}")
        print("请逐个运行 --source <编号> --backend msmf|dshow 用眼睛确认哪一个是 iPhone/Camo。")


def synthetic_frame(index: int, width: int, height: int):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :] = (30, 30, 30)
    x = int((index * 9) % max(1, width - 160))
    y = int((index * 5) % max(1, height - 160))
    cv2.rectangle(frame, (x, y), (x + 150, y + 150), (0, 180, 255), -1)
    cv2.putText(
        frame,
        "SYNTHETIC - NOT A CAMERA",
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"frame {index}",
        (30, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (200, 200, 200),
        2,
        cv2.LINE_AA,
    )
    return frame


def save_snapshot(frame, directory: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / time.strftime("frame_%Y%m%d_%H%M%S.png")
    cv2.imwrite(str(path), frame)
    return path


def run_preview(args: argparse.Namespace) -> int:
    source = parse_source(args.source) if not args.synthetic else None
    capture = None
    if not args.synthetic:
        capture = CameraSource(
            source,
            width=args.width,
            height=args.height,
            backend=args.backend,
        )
        if not capture.open():
            print(f"无法打开视频源：{args.source!r}")
            print("建议：")
            print("  1. 确认 Camo Studio 已运行，iPhone 已通过 USB 连接。")
            print("  2. 在 Windows 相机应用中先看到画面。")
            print("  3. 运行 python camera_preview.py --list 找编号。")
            print("  4. 若是 RTSP/HTTP 流，确认地址和网络可达。")
            return 2
        print(f"已打开视频源 {args.source!r}，按 q/ESC 退出，s 保存截图。")
    else:
        print("使用模拟画面（不是摄像头，仅用于离线验证显示链路）。")
        print("按 q/ESC 退出，s 保存截图。")

    frames = 0
    failures = 0
    started = time.monotonic()
    last_report = started
    fps = 0.0
    last_snapshot = None
    last_size = None

    try:
        while True:
            if args.synthetic:
                frame = synthetic_frame(frames, args.width or 1280, args.height or 720)
            else:
                assert capture is not None
                frame = capture.read()

            if frame is None:
                failures += 1
                if failures >= 30:
                    print("连续读取失败 30 次，停止。请检查摄像头/流。")
                    break
                time.sleep(0.05)
                continue
            failures = 0
            frames += 1

            now = time.monotonic()
            elapsed = now - last_report
            if elapsed >= 0.5:
                fps = (frames - 1) / max(1e-6, now - started)
                last_report = now

            height, width = frame.shape[:2]
            last_size = (width, height)
            cv2.putText(
                frame,
                f"{width}x{height}  {fps:5.1f} FPS  q=quit s=snapshot",
                (16, height - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            if not args.headless:
                cv2.imshow("Camera preview - no motion control", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                if key == ord("s"):
                    last_snapshot = save_snapshot(frame, Path(args.snapshot_dir))
                    print(f"截图已保存：{last_snapshot}")

            if args.headless and args.frames > 0 and frames >= args.frames:
                break
    finally:
        if capture is not None:
            capture.close()
        if not args.headless:
            cv2.destroyAllWindows()

    elapsed = max(1e-6, time.monotonic() - started)
    print(
        f"处理 {frames} 帧，用时 {elapsed:.2f}s，平均 {frames / elapsed:.2f} FPS。"
    )
    if last_size is not None:
        print(f"实际分辨率：{last_size[0]}x{last_size[1]}")
    if last_snapshot is not None:
        print(f"最后一帧截图：{last_snapshot}")
    return 0 if frames > 0 else 3


def parse_args() -> argparse.Namespace:
    config = load_config()
    source_default = str(config.get("source", "0"))
    backend_default = str(config.get("backend", "auto"))
    if backend_default not in ("auto", "dshow", "msmf"):
        backend_default = "auto"
    parser = argparse.ArgumentParser(description="独立摄像头预览（不连接运动控制）")
    parser.add_argument(
        "--names",
        action="store_true",
        help="列出 DirectShow 摄像头设备名（用于区分 Camo 和笔记本摄像头）",
    )
    parser.add_argument("--list", action="store_true", help="探测可用的摄像头编号")
    parser.add_argument(
        "--max-index", type=int, default=5, help="--list 探测的最大编号（默认 5）"
    )
    parser.add_argument(
        "--source",
        default=source_default,
        help="摄像头编号或视频流/文件地址；默认读取 camera_config.json",
    )
    parser.add_argument(
        "--backend",
        choices=["auto", "dshow", "msmf"],
        default=backend_default,
        help="Windows 摄像头后端；默认读取 camera_config.json",
    )
    parser.add_argument(
        "--width", type=int, default=config_int(config, "width", 1280)
    )
    parser.add_argument(
        "--height", type=int, default=config_int(config, "height", 720)
    )
    parser.add_argument(
        "--synthetic",
        action="store_true",
        help="生成模拟画面，用于离线验证显示链路",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="不打开窗口，只读取帧（用于自动检查）",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="headless 模式下处理的帧数；0 表示一直运行",
    )
    parser.add_argument("--snapshot-dir", default="snapshots")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.names:
        list_device_names()
        return 0
    if args.list:
        list_cameras(args.max_index)
        return 0
    if args.headless and args.frames == 0 and not args.synthetic:
        print("headless 模式请同时指定 --frames N，避免无限运行。")
        return 2
    return run_preview(args)


if __name__ == "__main__":
    raise SystemExit(main())
