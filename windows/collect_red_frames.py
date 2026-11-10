"""从手机图像流截帧，供红块检测训练集标注。

不分析、不控制底盘。

两种用法：

1) **手动抓帧（推荐，摆一次按一次 S）**：预览窗口里看着画面，摆好一批块就按 `S`
   存一帧，再摆下一批。静态场地只按间隔自动抓会全是重复帧，手动更适合做数据集。
2) 定时抓帧：`--interval` 秒存一帧，自动跳过几乎重复的画面。

用法：
    & 'E:\\scrcpy\\scrcpy-win64-v5.0.1\\.venv\\Scripts\\python.exe' windows\\collect_red_frames.py ^
        --scrcpy-dir 'E:\\scrcpy\\scrcpy-win64-v5.0.1' --manual ^
        --out windows\\test_data\\red_train_20261110

    # 定时（适合场地里有持续变化的场景）
    ... windows\\collect_red_frames.py --interval 0.7 --seconds 180 --out windows\\test_data\\red_train_b

按键：`S` 存一帧，`Q`/`Esc` 退出（手动模式自动打开预览）。
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

from capture_hi import open_camera, rotate                             # noqa: E402
from label_boxes import load_store, save_store                         # noqa: E402
from orientation import decide_rotation, field_rotation                # noqa: E402
from red_blocks import load_red_config, prelabel_boxes                  # noqa: E402

HERE = Path(__file__).resolve().parent
DEFAULT_SCRCPY = Path(r"E:\scrcpy\scrcpy-win64-v5.0.1")
RED_PROFILE = HERE / "red_block_profile.json"


def red_pixels(image) -> int:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = (cv2.inRange(hsv, (0, 70, 40), (10, 255, 255)) |
            cv2.inRange(hsv, (170, 70, 40), (180, 255, 255)))
    return int(np.count_nonzero(mask))


def thumbnail(image) -> np.ndarray:
    return cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (160, 120),
                      interpolation=cv2.INTER_AREA)


def save_frame(image, red: int, frames_dir: Path, rows: list, start_index: int = 0,
               quality: int = 92) -> str | None:
    name = f"frame_{start_index + len(rows):05d}.jpg"
    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        return None
    buffer.tofile(str(frames_dir / name))
    rows.append({"file": name, "wall": round(time.time(), 3), "red_px": red,
                 "width": int(image.shape[1]), "height": int(image.shape[0])})
    return name


def next_frame_index(frames_dir: Path) -> int:
    """Continue numbering after existing frames instead of overwriting them."""
    highest = -1
    for path in frames_dir.glob("frame_*.jpg"):
        digits = path.stem.split("_")[-1]
        if digits.isdigit():
            highest = max(highest, int(digits))
    return highest + 1


def decide_from_camera(camera, timeout: float = 8.0, samples: int = 12):
    """Sample the stream and vote on the rotation that puts yellow on top."""
    votes = []
    last = 0
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and len(votes) < samples:
        if camera.error:
            raise RuntimeError(camera.error)
        frame, _stamp, sequence = camera.latest.get()
        if frame is None or sequence == last:
            time.sleep(0.01)
            continue
        last = sequence
        votes.append(field_rotation(frame.to_ndarray(format="bgr24")))
    return decide_rotation(votes)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scrcpy-dir", type=Path, default=DEFAULT_SCRCPY)
    parser.add_argument("--serial")
    parser.add_argument("--rotate", default="auto",
                        choices=("0", "90", "180", "270", "auto"),
                        help="auto 表示按蓝黄地面自动判断黄区朝上（推荐）")
    parser.add_argument("--max-size", type=int, default=1280)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--manual", action="store_true",
                        help="手动模式：按 S 存一帧（自动打开预览）")
    parser.add_argument("--interval", type=float, default=0.7, help="定时模式的截帧间隔（秒）")
    parser.add_argument("--seconds", type=float, default=180.0, help="0 表示一直跑到 Q/Ctrl+C")
    parser.add_argument("--min-red-px", type=int, default=0,
                        help="红色像素少于此数的帧不存；0 表示不过滤")
    parser.add_argument("--no-dedup", action="store_true", help="不去掉几乎重复的帧")
    parser.add_argument("--show", action="store_true", help="显示预览并允许 Q 提前结束")
    parser.add_argument("--no-prelabel", action="store_true",
                        help="不预填 HSV 检测框（默认会预填，标注时只需改）")
    parser.add_argument("--profile", type=Path, default=RED_PROFILE,
                        help="预填用的红块颜色配置")
    parser.add_argument("--out", type=Path,
                        default=HERE / "test_data" / time.strftime("red_train_%Y%m%d"))
    args = parser.parse_args()
    if args.manual:
        args.show = True
        if args.seconds == 180.0:
            args.seconds = 0.0

    frames_dir = args.out / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    print(f"输出：{frames_dir}")
    if args.manual:
        print("手动模式：摆好一批块按 S 存帧，Q 退出")
    else:
        print(f"定时模式：间隔 {args.interval:.2f} s；时长 {args.seconds or '手动'}"
              f"；红色门槛 {args.min_red_px} px；去重 {'关' if args.no_dedup else '开'}")

    camera, used_size = open_camera(args.scrcpy_dir, args.serial, [args.max_size],
                                     args.fps, 12.0)
    if camera is None:
        return 2

    if args.rotate == "auto":
        decided = decide_from_camera(camera)
        if decided is None:
            print("自动判断朝向失败：画面里找不到蓝黄地面。"
                  "确认镜头对着场地，或用 --rotate 手动指定。", file=sys.stderr)
            camera.close()
            return 3
        args.rotate = decided
        print(f"自动判定朝向：逆时针 {decided}°（黄区在上）")
    else:
        args.rotate = int(args.rotate)

    saved = 0
    skipped_dup = 0
    skipped_red = 0
    last_thumb = None
    next_save = time.monotonic()
    started = time.monotonic()
    rows = []
    last_sequence = 0
    last_image = None
    window = "collect_red_frames | S save | Q quit"

    red_config = None
    if not args.no_prelabel:
        red_config = load_red_config(args.profile) if args.profile.exists() else None
        if red_config is None:
            print(f"提示：找不到预填配置 {args.profile}，这次不预填框")
    labels_path = frames_dir / "labels.json"
    start_index = next_frame_index(frames_dir)
    if start_index:
        print(f"目录里已有 {start_index} 帧，新帧从 frame_{start_index:05d} 继续编号")

    def store_prediction(name: str, image) -> None:
        if red_config is None:
            return
        store = load_store(labels_path)
        entry = store["images"].setdefault(name, {"boxes": [], "reviewed": False})
        if not entry.get("boxes"):
            entry["boxes"] = prelabel_boxes(image, red_config)
        entry["auto"] = True
        entry["width"], entry["height"] = int(image.shape[1]), int(image.shape[0])
        save_store(labels_path, store)

    def write_manifest() -> None:
        """Always write the manifest, even when the window is closed mid-run."""
        manifest = {
            "created": time.strftime("%Y-%m-%d %H:%M:%S"),
            "source": "phone camera stream via scrcpy",
            "mode": "manual" if args.manual else f"interval {args.interval}",
            "arguments": {key: (str(value) if isinstance(value, Path) else value)
                          for key, value in vars(args).items()},
            "camera_size": args.max_size,
            "rotate": args.rotate,
            "frames": rows,
            "saved": saved,
            "skipped_duplicate": skipped_dup,
            "skipped_red": skipped_red,
        }
        (args.out / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    try:
        while True:
            now = time.monotonic()
            if args.seconds and now - started >= args.seconds:
                break
            if camera.error:
                raise RuntimeError(camera.error)
            frame, received_at, sequence = camera.latest.get()
            if frame is None or sequence == last_sequence:
                if args.show and (cv2.waitKey(1) & 0xFF) in (27, ord("q"), ord("Q")):
                    break
                time.sleep(0.005)
                continue
            last_sequence = sequence
            image = rotate(frame.to_ndarray(format="bgr24"), args.rotate)
            last_image = image

            if args.manual:
                red = red_pixels(image)
                preview = cv2.resize(image, (720, 960))
                cv2.putText(preview, f"saved={saved} red={red}px  S=save Q=quit", (8, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow(window, preview)
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    print("预览窗口被关闭，结束采集")
                    break
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key in (ord("s"), ord("S")):
                    if red < args.min_red_px:
                        print(f"  红色像素 {red} 低于门槛 {args.min_red_px}，未保存")
                    else:
                        name = save_frame(image, red, frames_dir, rows, start_index)
                        if name:
                            saved += 1
                            store_prediction(name, image)
                            boxes = len(load_store(labels_path)["images"][name]["boxes"])
                            print(f"  已存 {name}（red={red}px，预填 {boxes} 个框）")
                continue

            if now < next_save:
                continue
            next_save = now + args.interval
            red = red_pixels(image)
            if red < args.min_red_px:
                skipped_red += 1
            else:
                small = thumbnail(image)
                if (not args.no_dedup and last_thumb is not None
                        and float(np.mean(cv2.absdiff(small, last_thumb))) < 1.5):
                    skipped_dup += 1
                else:
                    last_thumb = small
                    name = save_frame(image, red, frames_dir, rows, start_index)
                    if name:
                        saved += 1
                        store_prediction(name, image)
                        if saved % 10 == 0:
                            print(f"  已存 {saved} 帧（重复跳过 {skipped_dup}，"
                                  f"红色不足跳过 {skipped_red}）")
            if args.show:
                preview = cv2.resize(image, (720, 720))
                cv2.putText(preview, f"saved={saved} red={red}px", (8, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("collect_red_frames", preview)
                if (cv2.waitKey(1) & 0xFF) in (27, ord("q"), ord("Q")):
                    break
    except KeyboardInterrupt:
        print("手动停止")
    finally:
        camera.close()
        cv2.destroyAllWindows()
        write_manifest()

    print(f"\n共保存 {saved} 帧到 {frames_dir}")
    print(f"清单：{args.out / 'manifest.json'}")
    if 0 < saved < 30:
        print("提示：不足 30 帧，建议换几批摆法/光照再补；每批只需几帧，但要有差异。")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:                                   # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
