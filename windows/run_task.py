"""Run the competition policy on Windows using the phone and Bluetooth SPP."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import sys
import threading
import time

import cv2
import numpy as np

from calibrated_blocks import detect_calibrated_blocks
from block_tracks import BlockStabilizer
from capture_hi import rotate
from chassis_link import ChassisError, ChassisLink
from field_view import FieldCalibration
from field_vision import HSV, _obstacles, image_to_world
from orientation import decide_rotation, field_rotation
from pose_tag import detect
from tag_detector import TAG_ID
from usb_camera import UsbCamera

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src" / "sweep_robot"))
from sweep_robot.row_policy import HybridCompetitionPolicy

DURATION = 90.0
MAX_AGE = 0.35
PULSE_MS = 100
SEND_MARGIN = 0.04
WINDOW = "Sweep Robot | SPACE start | Q / Esc stop"


@dataclass(frozen=True)
class DriveConfig:
    port: str
    layout: str
    forward_mm_per_pct_s: float
    left_mm_per_pct_s: float
    turn_deg_per_pct_s: float
    max_wheel_percent: int

    def __post_init__(self):
        if not re.fullmatch(r"COM[1-9]\d*", self.port):
            raise ValueError("请输入蓝牙传出串口，例如 COM6")
        if not isinstance(self.layout, str) or sorted(self.layout) != list("ABCD"):
            raise ValueError("轮位必须为本车实测的 FL/FR/RL/RR 四路顺序")
        gains = (self.forward_mm_per_pct_s, self.left_mm_per_pct_s, self.turn_deg_per_pct_s)
        if not all(math.isfinite(gain) and gain > 0 for gain in gains):
            raise ValueError("三个实测运动换算值必须为正数")
        if type(self.max_wheel_percent) is not int or not 1 <= self.max_wheel_percent <= 40:
            raise ValueError("本任务入口的单轮输出上限必须为 1–40%")

    def convert(self, velocity):
        """Keep the mixed wheel budget and recheck the quantized body velocity."""
        gains = (self.forward_mm_per_pct_s / 1000, self.left_mm_per_pct_s / 1000,
                 math.radians(self.turn_deg_per_pct_s))
        requested = [value / gain for value, gain in zip(velocity, gains)]
        scale = min(1., self.max_wheel_percent / max(sum(map(abs, requested)), 1.))
        # Truncate toward zero so rounding cannot exceed the wheel budget.
        command = tuple(int(value * scale) for value in requested)
        estimated = tuple(value * gain for value, gain in zip(command, gains))
        return command, estimated


def configure(path):
    print("输入已在本车确认的参数。轮位是 FL/FR/RL/RR；运动数据使用开环短脉冲实测值。")
    print(f"用 {PULSE_MS} ms 脉冲实测。换算 = 位移(mm)或转角(度) ÷ 指令百分数(如 30) ÷ 脉冲时长(秒)。")
    settings = DriveConfig(
        input("蓝牙传出端口 [COM6]：").strip().upper() or "COM6",
        input("已核对的轮位顺序：").strip().upper(),
        float(input("前进 mm/(%·s)：")),
        float(input("左移 mm/(%·s)：")),
        float(input("逆时针 °/(%·s)：")),
        int(input("已验证的单轮输出上限（1–40%）：")),
    )
    pending = path.with_suffix(path.suffix + ".tmp")
    pending.write_text(json.dumps(vars(settings), indent=2) + "\n", encoding="utf-8")
    pending.replace(path)
    print(f"已保存 {path}")


def load_calibration(path):
    if not path.is_file():
        raise ValueError("缺少场地标定。先在日常工具选 1 拍完整场地，再选 4 点地面四角")
    calib = FieldCalibration.load(path)
    corners = calib.corners
    if (calib.field_mm != 1200 or calib.size != 960 or not calib.source_size
            or len(calib.source_size) != 2 or corners.shape != (4, 2)
            or not np.isfinite(corners).all() or not cv2.isContourConvex(corners)
            or cv2.contourArea(corners, oriented=True) < 100
            or np.any(corners < 2) or np.any(corners >= np.array(calib.source_size) - 2)):
        raise ValueError("需用完整入镜的 1.2 m 场地原图，按左上、右上、右下、左下重新标定")
    return calib


def find_seam(blue, yellow):
    """Require an observed, approximately horizontal blue/yellow transition."""
    height, width = blue.shape
    seams = []
    for x in np.linspace(.08, .92, 9) * width:
        half = width // 30
        band = slice(int(x) - half, int(x) + half)
        b = (blue[:, band] > 0).mean(axis=1)
        y = (yellow[:, band] > 0).mean(axis=1)
        if y[height//12:height//4].mean() < .6 or b[3*height//4:11*height//12].mean() < .6:
            continue
        lo, hi = height * 2 // 5, height * 3 // 5
        south = np.flatnonzero(b[lo:hi] > .65)
        north = np.flatnonzero(y[lo:hi] > .65)
        if south.size and north.size and 0 < south[0] - north[-1] <= height * .03:
            seams.append(lo + (south[0] + north[-1]) / 2)
    if len(seams) < 6 or np.ptp(seams) > height * .025:
        return None
    seam = float(np.median(seams))
    return seam if abs(seam - height / 2) <= height * .025 else None


def observe(image, calib, stamp, stats=None, detector=None):
    """Warp one frame and return the scene the policy sees.

    `stats` is an optional dict that receives this frame's decision counters
    (reason, seam, tag, component/split counts) for diagnostics; it does not
    change the returned scene.
    """
    if stats is not None:
        stats.clear()
        stats.update(reason="", seam_px=None, tag_ok=False, candidates=0,
                     instance_area=None, splits=0, kept=0, dropped=0)
    scene = {"stamp": stamp, "valid": False, "robot": None, "blocks": [], "obstacles": []}

    def fail(label, frame):
        if stats is not None:
            stats["reason"] = label
        return scene, frame, label

    if (image.shape[1], image.shape[0]) != calib.source_size:
        return fail("frame size differs from calibration", image)
    view = calib.warp(image)
    hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, *HSV["blue"])
    yellow = cv2.inRange(hsv, *HSV["yellow"])
    seam = find_seam(blue, yellow)
    if seam is None:
        return fail("field seam missing (yellow top, blue bottom)", view)
    if stats is not None:
        stats["seam_px"] = float(seam)
    pose, reason = detect(view, calib, unique=True)
    if pose is None:
        return fail(f"ID {TAG_ID}: {reason}", view)
    if stats is not None:
        stats["tag_ok"] = True
    metres = calib.mm_per_px / 1000
    # Detection scale remains fixed to the surveyed rectangle. The policy's
    # y=0 follows the observed color seam, which must lie near the field centre.
    centered = np.array([[metres, 0, -.6], [0, -metres, .6], [0, 0, 1]])
    homography = calib.homography()
    bias = .6 - metres * seam
    robot = (pose.x_mm / 1000 - .6, pose.y_mm / 1000 - .6 - bias,
             math.radians(pose.heading_deg))
    if not (-.6 < robot[0] < .6 and -.6 < robot[1] + bias < .6):
        return fail("tag outside field", view)
    if stats is not None:
        stats["bias"] = bias
    tag = {"id": pose.tag_id, "corners_px": image_to_world(np.linalg.inv(homography), pose.corners_px)}
    if detector is None:
        block_stats = {}
        detections = detect_calibrated_blocks(image, centered @ homography, [tag],
                                              stats=block_stats)
        if stats is not None:
            stats.update(block_stats)
    else:
        # Trained detector: same dict shape, still in original-image pixels, so
        # ownership, world coordinates and the policy stay unchanged.
        detections = detector.detect(image)
        if stats is not None:
            stats.update(candidates=len(detections), kept=len(detections))
    marked = view.copy()
    cv2.line(marked, (0, round(seam)), (calib.size - 1, round(seam)), (255, 255, 0), 2)
    for block in detections:
        contour = image_to_world(homography, block["contour_px"]).reshape(-1, 2)
        x, y = image_to_world(homography, [block["center_px"]])[0]
        if not (0 <= x < calib.size and 0 <= y < calib.size):
            continue
        dx, dy = np.max(np.abs(contour - [x, y]), axis=0) * metres
        radius = math.ceil(max(dx, dy) / metres + 10 / calib.mm_per_px)
        xx, yy = round(float(x)), round(float(y))
        x0, x1 = max(0, xx-radius), min(calib.size, xx+radius+1)
        y0, y1 = max(0, yy-radius), min(calib.size, yy+radius+1)
        around = np.ones((y1-y0, x1-x0), dtype=np.uint8)
        cv2.fillPoly(around, [np.int32(contour - [x0, y0])], 0)
        b = np.count_nonzero(blue[y0:y1, x0:x1] & around.astype(bool))
        g = np.count_nonzero(yellow[y0:y1, x0:x1] & around.astype(bool))
        scene["blocks"].append((x*metres-.6, (seam-y)*metres,
                                max(.005, dx), max(.005, dy), b > g))
        cv2.polylines(marked, [np.int32(contour)], True, (0, 235, 0) if b > g else (0, 180, 255), 2)
    size = calib.size - 1
    border = [(0, 0), (size, 0), (size, size), (0, size)]
    positions, extents = _obstacles(hsv, centered, border, bias, robot)
    scene.update(valid=True, robot=robot, obstacles=[(*p, *h) for p, h in zip(positions, extents)])
    scene["seam_bias"] = bias
    cv2.polylines(marked, [np.int32(pose.corners_px)], True, (255, 255, 255), 2)
    return scene, marked, "vision ready"


def latest_scene(new, held):
    """Serve the newest scene, or hold the last valid one inside MAX_AGE.

    One frame with a lost tag or a missing colour seam used to look like "no
    robot" to the main loop and ended the run. The freshness check in `fresh()`
    still bounds the hold to MAX_AGE, so a real vision loss aborts as before.
    """
    if new["valid"]:
        return new, new
    if held is not None and new["stamp"] - held["stamp"] <= MAX_AGE:
        return held, held
    return new, held


class VisionWorker:
    def __init__(self, camera, calib, rotation, stabilizer=None, detector=None):
        self.camera, self.calib, self.rotation = camera, calib, rotation
        self.stabilizer = stabilizer
        self.detector = detector
        self.latest = None
        self.held = None
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name="task-vision")

    def _decide_rotation(self):
        """Vote on the rotation that puts yellow above blue, and report the raw size."""
        votes = []
        last = 0
        size = None
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline and len(votes) < 12:
            frame, _stamp, sequence = self.camera.latest.get()
            if frame is None or sequence == last:
                time.sleep(.01)
                continue
            last = sequence
            image = frame.to_ndarray(format="bgr24")
            size = (image.shape[1], image.shape[0])
            votes.append(field_rotation(image))
        return decide_rotation(votes), size

    def run(self):
        last = 0
        try:
            if self.rotation is None:
                decided, size = self._decide_rotation()
                if decided is None or size is None:
                    raise RuntimeError("无法判断场地朝向：画面里找不到蓝黄地面，"
                                       "请调整取景或用 --rotate 指定")
                rotated = (size[0], size[1]) if decided in (0, 180) else (size[1], size[0])
                if tuple(self.calib.source_size or ()) != rotated:
                    raise RuntimeError(
                        f"自动判定需要旋转 {decided}°，旋转后画面 {rotated} 与标定 "
                        f"{tuple(self.calib.source_size or ())} 不一致；"
                        f"请按 --rotate {decided} 重新标定四角")
                self.rotation = decided
                print(f"自动判定朝向：逆时针 {decided}°（黄区在上）", flush=True)
            while not self.stop.is_set():
                frame, stamp, sequence = self.camera.latest.get()
                if frame is None or sequence == last:
                    self.stop.wait(.01)
                    continue
                last = sequence
                began = time.monotonic()
                image = rotate(frame.to_ndarray(format="bgr24"), self.rotation)
                scene, marked, label = observe(image, self.calib, stamp,
                                               detector=self.detector)
                # Merge over-split halves, drop one-frame ghosts and smooth the
                # centres. Invalid frames do not advance the tracker, so a short
                # vision gap does not erase blocks that are still there.
                if self.stabilizer is not None and scene["valid"]:
                    scene["blocks"] = self.stabilizer.update(scene["blocks"], stamp)
                self.latest, self.held = latest_scene(scene, self.held)
                self.stop.wait(max(0., .10 - (time.monotonic() - began)))
        except Exception as error:
            self.error = error


def fresh(scene, now):
    return scene is not None and scene["valid"] and 0 <= now - scene["stamp"] <= MAX_AGE


def can_start(scene):
    x, y, yaw = scene["robot"]
    return abs(x) < .49 and -.55 < y < -.20 and math.sin(yaw) >= math.cos(math.pi / 6)


def pose_changed_too_fast(scene, previous):
    dt = scene["stamp"] - previous["stamp"]
    if not 0 < dt <= MAX_AGE:
        return True
    yaw = scene["robot"][2] - previous["robot"][2]
    return (math.dist(scene["robot"][:2], previous["robot"][:2]) > .025 + .5 * dt
            or abs(math.atan2(math.sin(yaw), math.cos(yaw))) > .18 + 1.5 * dt)


def motion_safe(policy, scene, velocity, now):
    # The policy centres y on the seam. Keep its north/south limits relative
    # to the surveyed walls as well when that seam is off centre.
    y = scene["robot"][1] + scene["seam_bias"]
    yaw = scene["robot"][2]
    vx, vy, _ = velocity
    next_y = y + (math.sin(yaw) * vx + math.cos(yaw) * vy) * .20
    violation = max(0., -.548-y, y-.505)
    return max(0., -.548-next_y, next_y-.505) <= violation and policy.safe(velocity, now)


def run(args, calib, drive):
    policy = HybridCompetitionPolicy(duration=DURATION)
    camera = UsbCamera(args.scrcpy_dir, max_size=args.max_size)
    detector = None
    use_onnx = args.detector == "onnx" or (args.detector == "auto"
                                           and Path(args.onnx_model).exists())
    if use_onnx:
        from onnx_blocks import OnnxBlockDetector
        detector = OnnxBlockDetector(args.onnx_model, imgsz=(1280, 960),
                                     device=args.onnx_device)
        print(f"红块检测：ONNX {args.onnx_model}（后端 {detector.provider}）", flush=True)
    else:
        if args.detector == "onnx":
            raise RuntimeError(f"找不到模型 {args.onnx_model}")
        print("红块检测：HSV（未找到模型或指定了 hsv）", flush=True)
    vision = VisionWorker(camera, calib, args.rotate,
                          None if args.no_track else BlockStabilizer(), detector)
    link = ChassisLink(drive.port, layout=f"{drive.layout} 1 1 1 1", strict=True) if drive else None
    started = None
    last_scene = None
    stable = 0
    last_label = None
    active_velocity = None
    active_until = 0.
    try:
        camera.start()
        vision.thread.start()
        if link:
            link.start()
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 720, 720)
        waiting = np.zeros((calib.size, calib.size, 3), np.uint8)
        next_tick = time.monotonic()
        camera_deadline = next_tick + 15.
        while True:
            now = time.monotonic()
            if camera.error or vision.error:
                raise RuntimeError(camera.error or vision.error)
            if link and link.error:
                raise ChassisError(link.error)
            snapshot = vision.latest
            scene, frame, label = snapshot if snapshot is not None else (None, waiting, "waiting for camera")
            if scene is None and now > camera_deadline:
                raise RuntimeError("未收到手机画面")
            valid = fresh(scene, now)
            if scene is not None and scene is not last_scene:
                if started is not None and valid and pose_changed_too_fast(scene, last_scene):
                    raise RuntimeError("位姿跳变或识别帧间隔过长，任务终止")
                if started is None and valid and can_start(scene):
                    contiguous = fresh(last_scene, now)
                    steady = (contiguous
                              and math.dist(scene["robot"][:2], last_scene["robot"][:2]) < .01
                              and abs(scene["robot"][2] - last_scene["robot"][2]) < .10)
                    stable = stable + 1 if steady else 1
                else:
                    stable = 0
                policy.update(scene)
                last_scene = scene
            if valid and not policy.valid:
                valid = False
                label = "invalid scene geometry"
            if not valid:
                stable = 0
                if scene is not None and scene["valid"] and policy.valid:
                    label = "vision stale"
            usable = valid and now - scene["stamp"] < MAX_AGE - PULSE_MS / 1000 - SEND_MARGIN
            ready = usable and stable >= 3 and (link is None or link.ready and not link.busy)
            if started is None:
                if ready:
                    label = "SPACE to start" if link else "PREVIEW | SPACE to start policy"
                elif valid:
                    if not usable:
                        label = "waiting for newer frames"
                    elif not can_start(scene):
                        label = "place robot in blue start zone, facing yellow"
                    elif stable < 3:
                        label = f"waiting for stable ID {TAG_ID}"
                    else:
                        label = "waiting for Bluetooth"
            else:
                if now >= started + DURATION:
                    print("90 秒到时，停车")
                    return 0
                if not valid or (link and not link.ready):
                    raise RuntimeError(f"任务终止，视觉或底盘失效：{label}")
                target = policy.step(now)
                if link and active_velocity and now < active_until:
                    if not any(target) or not motion_safe(policy, scene, active_velocity, now):
                        link.stop()
                        active_velocity = None
                if link and not link.busy:
                    command, estimated = drive.convert(target)
                    latest = vision.latest[0]
                    after_plan = time.monotonic()
                    end = min(scene["stamp"] + MAX_AGE, started + DURATION)
                    if (any(command) and fresh(latest, after_plan)
                            and motion_safe(policy, scene, estimated, after_plan)):
                        if link.pulse(*command, PULSE_MS, send_by=end - PULSE_MS / 1000 - SEND_MARGIN):
                            active_velocity, active_until = estimated, end
                label = f"{'RUN' if link else 'PREVIEW'} | {policy.phase} | {now-started:.1f}s"
            display = cv2.resize(frame, (720, 720), interpolation=cv2.INTER_AREA)
            cv2.rectangle(display, (0, 0), (720, 34), (20, 20, 20), -1)
            cv2.putText(display, label, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, .56, (240, 240, 240), 1)
            cv2.imshow(WINDOW, display)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                return 0
            if (key == ord(" ") and ready and started is None
                    and fresh(vision.latest[0], time.monotonic())):
                started = time.monotonic()
                policy.start_time = started
                print("任务开始，90 秒计时")
            status = (policy.phase, policy.reason) if started is not None else label
            if status != last_label:
                print(status, flush=True)
                last_label = status
            next_tick = max(next_tick + 1/30, time.monotonic())
            time.sleep(max(0., next_tick - time.monotonic()))
    finally:
        if link:
            link.close()
        vision.stop.set()
        camera.close()
        if vision.thread.is_alive():
            vision.thread.join(timeout=1.)
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--configure", action="store_true", help="保存本车实测运动参数")
    modes.add_argument("--preview", action="store_true", help="运行视觉与策略，禁用串口输出")
    parser.add_argument("--config", type=Path, default=HERE / "task_drive.json")
    parser.add_argument("--calibration", type=Path, default=HERE / "field_calib.json")
    parser.add_argument("--scrcpy-dir", type=Path, default=Path(r"E:\scrcpy\scrcpy-win64-v5.0.1"))
    parser.add_argument("--rotate", default="auto",
                        choices=("0", "90", "180", "270", "auto"),
                        help="auto 表示按蓝黄地面自动判断黄区朝上（推荐）")
    parser.add_argument("--max-size", type=int, default=1280)
    parser.add_argument("--no-track", action="store_true",
                        help="关闭块级合并/确认/平滑，回到逐帧原始列表")
    parser.add_argument("--detector", choices=("auto", "hsv", "onnx"), default="auto",
                        help="红块来源：auto=有模型就用 onnx，否则 hsv")
    parser.add_argument("--onnx-model", type=Path,
                        default=HERE / "models" / "red_block.onnx")
    parser.add_argument("--onnx-device", choices=("auto", "gpu", "cpu"), default="auto",
                        help="ONNX 推理后端；auto 有 CUDA 就用 GPU")
    args = parser.parse_args()
    args.rotate = None if args.rotate == "auto" else int(args.rotate)
    try:
        if args.configure:
            configure(args.config)
            return 0
        calib = load_calibration(args.calibration)
        if not args.preview and not args.config.is_file():
            raise ValueError("缺少本车运动参数，请在日常工具选 11 保存本车配置")
        drive = None if args.preview else DriveConfig(**json.loads(args.config.read_text(encoding="utf-8")))
        return run(args, calib, drive)
    except (RuntimeError, OSError, ValueError, TypeError, KeyError, cv2.error) as error:
        print(f"任务退出：{error}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
