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
from chassis_link import ChassisError, ChassisLink, STREAM_MIN_MS, STREAM_SEND_MARGIN_S
from detection_flags import obstacles_enabled
from field_view import FieldCalibration
from field_vision import HSV, _obstacles, find_seam, image_to_world, rectified_floor_border
from orientation import decide_rotation, field_rotation
from pose_tag import detect
from tag_detector import TAG_ID, draw_tag_heading
from usb_camera import UsbCamera

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src" / "sweep_robot"))
from sweep_robot.competition_policy import wrap
from sweep_policy import OUTLINE, SweepPolicy

DURATION = 90.0
MAX_AGE = 0.35
STREAM_RESERVE = STREAM_MIN_MS / 1000 + STREAM_SEND_MARGIN_S
BLOCK_OVERLAP_LIMIT = .70
WINDOW = "Sweep Robot | B/Y home | SPACE start | Q / Esc stop"

# Roof tag sits ahead of chassis centre and right of its lateral midpoint.
# These outlines follow the observed real car in tag-relative metres (forward, left).
ROBOT_OUTLINE = np.array(OUTLINE)
# Wheels and bumpers sit below the tag plane and can project outside the template.
OBSTACLE_OUTLINE_PAD_MM = 18
OBSTACLE_OWN_RADIUS_MM = 1000 * np.linalg.norm(ROBOT_OUTLINE, axis=1).max() + OBSTACLE_OUTLINE_PAD_MM
ROBOT_WHEELS = (
    np.array([(-.014, .052), (-.014, .101), (-.144, .101), (-.144, .052)]),
    np.array([(-.014, -.071), (-.014, -.031), (-.144, -.031), (-.144, -.071)]),
)


@dataclass(frozen=True)
class DriveConfig:
    port: str
    layout: str
    forward_counts_per_mm: float
    left_counts_per_mm: float
    turn_counts_per_deg: float
    max_wheel_counts_per_s: int
    encoder_signs: tuple[int, ...]

    def __post_init__(self):
        if not re.fullmatch(r"COM[1-9]\d*", self.port):
            raise ValueError("请输入蓝牙传出串口，例如 COM6")
        if not isinstance(self.layout, str) or sorted(self.layout) != list("ABCD"):
            raise ValueError("轮位必须为本车实测的 FL/FR/RL/RR 四路顺序")
        gains = (self.forward_counts_per_mm, self.left_counts_per_mm, self.turn_counts_per_deg)
        if not all(math.isfinite(gain) and gain > 0 for gain in gains):
            raise ValueError("三个实测运动换算值必须为正数")
        if type(self.max_wheel_counts_per_s) is not int or not 1 <= self.max_wheel_counts_per_s <= 1_000_000:
            raise ValueError("单轮计数速度上限无效，请重新标定")
        if len(self.encoder_signs) != 4 or any(sign not in (-1, 1) for sign in self.encoder_signs):
            raise ValueError("编码器符号无效，请重新标定")

    def convert(self, velocity):
        """Keep the mixed wheel budget and recheck the quantized body velocity."""
        requested = [value * gain for value, gain in zip(velocity, self.gains)]
        scale = min(1., self.max_wheel_counts_per_s / max(sum(map(abs, requested)), 1.))
        # Truncate toward zero so rounding cannot exceed the wheel budget.
        command = tuple(int(value * scale) for value in requested)
        return command, self.estimate(command)

    @property
    def gains(self):
        return (self.forward_counts_per_mm * 1000, self.left_counts_per_mm * 1000,
                self.turn_counts_per_deg * math.degrees(1.0))

    def estimate(self, command):
        return tuple(value / gain for value, gain in zip(command, self.gains))

    def verify_board(self, info):
        limit, signs = info
        if tuple(self.encoder_signs) != signs or self.max_wheel_counts_per_s > limit:
            raise ValueError("底盘编码器符号或轮速量程已变，请重新运行菜单 11")


def configure(args, calib):
    from calibrate_drive import measure_drive

    port = input("蓝牙传出端口 [COM6]：").strip().upper() or "COM6"
    layout = input("已核对的 FL/FR/RL/RR 轮位 [CBDA]：").strip().upper() or "CBDA"
    if not re.fullmatch(r"COM[1-9]\d*", port) or sorted(layout) != list("ABCD"):
        raise ValueError("请核对蓝牙传出端口和四轮通道顺序")
    # 标定只在空场执行，检测开启时额外检查近车障碍。
    check_obstacles = args.obstacles
    vision = task_vision(args, calib, keep_source=check_obstacles, detect_obstacles=check_obstacles)
    vision.home_side = "blue"
    link = ChassisLink(port, layout=f"{layout} 1 1 1 1", strict=True,
                       require_speed_stream=True, stream_pipeline=True)
    measured = measure_drive(vision, link, layout, MAX_AGE, STREAM_RESERVE,
                             check_obstacles=check_obstacles)
    if measured is None:
        return 130
    gains, limit, signs = measured
    settings = DriveConfig(port, layout, *gains, limit, signs)
    path = args.config
    pending = path.with_suffix(path.suffix + ".tmp")
    pending.write_text(json.dumps(vars(settings), indent=2) + "\n", encoding="utf-8")
    pending.replace(path)
    print(f"已保存 {path}")
    return 0


def load_calibration(path):
    if not path.is_file():
        raise ValueError("缺少场地标定。选 1 按 S 拍完整场地，点四个地面内角后按回车保存")
    calib = FieldCalibration.load(path)
    corners = calib.corners
    if (calib.field_mm != 1200 or calib.size != 960 or not calib.source_size
            or len(calib.source_size) != 2 or corners.shape != (4, 2)
            or not np.isfinite(corners).all() or not cv2.isContourConvex(corners)
            or cv2.contourArea(corners, oriented=True) < 100
            or np.any(corners < 2) or np.any(corners >= np.array(calib.source_size) - 2)):
        raise ValueError("请选 1 按 S 拍完整的 1.2 m 场地，重新点四个地面内角后按回车保存")
    return calib


def project_robot_outline(points, pose, mm_per_px):
    yaw = math.radians(pose.heading_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    # Image y increases downwards; the 50 mm tag gives the local projected scale.
    pixels_per_metre = 1000 * pose.side_mm / (50 * mm_per_px)
    return np.float32(pose.center_px + pixels_per_metre * (points @ [[c, -s], [-s, -c]]))


@dataclass(frozen=True)
class ProjectedBlock:
    contour: np.ndarray
    center: tuple[float, float]
    bounds: tuple[float, float, float, float]
    priority: tuple[float, bool, float]


def overlapping_boxes(a, b):
    """Judge duplicate boxes by the fraction of the smaller box covered."""
    ax, ay, ar, ab = a
    bx, by, br, bb = b
    width = min(ar, br) - max(ax, bx)
    height = min(ab, bb) - max(ay, by)
    smaller_area = min((ar - ax) * (ab - ay), (br - bx) * (bb - by))
    return width > 0 and height > 0 and width * height > BLOCK_OVERLAP_LIMIT * smaller_area


def distinct_blocks(blocks):
    kept = []
    for index in sorted(range(len(blocks)), key=lambda i: blocks[i].priority, reverse=True):
        if not any(overlapping_boxes(blocks[index].bounds, blocks[other].bounds) for other in kept):
            kept.append(index)
    return [blocks[index] for index in sorted(kept)]


def observe(image, calib, stamp, stats=None, detector=None, *, home_side="blue", detect_obstacles=True):
    """Warp one frame and return the scene the policy sees.

    `stats` is an optional dict that receives this frame's decision counters
    (reason, seam, tag, component/split counts) for diagnostics; it does not
    change the returned scene.
    A None home_side tracks the robot's current half while staging a run.
    """
    if stats is not None:
        stats.clear()
        stats.update(reason="", seam_px=None, tag_ok=False, candidates=0,
                     instance_area=None, splits=0, kept=0, dropped=0, overlap_dropped=0)
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
    pose, reason = detect(view, calib, unique=True)
    if stats is not None:
        stats["tag_ok"] = pose is not None
    if seam is None:
        if pose is not None:
            draw_tag_heading(view, pose.corners_px, f"ID {pose.tag_id}")
        return fail("field seam missing (yellow top, blue bottom)", view)
    if stats is not None:
        stats["seam_px"] = float(seam)
    if pose is None:
        return fail(f"ID {TAG_ID}: {reason}", view)
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
    if home_side is None:
        home_side = "blue" if robot[1] < 0 else "yellow"
    direction = -1 if home_side == "yellow" else 1
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
    border = rectified_floor_border(hsv, blue, yellow)
    wheels = [project_robot_outline(wheel, pose, calib.mm_per_px) for wheel in ROBOT_WHEELS]
    wheel_bounds = [(wheel.min(axis=0), wheel.max(axis=0)) for wheel in wheels]
    projected = []
    for block in detections:
        contour = image_to_world(homography, block["contour_px"]).reshape(-1, 2)
        x, y = image_to_world(homography, [block["center_px"]])[0]
        if (not (0 <= x < calib.size and 0 <= y < calib.size)
                or cv2.pointPolygonTest(border, (float(x), float(y)), False) < 0):
            continue
        # Reject only contours wholly on a wheel strip. A block touching the
        # wheel and extending beyond the strip must remain visible to the planner.
        low, high = contour.min(axis=0), contour.max(axis=0)
        if any(np.all(low >= wheel_low) and np.all(high <= wheel_high)
               and all(cv2.pointPolygonTest(wheel, tuple(map(float, point)), False) >= 0 for point in contour)
               for wheel, (wheel_low, wheel_high) in zip(wheels, wheel_bounds)):
            continue
        priority = (float(block.get("score", 0.)), not block.get("size_inferred_seed", False),
                    cv2.contourArea(contour))
        projected.append(ProjectedBlock(contour, (x, y), (*low, *high), priority))
    unique = distinct_blocks(projected)
    if stats is not None:
        stats["overlap_dropped"] = len(projected) - len(unique)
        stats["dropped"] += stats["overlap_dropped"]
        stats["kept"] = len(unique)
    for block in unique:
        contour = block.contour
        x, y = block.center
        dx, dy = np.max(np.abs(contour - [x, y]), axis=0) * metres
        radius = math.ceil(max(dx, dy) / metres + 10 / calib.mm_per_px)
        xx, yy = round(float(x)), round(float(y))
        x0, x1 = max(0, xx-radius), min(calib.size, xx+radius+1)
        y0, y1 = max(0, yy-radius), min(calib.size, yy+radius+1)
        around = np.ones((y1-y0, x1-x0), dtype=np.uint8)
        cv2.fillPoly(around, [np.int32(contour - [x0, y0])], 0)
        b = np.count_nonzero(blue[y0:y1, x0:x1] & around.astype(bool))
        g = np.count_nonzero(yellow[y0:y1, x0:x1] & around.astype(bool))
        own = direction * (b - g) > 0
        scene["blocks"].append((direction * (x*metres-.6), direction * (seam-y)*metres,
                                max(.005, dx), max(.005, dy), own))
        cv2.polylines(marked, [np.int32(contour)], True, (0, 235, 0) if own else (0, 180, 255), 2)
    if detect_obstacles:
        outline = project_robot_outline(ROBOT_OUTLINE, pose, calib.mm_per_px)
        pixels_per_mm = pose.side_mm / (50 * calib.mm_per_px)
        outline_pad = math.ceil(OBSTACLE_OUTLINE_PAD_MM * pixels_per_mm)
        own_region = (tuple(map(round, pose.center_px)), math.ceil(OBSTACLE_OWN_RADIUS_MM * pixels_per_mm))
        positions, extents = _obstacles(hsv, centered, np.rint(border), bias, robot, robot_outline_px=outline,
                                        robot_outline_pad_px=outline_pad, floor_seam_px=seam,
                                        own_evidence_region=own_region,
                                        block_contours=(block.contour for block in unique))
        scene["obstacles"] = [(direction * x, direction * y, *extent)
                              for (x, y), extent in zip(positions, extents)]
    # The planner's negative-y half is always home. A half-turn preserves the
    # robot's local velocity axes for a yellow-side start.
    if direction == -1:
        robot = (-robot[0], -robot[1], wrap(robot[2] + math.pi))
    scene.update(valid=True, robot=robot, home_side=home_side,
                 seam_bias=direction * bias, tag_side_mm=pose.side_mm)
    draw_tag_heading(marked, pose.corners_px, f"ID {pose.tag_id}")
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
    def __init__(self, camera, calib, rotation, stabilizer=None, detector=None, *,
                 keep_source=False, detect_obstacles=True):
        self.camera, self.calib, self.rotation = camera, calib, rotation
        self.stabilizer = stabilizer
        self.detector = detector
        self.keep_source = keep_source
        self.detect_obstacles = detect_obstacles
        self.home_side = None
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
        tracked_side = None
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
                                               detector=self.detector, home_side=self.home_side,
                                               detect_obstacles=self.detect_obstacles)
                scene["vision_timing_ms"] = (round(1000 * (began - stamp)),
                                             round(1000 * (time.monotonic() - began)))
                # Merge over-split halves, drop one-frame ghosts and smooth the
                # centres. Invalid frames do not advance the tracker, so a short
                # vision gap does not erase blocks that are still there.
                if self.stabilizer is not None and scene["valid"]:
                    if tracked_side != scene["home_side"]:
                        self.stabilizer.reset()
                        tracked_side = scene["home_side"]
                    scene["blocks"] = self.stabilizer.update(scene["blocks"], stamp)
                selected, self.held = latest_scene(scene, self.held)
                # Hold the previous display together with its scene through a short vision gap.
                if selected is scene:
                    if self.keep_source:
                        scene["source_image"] = image
                    self.latest = (scene, marked, label)
                self.stop.wait(max(0., .10 - (time.monotonic() - began)))
        except Exception as error:
            self.error = error


def fresh(scene, now):
    return scene is not None and scene["valid"] and 0 <= now - scene["stamp"] <= MAX_AGE


def can_start(scene):
    x, y, yaw = scene["robot"]
    return (math.isfinite(yaw) and abs(x) < .6
            and abs(y + scene.get("seam_bias", 0.)) < .6)


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


def task_vision(args, calib, *, keep_source=False, detect_obstacles=True):
    camera = UsbCamera(args.scrcpy_dir, max_size=args.max_size)
    from onnx_blocks import build_detector
    detector, _label = build_detector(args.detector, args.onnx_model, args.onnx_device)
    return VisionWorker(camera, calib, args.rotate,
                        None if args.no_track else BlockStabilizer(), detector,
                        keep_source=keep_source, detect_obstacles=detect_obstacles)


def run(args, calib, drive):
    policy = SweepPolicy(duration=DURATION)
    vision = task_vision(args, calib, detect_obstacles=args.obstacles)
    camera = vision.camera
    link = (ChassisLink(drive.port, layout=f"{drive.layout} 1 1 1 1", strict=True,
                        require_speed_stream=True, stream_pipeline=True, renew_stream=True) if drive else None)
    started = None
    policy_side = None
    last_scene = None
    stable = 0
    last_label = None
    queued_output = None
    board_verified = False
    try:
        if not args.obstacles:
            print("障碍检测开关关闭：不识别、也不避让对方车体（菜单 14 可打开）。", flush=True)
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
            if link and link.ready and not board_verified:
                drive.verify_board(link.speed_info)
                board_verified = True
            snapshot = vision.latest
            scene, frame, label = snapshot if snapshot is not None else (None, waiting, "waiting for camera")
            if scene is None and now > camera_deadline:
                raise RuntimeError("未收到手机画面")
            valid = fresh(scene, now)
            if (valid and started is None and vision.home_side is not None
                    and scene["home_side"] != vision.home_side):
                valid = False
                label = "waiting for selected home side"
            if scene is not None and scene is not last_scene:
                if valid:
                    side = scene["home_side"]
                    if started is not None and side != vision.home_side:
                        raise RuntimeError("起跑侧识别不一致，任务终止")
                    if started is not None and pose_changed_too_fast(scene, last_scene):
                        raise RuntimeError("位姿跳变或识别帧间隔过长，任务终止")
                    if started is None and policy_side != side:
                        policy = SweepPolicy(duration=DURATION)
                        policy_side = side
                if started is None and valid and can_start(scene):
                    contiguous = fresh(last_scene, now)
                    steady = (contiguous
                              and scene["home_side"] == last_scene["home_side"]
                              and math.dist(scene["robot"][:2], last_scene["robot"][:2]) < .01
                              and abs(wrap(scene["robot"][2] - last_scene["robot"][2])) < .10)
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
            usable = valid and now - scene["stamp"] < MAX_AGE - STREAM_RESERVE
            ready = usable and stable >= 3 and (link is None or link.ready and not link.busy)
            if started is None:
                if ready:
                    side = scene["home_side"].upper()
                    label = f"{'READY' if link else 'PREVIEW'} | {side} home | SPACE start | B/Y change home"
                elif valid:
                    if not usable:
                        label = "waiting for newer frames"
                    elif not can_start(scene):
                        label = "place robot inside the surveyed field"
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
                motion_label = ""
                if link:
                    command, estimated = drive.convert(target)
                    latest = vision.latest[0]
                    after_plan = time.monotonic()
                    end = min(scene["stamp"] + MAX_AGE, started + DURATION)
                    active_outputs = set(link.stream_outputs)
                    if queued_output is not None:
                        active_outputs.add(queued_output)
                    frame_ready = (after_plan + STREAM_RESERVE < end and fresh(latest, after_plan)
                                   and latest["home_side"] == vision.home_side)
                    allowed = (frame_ready and any(command)
                               and motion_safe(policy, scene, estimated, after_plan)
                               and all(motion_safe(policy, scene, drive.estimate(output), after_plan)
                                       for output in active_outputs - {command}))
                    if not allowed:
                        if not frame_ready:
                            motion_label = " | HOLD: frame timing"
                        elif not any(command):
                            motion_label = " | HOLD: policy stop"
                        else:
                            motion_label = " | HOLD: motion safety"
                        if queued_output is not None:
                            link.stop()
                            queued_output = None
                            policy.smoother.reset()
                    elif vision.latest[0] is scene and link.stream(*command, valid_until=end):
                        queued_output = command
                    else:
                        motion_label = " | WAIT: next command"
                label = f"{'RUN' if link else 'PREVIEW'} | {policy.phase} | {now-started:.1f}s{motion_label}"
            display = cv2.resize(frame, (720, 720), interpolation=cv2.INTER_AREA)
            if valid:
                scale = display.shape[0] / 1.2
                direction = -1 if scene["home_side"] == "yellow" else 1
                if started is not None and policy.route is not None:
                    def route_pixel(point):
                        x, y = point
                        return (round((.6 + direction * x) * scale),
                                round((.6 - direction * (y + scene["seam_bias"])) * scale))

                    previous = route_pixel(scene["robot"][:2])
                    for leg in policy.route.legs[policy.leg_index:]:
                        goal = route_pixel(leg.point)
                        color = (40, 190, 255) if leg.mode == 'push' else (230, 220, 30)
                        cv2.line(display, previous, goal, color, 2)
                        previous = goal
                for obstacle in policy.obstacles:
                    x = .6 + direction * obstacle.x
                    y = .6 - direction * obstacle.y - scene["seam_bias"]
                    cv2.rectangle(display, (round((x - obstacle.hx) * scale), round((y - obstacle.hy) * scale)),
                                  (round((x + obstacle.hx) * scale), round((y + obstacle.hy) * scale)),
                                  (255, 100, 0), 2)
            cv2.rectangle(display, (0, 0), (720, 34), (20, 20, 20), -1)
            cv2.putText(display, label, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, .56, (240, 240, 240), 1)
            cv2.imshow(WINDOW, display)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                return 0
            if started is None and key in (ord("b"), ord("B"), ord("y"), ord("Y")):
                vision.home_side = "blue" if key in (ord("b"), ord("B")) else "yellow"
                stable = 0
            if key == ord(" ") and ready and started is None:
                latest = vision.latest[0]
                now = time.monotonic()
                if (fresh(latest, now) and latest["home_side"] == scene["home_side"]
                        and can_start(latest)):
                    vision.home_side = scene["home_side"]
                    started = now
                    policy.start_time = started
                    color = "蓝区" if vision.home_side == "blue" else "黄区"
                    print(f"任务开始，己方为{color}，90 秒计时")
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


def build_parser() -> argparse.ArgumentParser:
    """Task CLI; its defaults are also the detection defaults of menu 13's view."""
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--configure", action="store_true", help="用相机和四轮编码器自动标定车体运动")
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
    parser.add_argument("--obstacles", dest="obstacles", action="store_true", default=None,
                        help="打开对方车体障碍检测（默认按 windows/detection_flags.json）")
    parser.add_argument("--no-obstacles", dest="obstacles", action="store_false",
                        help="关闭障碍检测")
    parser.add_argument("--onnx-model", type=Path,
                        default=HERE / "models" / "red_block.onnx")
    parser.add_argument("--onnx-device", choices=("auto", "gpu", "cpu"), default="auto",
                        help="ONNX 推理后端；auto 有 CUDA 就用 GPU")
    return parser


def main():
    args = build_parser().parse_args()
    args.rotate = None if args.rotate == "auto" else int(args.rotate)
    if args.obstacles is None:
        args.obstacles = obstacles_enabled()
    try:
        calib = load_calibration(args.calibration)
        if args.configure:
            return configure(args, calib)
        if not args.preview and not args.config.is_file():
            raise ValueError("缺少本车运动参数，请在日常工具选 11 保存本车配置")
        drive = None
        if not args.preview:
            settings = json.loads(args.config.read_text(encoding="utf-8"))
            if "forward_counts_per_mm" not in settings:
                raise ValueError("旧版 PWM 参数需要升级，请选 11 自动标定编码器")
            drive = DriveConfig(**settings)
        return run(args, calib, drive)
    except (RuntimeError, OSError, ValueError, TypeError, KeyError, cv2.error) as error:
        print(f"任务退出：{error}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
