"""Measure chassis encoder scales against the roof tag in an empty arena."""
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
from statistics import median
import time

import cv2
import numpy as np

from chassis_link import ChassisError


WINDOW = "Drive calibration | SPACE start | Q / Esc stop"
OBSTACLE_LOG_DIR = Path(__file__).resolve().parents[1] / "log" / "drive_calibration"
TAG_SIDE_MM = 50.0
TARGET = (.120, .120, math.radians(45))
MAX_TRAVEL_FACTOR = 1.6
AXES = ("前进/后退", "横移", "转向")
DIRECTIONS = ((0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)) * 3
# The camera observes motion late and the chassis coasts after STOP. Use a
# lower wheel speed for these short measurements to limit that extra travel.
CALIBRATION_SPEED_DIVISOR = 3
START_LIMIT_M = .24
MOTION_LIMIT_M = .38
# Five samples must cover at least 200 ms even when frames arrive faster.
POSE_SAMPLE_INTERVAL_S = .055
# Allow for the tag-to-rear-corner distance, a full 120 mm translation, and a
# margin. Check this region on each frame as the car moves.
CLEARANCE_M = .34
DRIVE_BUDGET_S = 3.5
RECOVERY_LIMIT_S = 2.
TRIAL_LIMIT_S = 8.

# A glossy floor reflection flickers between "object" and "floor". Require a
# short run of stable position and size before treating a near-car candidate
# as a real obstacle. This only gates calibration; the task keeps its own
# obstacle tracking and does not use this confirmation.
OBSTACLE_MATCH_M = .08
OBSTACLE_FRAMES = 3
OBSTACLE_SPAN_S = .16
OBSTACLE_MAX_GAP_S = .25
OBSTACLE_STABLE_M = .006
OBSTACLE_SIZE_TOL = .25
OBSTACLE_DRIFT_M = .08
OBSTACLE_PENDING_S = .50
OBSTACLE_TTL_S = .60


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    heading: float
    side_mm: float

    @classmethod
    def from_scene(cls, scene):
        x, y, heading = scene["robot"]
        return cls(x, y + scene["seam_bias"], heading, scene["tag_side_mm"])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def between(start, end):
    # For parallel floor and tag planes, the floor homography scales every
    # relative roof displacement by the same factor as the marker edges.
    scale = TAG_SIDE_MM / ((start.side_mm + end.side_mm) * .5)
    dx, dy = (end.x - start.x) * scale, (end.y - start.y) * scale
    c, s = math.cos(start.heading), math.sin(start.heading)
    return c * dx + s * dy, -s * dx + c * dy, wrap(end.heading - start.heading)


def motion_fits(pose, axis, direction):
    if max(abs(pose.x), abs(pose.y)) >= MOTION_LIMIT_M:
        return False
    if axis == 2:
        return True
    # Include the accepted travel beyond the target and convert to the same
    # projected coordinates as the live boundary check.
    distance = direction * MAX_TRAVEL_FACTOR * TARGET[axis] * pose.side_mm / TAG_SIDE_MM
    heading = pose.heading + axis * math.pi / 2
    end_x = pose.x + distance * math.cos(heading)
    end_y = pose.y + distance * math.sin(heading)
    return max(abs(end_x), abs(end_y)) < MOTION_LIMIT_M


def stable_pose(samples):
    if len(samples) < 5 or samples[-1][0] - samples[0][0] < .20:
        return None
    poses = [pose for _, pose in samples]
    ref = poses[-1]
    cx = median(pose.x for pose in poses)
    cy = median(pose.y for pose in poses)
    heading = wrap(ref.heading + median(wrap(pose.heading - ref.heading) for pose in poses))
    # Tolerate one noisy tag frame: a single outlier must not make READY flip
    # to WAITING for a frame. A real move still fails because it changes the
    # majority, not just one sample.
    good = [pose for pose in poses
            if math.hypot(pose.x - cx, pose.y - cy) <= .005
            and abs(wrap(pose.heading - heading)) <= math.radians(2.5)]
    if len(good) < 4:
        return None
    return Pose(median(pose.x for pose in good), median(pose.y for pose in good),
                wrap(heading + median(wrap(pose.heading - heading) for pose in good)),
                median(pose.side_mm for pose in good))


def encoder_axes(before, after, layout, signs):
    counts = [(end - start) * sign for start, end, sign in zip(before, after, signs)]
    fl, fr, rl, rr = (counts[ord(channel) - ord("A")] for channel in layout)
    return (fl + fr + rl + rr) / 4, (-fl + fr + rl - rr) / 4, (-fl + fr - rl + rr) / 4


def trial_gain(axis, direction, start, end, before, after, layout, signs):
    advance, side, turn = between(start, end)
    motion = (advance, side, turn)
    travel = direction * motion[axis]
    ticks = direction * encoder_axes(before, after, layout, signs)[axis]
    units = 1000 * travel if axis < 2 else math.degrees(travel)
    target = 1000 * TARGET[axis] if axis < 2 else math.degrees(TARGET[axis])
    unit = "mm" if axis < 2 else "°"
    failures = []
    if travel < .75 * TARGET[axis] or travel > MAX_TRAVEL_FACTOR * TARGET[axis]:
        failures.append(f"有效位移 {units:.1f} {unit} 超出 {0.75 * target:g}～"
                        f"{MAX_TRAVEL_FACTOR * target:g} {unit}")
    if abs(start.side_mm - end.side_mm) > .07 * TAG_SIDE_MM:
        failures.append(f"tag 投影边长变化超过 {0.07 * TAG_SIDE_MM:g} mm")
    if ticks <= 0:
        failures.append("沿指令方向的编码器计数应为正")
    if axis < 2 and (abs(motion[1 - axis]) > .025 or abs(turn) > math.radians(8)):
        failures.append("走偏过多，请检查轮子接地和轮速标定")
    if axis == 2 and math.hypot(advance, side) > .050:
        failures.append("tag 中心偏移过大，请核对 tag 安装和轮位")
    if failures:
        raw = " ".join(f"{channel}={end_count - start_count:+d}"
                       for channel, start_count, end_count in zip("ABCD", before, after))
        sign_text = " ".join(f"{sign:+d}" for sign in signs)
        raise ValueError(
            f"{AXES[axis]}{'正向' if direction > 0 else '反向'}验算失败，"
            f"{'、'.join(failures)}。停稳后车体系前进 {1000 * advance:+.1f} mm、"
            f"左移 {1000 * side:+.1f} mm、转向 {math.degrees(turn):+.1f}°，"
            f"tag 投影边长 {start.side_mm:.1f} → {end.side_mm:.1f} mm，"
            f"编码器原始增量 {raw}，标定符号（A/B/C/D）{sign_text}，"
            f"沿指令方向计数 {ticks:+.1f}，轮位 {layout}（FL/FR/RL/RR）"
        )
    return ticks / units


def fit_gains(samples):
    gains = tuple(median(values) for values in samples)
    for axis, (gain, values) in enumerate(zip(gains, samples)):
        if max(abs(value - gain) for value in values) > .20 * gain:
            raise ValueError(f"{AXES[axis]}重复结果相差过大，已保留原配置")
    return gains


def stationary_counts(vision, link, pose, max_age):
    counts = link.encoder_snapshot()
    scene = vision.latest[0]
    if not scene["valid"] or time.monotonic() - scene["stamp"] >= max_age:
        raise RuntimeError("读取编码器时失去有效位姿，请重试")
    x, y, turn = between(pose, Pose.from_scene(scene))
    if math.hypot(x, y) > .005 or abs(turn) > math.radians(2.5):
        raise RuntimeError("读取编码器时车仍在移动，请重试")
    return counts


def stationary_wait(scene, age, valid, vision_label, blocking, pending, stable, link_ready):
    """Report what is blocking a reading or movement while stopped."""
    if scene is None or not scene["valid"]:
        return "Waiting for the roof tag and field", f"位姿不可用（{vision_label}）"
    if not valid:
        return "Waiting for a fresh frame", f"视觉帧龄 {age * 1000:.0f} ms"
    if scene["blocks"]:
        return "Clear the field", f"场地检测到 {len(scene['blocks'])} 个红块候选"
    if blocking:
        return "Clear the field", "车旁确认有障碍物，请检查蓝框所在位置"
    if pending:
        return "Checking obstacle candidate", "近车障碍候选还在确认"
    if not link_ready:
        return "Waiting for Bluetooth", "蓝牙底盘未就绪"
    if stable is None:
        return "Waiting for the robot to settle", "未取得连续稳定的 tag 位姿"
    return None


def obstacles_near_car(scene):
    x, y, _ = scene["robot"]
    return [(ox, oy, hx, hy) for ox, oy, hx, hy in scene["obstacles"]
            if math.hypot(max(0., abs(ox - x) - hx), max(0., abs(oy - y) - hy)) <= CLEARANCE_M]


def save_obstacle_images(scene, marked, stage, log_dir):
    """Save the stopped frame and its blue-box overlay for offline replay."""
    stem = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + f"_step{stage:02d}"
    log_dir.mkdir(parents=True, exist_ok=True)
    images = [("boxed", marked)]
    if scene.get("source_image") is not None:
        images.append(("input", scene["source_image"]))
    for label, frame in images:
        path = log_dir / f"{stem}_{label}.png"
        ok, encoded = cv2.imencode(".png", frame)
        if not ok:
            raise OSError(f"无法编码 {path.name}")
        path.write_bytes(encoded.tobytes())
        print(f"障碍排查图 {path}", flush=True)


class ObstacleGate:
    """Confirm a near-car candidate only after it holds still for a moment.

    The candidate mask is per-frame. A light reflection on the glossy floor
    jitters inside a small area, while a real object either stays put or moves
    across the field. Keep a candidate pending until it confirms (stable
    position and size) or shows it is only local flicker; stable objects and
    candidates that travel beyond the local area block the calibration.

    The match radius is deliberately wider than the stability tolerance so a
    jittering box stays in one track and can be rejected, instead of starting
    a new track every frame.
    """

    def __init__(self):
        self.tracks = []

    def update(self, stamp, candidates):
        self.tracks = [track for track in self.tracks
                       if stamp - track["last"] <= OBSTACLE_TTL_S]
        used = set()
        for x, y, hx, hy in candidates:
            track = self._match(x, y, hx, hy, used)
            if track is None:
                track = {"x": x, "y": y, "hx": hx, "hy": hy,
                         "first": stamp, "last": stamp,
                         "samples": [(x, y, hx, hy)], "stamps": [stamp]}
                self.tracks.append(track)
            else:
                track.update(x=x, y=y, hx=hx, hy=hy, last=stamp)
                track["samples"] = (track["samples"] + [(x, y, hx, hy)])[-8:]
                track["stamps"] = (track["stamps"] + [stamp])[-8:]
            used.add(id(track))
        blocking = [(track["x"], track["y"], track["hx"], track["hy"])
                    for track in self.tracks
                    if stamp - track["last"] <= OBSTACLE_TTL_S
                    and (self._confirmed(track) or self._drifting(track, stamp))]
        pending = any(stamp - track["last"] <= OBSTACLE_TTL_S
                      and stamp - track["first"] < OBSTACLE_PENDING_S
                      and not self._confirmed(track) for track in self.tracks)
        return blocking, pending

    def _match(self, x, y, hx, hy, used):
        best = None
        for track in self.tracks:
            if id(track) in used:
                continue
            distance = math.hypot(x - track["x"], y - track["y"])
            if (distance <= OBSTACLE_MATCH_M
                    and abs(hx - track["hx"]) <= .05 and abs(hy - track["hy"]) <= .05):
                if best is None or distance < best[0]:
                    best = (distance, track)
        return best[1] if best is not None else None

    @staticmethod
    def _confirmed(track):
        stamps = track["stamps"]
        if len(stamps) < OBSTACLE_FRAMES or stamps[-1] - stamps[0] < OBSTACLE_SPAN_S:
            return False
        if any(later - earlier > OBSTACLE_MAX_GAP_S
               for earlier, later in zip(stamps, stamps[1:])):
            return False
        samples = track["samples"]
        xs = [sample[0] for sample in samples]
        ys = [sample[1] for sample in samples]
        hxs = [sample[2] for sample in samples]
        hys = [sample[3] for sample in samples]
        return (max(xs) - min(xs) <= 2 * OBSTACLE_STABLE_M
                and max(ys) - min(ys) <= 2 * OBSTACLE_STABLE_M
                and max(hxs) - min(hxs) <= OBSTACLE_SIZE_TOL * max(median(hxs), .01)
                and max(hys) - min(hys) <= OBSTACLE_SIZE_TOL * max(median(hys), .01))

    @staticmethod
    def _drifting(track, stamp):
        """A candidate that left the small area where floor flicker lives."""
        if stamp - track["first"] < OBSTACLE_PENDING_S:
            return False
        samples = track["samples"]
        xs = [sample[0] for sample in samples]
        ys = [sample[1] for sample in samples]
        return (max(xs) - min(xs) > OBSTACLE_DRIFT_M
                or max(ys) - min(ys) > OBSTACLE_DRIFT_M)


def measure_drive(vision, link, layout, max_age, stream_reserve, *, check_obstacles=True):
    """Press space once to run paired single-axis samples; save only on success."""
    print("清空场地，把车放在场地中心、tag 朝上且无遮挡。关闭其它取图和底盘工具。", flush=True)
    if check_obstacles:
        print("近车障碍候选会先观察约半秒，稳定时通常约 0.2 秒确认；退出时会保存排查图。", flush=True)
    else:
        print("菜单 11 暂停近车障碍判定，请在空场地标定。", flush=True)
    print("画面显示 READY 后按空格，车会依次往返平移和转向。Q / Esc 停车退出。", flush=True)
    history = deque(maxlen=5)
    gains = [[], [], []]
    index = 0
    state = "waiting"
    last_scene = None
    entered = time.monotonic()
    first = entered
    saved_start = None
    before = None
    stop_motion = stop_age = None
    drive_deadline = None
    trial_deadline = None
    remaining_drive = DRIVE_BUDGET_S
    pause_reason = ""
    gate = ObstacleGate() if check_obstacles else None
    blocking, pending = [], False
    last_stream_warn = 0.0
    camera = vision.camera
    blank = np.zeros((480, 480, 3), dtype=np.uint8)
    shown_frame = shown_overlay = None
    obstacle_images = None
    try:
        camera.start()
        vision.thread.start()
        link.start()
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 650, 650)
        while True:
            if link.error:
                raise ChassisError(link.error)
            if vision.error or camera.error:
                raise RuntimeError(vision.error or camera.error)
            scene, frame, vision_label = vision.latest or (None, blank, "未收到手机画面")
            now = time.monotonic()
            age = now - scene["stamp"] if scene is not None else math.inf
            valid = scene is not None and scene["valid"] and 0 <= age < max_age
            pose = Pose.from_scene(scene) if valid else None
            frame_gap = None
            if valid and scene is not last_scene:
                if gate is not None:
                    blocking, pending = gate.update(scene["stamp"], obstacles_near_car(scene))
                if state == "driving" and last_scene is not None:
                    previous = Pose.from_scene(last_scene)
                    dt = scene["stamp"] - last_scene["stamp"]
                    if not 0 < dt <= max_age:
                        frame_gap = dt
                    elif (math.hypot(pose.x - previous.x, pose.y - previous.y) > .025 + .5 * dt or
                            abs(wrap(pose.heading - previous.heading)) > .18 + 1.5 * dt):
                        raise RuntimeError("标定时位姿跳变，已停止")
                stamp = scene["stamp"]
                if (state in ("waiting", "next", "settling", "paused") and stamp >= entered
                        and (not history or stamp - history[-1][0] >= POSE_SAMPLE_INTERVAL_S)):
                    history.append((stamp, pose))
                last_scene = scene
            elif not valid:
                blocking, pending = [], False
                history.clear()
            axis, direction = DIRECTIONS[index]
            stable = stable_pose(history)
            waiting = None
            label = state.upper()
            if state == "arming":
                label = "Waiting for a new frame before moving"
            elif state == "paused":
                label = "Stopped | Waiting for clear, fresh vision"
            elif state in ("waiting", "next", "settling"):
                waiting = stationary_wait(scene, age, valid, vision_label, blocking,
                                          pending, stable, link.ready)
                if waiting is None and state != "settling":
                    if state == "waiting" and max(abs(stable.x), abs(stable.y)) >= START_LIMIT_M:
                        waiting = "Place the robot near the field center", "车未放在场地中心附近"
                    elif not motion_fits(stable, axis, direction):
                        waiting = "Not enough room for the next move", (
                            f"预计行程超出标定活动范围（车顶 tag x={stable.x * 1000:+.0f} mm、"
                            f"y={stable.y * 1000:+.0f} mm），请将车放回场地中心重试"
                        )
                if waiting is not None:
                    label = waiting[0]
                elif state == "waiting":
                    label = "READY | SPACE to start"
            ready = waiting is None
            overlay = (index, label, tuple(blocking))
            if frame is not shown_frame or overlay != shown_overlay:
                canvas = cv2.resize(frame, (650, 650))
                for x, y, hx, hy in blocking:
                    # Calibration vision uses the blue-side coordinate convention.
                    y += scene["seam_bias"]
                    scale = canvas.shape[0] / 1.2
                    cv2.rectangle(canvas, (round((.6 + x - hx) * scale), round((.6 - y - hy) * scale)),
                                  (round((.6 + x + hx) * scale), round((.6 - y + hy) * scale)), (255, 100, 0), 2)
                cv2.rectangle(canvas, (0, 0), (650, 34), (15, 15, 15), -1)
                cv2.putText(canvas, f"{index + 1}/{len(DIRECTIONS)}  {label}", (9, 23),
                            cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
                cv2.imshow(WINDOW, canvas)
                shown_frame, shown_overlay = frame, overlay
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                return None
            if (state == "waiting" and key == ord(" ") and ready) or (state == "next" and ready):
                limit, signs = link.speed_info
                speed = max(1, limit // CALIBRATION_SPEED_DIVISOR)
                print(f"标定速度 {speed} 计数/秒（固件上限 {limit}，"
                      f"约 {speed / limit:.0%}）", flush=True)
                before = stationary_counts(vision, link, stable, max_age)
                saved_start = stable
                drive_deadline = None
                trial_deadline = None
                remaining_drive = DRIVE_BUDGET_S
                state, entered = "arming", time.monotonic()
            elif state in ("arming", "driving", "paused"):
                if not link.ready:
                    raise ChassisError("标定时底盘连接中断，已停止")
                now = time.monotonic()
                age = now - scene["stamp"] if scene is not None else math.inf
                usable = valid and age < max_age - stream_reserve
                if valid:
                    if max(abs(pose.x), abs(pose.y)) >= MOTION_LIMIT_M:
                        raise RuntimeError("车辆离开标定活动范围，请放回场地中央")
                    if scene["blocks"]:
                        raise RuntimeError(f"场地检测到 {len(scene['blocks'])} 个红块候选，已停止")
                    if blocking:
                        link.stop()
                        obstacle_images = scene, canvas, index + 1
                        raise RuntimeError(f"{index + 1}/{len(DIRECTIONS)} 车旁确认有障碍物，已停止。"
                                           "请检查蓝框所在位置")
                if trial_deadline is not None and now >= trial_deadline:
                    raise RuntimeError("本段标定反复等待后超时，已停止")
                if drive_deadline is not None and now >= drive_deadline:
                    raise RuntimeError("标定连续驱动时间已用完，车辆未到达目标")
                if state == "driving" and (not usable or frame_gap is not None or pending):
                    link.stop()
                    history.clear()
                    remaining_drive = drive_deadline - now
                    drive_deadline = None
                    state, entered = "paused", now
                    if pending:
                        pause_reason = "发现新的近车障碍候选"
                    elif frame_gap is not None:
                        pause_reason = f"有效画面间隔 {frame_gap * 1000:.0f} ms"
                    elif scene is None or not scene["valid"]:
                        pause_reason = f"位姿暂不可用（{vision_label}）"
                    else:
                        pause_reason = f"视觉帧龄 {age * 1000:.0f} ms"
                        timing = scene.get("vision_timing_ms")
                        if timing is not None:
                            pause_reason += f"，取帧等待 {timing[0]} ms、识别处理 {timing[1]} ms"
                    print(f"{pause_reason}，已停车等待新画面和车辆停稳", flush=True)
                    continue
                ready_to_move = (usable and not pending
                                 and (state == "driving" or scene["stamp"] > entered)
                                 and (state != "paused" or (stable and now - entered >= .35)))
                if not ready_to_move:
                    if state != "driving" and now - entered >= RECOVERY_LIMIT_S:
                        if pending:
                            raise RuntimeError("近车障碍候选未消失，已停止。请检查场地或灯光")
                        if state == "paused":
                            raise RuntimeError(f"{pause_reason}；停车后未恢复稳定的新位姿，已停止")
                        raise RuntimeError(f"标定起步前未收到可用的新位姿（{vision_label}），未继续驱动")
                    time.sleep(.025)
                    continue
                displacement = between(saved_start, pose)
                motion = direction * displacement[axis]
                if state == "arming" and (math.hypot(*displacement[:2]) > .010
                                          or abs(displacement[2]) > math.radians(4)):
                    raise RuntimeError("读取编码器后车体位置改变，请放稳再标定")
                if state == "arming" and not motion_fits(pose, axis, direction):
                    raise RuntimeError("起步前预计行程超出标定活动范围，请将车放回场地中心")
                if motion >= TARGET[axis]:
                    link.stop()
                    stop_motion, stop_age = motion, age
                    history.clear()
                    state, entered = "settling", now
                elif motion < -.15 * TARGET[axis]:
                    raise RuntimeError("未按预期方向完成标定动作，已停止")
                else:
                    target = [0, 0, 0]
                    target[axis] = direction * speed
                    motion_end = drive_deadline if drive_deadline is not None else now + remaining_drive
                    if motion_end - now <= stream_reserve:
                        raise RuntimeError("标定连续驱动时间已用完，车辆未到达目标")
                    deadline = min(scene["stamp"] + max_age, motion_end)
                    if link.stream(*target, valid_until=deadline):
                        drive_deadline = motion_end
                        if trial_deadline is None:
                            trial_deadline = now + TRIAL_LIMIT_S
                            print(f"{index + 1}/{len(DIRECTIONS)} {AXES[axis]} {'正向' if direction > 0 else '反向'}",
                                  flush=True)
                        if state == "paused":
                            print("新画面已恢复、车辆已停稳，继续本段标定", flush=True)
                        state = "driving"
                    elif now - last_stream_warn >= 1.:
                        print(f"连续驱动目标未发出：ready={link.ready} error={link.error}", flush=True)
                        last_stream_warn = now
            elif state == "settling" and ready and now - entered >= .35:
                after = stationary_counts(vision, link, stable, max_age)
                try:
                    value = trial_gain(axis, direction, saved_start, stable, before, after, layout, signs)
                except ValueError as exc:
                    reading = (f"{1000 * stop_motion:.1f} mm" if axis < 2
                               else f"{math.degrees(stop_motion):.1f}°")
                    raise ValueError(f"{exc}。触发停车时沿指令方向 {reading}，"
                                     f"该帧帧龄 {1000 * stop_age:.0f} ms") from None
                gains[axis].append(value)
                print(f"  {value:.3f} 计数/{'mm' if axis < 2 else '度'}", flush=True)
                index += 1
                if index == len(DIRECTIONS):
                    return fit_gains(gains), limit, signs
                state, entered = "next", time.monotonic()
                history.clear()
            elif ((state in ("settling", "next") and now - entered > 4.) or
                  (state == "waiting" and scene is None and now - first > 15.)):
                if blocking:
                    obstacle_images = scene, canvas, index + 1
                action = f"{index + 1}/{len(DIRECTIONS)} {AXES[axis]}{'正向' if direction > 0 else '反向'}"
                phase = "停车后" if state == "settling" else "起步前"
                raise RuntimeError(f"{action} {phase}等待超时，{waiting[1]}")
            time.sleep(.025)
    finally:
        link.close()
        vision.stop.set()
        camera.close()
        if vision.thread.is_alive():
            vision.thread.join(timeout=1.)
        cv2.destroyAllWindows()
        if obstacle_images is not None:
            try:
                save_obstacle_images(*obstacle_images, OBSTACLE_LOG_DIR)
            except (OSError, cv2.error) as exc:
                print(f"障碍排查图保存失败，{exc}", flush=True)
