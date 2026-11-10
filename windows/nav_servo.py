"""视觉伺服点动：把 tag 位姿变成有界的 MECANUM 脉冲，逐拍逼近目标。

这一层只做“位姿 -> 车体误差 -> 百分比 + 时长”，不代表连续速度控制：
现用固件是定时开环动作（运行中拒绝、停后约 1 s 冷却），所以每拍都是
“发一个短脉冲 -> 停 -> 重新测位姿”。真正 10–20 Hz 连续闭环要等固件
增加流式目标接口。

坐标约定（与 field_view / pose_tag 一致）：
- 场地 x 向东、y 向北，单位 mm；
- heading 0° 指东，逆时针为正；
- 底盘 forward 向前、left 向左、ccw 逆时针，均为 -100..100 百分比。

离线自检（不需要相机、底盘或 pyserial）：
    python windows/nav_servo.py --sim --target 400 300 --heading 0 --start 150 200 90
"""
from __future__ import annotations

import argparse
import csv
import math
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from tag_detector import TAG_ID


def wrap_deg(angle: float) -> float:
    """归一到 (-180, 180]。"""
    angle = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if angle == -180.0 else angle


def body_error(x_mm, y_mm, heading_deg, target_x, target_y):
    """场地误差 -> 车体 (forward, left) 毫米。"""
    dx, dy = target_x - x_mm, target_y - y_mm
    h = math.radians(heading_deg)
    forward = dx * math.cos(h) + dy * math.sin(h)
    left = -dx * math.sin(h) + dy * math.cos(h)
    return forward, left


@dataclass
class ServoConfig:
    # 增益与限幅都是待标定值；标定前只用小百分比、短脉冲。
    kp_pos: float = 0.45            # 百分比 / mm
    kp_turn: float = 1.0            # 百分比 / 度
    max_percent: int = 40
    start_percent: int = 12         # 低于此值电机可能不起转，直接取该值
    pos_tol_mm: float = 10.0
    heading_tol_deg: float = 4.0
    min_pulse_ms: int = 150
    max_pulse_ms: int = 600
    # 车辆响应估计，用于把误差折算成脉冲时长；必须实测后修改。
    mm_per_percent_second: float = 3.0
    deg_per_percent_second: float = 1.2
    turn_first: bool = True         # 先原地转到目标朝向，再平移


@dataclass
class Command:
    kind: str                       # "turn" | "move"
    forward: int
    left: int
    ccw: int
    ms: int
    reason: str = ""


@dataclass
class Step:
    action: str                     # "pulse" | "stop" | "done"
    command: Command | None = None
    reason: str = ""


def _clamp(value, low, high):
    return max(low, min(high, value))


class PoseServo:
    def __init__(self, target_x, target_y, heading_deg=None, config=None):
        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.heading = None if heading_deg is None else float(heading_deg)
        self.config = config or ServoConfig()
        self.last_error = None
        self.pulses = 0

    def command(self, pose):
        """返回下一拍 Command；已到容差内返回 None。"""
        cfg = self.config
        forward, left = body_error(pose.x_mm, pose.y_mm, pose.heading_deg,
                                   self.target_x, self.target_y)
        dh = 0.0 if self.heading is None else wrap_deg(self.heading - pose.heading_deg)
        self.last_error = (forward, left, dh)
        linear = math.hypot(forward, left)
        if linear <= cfg.pos_tol_mm and abs(dh) <= cfg.heading_tol_deg:
            return None

        if cfg.turn_first and abs(dh) > cfg.heading_tol_deg:
            percent = _clamp(round(cfg.kp_turn * abs(dh)), cfg.start_percent, cfg.max_percent)
            ms = 1000.0 * abs(dh) / (cfg.deg_per_percent_second * percent)
            command = Command("turn", 0, 0, int(math.copysign(percent, dh)),
                              _clamp(round(ms), cfg.min_pulse_ms, cfg.max_pulse_ms))
        else:
            mf = cfg.kp_pos * forward
            ml = cfg.kp_pos * left
            magnitude = math.hypot(mf, ml)
            if magnitude <= 1e-9:
                return None
            scale = min(1.0, cfg.max_percent / magnitude)
            fwd = round(mf * scale)
            lft = round(ml * scale)
            for name, value in (("forward", fwd), ("left", lft)):
                if value and abs(value) < cfg.start_percent:
                    # 起转死区：把该轴抬到起步值，方向不变。
                    if name == "forward":
                        fwd = int(math.copysign(cfg.start_percent, value))
                    else:
                        lft = int(math.copysign(cfg.start_percent, value))
            used = math.hypot(fwd, lft)
            if used <= 0:
                # 百分比量化后全为 0（误差极小时才会出现）：本拍不动，等下一拍。
                return None
            ms = 1000.0 * linear / (cfg.mm_per_percent_second * used)
            command = Command("move", fwd, lft, 0,
                              _clamp(round(ms), cfg.min_pulse_ms, cfg.max_pulse_ms))
        self.pulses += 1
        return command


def plan_step(pose, age_s, elapsed_s, servo, *, deadline_s=90.0, pose_timeout_s=0.35):
    """一拍决策：先判安全，再判到达，最后才给脉冲。"""
    if elapsed_s >= deadline_s:
        return Step("stop", reason="deadline")
    if pose is None:
        return Step("stop", reason="no_pose")
    if age_s > pose_timeout_s:
        return Step("stop", reason="stale_pose")
    command = servo.command(pose)
    if command is None:
        return Step("done", reason="reached")
    return Step("pulse", command=command)


# ---------- 离线仿真：只验证控制律与状态机，不代表真车 ----------

@dataclass
class SimPose:
    x_mm: float
    y_mm: float
    heading_deg: float


class SimChassis:
    """一阶响应模型：脉冲期间按比例速度积分，附加可选增益误差与噪声。"""

    def __init__(self, x_mm, y_mm, heading_deg, config=None, *,
                 gain_error=1.0, noise_mm=0.0, seed=0):
        self.pose = SimPose(x_mm, y_mm, heading_deg)
        self.config = config or ServoConfig()
        self.gain_error = gain_error
        self.noise_mm = noise_mm
        self._rng = random.Random(seed)
        self.pulses = 0

    @property
    def busy(self):
        return False

    def pulse(self, forward, left, ccw, ms):
        cfg, pose = self.config, self.pose
        seconds = ms / 1000.0
        fwd_mm = cfg.mm_per_percent_second * forward * seconds * self.gain_error
        left_mm = cfg.mm_per_percent_second * left * seconds * self.gain_error
        h = math.radians(pose.heading_deg)
        pose.x_mm += fwd_mm * math.cos(h) - left_mm * math.sin(h)
        pose.y_mm += fwd_mm * math.sin(h) + left_mm * math.cos(h)
        pose.heading_deg = wrap_deg(pose.heading_deg
                                    + cfg.deg_per_percent_second * ccw * seconds * self.gain_error)
        if self.noise_mm:
            pose.x_mm += self._rng.uniform(-self.noise_mm, self.noise_mm)
            pose.y_mm += self._rng.uniform(-self.noise_mm, self.noise_mm)
        self.pulses += 1
        return True

    def stop(self):
        return True


def run_sim(args) -> int:
    config = ServoConfig(max_percent=args.max_percent)
    servo = PoseServo(args.target[0], args.target[1], args.heading, config)
    sim = SimChassis(args.start[0], args.start[1], args.start[2], config,
                     noise_mm=args.noise)
    clock = 0.0
    rows = []
    print(f"仿真：起点 ({sim.pose.x_mm:.0f},{sim.pose.y_mm:.0f},{sim.pose.heading_deg:.0f}) "
          f"目标 ({args.target[0]},{args.target[1]},{args.heading})")
    for _ in range(400):
        step = plan_step(sim.pose, 0.0, clock, servo, deadline_s=args.deadline)
        if step.action == "done":
            print(f"到达：{sim.pose.x_mm:.1f},{sim.pose.y_mm:.1f},{sim.pose.heading_deg:.1f} "
                  f"用了 {sim.pulses} 拍，{clock:.1f} s")
            break
        if step.action == "stop":
            print(f"提前停止：{step.reason}")
            break
        cmd = step.command
        rows.append((round(clock, 2), round(sim.pose.x_mm, 1), round(sim.pose.y_mm, 1),
                     round(sim.pose.heading_deg, 1), cmd.kind, cmd.forward, cmd.left,
                     cmd.ccw, cmd.ms))
        sim.pulse(cmd.forward, cmd.left, cmd.ccw, cmd.ms)
        clock += cmd.ms / 1000.0
    else:
        print("超过 400 拍仍未到达")
        return 1
    if args.log:
        _write_log(args.log, rows)
    err_x = args.target[0] - sim.pose.x_mm
    err_y = args.target[1] - sim.pose.y_mm
    ok = math.hypot(err_x, err_y) <= config.pos_tol_mm + 1e-6 and (
        args.heading is None or abs(wrap_deg(args.heading - sim.pose.heading_deg))
        <= config.heading_tol_deg + 1e-6)
    print(f"残差 {math.hypot(err_x, err_y):.1f} mm，"
          f"朝向 {abs(wrap_deg((args.heading or 0) - sim.pose.heading_deg)):.1f}°")
    return 0 if ok else 1


def _write_log(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "x_mm", "y_mm", "heading_deg", "kind",
                         "forward", "left", "ccw", "ms"])
        writer.writerows(rows)


# ---------- 实机：相机 + 串口 ----------

def run_camera(args) -> int:
    import cv2

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from chassis_link import ChassisError, ChassisLink
    from field_view import FieldCalibration
    from pose_tag import PoseFilter, detect
    from usb_camera import UsbCamera

    calib = FieldCalibration.load(args.config)
    config = ServoConfig(max_percent=args.max_percent)
    servo = PoseServo(args.target[0], args.target[1], args.heading, config)
    link = ChassisLink(args.port) if args.port else None
    camera = UsbCamera(args.scrcpy_dir, args.serial, max_size=args.max_size)
    pose_filter = PoseFilter()
    started = time.monotonic()
    rows = []
    try:
        camera.start()
        if link:
            link.start()
            link.wait_ready()
            print(f"底盘就绪：{args.port}，layout={link.layout}")
        else:
            print("未给 --port：只计算并打印脉冲，不驱动电机。")
        last_sequence = 0
        pose = None
        halted = False
        last_warn = 0.0
        for _ in range(args.max_frames):
            frame, received_at, sequence = camera.latest.get()
            if camera.error:
                raise RuntimeError(camera.error)
            now = time.monotonic()
            age = now - received_at
            if sequence and sequence != last_sequence:
                last_sequence = sequence
                image = frame.to_ndarray(format="bgr24")
                if args.rotate:
                    image = cv2.rotate(image, {90: cv2.ROTATE_90_CLOCKWISE,
                                               180: cv2.ROTATE_180,
                                               270: cv2.ROTATE_90_COUNTERCLOCKWISE}[args.rotate])
                pose = pose_filter.update(detect(calib.warp(image), calib,
                                                 args.tag_mm, args.tag_id)[0])
            else:
                time.sleep(0.01)
            step = plan_step(pose, age, now - started, servo,
                             deadline_s=args.deadline)
            if step.action == "done":
                if link:
                    link.stop()
                print(f"到达：{pose.x_mm:.1f},{pose.y_mm:.1f},{pose.heading_deg:.1f}，"
                      f"{servo.pulses} 拍")
                break
            if step.action == "stop":
                # 位姿丢失/过期只停车等待，不退出：比赛里视觉闪断不该结束程序。
                if link and not halted:
                    link.stop()
                    halted = True
                if step.reason == "deadline":
                    print(f"到达 {args.deadline:.0f} s 截止，已停车")
                    break
                if now - last_warn >= 1.0:
                    last_warn = now
                    print(f"停车等待：{step.reason}"
                          f"（age={age * 1000:.0f} ms，{now - started:.0f} s）")
                time.sleep(0.05)
                continue
            halted = False
            cmd = step.command
            if link is None:
                print(f"dry {cmd.kind} fwd={cmd.forward} left={cmd.left} "
                      f"ccw={cmd.ccw} ms={cmd.ms} "
                      f"pose=({pose.x_mm:.0f},{pose.y_mm:.0f},{pose.heading_deg:.0f})")
                time.sleep(0.2)
                continue
            if link.pulse(cmd.forward, cmd.left, cmd.ccw, cmd.ms):
                rows.append((round(now - started, 2), round(pose.x_mm, 1),
                             round(pose.y_mm, 1), round(pose.heading_deg, 1),
                             cmd.kind, cmd.forward, cmd.left, cmd.ccw, cmd.ms))
                wait_until = time.monotonic() + cmd.ms / 1000.0 + 1.5
                while link.busy and time.monotonic() < wait_until:
                    time.sleep(0.02)
            else:
                time.sleep(0.05)
        return 0
    except ChassisError as exc:
        print(f"底盘错误：{exc}", file=sys.stderr)
        return 2
    finally:
        if link:
            link.stop()
            link.close()
        camera.close()
        if args.log and rows:
            _write_log(args.log, rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", action="store_true", help="离线仿真，不连相机和串口")
    parser.add_argument("--start", nargs=3, type=float, default=(150, 200, 90),
                        metavar=("X", "Y", "H"), help="仿真起点 mm/度")
    parser.add_argument("--target", nargs=2, type=float, required=True, metavar=("X", "Y"),
                        help="目标点，场地 mm")
    parser.add_argument("--heading", type=float, default=None, help="目标朝向，度，0=东")
    parser.add_argument("--max-percent", type=int, default=40)
    parser.add_argument("--deadline", type=float, default=90.0)
    parser.add_argument("--noise", type=float, default=0.0, help="仿真位置噪声 mm")
    parser.add_argument("--port", help="ESP32 串口，如 COM4；省略则只打印脉冲")
    parser.add_argument("--config", type=Path, help="field_view 标定文件")
    parser.add_argument("--scrcpy-dir", type=Path,
                        help="含 adb.exe/scrcpy.exe 的目录；实机必填")
    parser.add_argument("--serial")
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=90)
    parser.add_argument("--max-size", type=int, default=1920)
    parser.add_argument("--tag-mm", type=float, default=50.0)
    parser.add_argument("--tag-id", type=int, default=TAG_ID)
    parser.add_argument("--max-frames", type=int, default=20000)
    parser.add_argument("--log", type=Path, help="CSV 日志路径")
    args = parser.parse_args()
    if args.sim:
        return run_sim(args)
    if not args.config:
        parser.error("实机模式需要 --config field_calib.json")
    if not args.scrcpy_dir:
        parser.error("实机模式需要 --scrcpy-dir")
    return run_camera(args)


if __name__ == "__main__":
    raise SystemExit(main())
