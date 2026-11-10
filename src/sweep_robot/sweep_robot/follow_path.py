"""旧固定路线对照控制器，当前默认入口是 adaptive_controller。

开局横推两排，随后沿东墙北推，再进入己方残余清扫。
_begin_pile2 没有调用；不作为当前决策。保留此节点用于回归对照。
"""

import math
import os
import termios
import threading
import tty
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

Point = Tuple[float, float]
Goal = Tuple[float, float, float]

CRUISE = 0.45
EAST_X = 0.51
WEST_X = -0.53
NORTH_Y = 0.50
OWN_CLEAR_Y = 0.06
OPP_LANE = 0.06
OWN_LANE = -0.06
WALL_LANE_X = 0.515
WALL_START_Y = -0.53
SOUTH_STAGING_Y = -0.42
WALL_TURN_X = 0.20


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def world_to_body(vx_w: float, vy_w: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * vx_w + s * vy_w, -s * vx_w + c * vy_w


def body_to_world(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * vx - s * vy, s * vx + c * vy


def advance_pose(
    pose: Tuple[float, float, float],
    cmd: Tuple[float, float, float],
    dt: float,
) -> Tuple[float, float, float]:
    """用刚发出的车体速度，把场地坐标补到下一帧视觉到来之前。"""
    x, y, yaw = pose
    vx, vy, wz = cmd
    dx, dy = body_to_world(vx, vy, yaw)
    return x + dx * dt, y + dy * dt, wrap_angle(yaw + wz * dt)


# 车体坐标：+x 朝前。前挡板是滚刷那一面，只有这一面可以碰块。
# 皮肤按车体外形加半个物块，块心进了这圈就算侧板或车尾碰到了。
_SKIN_X0 = -0.060
_SKIN_X1 = 0.088
_SKIN_Y = 0.074
_FRONT_X0 = 0.005
_FRONT_HALF = 0.090
_LOOK_AHEAD = 0.08


def _project_blocks(pose: Tuple[float, float, float], blocks: List[Point]) -> List[Point]:
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    projected = []
    for bx, by in blocks:
        dx = bx - x
        dy = by - y
        projected.append((c * dx + s * dy, -s * dx + c * dy))
    return projected


def _pushable(px: float, py: float) -> bool:
    """落在前挡板正前方，往前走是用挡板推，不算侧面碰撞。"""
    return px >= _FRONT_X0 and abs(py) <= _FRONT_HALF


def _captured(px: float, py: float) -> bool:
    """已经进入三面腔体的块；前进时由腔体带走是设计行为。"""
    return -0.045 <= px <= 0.055 and abs(py) <= 0.035


def _skin_clearance(px: float, py: float) -> float:
    """块心到整车安全皮肤的距离。负值表示已经压进去了。"""
    if abs(px) > 0.40 or abs(py) > 0.40:
        return 1.0
    dx = 0.0
    if px < _SKIN_X0:
        dx = _SKIN_X0 - px
    elif px > _SKIN_X1:
        dx = px - _SKIN_X1
    dy = 0.0
    if py < -_SKIN_Y:
        dy = -_SKIN_Y - py
    elif py > _SKIN_Y:
        dy = py - _SKIN_Y
    if dx == 0.0 and dy == 0.0:
        return -min(px - _SKIN_X0, _SKIN_X1 - px, py + _SKIN_Y, _SKIN_Y - py)
    return math.hypot(dx, dy)


def _worst_clearance(projected: List[Point], ignore: set) -> float:
    worst = 1.0
    for index, (px, py) in enumerate(projected):
        if index in ignore:
            continue
        worst = min(worst, _skin_clearance(px, py))
    return worst


def _contact_ok(
    pose: Tuple[float, float, float],
    cmd: Tuple[float, float, float],
    blocks: List[Point],
    allow_front_contact: bool = True,
) -> bool:
    """这一下不会让侧板或车尾靠近物块。已经贴上时，必须是在分开。"""
    current = _project_blocks(pose, blocks)
    vx, vy, wz = cmd
    if abs(wz) > 0.40:
        for px, py in current:
            if math.hypot(px, py) < 0.11:
                return False
    allow_front = (
        allow_front_contact
        and vx >= 0.06
        and abs(vy) <= abs(vx) * 0.55
        and abs(wz) < 0.35
    )
    # 倒车时腔里的块从开口退出，不算侧撞。
    dumping = vx < -0.05 and abs(vy) <= abs(vx) * 0.4 and abs(wz) < 0.35
    ignore = set()
    if allow_front or dumping:
        ignore = {
            index
            for index, (px, py) in enumerate(current)
            if (_pushable(px, py) and allow_front) or (_captured(px, py) and (allow_front or dumping))
        }
    now = _worst_clearance(current, ignore)
    later = _worst_clearance(_project_blocks(advance_pose(pose, cmd, _LOOK_AHEAD), blocks), ignore)
    if now >= 0.0:
        return later >= 0.004
    return later > now + 0.003


def limit_block_contact(
    pose: Tuple[float, float, float],
    cmd: Tuple[float, float, float],
    blocks: List[Point],
    previous: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    allow_front_contact: bool = True,
) -> Tuple[float, float, float]:
    """从安全速度里选最接近原目标和上一拍的一项，避免左右来回跳。"""
    if not blocks or _contact_ok(pose, cmd, blocks, allow_front_contact):
        return cmd
    vx, vy, wz = cmd
    projected = _project_blocks(pose, blocks)
    if not allow_front_contact and any(_captured(px, py) for px, py in projected):
        dump = (-0.12, 0.0, 0.0)
        if _contact_ok(pose, dump, blocks, False):
            return dump
    if abs(wz) > 0.40:
        close = [(px, py) for px, py in projected if math.hypot(px, py) < 0.11]
        if close:
            px, py = min(close, key=lambda point: math.hypot(*point))
            # 转动会用侧板扫到近处物块，先沿最短方向拉开到 11cm，再原地转。
            escapes = []
            if abs(px) >= abs(py):
                escapes.append((-0.12 if px > 0.0 else 0.12, 0.0, 0.0))
            if abs(py) > 0.01:
                escapes.append((0.0, -0.12 if py > 0.0 else 0.12, 0.0))
            escapes.extend(((-0.12, 0.0, 0.0), (0.12, 0.0, 0.0)))
            for item in escapes:
                if _contact_ok(pose, item, blocks, False):
                    return item
    if allow_front_contact:
        # 推送时必须守住当前车道，只能直推、减速或沿原路退开。
        candidates = [
            (vx, 0.0, max(-0.30, min(0.30, wz))),
            (0.7 * vx, 0.0, 0.0),
            (0.4 * vx, 0.0, 0.0),
            (-0.10, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ]
        if abs(previous[1]) < 0.02:
            candidates.insert(0, previous)
    else:
        candidates = [previous, (vx, vy, 0.0), (0.7 * vx, 0.7 * vy, 0.0)]
        for scale in (0.5, 0.25):
            candidates.extend(
                (
                    (scale * vx, scale * vy, scale * wz),
                    (scale * vx, 0.0, 0.0),
                    (0.0, scale * vy, 0.0),
                )
            )
        candidates.extend(
            (
                (0.12, 0.0, 0.0),
                (-0.12, 0.0, 0.0),
                (0.0, 0.12, 0.0),
                (0.0, -0.12, 0.0),
                (0.0, 0.0, 0.0),
            )
        )
    safe = [item for item in candidates if _contact_ok(pose, item, blocks, allow_front_contact)]
    # 避碰不能往目标的反方向开。宁可停住，交给卡住脱困，也不横穿整场。
    if math.hypot(vx, vy) > 0.05:
        aligned = [item for item in safe if item[0] * vx + item[1] * vy >= -0.002]
        safe = aligned
    if not safe:
        return (0.0, 0.0, 0.0)

    def cost(item: Tuple[float, float, float]) -> float:
        change = sum((item[i] - previous[i]) ** 2 for i in range(3))
        error = sum((item[i] - cmd[i]) ** 2 for i in range(3))
        return error + 0.7 * change

    return min(safe, key=cost)


def _lead_face(
    pose: Tuple[float, float, float],
    blocks: List[Point],
    extents: List[Point],
    gx: float,
    gy: float,
    gyaw: float,
) -> Optional[float]:
    """往东墙或北墙推时，最靠墙的那一侧外沿。别的路段不管。"""
    x, y, _yaw = pose
    east = gx >= 0.42 and abs(gyaw) < 0.5
    north = gy >= 0.42 and abs(wrap_angle(gyaw - math.pi / 2)) < 0.5
    if not east and not north:
        return None
    best: Optional[float] = None
    # 整排合成一条时长边能到二十多厘米，不能拿来判断贴墙，否则人还在场地中间就停。
    del extents
    for _index, (bx, by) in enumerate(blocks):
        # 只看眼前这一截。北墙角已经堆住的块不能把南边这一铲的速度一起压没。
        if east and x + 0.02 < bx < x + 0.25 and abs(by - y) < 0.07:
            face = bx + 0.012
        elif north and y + 0.02 < by < y + 0.25 and abs(bx - x) < 0.07:
            face = by + 0.012
        else:
            continue
        best = face if best is None else max(best, face)
    return best


def _opening() -> List[Goal]:
    """从西南空区直接转向；横推两排后贴东墙北推。"""
    north = math.pi / 2
    return [
        (WEST_X, -0.22, north),
        (WEST_X, -0.22, 0.0),
        (WEST_X, OPP_LANE, 0.0),
        (EAST_X, OPP_LANE, 0.0),
        (WEST_X, OPP_LANE, 0.0),
        (WEST_X, OWN_LANE, 0.0),
        (EAST_X, OWN_LANE, 0.0),
        (WALL_TURN_X, OWN_LANE, 0.0),
        (WALL_TURN_X, WALL_START_Y, 0.0),
        (WALL_TURN_X, WALL_START_Y, north),
        (WALL_LANE_X, WALL_START_Y, north),
        (WALL_LANE_X, NORTH_Y, north),
    ]


def _opening_modes() -> List[str]:
    return [
        "transit",
        "turn",
        "align",
        "push_east",
        "retreat",
        "align",
        "push_east",
        "retreat",
        "transit",
        "turn",
        "align",
        "push_north",
    ]


class FollowPath(Node):
    def __init__(self) -> None:
        super().__init__("follow_path")
        self.declare_parameter("autostart", False)
        autostart = self.get_parameter("autostart").value
        if isinstance(autostart, str):
            autostart = autostart.lower() in ("1", "true", "yes")
        self.started = bool(autostart)
        if self.started:
            self.get_logger().info("已自动开始")
        else:
            threading.Thread(target=self._wait_space, daemon=True).start()
            self.get_logger().info("车已停住。在这个终端按空格开始")

        self.pose: Optional[Tuple[float, float, float]] = None
        self.measured: Optional[Tuple[float, float, float]] = None
        self.vision_velocity = (0.0, 0.0, 0.0)
        self.pose_time = 0.0
        self.tick_time = 0.0
        self.blocks: List[Point] = []
        self.extents: List[Point] = []
        self.blocks_seq = 0
        self.verify_after_seq = 0
        self.goals: List[Goal] = _opening()
        self.goal_modes = _opening_modes()
        self.index = 0
        self.phase = "route"
        self.done = False
        self.lane_set = False
        self.current_x: Optional[float] = None
        self.current_anchor_x: Optional[float] = None
        self.wall_hits = 0
        self.recovery_cmd: Optional[Tuple[float, float, float]] = None
        self.recovery_safe_ticks = 0
        self.cmd = (0.0, 0.0, 0.0)
        self.target = (0.0, 0.0, 0.0)
        self.progress_key = None
        self.progress_anchor = (0.0, 0.0, 0.0)
        self.progress_time = 0.0
        self.force_until = 0.0
        self.force_cmd = (0.0, 0.0, 0.0)
        self.unstick_step = 0
        self.last_detected_x: Optional[float] = None
        self.own_repeats = 0

        self.create_subscription(Pose2D, "/perception/robot_pose", self._on_pose, 10)
        self.create_subscription(Float32MultiArray, "/perception/blocks", self._on_blocks, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info("先推对方那一排到东墙，再推己方那一排")

    def _wait_space(self) -> None:
        try:
            fd = os.open("/dev/tty", os.O_RDONLY)
        except OSError as exc:
            self.get_logger().error(f"读不到启动终端的键盘: {exc}")
            return
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok() and not self.started:
                key = os.read(fd, 1)
                if key == b" ":
                    self.started = True
                    self.get_logger().info("已按空格，开始")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            os.close(fd)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg: Pose2D) -> None:
        fresh = (float(msg.x), float(msg.y), float(msg.theta))
        now = self._now()
        if self.measured is not None and self.pose_time > 0.0:
            dt = max(0.04, min(0.25, now - self.pose_time))
            vx = max(-0.55, min(0.55, (fresh[0] - self.measured[0]) / dt))
            vy = max(-0.55, min(0.55, (fresh[1] - self.measured[1]) / dt))
            wz = max(-3.0, min(3.0, wrap_angle(fresh[2] - self.measured[2]) / dt))
            old_vx, old_vy, old_wz = self.vision_velocity
            self.vision_velocity = (
                0.65 * vx + 0.35 * old_vx,
                0.65 * vy + 0.35 * old_vy,
                0.65 * wz + 0.35 * old_wz,
            )
        self.measured = fresh
        self.pose = fresh
        self.pose_time = now

    def _predict(self, dt: float) -> None:
        del dt
        if self.measured is None:
            return
        # 10Hz 一帧最多走约 3cm，小于到点容差；直接信最新视觉比预测后过冲更稳。
        self.pose = self.measured

    def _on_blocks(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) >= 4 and len(data) % 4 == 0:
            self.blocks = [(data[i], data[i + 1]) for i in range(0, len(data), 4)]
            self.extents = [(data[i + 2], data[i + 3]) for i in range(0, len(data), 4)]
        else:
            self.blocks = [(data[i], data[i + 1]) for i in range(0, len(data) - 1, 2)]
            self.extents = [(0.01, 0.01) for _ in self.blocks]
        self.blocks_seq += 1

    def _tick(self) -> None:
        if not self.started:
            self._emit(0.0, 0.0, 0.0)
            return
        now = self._now()
        dt = 0.0 if self.tick_time <= 0.0 else min(0.10, max(0.0, now - self.tick_time))
        self.tick_time = now
        if self.pose is None or now - self.pose_time > 0.8:
            self.target = (0.0, 0.0, 0.0)
            self._flush()
            return
        if self.done:
            self.target = (0.0, 0.0, 0.0)
            self._flush()
            return
        if self.phase == "verify":
            self.target = (0.0, 0.0, 0.0)
            if self.blocks_seq >= self.verify_after_seq:
                self._begin_own()
            self._flush()
            return
        self._predict(dt)
        self._nudge()
        self._advance()
        if self.done or self.index >= len(self.goals):
            self.target = (0.0, 0.0, 0.0)
        else:
            self._steer()
        self._limit_contact()
        self._watch_progress()
        self._flush()

    def _nudge(self) -> None:
        """第二铲和单个己方块的 x 跟着相机慢慢偏，不一下子跳过去。"""
        if self.phase not in ("pile2", "own") or self.current_x is None:
            return
        if self.pose[1] > 0.12:
            return
        seen = [block for block in self.blocks if abs(block[0] - self.current_x) < 0.08 and block[1] < 0.35]
        if not seen:
            return
        aim = min(seen, key=lambda block: abs(block[0] - self.current_x))[0]
        old_x = self.current_x
        step = max(-0.006, min(0.006, aim - self.current_x))
        anchor = self.current_x if self.current_anchor_x is None else self.current_anchor_x
        limit = 0.515 if self.phase == "pile2" else 0.525
        self.current_x = min(limit, max(-0.52, max(anchor - 0.035, min(anchor + 0.035, self.current_x + step))))
        self.goals = [
            (self.current_x, gy, gyaw) if abs(gx - old_x) < 0.08 else (gx, gy, gyaw)
            for gx, gy, gyaw in self.goals
        ]

    def _advance(self) -> None:
        x, y, yaw = self.pose
        while self.index < len(self.goals):
            gx, gy, gyaw = self.goals[self.index]
            mode = self._current_mode()
            if (
                mode == "align"
                and abs(wrap_angle(gyaw - yaw)) < 0.40
                and abs(wrap_angle(gyaw - math.pi / 2)) < 0.25
                and x > gx - 0.12
            ):
                # 刷子已经碰上这一列，就转入北推。继续横移会把块挤到南墙角。
                ahead = _project_blocks(self.pose, self.blocks)
                if any(-0.02 <= px <= 0.14 and abs(py) <= 0.12 for px, py in ahead):
                    if not self._complete_goal():
                        return
                    continue
            at_wall = mode.startswith("push_") and self._against_wall(gx, gy, gyaw)
            self.wall_hits = self.wall_hits + 1 if at_wall else 0
            if self.wall_hits >= 2:
                self.get_logger().info("物块已经贴墙，不再往上顶")
                if not self._complete_goal():
                    return
                continue
            crossed = (
                mode == "push_north"
                and y >= gy - 0.03
                and abs(x - gx) < 0.08
            ) or (
                mode == "push_east"
                and x >= gx - 0.03
                and abs(y - gy) < 0.08
            )
            if crossed and abs(wrap_angle(gyaw - yaw)) < 0.22:
                if not self._complete_goal():
                    return
                continue
            if mode == "turn" and abs(wrap_angle(gyaw - yaw)) < 0.10:
                if not self._complete_goal():
                    return
                continue
            dist = math.hypot(gx - x, gy - y)
            # 过路点下一拍才转身，角度没摆正也不能冲过点位。
            tolerance = 0.050 if mode in ("transit", "retreat") else 0.028
            yaw_tol = 0.45 if mode == "transit" else 0.14
            if dist < tolerance and abs(wrap_angle(gyaw - yaw)) < yaw_tol:
                if not self._complete_goal():
                    return
                continue
            return

    def _complete_goal(self) -> bool:
        if self.phase == "route" and self.index == 2:
            self._apply_opponent_lane()
        elif self.phase == "route" and self.index == 4:
            self._apply_own_lane()
        elif self.phase == "route" and self.index == 7:
            self._apply_wall_lane()
        self.wall_hits = 0
        self.recovery_cmd = None
        self.index += 1
        if self.index >= len(self.goals):
            self._on_phase_done()
            return False
        gx, gy, gyaw = self.goals[self.index]
        self.get_logger().info(
            f"下一动作 {self._current_mode()}，终点({gx:.2f},{gy:.2f})，朝向 {gyaw:.2f}"
        )
        return True

    def _current_mode(self) -> str:
        if 0 <= self.index < len(self.goal_modes):
            return self.goal_modes[self.index]
        return "transit"

    def _against_wall(self, gx: float, gy: float, gyaw: float) -> bool:
        """正在往墙里推，而且最外侧的块已经贴上墙。"""
        east = gx >= 0.42 and abs(gyaw) < 0.5
        north = gy >= 0.42 and abs(wrap_angle(gyaw - math.pi / 2)) < 0.5
        # 墙边本来就有别的块。车还没把这一铲送到墙边时，不能拿别的块冒充当前块。
        if east and self.pose[0] < 0.40:
            return False
        if north and self.pose[1] < 0.40:
            return False
        face = _lead_face(self.pose, self.blocks, self.extents, gx, gy, gyaw)
        return face is not None and face >= 0.585

    def _on_phase_done(self) -> None:
        if self.phase == "route":
            self._begin_own()
        elif self.phase == "pile2":
            self._begin_own()
        elif self.phase == "own":
            # 等下一张相机画面再复查，不能拿推之前的旧列表立即重选同一块。
            self.phase = "verify"
            self.current_x = None
            self.current_anchor_x = None
            self.goals = []
            self.index = 0
            self.verify_after_seq = self.blocks_seq + 1

    def _apply_opponent_lane(self) -> None:
        ys = sorted(
            block[1] for block in self.blocks if 0.02 < block[1] < 0.14 and abs(block[0]) < 0.35
        )
        lane = OPP_LANE if not ys else min(0.10, max(0.04, ys[len(ys) // 2]))
        for index in (2, 3, 4):
            gx, _gy, gyaw = self.goals[index]
            self.goals[index] = (gx, lane, gyaw)
        self.get_logger().info(f"对方这一排在 y={lane:.2f}，从西侧只推这一排")

    def _apply_own_lane(self) -> None:
        self.lane_set = True
        ys = sorted(
            block[1]
            for block in self.blocks
            if -0.14 < block[1] < -0.02 and abs(block[0]) < 0.35
        )
        lane = OWN_LANE if not ys else min(-0.04, max(-0.10, ys[len(ys) // 2]))
        for index in (5, 6, 7):
            gx, _gy, gyaw = self.goals[index]
            self.goals[index] = (gx, lane, gyaw)
        self.get_logger().info(f"己方这一排大约在 y={lane:.2f}，沿这一排推到东墙")

    def _apply_wall_lane(self) -> None:
        xs = sorted(block[0] for block in self.blocks if block[0] > 0.35 and block[1] < 0.25)
        lane = WALL_LANE_X
        if xs:
            lane = min(0.530, max(0.49, xs[-1] - 0.060))
        for index in (10, 11):
            _gx, gy, gyaw = self.goals[index]
            self.goals[index] = (lane, gy, gyaw)
        self.get_logger().info(f"东墙物块在 x≈{lane + 0.045:.2f}，车心贴墙线 x={lane:.2f}")

    def _begin_pile2(self) -> None:
        aim = self._half_pile()
        if aim is None:
            self.get_logger().info("没有看到侧漏的半堆，不再空跑第二铲")
            self._begin_own()
            return
        px, py = aim
        north = math.pi / 2
        self.phase = "pile2"
        self.current_x = px
        self.current_anchor_x = px
        self.goals = [(px, py, north), (px, NORTH_Y, north)]
        self.goal_modes = ["align", "push_north"]
        self.index = 0
        self.get_logger().info(f"第二铲对准侧漏 x={px:.2f}")

    def _half_pile(self) -> Optional[Point]:
        """东墙边还没推到北墙的那一堆。只对准最东边的一团，不往场中空跑。"""
        leaked = [block for block in self.blocks if block[0] > 0.12 and -0.15 < block[1] < 0.32]
        if not leaked:
            return None
        east = max(block[0] for block in leaked)
        pile = [block for block in leaked if block[0] >= east - 0.10]
        xs = sorted(block[0] for block in pile)
        ys = sorted(block[1] for block in pile)
        px = min(0.52, max(0.25, xs[len(xs) // 2]))
        py = min(0.0, max(-0.40, ys[0] - 0.14))
        return px, py

    def _own_targets(self) -> List[Point]:
        pending = [block for block in self.blocks if block[1] < 0.0]
        pending.sort(key=lambda block: block[0])
        return pending

    def _begin_own(self) -> None:
        pending = sorted(self._own_targets(), key=lambda block: -block[0])
        if not pending:
            self.done = True
            self.phase = "done"
            self.current_x = None
            self.get_logger().info("己方半区剩下的块已经逐个推完")
            return
        bx, _by = pending[0]
        detected = bx
        column = [block for block in pending if abs(block[0] - detected) < 0.10] or [pending[0]]
        south = min(block[1] for block in column)
        if self.last_detected_x is not None and abs(detected - self.last_detected_x) < 0.04:
            self.own_repeats = min(3, self.own_repeats + 1)
        else:
            self.own_repeats = 0
        self.last_detected_x = detected
        if detected > 0.35:
            # 贴东墙的块在刷子外侧，再往西让会越让越空。往墙靠一点。
            bx = min(0.535, detected + 0.02 * self.own_repeats)
        else:
            bx = detected - 0.05 * self.own_repeats
        bx = min(0.535, max(-0.52, bx))
        # 东墙残堆又长又贴墙，只送到中线会把南边一截留下，下一轮还是同一堆。
        # 横移对准用这一列最南端，不能用最西那一块的 y，否则会从堆中间穿过去。
        clear_y = NORTH_Y if bx > 0.40 else OWN_CLEAR_Y
        approach_y = min(south - 0.14, -0.20 if bx > 0.40 else -0.16)
        approach_y = max(WALL_START_Y, approach_y)
        stage_y = max(WALL_START_Y, min(SOUTH_STAGING_Y, approach_y))
        escape_y = min(self.pose[1], 0.28)
        escape_x = max(-0.50, min(0.35, min(self.pose[0] - 0.14, bx - 0.14)))
        north = math.pi / 2
        self.phase = "own"
        self.current_x = bx
        self.current_anchor_x = bx
        if abs(bx - self.pose[0]) > 0.30:
            # 横推后场地中部已经清空。远目标走对角线，比横一段、竖一段更快，
            # 每拍仍经过侧板/车尾碰撞检查。
            self.goals = [
                (self.pose[0], escape_y, north),
                (bx, stage_y, north),
                (bx, approach_y, north),
                (bx, clear_y, north),
            ]
            self.goal_modes = ["retreat", "transit", "approach", "push_north"]
        else:
            self.goals = [
                # 先横移离开墙边那一堆，再南下。倒车会从刚推上去的堆里穿过去。
                (escape_x, min(self.pose[1], 0.42), north),
                (escape_x, stage_y, north),
                (bx, stage_y, north),
                (bx, approach_y, north),
                (bx, clear_y, north),
            ]
            self.goal_modes = ["align", "retreat", "align", "approach", "push_north"]
        self.index = 0
        self.get_logger().info(
            f"推己方剩余 {len(column)} 块，对准 x={bx:.3f}，南端 y={south:.2f}，从 y={stage_y:.2f} 往北"
        )

    def _steer(self) -> None:
        x, y, yaw = self.pose
        gx, gy, gyaw = self.goals[self.index]
        mode = self._current_mode()
        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)
        yaw_err = wrap_angle(gyaw - yaw)
        # 10Hz 视觉下高速转动会一帧越过目标十几度。转身、行驶、推送分别限速，
        # 让角度误差单调收敛；推送时还要避免侧向扫动物块。
        if mode == "turn":
            yaw_gain, yaw_limit = 1.2, 0.80
        elif mode.startswith("push_"):
            yaw_gain, yaw_limit = 1.5, 0.30
        else:
            yaw_gain, yaw_limit = 1.4, 0.65
        wz = max(-yaw_limit, min(yaw_limit, yaw_gain * yaw_err))
        if mode == "turn":
            self.target = (0.0, 0.0, wz)
            return

        top_speed = 0.30 if mode.startswith("push_") else CRUISE
        remaining = max(0.0, dist - 0.015)
        speed = min(top_speed, 1.8 * remaining)
        ayaw = abs(yaw_err)
        # 对准时先把车头摆正再平移。贴边横移再放慢，避免把块挤进刷子够不到的墙角。
        if mode == "align":
            speed = min(speed, 0.22)
        if (mode == "align" and ayaw > 0.12) or (
            mode in ("approach", "retreat") and ayaw > 0.18
        ):
            self.target = (0.0, 0.0, wz)
            return
        # 连续地收平移，不设会让速度突然跳变的角度开关。
        speed *= max(0.0, 1.0 - (ayaw / 0.90) ** 2)
        face = _lead_face(self.pose, self.blocks, self.extents, gx, gy, gyaw)
        near_wall = (
            abs(gyaw) < 0.5 and gx >= 0.42 and x > 0.35
        ) or (
            abs(wrap_angle(gyaw - math.pi / 2)) < 0.5 and gy >= 0.42 and y > 0.35
        )
        if face is not None and near_wall:
            gap = 0.60 - face
            if gap < 0.10:
                speed = min(speed, max(0.08, 0.28 * gap / 0.10))

        if mode == "push_east":
            vx_w = max(0.0, speed if dx > 0.0 else 0.0)
            vy_w = max(-0.04, min(0.04, 1.2 * dy))
        elif mode == "push_north":
            vx_w = max(-0.04, min(0.04, 1.2 * dx))
            vy_w = max(0.0, speed if dy > 0.0 else 0.0)
        elif dist < 1e-4:
            vx_w = vy_w = 0.0
        else:
            vx_w = speed * dx / dist
            vy_w = speed * dy / dist
        vx, vy = world_to_body(vx_w, vy_w, yaw)
        self.target = (vx, vy, wz)

    def _limit_contact(self) -> None:
        allow_front = self._current_mode().startswith("push_")
        if self.recovery_cmd is not None:
            if _contact_ok(self.pose, self.target, self.blocks, allow_front):
                self.recovery_safe_ticks += 1
            else:
                self.recovery_safe_ticks = 0
            opposes = (
                self.recovery_cmd[0] * self.target[0] + self.recovery_cmd[1] * self.target[1] < -0.002
                and math.hypot(self.target[0], self.target[1]) > 0.05
            )
            if self.recovery_safe_ticks >= 3 or opposes:
                self.recovery_cmd = None
            elif _contact_ok(self.pose, self.recovery_cmd, self.blocks, False):
                self.target = self.recovery_cmd
                return
            else:
                self.recovery_cmd = None

        limited = limit_block_contact(
            self.pose,
            self.target,
            self.blocks,
            self.cmd,
            allow_front,
        )
        if limited != self.target:
            nearby = sorted(
                (
                    (round(px, 3), round(py, 3), round(_skin_clearance(px, py), 3))
                    for px, py in _project_blocks(self.pose, self.blocks)
                    if math.hypot(px, py) < 0.24
                ),
                key=lambda item: item[2],
            )
            self.get_logger().info(
                f"{self._current_mode()} 检测到侧边或车尾碰撞风险，"
                f"目标={tuple(round(value, 2) for value in self.target)}，"
                f"限制={tuple(round(value, 2) for value in limited)}，近块={nearby[:4]}",
                throttle_duration_sec=1.0,
            )
            if (
                math.hypot(limited[0], limited[1]) >= 0.08
                and (abs(self.target[2]) > 0.40 or math.hypot(self.target[0], self.target[1]) < 0.05)
            ):
                self.recovery_cmd = limited
                self.recovery_safe_ticks = 0
        self.target = limited

    def _watch_progress(self) -> None:
        """位置停太久就换一个安全方向挪开，避免在同一处顶墙。"""
        if self.pose is None or self.done:
            return
        now = self._now()
        if now < self.force_until and _contact_ok(self.pose, self.force_cmd, self.blocks, False):
            self.target = self.force_cmd
            self.recovery_cmd = None
            return
        key = (self.phase, self.index)
        x, y, _yaw = self.pose
        if key != self.progress_key:
            self.progress_key = key
            self.progress_anchor = (x, y, self.pose[2])
            self.progress_time = now
            self.unstick_step = 0
            return
        moved = math.hypot(x - self.progress_anchor[0], y - self.progress_anchor[1])
        turned = abs(wrap_angle(self.pose[2] - self.progress_anchor[2])) if len(self.progress_anchor) > 2 else 0.0
        if moved > 0.04 or (self._current_mode() == "turn" and turned > 0.12):
            self.progress_anchor = (x, y, self.pose[2])
            self.progress_time = now
            return
        if now - self.progress_time < 1.3:
            return
        if self.pose[1] < -0.40 and abs(wrap_angle(self.pose[2] - math.pi / 2)) < 0.6:
            options = ((0.0, 0.14, 0.0), (0.14, 0.0, 0.0), (-0.14, 0.0, 0.0), (0.0, -0.14, 0.0))
        else:
            options = ((-0.14, 0.0, 0.0), (0.0, -0.14, 0.0), (0.0, 0.14, 0.0), (0.14, 0.0, 0.0))
        for offset in range(len(options)):
            cmd = options[(self.unstick_step + offset) % len(options)]
            if _contact_ok(self.pose, cmd, self.blocks, False):
                self.force_cmd = cmd
                self.force_until = now + 0.55
                self.target = cmd
                self.recovery_cmd = None
                self.unstick_step += 1
                self.progress_time = now
                self.get_logger().info(
                    f"卡住，沿车体方向 {tuple(round(value, 2) for value in cmd)} 挪开"
                )
                return

    def _flush(self) -> None:
        tx, ty, tw = self.target
        x, y, w = self.cmd
        x = _approach(x, tx, 1.6)
        y = _approach(y, ty, 1.6)
        w = _approach(w, tw, 5.0)
        self.cmd = (x, y, w)
        self._emit(x, y, w)

    def _emit(self, vx: float, vy: float, wz: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.pub.publish(cmd)


def _approach(current: float, goal: float, accel: float) -> float:
    step = accel * 0.05
    delta = goal - current
    if delta > step:
        return current + step
    if delta < -step:
        return current - step
    return goal


def main() -> None:
    rclpy.init()
    node = FollowPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
