"""先沿 +y 直走到交界，再原地转到略偏 +y 的方向，用挡板一次把能碰到的块推到侧墙。

不要求一整列都贴墙才算完成。靠墙之后去推己方半区剩下的块，同样用车头挡板，推到交界就换下一铲。
倒车前先确认轮子正后方没有块。前进方向上出现比物块大、又不是红块和色标的物体时先停 2 秒。
"""

import math
import os
import termios
import threading
import tty
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

Point = Tuple[float, float]
Pose = Tuple[float, float, float]

WALL_Y = 0.50
SIDE_X = 0.50
ROW_Y = 0.05
OWN_Y = 0.02
OPEN_Y = -0.08
OWN_GAP = 0.04
OWN_SPAN = 0.10
PUSH_SPEED = 0.38
# 横推时车头不严格朝 +x，略偏 +y，一铲能罩住更多块。
RAKE_YAW = 0.18
# 车体中心超过这里，挡板前的块已经进了对方半区，交给后面的横推。
CROSS_Y = 0.16
# 倒车时轮子左右大约各 7.5cm，这个宽度里有块就不要退。
WHEEL_HALF = 0.075
# 转向前先离开两排。原地转外沿约 9cm，中心放在 -0.46 才不会扫到西墙。
TURN_X = -0.46
TURN_Y = -0.30


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def world_to_body(vx_w: float, vy_w: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * vx_w + s * vy_w, -s * vx_w + c * vy_w


def mean_x(group: Sequence[Point]) -> float:
    return sum(block[0] for block in group) / len(group)


def cluster_by_gap(blocks: Sequence[Point], gap: float) -> List[List[Point]]:
    """相距 gap 以内的块合成一堆，叠在一起的也算。按堆中心的 x 排序。"""
    points = list(blocks)
    parent = list(range(len(points)))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for i, left in enumerate(points):
        for j in range(i + 1, len(points)):
            right = points[j]
            if math.hypot(left[0] - right[0], left[1] - right[1]) <= gap:
                parent[find(i)] = find(j)
    buckets: dict = {}
    for index, block in enumerate(points):
        buckets.setdefault(find(index), []).append(block)
    groups = list(buckets.values())
    groups.sort(key=mean_x)
    return groups


def cluster_by_span(blocks: Sequence[Point], span: float) -> List[List[Point]]:
    """按 x 从西向东装，每堆跨度不超过挡板宽度，一铲能推完。"""
    ordered = sorted(blocks, key=lambda block: block[0])
    groups: List[List[Point]] = []
    current: List[Point] = []
    for block in ordered:
        if current and block[0] - current[0][0] > span:
            groups.append(current)
            current = []
        current.append(block)
    if current:
        groups.append(current)
    return groups


class PushBlocks(Node):
    def __init__(self) -> None:
        super().__init__("push_blocks")
        self.declare_parameter("vmax", 0.35)
        self.declare_parameter("wmax", 1.5)
        self.vmax = float(self.get_parameter("vmax").value)
        self.wmax = float(self.get_parameter("wmax").value)

        self.blocks: List[Point] = []
        self.obstacles: List[Point] = []
        self.pose: Optional[Pose] = None
        self.blocks_time = 0.0
        self.pose_time = 0.0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.target_cmd = (0.0, 0.0, 0.0)
        self.seen_stamp = -1.0
        self.started = False
        self.declare_parameter("autostart", False)
        autostart = self.get_parameter("autostart").value
        if isinstance(autostart, str):
            autostart = autostart.lower() in ("1", "true", "yes")
        if autostart:
            self.started = True
            self.get_logger().info("已自动开始推块")
        else:
            threading.Thread(target=self._wait_space, daemon=True).start()
            self.get_logger().info("车已停住。在这个终端按空格，开始推块")

        self.phase = "deliver"
        self.mode = "push"
        self.end_x: Optional[float] = None
        self.target_x: Optional[float] = None
        self.skipped: List[float] = []
        self.wait_until = 0.0
        self.wait_obs: Optional[Point] = None
        self.probing = False
        self.probe_until = 0.0
        self.probe_xy: Optional[Point] = None
        self.passed_wait = False
        self.sweep_returns = 0
        self.second_pass = False
        self.push_since = 0.0
        self.pack_since = 0.0
        self.lane_y: Optional[float] = None

        self.create_subscription(Float32MultiArray, "/perception/blocks", self._on_blocks, 10)
        self.create_subscription(Float32MultiArray, "/perception/obstacles", self._on_obstacles, 10)
        self.create_subscription(Pose2D, "/perception/robot_pose", self._on_pose, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.05, self._tick)

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
                    self.get_logger().info("已按空格，开始推块")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            os.close(fd)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_blocks(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        self.blocks = [(data[i], data[i + 1]) for i in range(0, len(data) - 1, 2)]
        self.blocks_time = self._now()

    def _on_obstacles(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        self.obstacles = [(data[i], data[i + 1]) for i in range(0, len(data) - 1, 2)]

    def _on_pose(self, msg: Pose2D) -> None:
        now = self._now()
        fresh = (msg.x, msg.y, msg.theta)
        if self.pose is None:
            self.pose = fresh
        else:
            yaw = self.pose[2] + 0.8 * wrap_angle(fresh[2] - self.pose[2])
            self.pose = (
                0.75 * fresh[0] + 0.25 * self.pose[0],
                0.75 * fresh[1] + 0.25 * self.pose[1],
                yaw,
            )
        self.pose_time = now

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        vx = float(max(-self.vmax, min(self.vmax, vx)))
        vy = float(max(-self.vmax, min(self.vmax, vy)))
        wz = float(max(-self.wmax, min(self.wmax, wz)))
        self.target_cmd = (vx, vy, wz)

    def _flush(self) -> None:
        """每拍只把速度往目标挪一小步，避免目标一变就把车速打成台阶。"""
        tx, ty, tw = self.target_cmd
        x, y, w = self.last_cmd
        x = self._approach(x, tx, 2.0)
        y = self._approach(y, ty, 2.0)
        w = self._approach(w, tw, 8.0)
        self.last_cmd = (x, y, w)
        self._emit(x, y, w)

    def _approach(self, current: float, goal: float, accel: float) -> float:
        step = accel * 0.05
        delta = goal - current
        if delta > step:
            return current + step
        if delta < -step:
            return current - step
        return goal

    def _emit(self, vx: float, vy: float, wz: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.pub.publish(cmd)

    def _drive(self, vx_w: float, vy_w: float, yaw: float, goal: float = math.pi / 2, hold: bool = True) -> None:
        """世界坐标速度。航向差得不大就边走边转，差过直角才停下来转。"""
        err = wrap_angle(goal - yaw)
        scale = 1.0
        if hold and abs(err) > 1.0:
            vx_w = 0.0
            vy_w = 0.0
        elif abs(err) > 0.5:
            scale = 0.6
        vx, vy = world_to_body(scale * vx_w, scale * vy_w, yaw)
        self._publish(vx, vy, max(-1.2, min(1.2, 2.2 * err)))

    def _groups(self) -> List[List[Point]]:
        if self.phase != "own":
            return []
        pool = [block for block in self.blocks if -0.55 < block[1] < -0.02]
        groups = cluster_by_span(pool, OWN_SPAN)
        kept = []
        for group in groups:
            center = mean_x(group)
            if any(abs(center - banned) < 0.05 for banned in self.skipped):
                continue
            kept.append(group)
        return kept

    def _approach_y(self, group: Sequence[Point]) -> float:
        # 靠蓝区墙一侧的空区。-0.18 附近会有被碰下来的块，横移会顶住。
        return -0.40

    def _row_blocks(self) -> List[Point]:
        """还在对方那一排、没到 +x 侧墙的块。"""
        return [block for block in self.blocks if 0.0 < block[1] < 0.20 and block[0] < SIDE_X]

    def _row_y(self) -> float:
        row = self._row_blocks()
        if row:
            ys = [block[1] for block in row]
            if max(ys) - min(ys) <= 0.09:
                return (min(ys) + max(ys)) / 2.0
            ordered = sorted(ys)
            return ordered[len(ordered) // 2]
        return ROW_Y

    def _motion(self, x: float, y: float, group: Optional[Sequence[Point]]) -> Tuple[float, float]:
        """当前打算走的世界方向，用来判断前方有没有挡路。"""
        if self.phase == "done":
            return 0.0, 0.0
        if self.phase == "deliver":
            return 0.0, 1.0
        if self.phase == "retreat":
            return 0.0, -1.0
        if self.phase in ("turn", "spin"):
            return 0.0, 0.0
        if self.phase == "own" and self.mode == "south":
            return 0.0, -1.0
        if self.phase == "own" and self.mode == "west":
            return -1.0, 0.0
        if self.phase == "own" and self.mode in ("aim", "push", "back", "exit"):
            return 0.0, 1.0 if self.mode != "back" else -1.0
        if self.phase == "recover":
            return 0.0, -1.0
        return 1.0, 0.0

    def _ahead(self, x: float, y: float, dx: float, dy: float) -> Optional[Point]:
        if abs(dx) + abs(dy) < 0.5:
            return None
        found = None
        best = 1.0
        for ox, oy in self.obstacles:
            along = (ox - x) * dx + (oy - y) * dy
            cross = abs(-(oy - y) * dx + (ox - x) * dy)
            if 0.03 < along < 0.18 and cross < 0.09 and along < best:
                best = along
                found = (ox, oy)
        return found

    def _obs_shift(self, old: Point) -> float:
        if not self.obstacles:
            return 1.0
        return min(math.hypot(ox - old[0], oy - old[1]) for ox, oy in self.obstacles)

    def _skip(self) -> None:
        if self.phase == "end":
            if self.end_x is not None:
                self.skipped.append(self.end_x)
            self._begin_turn()
        elif self.phase in ("turn", "sweep"):
            self.phase = "own"
        elif self.target_x is not None:
            self.skipped.append(self.target_x)
        self.target_x = None
        if self.phase != "turn":
            self.mode = "approach"
        self.passed_wait = False
        self.probing = False
        self.probe_xy = None

    def _tick(self) -> None:
        now = self._now()
        if not self.started:
            self.target_cmd = (0.0, 0.0, 0.0)
            self.last_cmd = (0.0, 0.0, 0.0)
            self._emit(0.0, 0.0, 0.0)
            return
        # 仿真比墙钟慢时，画面大约一秒才来一帧。位姿稍晚就沿用上一条速度，
        # 只有真的丢了超过一秒才停，避免直行和转向都走一下停一下。
        pose_late = self.pose is None or now - self.pose_time > 1.0
        blocks_late = now - self.blocks_time > 2.0
        if pose_late or blocks_late:
            self.target_cmd = (0.0, 0.0, 0.0)
            self._flush()
            return
        stamp = min(self.pose_time, self.blocks_time)
        if stamp == self.seen_stamp:
            self._flush()
            return
        self.seen_stamp = stamp

        x, y, yaw = self.pose
        if self.wait_until and now < self.wait_until:
            self.target_cmd = (0.0, 0.0, 0.0)
            self._flush()
            return

        self._advance_phase()
        group = None
        dx, dy = self._motion(x, y, group)

        if self.wait_obs is not None and self.wait_until and now >= self.wait_until:
            shift = self._obs_shift(self.wait_obs)
            ahead = self._ahead(x, y, dx, dy)
            self.wait_obs = None
            self.wait_until = 0.0
            self.passed_wait = True
            if shift < 0.03 or ahead is not None:
                self.probing = True
                self.probe_xy = (x, y)
                self.probe_until = now + 1.5
                self.get_logger().info("挡路的物体几乎没动，按原路线继续")
            else:
                self.probing = False
                self.passed_wait = False

        ahead = self._ahead(x, y, dx, dy)
        if ahead is not None and not self.passed_wait and not self.probing:
            self.wait_until = now + 2.0
            self.wait_obs = ahead
            self.target_cmd = (0.0, 0.0, 0.0)
            self._flush()
            self.get_logger().info("前方有物体挡路，先停 2 秒")
            return

        if self.probing and self.probe_xy is not None and now >= self.probe_until:
            moved = math.hypot(x - self.probe_xy[0], y - self.probe_xy[1])
            self.probing = False
            if moved < 0.02 and ahead is not None:
                self.get_logger().info("还是过不去，绕到下一堆")
                self._skip()
                self._advance_phase()
                group = self._select_group()
            else:
                self.passed_wait = ahead is not None

        self._act(x, y, yaw, group)
        self._flush()

    def _advance_phase(self) -> None:
        if self.phase == "deliver" and self.end_x is None and self.pose is not None and self.pose[1] < -0.40:
            self.end_x = self.pose[0]
        if self.phase == "deliver" and self._at_boundary():
            self.phase = "turn"
            self.get_logger().info("到交界处，原地转向后再前进")

    def _at_boundary(self) -> bool:
        """挡板前缘大约在车心前方 0.065。车心过 -0.07 时铲面在交界，不进入对方半区。"""
        return self.pose is not None and self.pose[1] > -0.07

    def _path_clear(self, x: float, y: float, dx: float, dy: float, reach: float = 0.16) -> bool:
        for bx, by in self.blocks:
            along = (bx - x) * dx + (by - y) * dy
            cross = abs(-(by - y) * dx + (bx - x) * dy)
            if 0.025 < along < reach and cross < 0.055:
                return False
        return True

    def _begin_turn(self) -> None:
        self.phase = "turn"
        self.mode = "turn"

    def _begin_own(self) -> None:
        self.phase = "own"
        self.skipped.clear()
        self.target_x = None
        self.mode = "exit"
        self.passed_wait = False

    def _sweep_done(self, robot_x: float) -> bool:
        band = [block for block in self.blocks if 0.0 < block[1] < 0.20 and block[0] < SIDE_X]
        if not band:
            return True
        # 车到侧墙但这一排还有块，再从西侧推一次。两次之后交给己方。
        if robot_x >= SIDE_X and self.mode == "push":
            self.sweep_returns += 1
            self.mode = "lineup"
            return self.sweep_returns >= 2
        return False

    def _select_group(self) -> Optional[List[Point]]:
        groups = self._groups()
        if not groups:
            self.target_x = None
            return None
        if self.target_x is not None:
            for group in groups:
                if abs(mean_x(group) - self.target_x) < 0.10:
                    self.target_x = mean_x(group)
                    return group
        group = groups[0]
        new_x = mean_x(group)
        if self.target_x is None or abs(new_x - self.target_x) > 0.06:
            self.mode = "approach"
            self.passed_wait = False
        self.target_x = new_x
        return group

    def _rear_blocked(self, x: float, y: float) -> bool:
        """车头朝 +y 时，正后方轮距里有没有块。有的话倒车会把它卷到轮下。"""
        for bx, by in self.blocks:
            if y - 0.18 < by < y - 0.02 and abs(bx - x) < WHEEL_HALF:
                return True
        return False

    def _evade_left(self, x: float, y: float) -> float:
        """车头朝 +y 时，车体向左是世界 -x。选块更少的一侧让开。"""
        def count(x0: float, x1: float) -> int:
            lo, hi = min(x0, x1), max(x0, x1)
            return sum(1 for bx, by in self.blocks if lo <= bx <= hi and y - 0.18 < by < y + 0.06)

        west = count(x - 0.22, x - WHEEL_HALF)
        east = count(x + WHEEL_HALF, x + 0.22)
        if west <= east and x > -0.50:
            return 0.24
        return -0.24

    def _safe_back(self, x: float, y: float, yaw: float) -> None:
        if self._rear_blocked(x, y):
            self._north(yaw, 0.0, self._evade_left(x, y))
            return
        self._north(yaw, -0.22, 0.0)

    def _column_delivered(self, goal_x: float, robot_y: float) -> bool:
        if robot_y > 0.24:
            return True
        if robot_y <= CROSS_Y:
            return False
        near = [block for block in self.blocks if abs(block[0] - goal_x) < 0.07 and block[1] < 0.28]
        return not near or all(block[1] > 0.06 for block in near)

    def _sweep_row(self) -> List[Point]:
        """已经在对方半区、还没靠到 +x 墙的块。"""
        return [block for block in self.blocks if -0.02 < block[1] < 0.28 and block[0] < 0.48]

    def _act(self, x: float, y: float, yaw: float, group: Optional[Sequence[Point]]) -> None:
        self.get_logger().info(
            f"{self.phase} 车 ({x:.2f},{y:.2f},{yaw:.2f})",
            throttle_duration_sec=1.0,
        )
        if self.phase == "done":
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().info("己方剩下的块已经推完", throttle_duration_sec=2.0)
            return
        if self.phase == "deliver":
            self._deliver(x, y, yaw)
            return
        if self.phase == "turn":
            self._turn_here(yaw)
            return
        if self.phase == "own":
            self._own(x, y, yaw)
            return
        if self.phase == "retreat":
            self._retreat(x, y, yaw)
            return
        if self.phase == "spin":
            self._spin(x, y, yaw)
            return
        if self.phase == "lineup":
            self._lineup(x, y, yaw)
            return
        if self.phase == "recover":
            self._recover(x, y, yaw)
            return
        if self.phase == "rake":
            self._rake(x, y, yaw)
            return
        self._publish(0.0, 0.0, 0.0)

    def _deliver(self, x: float, y: float, yaw: float) -> None:
        """车头朝 +y，挡板把对准的那一块直推到对方那一排。"""
        goal = self.end_x if self.end_x is not None else x
        lateral = 0.0
        if y < -0.16 and abs(x - goal) > 0.015:
            lateral = max(-0.05, min(0.05, 1.2 * (x - goal)))
        speed = 0.12 if y > -0.16 else PUSH_SPEED
        self._shove(yaw, math.pi / 2, lateral, speed)

    def _turn_here(self, yaw: float) -> None:
        """就地转到略偏 +y 的方向。平移速度立刻清掉，避免边转边往前滑。"""
        err = wrap_angle(RAKE_YAW - yaw)
        self.last_cmd = (0.0, 0.0, self.last_cmd[2])
        if abs(err) < 0.08:
            self.phase = "rake"
            self.lane_y = None
            self.get_logger().info("车头略偏 +y，一次把能碰到的块推向侧墙")
            return
        self._publish(0.0, 0.0, max(-1.3, min(1.3, 2.6 * err)))

    def _retreat(self, x: float, y: float, yaw: float) -> None:
        """退到南侧空地再转向。只在后方轮距是空的时候倒车，不往西墙里挤。"""
        if y < -0.34 and -0.50 <= x <= -0.42:
            self.phase = "spin"
            self._publish(0.0, 0.0, 0.0)
            return
        if y < -0.34 and x < -0.50:
            self._north(yaw, 0.0, -0.16)
            return
        if y < -0.34:
            if self._path_clear(x, y, -1.0, 0.0):
                self._north(yaw, 0.0, 0.28)
            else:
                self._publish(0.0, 0.0, 0.0)
            return
        if self._path_clear(x, y, 0.0, -1.0):
            self._north(yaw, -0.28, 0.0)
            return
        if self._path_clear(x, y, -1.0, 0.0):
            self._north(yaw, 0.0, 0.22)
            return
        self._publish(0.0, 0.0, 0.0)

    def _spin(self, x: float, y: float, yaw: float) -> None:
        if x > -0.40 or y > -0.28:
            self.phase = "retreat"
            return
        if x < -0.50:
            self._north(yaw, 0.0, -0.16)
            return
        err = wrap_angle(0.0 - yaw)
        if abs(err) < 0.12:
            self.lane_y = 0.062
            self.phase = "lineup"
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().info("车头已朝 +x，对齐对方那一排")
            return
        self._publish(0.0, 0.0, max(-1.3, min(1.3, 2.8 * err)))

    def _row_y(self) -> float:
        row = self._row_blocks()
        if not row:
            return ROW_Y
        ys = [block[1] for block in row]
        if max(ys) - min(ys) <= 0.09:
            return (min(ys) + max(ys)) / 2.0
        ordered = sorted(ys)
        return ordered[len(ordered) // 2]

    def _clear_x(self) -> float:
        """挡板前缘要留在最西边那块的西侧，北上时才不会用铲的侧面碰到它。"""
        row = self._row_blocks()
        left = min((block[0] for block in row), default=-0.40)
        return max(-0.545, min(-0.40, left - 0.12))

    def _face_east(self, yaw: float) -> Optional[float]:
        err = wrap_angle(0.0 - yaw)
        if abs(err) > 0.18:
            self._publish(0.0, 0.0, max(-1.2, min(1.2, 2.6 * err)))
            return None
        return err

    def _lineup(self, x: float, y: float, yaw: float) -> None:
        """车头朝 +x。先在南侧空地让到这一排西侧，再北上，挡板正面贴上这一排。"""
        err = self._face_east(yaw)
        if err is None:
            return
        if self.lane_y is None:
            self.lane_y = self._row_y()
        lane = self.lane_y
        clear_x = self._clear_x()
        wz = max(-0.6, min(0.6, 1.8 * err))
        if y > -0.28 and x > clear_x + 0.02:
            if self._path_clear(x, y, 0.0, -1.0, reach=0.14):
                self._publish(0.0, -0.24, wz)
            else:
                self._publish(0.0, 0.0, wz)
            return
        if abs(x - clear_x) > 0.025:
            step = max(-0.22, min(0.22, 2.0 * (clear_x - x)))
            if step < 0 and not self._path_clear(x, y, -1.0, 0.0):
                self._publish(0.0, 0.0, wz)
                return
            self._publish(step, 0.0, wz)
            return
        if abs(y - lane) > 0.02:
            step = max(-0.28, min(0.28, 2.4 * (lane - y)))
            if step > 0 and x > clear_x + 0.03:
                self._publish(0.0, 0.0, wz)
                return
            self._publish(0.0, step, wz)
            return
        self.phase = "rake"
        self.get_logger().info(f"已对齐 y={lane:.2f}，沿 +x 推到侧墙")

    def _recover(self, x: float, y: float, yaw: float) -> None:
        """还有块留在西边。先退到南侧空地，再从西侧用挡板正面推一次。"""
        err = self._face_east(yaw)
        if err is None:
            return
        wz = max(-0.6, min(0.6, 1.8 * err))
        if y > -0.34:
            if self._path_clear(x, y, 0.0, -1.0, reach=0.16):
                self._publish(0.0, max(-0.28, min(-0.12, 2.0 * (-0.40 - y))), wz)
            else:
                self._publish(0.0, 0.0, wz)
            return
        self.phase = "lineup"
        self._publish(0.0, 0.0, 0.0)

    def _row_lane(self, x: float) -> float:
        """前方还在这一排上的块，取它们的中线，横推时就贴着这条道走。"""
        ahead = [
            block[1]
            for block in self.blocks
            if -0.02 < block[1] < 0.16 and block[0] > x - 0.02 and block[0] < 0.55
        ]
        if not ahead:
            return ROW_Y
        ahead.sort()
        return ahead[len(ahead) // 2]

    def _rake(self, x: float, y: float, yaw: float) -> None:
        """铲面略朝 +y，但车心沿世界 +x 走，并保持在这一排的 y 上。"""
        if self.lane_y is None:
            self.lane_y = self._row_lane(x)
            self.get_logger().info(f"对齐这一排 y={self.lane_y:.2f}，再沿 +x 推")
        if x >= 0.45:
            self._begin_own()
            self.get_logger().info("这一铲已经靠到侧墙，接着推己方剩下的块")
            return
        lane_err = self.lane_y - y
        vy = max(-0.12, min(0.12, 1.6 * lane_err))
        speed = 0.28
        if abs(lane_err) > 0.035 and x < 0.05:
            speed = 0.10
        self._drive(speed, vy, yaw, goal=RAKE_YAW, hold=False)

    def _own(self, x: float, y: float, yaw: float) -> None:
        """靠墙后先退开，再转到车头朝 +y，对准己方下一铲，只推到交界。"""
        groups = self._groups()
        center = mean_x(groups[0]) if groups else None
        if self.mode == "exit":
            if x > 0.40:
                self._drive(-0.16, 0.0, yaw, goal=RAKE_YAW, hold=False)
                return
            if y < 0.22:
                self._drive(0.0, 0.18, yaw, goal=math.pi / 2, hold=True)
                return
            self.mode = "west"
        if self.mode == "west":
            if center is None:
                self.phase = "done"
                self._publish(0.0, 0.0, 0.0)
                self.get_logger().info("己方剩下的块已经推完")
                return
            if abs(x - center) > 0.04 or abs(y - 0.22) > 0.05:
                step = max(-0.22, min(0.22, 1.8 * (center - x)))
                hold_y = max(-0.10, min(0.10, 1.4 * (0.22 - y)))
                self._drive(step, hold_y, yaw, goal=math.pi / 2, hold=True)
                return
            self.target_x = center
            self.mode = "drop"
            self.get_logger().info(f"对准己方一铲 x={center:.2f}，退到块后面再推")
        if self.mode == "drop":
            if y > -0.36:
                if self._rear_blocked(x, y):
                    slide = 0.14 if self._path_clear(x, y, -1.0, 0.0, reach=0.10) else -0.14
                    self._north(yaw, 0.0, slide)
                else:
                    self._north(yaw, -0.18, 0.0)
                return
            self.mode = "push"
        if center is None:
            self.phase = "done"
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().info("己方剩下的块已经推完")
            return
        if abs(x - center) > 0.05:
            # 人已经在己方空区时，沿空区挪到下一铲，不再往北穿回去。
            if y < -0.28:
                step = max(-0.20, min(0.20, 1.8 * (center - x)))
                self._drive(step, 0.0, yaw, goal=math.pi / 2, hold=True)
                return
            self.mode = "west"
            self._drive(0.0, 0.16, yaw, goal=math.pi / 2, hold=True)
            return
        if y > -0.06:
            self.skipped.append(center)
            self.target_x = None
            self.mode = "drop"
            self.get_logger().info("这一铲已到交界，退回去推下一铲")
            self._north(yaw, -0.18, 0.0)
            return
        lateral = max(-0.05, min(0.05, 1.4 * (center - x)))
        self._shove(yaw, math.pi / 2, lateral, 0.22)

    def _north(self, yaw: float, forward: float, left: float) -> None:
        """车头朝 +y 时边走边修正。倒车且航向差得多时先不退，免得车尾扫到块。"""
        err = wrap_angle(math.pi / 2 - yaw)
        wz = max(-1.2, min(1.2, 2.2 * err))
        if forward < 0 and abs(err) > 0.5:
            forward = 0.0
        elif abs(err) > 1.0:
            forward = 0.0
            left = 0.0
        elif abs(err) > 0.5:
            forward *= 0.55
            left *= 0.55
        self._publish(forward, left, wz)

    def _shove(self, yaw: float, goal: float, lateral: float, speed: float) -> None:
        """沿车头推，航向连续修正，不把小偏差当成要停车。"""
        err = wrap_angle(goal - yaw)
        wz = max(-0.7, min(0.7, 1.8 * err))
        if abs(err) > 0.6:
            speed *= 0.55
        lateral = max(-0.05, min(0.05, lateral))
        self._publish(speed, lateral, wz)


def main() -> None:
    rclpy.init()
    node = PushBlocks()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
