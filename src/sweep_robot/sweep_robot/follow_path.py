"""按比赛意图走，不再逐段复读键盘录像。

开局沿固定路线：先把对方那一排推到东墙，倒回出发的那一列，再把己方那一排推到东墙。
然后从己方半区把墙边这一堆推进对方角落。第一铲按路线走，第二铲用俯视看到的半堆位置对准。
最后把己方半区还剩的块一个一个推到对方半区靠北墙的位置。块已经没了的情况以后再处理。
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

CRUISE = 0.40
W_MAX = 2.2
EAST_X = 0.50
NORTH_Y = 0.50


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def world_to_body(vx_w: float, vy_w: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * vx_w + s * vy_w, -s * vx_w + c * vy_w


def _opening() -> List[Goal]:
    """对方那一排、倒回、己方那一排、再绕到东墙南侧把整堆往北推。"""
    north = math.pi / 2
    return [
        (-0.42, 0.04, north),
        (-0.42, 0.05, 0.0),
        (EAST_X, 0.05, 0.0),
        (-0.42, 0.05, 0.0),
        (-0.42, 0.05, north),
        (-0.42, -0.05, north),
        (-0.42, -0.05, 0.0),
        (EAST_X, -0.05, 0.0),
        (-0.15, -0.05, 0.0),
        (-0.15, -0.05, north),
        (-0.15, -0.46, north),
        (0.48, -0.46, north),
        (0.48, NORTH_Y, north),
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
        self.pose_time = 0.0
        self.blocks: List[Point] = []
        self.goals: List[Goal] = _opening()
        self.index = 0
        self.phase = "route"
        self.done = False
        self.lane_set = False
        self.current_x: Optional[float] = None
        self.cleared: List[float] = []
        self.cmd = (0.0, 0.0, 0.0)
        self.target = (0.0, 0.0, 0.0)

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
        if self.pose is None:
            self.pose = fresh
        else:
            yaw = self.pose[2] + 0.65 * wrap_angle(fresh[2] - self.pose[2])
            self.pose = (
                0.7 * fresh[0] + 0.3 * self.pose[0],
                0.7 * fresh[1] + 0.3 * self.pose[1],
                yaw,
            )
        self.pose_time = self._now()

    def _on_blocks(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        self.blocks = [(data[i], data[i + 1]) for i in range(0, len(data) - 1, 2)]

    def _tick(self) -> None:
        if not self.started:
            self._emit(0.0, 0.0, 0.0)
            return
        now = self._now()
        if self.pose is None or now - self.pose_time > 0.8:
            self.target = (0.0, 0.0, 0.0)
            self._flush()
            return
        if self.done:
            self.target = (0.0, 0.0, 0.0)
            self._flush()
            return
        self._nudge()
        self._advance()
        if self.done or self.index >= len(self.goals):
            self.target = (0.0, 0.0, 0.0)
        else:
            self._steer()
        self._flush()

    def _nudge(self) -> None:
        """第二铲和单个己方块的 x 跟着相机慢慢偏，不一下子跳过去。"""
        if self.phase not in ("pile2", "own") or self.current_x is None:
            return
        if self.pose[1] > 0.12:
            return
        seen = [
            block[0]
            for block in self.blocks
            if abs(block[0] - self.current_x) < 0.10 and block[1] < 0.35
        ]
        if not seen:
            return
        aim = sum(seen) / len(seen)
        step = max(-0.012, min(0.012, aim - self.current_x))
        self.current_x += step
        self.goals = [
            (self.current_x, gy, gyaw) if abs(gx - (self.current_x - step)) < 0.08 else (gx, gy, gyaw)
            for gx, gy, gyaw in self.goals
        ]

    def _advance(self) -> None:
        x, y, yaw = self.pose
        while self.index < len(self.goals):
            gx, gy, gyaw = self.goals[self.index]
            dist = math.hypot(gx - x, gy - y)
            if dist < 0.045 and abs(wrap_angle(gyaw - yaw)) < 0.16:
                if self.phase == "route" and self.index == 4 and not self.lane_set:
                    self._apply_own_lane()
                self.index += 1
                if self.index >= len(self.goals):
                    self._on_phase_done()
                    return
                gx, gy, gyaw = self.goals[self.index]
                self.get_logger().info(f"下一段 终点({gx:.2f},{gy:.2f}) 朝向 {gyaw:.2f}")
                continue
            return

    def _on_phase_done(self) -> None:
        if self.phase == "route":
            self._begin_pile2()
        elif self.phase == "pile2":
            self._begin_own()
        elif self.phase == "own":
            if self.current_x is not None:
                self.cleared.append(self.current_x)
            self._begin_own()

    def _apply_own_lane(self) -> None:
        self.lane_set = True
        ys = [block[1] for block in self.blocks if -0.20 < block[1] < 0.02 and block[0] < 0.35]
        if not ys:
            self.get_logger().info("没看清己方那一排，仍沿 y=-0.05 推")
            return
        ys.sort()
        lane = min(-0.02, max(-0.12, ys[len(ys) // 2]))
        for index in (5, 6, 7, 8, 9):
            gx, _gy, gyaw = self.goals[index]
            self.goals[index] = (gx, lane, gyaw)
        self.get_logger().info(f"己方这一排大约在 y={lane:.2f}，沿这一排推到东墙")

    def _begin_pile2(self) -> None:
        px, py = self._half_pile()
        north = math.pi / 2
        self.phase = "pile2"
        self.current_x = px
        self.goals = [(px, py, north), (px, NORTH_Y, north)]
        self.index = 0
        self.get_logger().info(f"第一铲可能侧漏，第二铲对准半堆 x={px:.2f}")

    def _half_pile(self) -> Point:
        leaked = [block for block in self.blocks if block[0] > 0.12 and block[1] < 0.32]
        if len(leaked) < 2:
            return 0.40, -0.18
        xs = sorted(block[0] for block in leaked)
        ys = sorted(block[1] for block in leaked)
        px = min(0.52, max(0.25, xs[len(xs) // 2]))
        py = min(0.0, max(-0.40, ys[0] - 0.14))
        return px, py

    def _own_targets(self) -> List[Point]:
        pending = []
        for block in self.blocks:
            if block[1] >= -0.02:
                continue
            if any(abs(block[0] - done) < 0.07 for done in self.cleared):
                continue
            pending.append(block)
        pending.sort(key=lambda block: block[0])
        return pending

    def _begin_own(self) -> None:
        pending = self._own_targets()
        if not pending:
            self.done = True
            self.phase = "done"
            self.current_x = None
            self.get_logger().info("己方半区剩下的块已经逐个推完")
            return
        bx, by = pending[0]
        approach_y = min(by - 0.12, -0.22)
        approach_y = max(-0.46, approach_y)
        north = math.pi / 2
        self.phase = "own"
        self.current_x = bx
        self.goals = [
            (bx, approach_y, north),
            (bx, NORTH_Y, north),
            (bx, -0.28, north),
        ]
        self.index = 0
        self.get_logger().info(f"推己方剩余的一块 x={bx:.2f}，送到对方半区靠墙")

    def _steer(self) -> None:
        x, y, yaw = self.pose
        gx, gy, gyaw = self.goals[self.index]
        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)
        yaw_err = wrap_angle(gyaw - yaw)
        speed = CRUISE
        if dist < 0.16 and self._sharp_corner():
            speed = max(0.18, CRUISE * dist / 0.16)
        if abs(yaw_err) > 0.9:
            speed = 0.0
        elif abs(yaw_err) > 0.45:
            speed *= 0.4
        if dist < 1e-4:
            vx_w = vy_w = 0.0
        else:
            vx_w = speed * dx / dist
            vy_w = speed * dy / dist
        vx, vy = world_to_body(vx_w, vy_w, yaw)
        wz = max(-W_MAX, min(W_MAX, 2.4 * yaw_err))
        self.target = (vx, vy, wz)

    def _sharp_corner(self) -> bool:
        if self.index + 1 >= len(self.goals):
            return True
        ax, ay, _ayaw = self.goals[self.index]
        bx, by, _byaw = self.goals[self.index + 1]
        if math.hypot(bx - ax, by - ay) < 0.05:
            return True
        if self.index == 0:
            px, py = self.pose[0], self.pose[1]
        else:
            px, py, _pyaw = self.goals[self.index - 1]
        in_x, in_y = ax - px, ay - py
        out_x, out_y = bx - ax, by - ay
        in_n = math.hypot(in_x, in_y)
        out_n = math.hypot(out_x, out_y)
        if in_n < 1e-3 or out_n < 1e-3:
            return True
        dot = (in_x * out_x + in_y * out_y) / (in_n * out_n)
        return dot < 0.5

    def _flush(self) -> None:
        tx, ty, tw = self.target
        x, y, w = self.cmd
        x = _approach(x, tx, 1.6)
        y = _approach(y, ty, 1.6)
        w = _approach(w, tw, 8.0)
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
