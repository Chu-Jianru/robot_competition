"""按一次键盘录像走完整场，去掉中间的停顿。

路线来自 records/traj_20260929_151848_segments.txt。
录像里每次松开按键都会停一下，这里把同一方向并成一段，只在该转向时原地转，转完接着走。
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

# 每段的终点：(x, y, 车头朝向)。朝向变了、位置几乎没变的，就是原地转。
PATH: List[Tuple[float, float, float]] = [
    (-0.42, 0.04, math.pi / 2),
    (-0.42, 0.05, 0.00),
    (0.44, 0.06, 0.00),
    (-0.47, 0.06, 0.00),
    (-0.47, 0.06, math.pi / 2),
    (-0.48, -0.09, math.pi / 2),
    (-0.48, -0.09, 0.05),
    (0.40, -0.05, 0.00),
    (-0.12, 0.00, 0.00),
    (-0.12, 0.02, math.pi / 2),
    (-0.13, -0.47, math.pi / 2),
    (0.52, -0.48, math.pi / 2),
    (0.52, 0.44, math.pi / 2),
    (0.26, 0.44, math.pi / 2),
    (0.26, -0.09, math.pi / 2),
    (0.52, -0.13, math.pi / 2),
    (0.52, 0.35, math.pi / 2),
    (0.52, -0.17, math.pi / 2),
    (0.45, -0.18, math.pi / 2),
    (0.44, 0.39, math.pi / 2),
    (0.45, -0.20, math.pi / 2),
    (-0.45, -0.23, math.pi / 2),
    (-0.46, 0.50, math.pi / 2),
]

CRUISE = 0.40
W_MAX = 2.2


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def world_to_body(vx_w: float, vy_w: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * vx_w + s * vy_w, -s * vx_w + c * vy_w


class FollowPath(Node):
    def __init__(self) -> None:
        super().__init__("follow_path")
        self.declare_parameter("autostart", False)
        autostart = self.get_parameter("autostart").value
        if isinstance(autostart, str):
            autostart = autostart.lower() in ("1", "true", "yes")
        self.started = bool(autostart)
        if self.started:
            self.get_logger().info("已自动开始按录像路线走")
        else:
            threading.Thread(target=self._wait_space, daemon=True).start()
            self.get_logger().info("车已停住。在这个终端按空格，按录像路线走")

        self.pose: Optional[Tuple[float, float, float]] = None
        self.pose_time = 0.0
        self.index = 0
        self.done = False
        self.cmd = (0.0, 0.0, 0.0)
        self.target = (0.0, 0.0, 0.0)

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
                    self.get_logger().info("已按空格，开始按录像路线走")
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
        self._advance()
        if self.done:
            self.target = (0.0, 0.0, 0.0)
        else:
            self._steer()
        self._flush()

    def _advance(self) -> None:
        x, y, yaw = self.pose
        while self.index < len(PATH):
            gx, gy, gyaw = PATH[self.index]
            dist = math.hypot(gx - x, gy - y)
            if dist < 0.045 and abs(wrap_angle(gyaw - yaw)) < 0.16:
                self.index += 1
                if self.index >= len(PATH):
                    self.done = True
                    self.get_logger().info("录像路线走完")
                    return
                gx, gy, gyaw = PATH[self.index]
                self.get_logger().info(f"下一段 终点({gx:.2f},{gy:.2f}) 朝向 {gyaw:.2f}")
                continue
            return

    def _steer(self) -> None:
        x, y, yaw = self.pose
        gx, gy, gyaw = PATH[self.index]
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
        """下一段要转向，或者行进方向折了，先减速再拐。"""
        if self.index + 1 >= len(PATH):
            return True
        ax, ay, _ayaw = PATH[self.index]
        bx, by, _byaw = PATH[self.index + 1]
        if math.hypot(bx - ax, by - ay) < 0.05:
            return True
        if self.index == 0:
            return False
        px, py, _pyaw = PATH[self.index - 1]
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
