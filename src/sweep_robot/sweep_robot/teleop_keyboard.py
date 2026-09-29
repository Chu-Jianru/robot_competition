"""键盘遥控。先读懂这个节点，再改速度。

发布: /cmd_vel  geometry_msgs/Twist
  linear.x  前进/后退
  linear.y  左移/右移   （麦克轮才有，差速车没有）
  angular.z 原地转

实物阶段这个节点不用改，STM32 或逆解节点接着听 /cmd_vel。
"""

import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# 遥控期间把键盘连发间隔缩短后，松开超过这个时间就停车。
HOLD_S = 0.08

HELP = """
控制（终端要保持焦点，按住才动，松开就停）:
  w/s  前进/后退
  a/d  左平移/右平移
  q/e  左转/右转
  空格  立刻停车
  Ctrl+C 退出
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        # 作业：把这三个数改成你觉得合适的比赛速度，观察 Gazebo 里车的表现
        self.vx = 0.45
        self.vy = 0.45
        self.wz = 3.0
        self.cmd = Twist()
        self.key_time = 0.0
        self.hold_s = HOLD_S
        self.create_timer(0.05, self._publish)
        self.get_logger().info(HELP)

    def set_key(self, key: str) -> None:
        cmd = Twist()
        if key == "w":
            cmd.linear.x = self.vx
        elif key == "s":
            cmd.linear.x = -self.vx
        elif key == "a":
            cmd.linear.y = self.vy
        elif key == "d":
            cmd.linear.y = -self.vy
        elif key == "q":
            cmd.angular.z = self.wz
        elif key == "e":
            cmd.angular.z = -self.wz
        elif key == " ":
            self.cmd = Twist()
            self.key_time = 0.0
            return
        else:
            return
        self.cmd = cmd
        self.key_time = time.monotonic()

    def _publish(self) -> None:
        if self.key_time == 0.0 or time.monotonic() - self.key_time > self.hold_s:
            self.cmd = Twist()
        self.pub.publish(self.cmd)


def _stdin_loop(node: TeleopKeyboard) -> None:
    try:
        import tty
        import termios

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while rclpy.ok():
                ch = sys.stdin.read(1)
                node.set_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"键盘读取失败，改用 ros2 topic pub。原因: {exc}")


def _read_repeat():
    """当前键盘连发：第一次重复前的等待（毫秒）和每秒重复次数。"""
    try:
        out = subprocess.check_output(["xset", "q"], text=True, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        return None
    for line in out.splitlines():
        if "auto repeat delay:" not in line or "repeat rate:" not in line:
            continue
        nums = [int(part) for part in line.split() if part.isdigit()]
        if len(nums) >= 2:
            return nums[0], nums[1]
    return None


def _write_repeat(delay: int, rate: int) -> None:
    subprocess.call(
        ["xset", "r", "rate", str(delay), str(rate)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def main() -> None:
    rclpy.init()
    saved_repeat = _read_repeat()
    if saved_repeat is not None:
        # 系统默认要等 0.5 秒才连发，松开后会多走一截。遥控时改成马上连发。
        _write_repeat(30, 50)
    node = TeleopKeyboard()
    if saved_repeat is None:
        node.hold_s = 0.55
        node.get_logger().warn("读不到键盘连发设置，松开后仍会多走一会儿")
    thread = threading.Thread(target=_stdin_loop, args=(node,), daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd = Twist()
    node.key_time = 0.0
    node.pub.publish(node.cmd)
    node.destroy_node()
    if saved_repeat is not None:
        _write_repeat(*saved_repeat)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
