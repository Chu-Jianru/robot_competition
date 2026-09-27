"""键盘遥控。先读懂这个节点，再改速度。

发布: /cmd_vel  geometry_msgs/Twist
  linear.x  前进/后退
  linear.y  左移/右移   （麦克轮才有，差速车没有）
  angular.z 原地转

实物阶段这个节点不用改，STM32 或逆解节点接着听 /cmd_vel。
"""

import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = """
控制（终端要保持焦点）:
  w/s  前进/后退
  a/d  左平移/右平移
  q/e  左转/右转
  空格  停车
  Ctrl+C 退出
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        # 作业：把这三个数改成你觉得合适的比赛速度，观察 Gazebo 里车的表现
        self.vx = 0.15
        self.vy = 0.15
        self.wz = 0.4
        self.cmd = Twist()
        self.create_timer(0.05, self._publish)
        self.get_logger().info(HELP)

    def set_key(self, key: str) -> None:
        if key == "w":
            self.cmd.linear.x = self.vx
        elif key == "s":
            self.cmd.linear.x = -self.vx
        elif key == "a":
            self.cmd.linear.y = self.vy
        elif key == "d":
            self.cmd.linear.y = -self.vy
        elif key == "q":
            self.cmd.angular.z = self.wz
        elif key == "e":
            self.cmd.angular.z = -self.wz
        elif key == " ":
            self.cmd = Twist()

    def _publish(self) -> None:
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


def main() -> None:
    rclpy.init()
    node = TeleopKeyboard()
    thread = threading.Thread(target=_stdin_loop, args=(node,), daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd = Twist()
    node.pub.publish(node.cmd)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
