"""麦克纳姆轮逆解。这是仿真和 STM32 共用的核心公式。

订阅: /cmd_vel
发布: /wheel_speeds  (std_msgs/Float32MultiArray, 顺序 fl, fr, rl, rr)
      单位 rad/s

作业：补全 twist_to_wheels()。先在终端打印四个轮速，对上了再接到 STM32。

轮子布局（俯视，x 向前，y 向左）:

    fl ---- fr
    |  车体  |
    rl ---- rr

标准 X 型麦克轮（辊子 45°）:
  v_fl = vx - vy - (lx + ly) * wz
  v_fr = vx + vy + (lx + ly) * wz
  v_rl = vx + vy - (lx + ly) * wz
  v_rr = vx - vy + (lx + ly) * wz
  轮角速度 = 线速度 / 轮半径
"""

from typing import Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def twist_to_wheels(
    vx: float,
    vy: float,
    wz: float,
    lx: float,
    ly: float,
    radius: float,
) -> Tuple[float, float, float, float]:
    """把底盘速度变成四个轮的角速度 (fl, fr, rl, rr)。

    lx: 轮距在 x 方向的一半（前后轴距/2）
    ly: 轮距在 y 方向的一半（左右轮距/2）
    radius: 轮半径，米
    """
    # TODO: 删掉下面三行，按文件头公式自己写
    raise NotImplementedError(
        "请实现麦克轮逆解。提示: k = lx + ly，然后四个轮的 vx±vy±k*wz，再除以 radius"
    )


class MecanumIk(Node):
    def __init__(self):
        super().__init__("mecanum_ik")
        # 与当前仿真模型大致对应，实物时改成实测值
        self.declare_parameter("lx", 0.03)
        self.declare_parameter("ly", 0.047)
        self.declare_parameter("wheel_radius", 0.018)
        self.lx = float(self.get_parameter("lx").value)
        self.ly = float(self.get_parameter("ly").value)
        self.radius = float(self.get_parameter("wheel_radius").value)
        self.pub = self.create_publisher(Float32MultiArray, "wheel_speeds", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.get_logger().info(
            f"麦克轮逆解已启动 lx={self.lx:.3f} ly={self.ly:.3f} r={self.radius:.3f}"
        )

    def _on_cmd(self, msg: Twist) -> None:
        try:
            wheels = twist_to_wheels(
                msg.linear.x,
                msg.linear.y,
                msg.angular.z,
                self.lx,
                self.ly,
                self.radius,
            )
        except NotImplementedError as exc:
            self.get_logger().warn(str(exc), once=True)
            return
        out = Float32MultiArray()
        out.data = [float(v) for v in wheels]
        self.pub.publish(out)
        self.get_logger().info(
            f"fl={wheels[0]:.2f} fr={wheels[1]:.2f} rl={wheels[2]:.2f} rr={wheels[3]:.2f}",
            throttle_duration_sec=0.5,
        )


def main() -> None:
    rclpy.init()
    node = MecanumIk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
