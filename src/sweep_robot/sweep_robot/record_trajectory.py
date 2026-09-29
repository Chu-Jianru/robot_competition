"""键盘遥控时记下发出的速度和车的里程。

订阅:
  /cmd_vel   geometry_msgs/Twist，键盘正在发的速度
  /odom      nav_msgs/Odometry，Gazebo 里的位置

运行中每隔 0.1 秒写一行样本。按 Ctrl+C 结束时，再按速度方向切成一段一段，
方便后面对照着改自主运动。这一步只记录，不改推块策略。
"""

import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

SAMPLE_DT = 0.1


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def action_name(vx: float, vy: float, wz: float) -> str:
    parts = []
    if vx > 0.02:
        parts.append("前进")
    elif vx < -0.02:
        parts.append("后退")
    if vy > 0.02:
        parts.append("左移")
    elif vy < -0.02:
        parts.append("右移")
    if wz > 0.05:
        parts.append("左转")
    elif wz < -0.05:
        parts.append("右转")
    if not parts:
        return "停车"
    return "+".join(parts)


class RecordTrajectory(Node):
    def __init__(self) -> None:
        super().__init__("record_trajectory")
        self.declare_parameter("output_dir", "/home/cjr/robot_competition/records")
        out_dir = str(self.get_parameter("output_dir").value)
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(out_dir, f"traj_{stamp}.csv")
        self.segment_path = os.path.join(out_dir, f"traj_{stamp}_segments.txt")

        self.cmd = (0.0, 0.0, 0.0)
        self.pose = None
        self.path_m = 0.0
        self.samples = []
        self.t0 = None
        self.last_sample = 0.0

        self.csv = open(self.csv_path, "w", encoding="utf-8")
        self.csv.write("t,vx,vy,wz,x,y,yaw,path_m\n")
        self.csv.flush()

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_timer(SAMPLE_DT, self._sample)
        self.get_logger().info(f"开始记录。样本写到 {self.csv_path}")
        self.get_logger().info("键盘开着走完后，在这个终端按 Ctrl+C 结束并写出分段")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        pose = (float(p.x), float(p.y), yaw)
        if self.pose is not None:
            self.path_m += math.hypot(pose[0] - self.pose[0], pose[1] - self.pose[1])
        self.pose = pose

    def _sample(self) -> None:
        now = self._now()
        if self.t0 is None:
            self.t0 = now
        if self.pose is None:
            return
        t = now - self.t0
        if self.samples and t - self.last_sample < SAMPLE_DT * 0.5:
            return
        self.last_sample = t
        vx, vy, wz = self.cmd
        x, y, yaw = self.pose
        row = (t, vx, vy, wz, x, y, yaw, self.path_m)
        self.samples.append(row)
        self.csv.write(
            f"{t:.3f},{vx:.4f},{vy:.4f},{wz:.4f},{x:.4f},{y:.4f},{yaw:.4f},{self.path_m:.4f}\n"
        )
        self.csv.flush()

    def finish(self) -> None:
        if not self.csv.closed:
            self.csv.close()
        self._write_segments()
        self.get_logger().info(f"分段写到 {self.segment_path}")

    def _write_segments(self) -> None:
        lines = []
        if not self.samples:
            lines.append("没有样本。确认 Gazebo 已打开，并且 /odom 有数据。")
        else:
            groups = []
            current = [self.samples[0]]
            for sample in self.samples[1:]:
                if _same_cmd(current[-1], sample):
                    current.append(sample)
                else:
                    groups.append(current)
                    current = [sample]
            groups.append(current)
            total_t = self.samples[-1][0]
            lines.append(f"总用时 {total_t:.2f} s，总里程 {self.path_m:.3f} m")
            lines.append("下面每一段是一次按住不放的动作。")
            lines.append("")
            for group in groups:
                start = group[0]
                end = group[-1]
                name = action_name(start[1], start[2], start[3])
                dist = end[7] - start[7]
                lines.append(
                    f"{start[0]:6.2f}-{end[0]:6.2f}s  {name:8s}"
                    f"  vx={start[1]:6.2f} vy={start[2]:6.2f} wz={start[3]:6.2f}"
                    f"  起点({start[4]:7.3f},{start[5]:7.3f},{start[6]:6.2f})"
                    f"  终点({end[4]:7.3f},{end[5]:7.3f},{end[6]:6.2f})"
                    f"  里程 {dist:.3f} m"
                )
        text = "\n".join(lines) + "\n"
        with open(self.segment_path, "w", encoding="utf-8") as handle:
            handle.write(text)
        self.get_logger().info("\n" + text.rstrip())


def _same_cmd(left, right) -> bool:
    return (
        abs(left[1] - right[1]) < 0.02
        and abs(left[2] - right[2]) < 0.02
        and abs(left[3] - right[3]) < 0.05
    )


def main() -> None:
    rclpy.init()
    node = RecordTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.finish()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
