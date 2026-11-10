"""ROS 适配层。比赛决策和运动安全在 competition_policy 内，可独立运行。"""
import json
import os
import termios
import threading
import time
import tty
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from sweep_robot.row_policy import HybridCompetitionPolicy


class AdaptiveController(Node):
    def __init__(self):
        super().__init__('follow_path')
        self.declare_parameter('autostart', False)
        self.declare_parameter('duration', 90.0)
        self.declare_parameter('sweep_rows',True)
        value = self.get_parameter('autostart').value
        self.started = value is True or str(value).lower() == 'true'
        self.policy = HybridCompetitionPolicy(float(self.get_parameter('duration').value))
        row_value=self.get_parameter('sweep_rows').value
        if row_value is False or str(row_value).lower()=='false':self.policy.opening_checked=True
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status = self.create_publisher(String, '/strategy/status', 1)
        self.create_subscription(String, '/perception/scene', self.scene, 1)
        self.create_timer(0.05, self.tick)
        self.last_event = None
        self.received_wall = time.monotonic()
        self.wall_start = None
        self.stopping = False
        self.lock = threading.RLock()
        # 仿真时钟停住/图像流中断时也能发零速；不用墙钟推算位姿。
        self.watchdog_thread = threading.Thread(target=self.watchdog, daemon=True)
        self.watchdog_thread.start()
        if not self.started: threading.Thread(target=self.wait_space, daemon=True).start()
        self.get_logger().info('整排连续推送优先，受阻切换局部清理，清空后持续防回推；90秒截止')

    def wait_space(self):
        try:
            fd = os.open('/dev/tty', os.O_RDONLY)
            old = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                while rclpy.ok() and not self.started:
                    if os.read(fd, 1) == b' ': self.started = True
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old); os.close(fd)
        except OSError as exc: self.get_logger().error(f'无法读取空格启动: {exc}')

    def scene(self, message):
        with self.lock:
            try: self.policy.update(json.loads(message.data))
            except (ValueError, TypeError, KeyError, OverflowError): self.policy.valid = False
            self.received_wall = time.monotonic()

    def emit(self, command):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = map(float, command)
        self.pub.publish(message)

    def tick(self):
        with self.lock:
            now = self.get_clock().now().nanoseconds * 1e-9
            if not self.started:
                self.emit((0, 0, 0)); return
            if self.wall_start is None: self.wall_start = time.monotonic()
            command = self.policy.step(now)
            self.emit(command)
            state = String()
            state.data = json.dumps(dict(t=now, phase=self.policy.phase, reason=self.policy.reason,
                                         lane=self.policy.lane, own=len(self.policy.pending()), command=command,
                                         scene_age=now-self.policy.scene_time if math.isfinite(self.policy.scene_time) else None), ensure_ascii=False)
            self.status.publish(state)
            if self.policy.events and self.policy.events[-1] != self.last_event:
                self.last_event = self.policy.events[-1]
                self.get_logger().info(f'{self.policy.phase}: {self.policy.reason}')

    def watchdog(self):
        while not self.stopping:
            time.sleep(0.1)
            with self.lock:
                if self.stopping or not rclpy.ok(): break
                if time.monotonic() - self.received_wall > 0.5:
                    self.policy.valid = False
                    self.policy.smoother.reset()
                    self.emit((0, 0, 0))


def main():
    rclpy.init()
    node = AdaptiveController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stopping = True
        node.watchdog_thread.join(timeout=0.5)
        if rclpy.ok(): node.emit((0, 0, 0))
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
