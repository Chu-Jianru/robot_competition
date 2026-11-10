"""订阅俯视相机，发布红块位置、车体位姿和己方剩余块数。

话题:
  订阅 /overhead/camera/image_raw
  发布 /perception/blocks      Float32MultiArray，[x, y, 半宽x, 半宽y, ...]，米
  发布 /perception/robot_pose  Pose2D，场地坐标，theta 从 +x 逆时针
  发布 /perception/own_count   Int32，落在蓝色地面上的红块数量；看不到场地时为 -1
  发布 /perception/obstacles  Float32MultiArray，比物块大、又不是红块和己方色标的物体 [x, y, ...]

有显示器时另开「俯视相机」窗口，只用于观看。弹窗在独立定时器里刷新，不挡检测。
"""

import math
import os
import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32, String

from sweep_robot.field_vision import FieldLock, detect_field


class HsvDetector(Node):
    def __init__(self) -> None:
        super().__init__("hsv_detector")
        self.declare_parameter("image_topic", "/overhead/camera/image_raw")
        topic = str(self.get_parameter("image_topic").value)
        self.bridge = CvBridge()
        cv2.setNumThreads(1)
        # Gazebo 图像发布器支持可靠传输；大图分片丢失时 best effort
        # 会整帧丢弃。其他相机可保留传感器常用的 best effort。
        self.declare_parameter('reliable_images',False)
        reliability=ReliabilityPolicy.RELIABLE if self.get_parameter('reliable_images').value else ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, topic, self._on_image,
                                 QoSProfile(depth=1,reliability=reliability))
        self.blocks_pub = self.create_publisher(Float32MultiArray, "/perception/blocks", 10)
        self.pose_pub = self.create_publisher(Pose2D, "/perception/robot_pose", 10)
        self.count_pub = self.create_publisher(Int32, "/perception/own_count", 10)
        self.obstacle_pub = self.create_publisher(Float32MultiArray, "/perception/obstacles", 10)
        self.scene_pub = self.create_publisher(String, "/perception/scene", 1)
        self.declare_parameter("show_window", True)
        self.show_window = bool(self.get_parameter("show_window").value) and bool(os.environ.get("DISPLAY"))
        self._window_ready = False
        self._preview = None
        self._preview_time = 0.0
        self._held_pose = None
        self._held_time = 0.0
        self.field_lock = FieldLock()
        self.declare_parameter('camera_height', 0.0)
        self.camera_height = float(self.get_parameter('camera_height').value)
        if self.show_window:
            self.create_timer(0.2, self._show)
        self.get_logger().info(f"俯视检测已启动，图像来自 {topic}")

    def _on_image(self, msg: Image) -> None:
        processing_start = time.perf_counter()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        result = detect_field(frame, self.field_lock)
        # 仅用于明确标定为正俯视的相机；0 表示未知高度，保持普通平面单应性。
        # Gazebo 相机在(0,0,2.55)，车顶标记和方块顶面不在地面平面上。
        if self.camera_height > 0.2:
            marker_scale = 1.0 - 0.055 / self.camera_height
            block_scale = 1.0 - 0.020 / self.camera_height
            if result['robot'] is not None:
                x,y,yaw = result['robot']
                result['robot'] = (x*marker_scale,y*marker_scale,yaw)
            result['blocks'] = [(x*block_scale,y*block_scale) for x,y in result['blocks']]
            result['extents'] = [(hx*block_scale,hy*block_scale) for hx,hy in result.get('extents',[])]

        blocks = Float32MultiArray()
        flat = []
        extents = result.get("extents") or []
        for index, (x, y) in enumerate(result["blocks"]):
            hx, hy = extents[index] if index < len(extents) else (0.01, 0.01)
            flat.extend((float(x), float(y), float(hx), float(hy)))
        blocks.data = flat
        self.blocks_pub.publish(blocks)

        count = Int32()
        count.data = int(result["own_count"])
        self.count_pub.publish(count)

        obstacles = Float32MultiArray()
        obstacle_flat = []
        for x, y in result.get("obstacles") or []:
            obstacle_flat.extend((float(x), float(y)))
        obstacles.data = obstacle_flat
        self.obstacle_pub.publish(obstacles)

        now = self.get_clock().now().nanoseconds * 1e-9
        robot = self._gate_pose(result["robot"], now)
        # 一个有时间戳的原子场景，避免控制器把不同帧的块、位姿和有效性混用。
        scene = String()
        scene.data = json.dumps({
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "publish_stamp": now,
            "processing_ms": (time.perf_counter()-processing_start)*1000,
            "valid": result["homography"] is not None,
            "robot": robot if result["robot"] is not None else None,
            "blocks": [list(xy) + list(extents[i]) + [bool(result["own_flags"][i])]
                       for i, xy in enumerate(result["blocks"])],
            "obstacles": [list(xy) + list(result["obstacle_extents"][i])
                          for i, xy in enumerate(result["obstacles"])],
        }, separators=(",", ":"))
        self.scene_pub.publish(scene)
        if robot is not None:
            pose = Pose2D()
            pose.x = float(robot[0])
            pose.y = float(robot[1])
            pose.theta = float(robot[2])
            self.pose_pub.publish(pose)

        if self.show_window:
            if self._preview is None or now - self._preview_time >= 0.2:
                self._preview = self._make_preview(frame, result)
                self._preview_time = now

        opponent = len(result["blocks"]) - max(result["own_count"], 0)
        self.get_logger().info(
            f"红块 {len(result['blocks'])} 个，己方 {result['own_count']} 个，对方 {opponent} 个，"
            f"车 {'无' if robot is None else f'({robot[0]:.2f},{robot[1]:.2f},{robot[2]:.2f})'}",
            throttle_duration_sec=1.0,
        )

    def _gate_pose(self, robot, now: float):
        """按车的物理速度限制视觉跳变；不再憋三帧后突然接受错误位置。"""
        if robot is None:
            if self._held_pose is not None and now - self._held_time <= 0.3:
                return self._held_pose
            return None
        if self._held_pose is not None:
            dt = min(0.25, max(0.04, now - self._held_time))
            dx = math.hypot(robot[0] - self._held_pose[0], robot[1] - self._held_pose[1])
            max_move = 0.60 * dt + 0.012
            if dx > max_move:
                ratio = max_move / dx
                rx = self._held_pose[0] + ratio * (robot[0] - self._held_pose[0])
                ry = self._held_pose[1] + ratio * (robot[1] - self._held_pose[1])
            else:
                rx, ry = robot[0], robot[1]
            dyaw = math.atan2(
                math.sin(robot[2] - self._held_pose[2]),
                math.cos(robot[2] - self._held_pose[2]),
            )
            max_turn = 3.0 * dt + 0.12
            dyaw = max(-max_turn, min(max_turn, dyaw))
            robot = (
                rx,
                ry,
                math.atan2(
                    math.sin(self._held_pose[2] + dyaw),
                    math.cos(self._held_pose[2] + dyaw),
                ),
            )
        self._held_pose = robot
        self._held_time = now
        return robot

    def _make_preview(self, frame, result):
        debug = frame
        corners = result["field_px"]
        if corners is not None:
            debug = frame.copy()
            center = corners.mean(axis=0)
            order = np.argsort(np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0]))
            pts = corners[order].reshape(-1, 1, 2).astype("int32")
            cv2.polylines(debug, [pts], True, (255, 255, 255), 1)
            flags = result.get("own_flags") or []
            for index, (px, py) in enumerate(result["block_px"]):
                own = index < len(flags) and flags[index]
                color = (0, 255, 0) if own else (0, 0, 255)
                cv2.circle(debug, (int(px), int(py)), 4, color, 1)
            robot_px = result["robot_px"]
            if robot_px is not None:
                mx, my = robot_px[0]
                cv2.circle(debug, (int(mx), int(my)), 5, (255, 0, 255), 1)
                if robot_px[1] is not None:
                    hx, hy = robot_px[1]
                    cv2.line(debug, (int(mx), int(my)), (int(hx), int(hy)), (255, 255, 0), 1)
        return cv2.resize(debug, (320, 240), interpolation=cv2.INTER_NEAREST)

    def _show(self) -> None:
        if self._preview is None:
            return
        try:
            if not self._window_ready:
                cv2.namedWindow("俯视相机", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("俯视相机", 320, 240)
                self._window_ready = True
            cv2.imshow("俯视相机", self._preview)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.show_window = False
            self.get_logger().warn(f"打不开俯视窗口: {exc}")


def main() -> None:
    rclpy.init()
    node = HsvDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
