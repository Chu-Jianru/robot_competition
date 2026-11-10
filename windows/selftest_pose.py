"""端到端合成自检：不连相机，验证 field_view + pose_tag 的精度。

做法：先在"场地毫米坐标系"画一张 2400x2400 的世界图（蓝黄场地 + 20 个红块 + 5 cm tag），
用带轻微透视的 1920x1080 相机模型投影成仿真俯视图，再加模糊/噪声/JPEG 失真，
最后走真实链路（四角标定 -> 透视校正 -> tag 检测）与真值比较。

用法：
    python windows/selftest_pose.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from field_view import FieldCalibration          # noqa: E402
from pose_tag import detect as detect_pose      # noqa: E402
from tag_detector import TAG_ID                 # noqa: E402

FIELD_MM = 1200.0
WORLD_PX_PER_MM = 2.0
CAMERA = (1920, 1080)
TAG_MM = 50.0
TRIALS = 15

# 相机看到的场地四角（带透视），顺序 左上/右上/右下/左下
CAMERA_QUAD = np.array([[430, 150], [1510, 118], [1556, 942], [392, 972]], np.float32)


def world_px(x_mm: float, y_mm: float) -> tuple[float, float]:
    """场地毫米（y 向北）-> 世界图像像素（图像上方为北）。"""
    return x_mm * WORLD_PX_PER_MM, (FIELD_MM - y_mm) * WORLD_PX_PER_MM


def make_world(rng: np.random.Generator, tag_xy: tuple[float, float], heading_deg: float) -> np.ndarray:
    side = int(FIELD_MM * WORLD_PX_PER_MM)
    world = np.full((side, side, 3), 120, np.uint8)
    half = side // 2
    cv2.rectangle(world, (0, half), (side, side), (150, 95, 60), -1)   # 南半（蓝）
    cv2.rectangle(world, (0, 0), (side, half), (70, 165, 195), -1)     # 北半（黄）

    for _ in range(20):
        bx = float(rng.uniform(40, FIELD_MM - 40))
        by = float(rng.uniform(40, FIELD_MM - 40))
        px, py = world_px(bx, by)
        r = int(20 * WORLD_PX_PER_MM / 2)                              # 20 mm 方块
        cv2.rectangle(world, (int(px) - r, int(py) - r), (int(px) + r, int(py) + r), (40, 45, 195), -1)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_px = int(TAG_MM * WORLD_PX_PER_MM)
    marker = cv2.aruco.generateImageMarker(dictionary, TAG_ID, marker_px)
    marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    h = math.radians(heading_deg)
    up = np.array([math.cos(h), -math.sin(h)], np.float32)             # 世界像素里的"上"方向
    right = np.array([-up[1], up[0]], np.float32)
    cx, cy = world_px(*tag_xy)
    center = np.array([cx, cy], np.float32)
    s = marker_px / 2.0
    quad = np.array([
        center + up * s - right * s,
        center + up * s + right * s,
        center - up * s + right * s,
        center - up * s - right * s,
    ], np.float32)
    src = np.array([[0, 0], [marker_px, 0], [marker_px, marker_px], [0, marker_px]], np.float32)
    warp = cv2.getPerspectiveTransform(src, quad)
    u8 = marker.astype(np.uint8)
    warped = cv2.warpPerspective(u8, warp, (side, side))
    mask = cv2.warpPerspective(np.full((marker_px, marker_px), 255, np.uint8), warp, (side, side))
    world[mask > 0] = warped[mask > 0]
    return world


def shoot(world: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    side = world.shape[0]
    src = np.array([[0, 0], [side, 0], [side, side], [0, side]], np.float32)
    h = cv2.getPerspectiveTransform(src, CAMERA_QUAD)
    frame = cv2.warpPerspective(world, h, CAMERA, flags=cv2.INTER_AREA)
    frame = cv2.GaussianBlur(frame, (0, 0), 1.0)
    frame = np.clip(frame.astype(np.float32) + rng.normal(0, 3.0, frame.shape), 0, 255).astype(np.uint8)
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def main() -> None:
    rng = np.random.default_rng(20261102)
    calib = FieldCalibration(CAMERA_QUAD.copy(), size=960, field_mm=FIELD_MM, source_size=CAMERA)
    print(f"校正后 {calib.size}x{calib.size}，{calib.mm_per_px:.3f} mm/px；"
          f"tag 约 {TAG_MM / calib.mm_per_px:.1f} px，红块约 {20 / calib.mm_per_px:.1f} px")

    pos_err, head_err, hits = [], [], 0
    for i in range(TRIALS):
        x = float(rng.uniform(150, FIELD_MM - 150))
        y = float(rng.uniform(150, FIELD_MM - 150))
        heading = float(rng.uniform(0, 360))
        frame = shoot(make_world(rng, (x, y), heading), rng)
        view = calib.warp(frame)
        pose, reason = detect_pose(view, calib, TAG_MM, expected_id=TAG_ID)
        if pose is None:
            print(f"  #{i:02d} 未检出：{reason}")
            continue
        hits += 1
        dx, dy = pose.x_mm - x, pose.y_mm - y
        dh = abs(pose.heading_deg - heading) % 360.0
        dh = min(dh, 360.0 - dh)
        pos_err.append(math.hypot(dx, dy))
        head_err.append(dh)
        print(f"  #{i:02d} 真值 ({x:6.1f},{y:6.1f},{heading:6.1f})  "
              f"实测 ({pose.x_mm:6.1f},{pose.y_mm:6.1f},{pose.heading_deg:6.1f})  "
              f"误差 {math.hypot(dx, dy):.2f} mm / {dh:.2f}°")

    print(f"\n检出 {hits}/{TRIALS}")
    if pos_err:
        print(f"位置误差 P50 {np.percentile(pos_err, 50):.2f} mm，P95 {np.percentile(pos_err, 95):.2f} mm")
        print(f"朝向误差 P50 {np.percentile(head_err, 50):.2f}°，P95 {np.percentile(head_err, 95):.2f}°")


if __name__ == "__main__":
    main()
