"""用倾斜、留白、非正方形的合成图检查场地检测。不需要 Gazebo。"""

import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sweep_robot.field_vision import FieldLock, detect_field


def world_to_px(x: float, y: float, scale: float) -> tuple:
    return (x + 0.6) * scale, (0.6 - y) * scale


def fill_rect(image: np.ndarray, cx: float, cy: float, sx: float, sy: float, color: tuple, scale: float) -> None:
    x0, y0 = world_to_px(cx - sx / 2, cy + sy / 2, scale)
    x1, y1 = world_to_px(cx + sx / 2, cy - sy / 2, scale)
    cv2.rectangle(
        image,
        (int(round(x0)), int(round(y0))),
        (int(round(x1)), int(round(y1))),
        color,
        thickness=-1,
    )


def make_frame(dst_quad: np.ndarray, canvas_wh: tuple) -> tuple:
    scale = 800.0
    side = int(1.2 * scale)
    field = np.full((side, side, 3), 40, np.uint8)
    # 上半是黄区（世界 +y），下半是蓝区。颜色接近 Gazebo 材质。
    mid = side // 2
    field[:mid, :] = (13, 204, 242)
    field[mid:, :] = (230, 64, 13)

    blocks = []
    for i in range(10):
        x = -0.45 + 0.10 * i
        blocks.append((x, -0.06 - 0.004 * (i % 3)))
        blocks.append((x + 0.02, 0.07 + 0.003 * (i % 4)))

    for x, y in blocks:
        fill_rect(field, x, y, 0.02, 0.02, (13, 13, 242), scale)

    robot = (0.12, -0.42, 1.15)
    rx, ry, theta = robot
    fill_rect(field, rx, ry, 0.09, 0.085, (51, 140, 26), scale)
    c = math.cos(theta)
    s = math.sin(theta)
    for lx, ly, color in (
        (0.032, 0.026, (255, 0, 255)),
        (0.032, -0.026, (255, 0, 255)),
        (-0.032, 0.026, (255, 255, 0)),
        (-0.032, -0.026, (255, 255, 0)),
    ):
        fill_rect(field, rx + lx * c - ly * s, ry + lx * s + ly * c, 0.02, 0.02, color, scale)

    src = np.float32([[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]])
    width, height = canvas_wh
    canvas = np.full((height, width, 3), 170, np.uint8)
    warped = cv2.warpPerspective(field, cv2.getPerspectiveTransform(src, dst_quad), (width, height))
    mask = cv2.warpPerspective(np.full((side, side), 255, np.uint8), cv2.getPerspectiveTransform(src, dst_quad), (width, height))
    canvas[mask > 0] = warped[mask > 0]
    gradient = np.linspace(0.68, 1.0, height, dtype=np.float32)[:, None, None]
    canvas = np.clip(canvas.astype(np.float32) * gradient, 0, 255).astype(np.uint8)
    return canvas, blocks, robot


def check(name: str, frame: np.ndarray, blocks: list, robot: tuple) -> None:
    result = detect_field(frame)
    assert result["homography"] is not None, f"{name}: 没有找到场地"
    assert result["own_count"] == 10, f"{name}: 己方数量 {result['own_count']}"
    assert len(result["blocks"]) == 20, f"{name}: 红块数量 {len(result['blocks'])}"
    assert result["robot"] is not None, f"{name}: 没有找到车"
    for truth in blocks:
        err = min(math.hypot(truth[0] - found[0], truth[1] - found[1]) for found in result["blocks"])
        assert err < 0.025, f"{name}: 方块 {truth} 误差 {err:.3f} m"
    rx, ry, theta = result["robot"]
    pos_err = math.hypot(rx - robot[0], ry - robot[1])
    ang_err = abs(math.atan2(math.sin(theta - robot[2]), math.cos(theta - robot[2])))
    assert pos_err < 0.025, f"{name}: 车位置误差 {pos_err:.3f} m"
    assert ang_err < 0.20, f"{name}: 朝向误差 {ang_err:.3f} rad"
    print(f"{name}: 通过，位置误差 {pos_err * 100:.1f} cm，朝向误差 {math.degrees(ang_err):.1f} 度")


def check_offset_boundary() -> None:
    """黄区占画面的大部分，四边形中心落在黄区，分界不在几何中心。"""
    height, width = 960, 960
    image = np.full((height, width, 3), 40, np.uint8)
    split = int(height * 0.72)
    image[:split, :] = (13, 204, 242)
    image[split:, :] = (230, 64, 13)
    for i in range(10):
        cx = 80 + i * 85
        cv2.rectangle(image, (cx - 8, split + 36), (cx + 8, split + 52), (13, 13, 242), -1)
        cv2.rectangle(image, (cx - 8, split - 52), (cx + 8, split - 36), (13, 13, 242), -1)
    cv2.rectangle(image, (470, 790), (490, 810), (255, 0, 255), -1)
    cv2.rectangle(image, (510, 790), (530, 810), (255, 0, 255), -1)
    cv2.rectangle(image, (470, 830), (490, 850), (255, 255, 0), -1)
    cv2.rectangle(image, (510, 830), (530, 850), (255, 255, 0), -1)

    src = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
    dst = np.float32([[140, 40], [1140, 160], [1180, 900], [80, 760]])
    canvas = np.full((960, 1280, 3), 170, np.uint8)
    warped = cv2.warpPerspective(image, cv2.getPerspectiveTransform(src, dst), (1280, 960))
    mask = cv2.warpPerspective(
        np.full((height, width), 255, np.uint8),
        cv2.getPerspectiveTransform(src, dst),
        (1280, 960),
    )
    canvas[mask > 0] = warped[mask > 0]

    result = detect_field(canvas)
    assert result["own_count"] == 10, f"偏移中线: 己方 {result['own_count']}"
    assert len(result["blocks"]) == 20, f"偏移中线: 红块 {len(result['blocks'])}"
    flags = result["own_flags"]
    for (x, y), own in zip(result["blocks"], flags):
        if own:
            assert y < 0.0, f"蓝区块被标成黄区 y={y:.3f}"
        else:
            assert y > 0.0, f"黄区块被标成蓝区 y={y:.3f}"
    assert result["robot"] is not None and result["robot"][1] < 0.0
    print("偏移中线: 通过，己方 10 个，符号和颜色一致")


def check_field_lock() -> None:
    lock = FieldLock()
    base = np.float32([[10, 10], [400, 12], [398, 300], [14, 302]])
    assert np.allclose(lock.corners_of(base), base)
    noisy = base + np.float32([[1.0, -1.0], [0.0, 2.0], [-1.0, 0.0], [2.0, 1.0]])
    smoothed = lock.corners_of(noisy)
    assert np.max(np.abs(smoothed - base)) < 0.5
    spike = base.copy()
    spike[0] += (40.0, 0.0)
    assert np.allclose(lock.corners_of(spike), smoothed)
    assert abs(lock.bias_of(0.01) - 0.01) < 1e-9
    assert abs(lock.bias_of(0.012) - 0.0103) < 1e-4
    assert abs(lock.bias_of(0.08) - 0.0103) < 1e-4
    print("四角锁定: 通过，单帧跳动被按住")


def main() -> None:
    tilted = np.float32([[150, 60], [1120, 140], [1160, 880], [110, 800]])
    frame, blocks, robot = make_frame(tilted, (1280, 960))
    check("倾斜宽画面", frame, blocks, robot)

    other = np.float32([[220, 150], [980, 80], [1040, 820], [80, 760]])
    frame, blocks, robot = make_frame(other, (1100, 900))
    check("另一组倾斜", frame, blocks, robot)
    check_offset_boundary()
    check_field_lock()


if __name__ == "__main__":
    main()
