"""俯视画面里的场地、红块和车体色标。

不假设画面是正方形，也不假设场地铺满画面、边和图像轴对齐。
先用蓝/黄找到场地四角，再变到场地坐标（中心为原点，黄区为 +y）。
"""

from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

FIELD_HALF = 0.6
Point = Tuple[float, float]

# OpenCV 的 H 是 0–180。范围按 Gazebo 材质留了余量，现场灯光不对时再收紧。
HSV = {
    "blue": ((100, 80, 40), (130, 255, 255)),
    "yellow": ((15, 80, 40), (40, 255, 255)),
    "red_lo": ((0, 70, 40), (10, 255, 255)),
    "red_hi": ((170, 70, 40), (180, 255, 255)),
    "magenta": ((135, 80, 40), (170, 255, 255)),
    "cyan": ((80, 80, 40), (100, 255, 255)),
}


def _mask(hsv: np.ndarray, low: Sequence[int], high: Sequence[int]) -> np.ndarray:
    return cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))


def _centroid(mask: np.ndarray) -> Optional[np.ndarray]:
    moments = cv2.moments(mask, binaryImage=True)
    if moments["m00"] < 1.0:
        return None
    return np.array(
        [moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]],
        dtype=np.float32,
    )


def _field_quad(field_mask: np.ndarray) -> Optional[np.ndarray]:
    contours, _ = cv2.findContours(field_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    image_area = field_mask.shape[0] * field_mask.shape[1]
    if cv2.contourArea(contour) < 0.05 * image_area:
        return None
    hull = cv2.convexHull(contour)
    peri = cv2.arcLength(hull, True)
    for ratio in (0.01, 0.02, 0.035, 0.05, 0.08):
        approx = cv2.approxPolyDP(hull, ratio * peri, True)
        if len(approx) == 4:
            return approx.reshape(4, 2).astype(np.float32)
    rect = cv2.minAreaRect(hull)
    return cv2.boxPoints(rect).astype(np.float32)


def _axes_from_colors(blue_xy: np.ndarray, yellow_xy: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    y_axis = yellow_xy - blue_xy
    norm = float(np.linalg.norm(y_axis))
    if norm < 5.0:
        return None
    y_axis = y_axis / norm
    # 图像 y 向下。顺着黄区方向看，右侧才是场地 +x。
    x_axis = np.array([-y_axis[1], y_axis[0]], dtype=np.float32)
    return x_axis, y_axis


def _pair_corners(
    corners: np.ndarray,
    x_axis: np.ndarray,
    y_axis: np.ndarray,
    half: float = FIELD_HALF,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    center = corners.mean(axis=0)
    rel = corners - center
    proj_x = rel @ x_axis
    proj_y = rel @ y_axis
    src = []
    dst = []
    used = set()
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            score = sx * proj_x + sy * proj_y
            idx = int(np.argmax(score))
            if idx in used:
                return None
            used.add(idx)
            src.append(corners[idx])
            dst.append((sx * half, sy * half))
    return np.float32(src), np.float32(dst)


def image_to_world(homography: np.ndarray, points: Sequence[Sequence[float]]) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, homography).reshape(-1, 2)


def _blobs(
    mask: np.ndarray,
    homography: np.ndarray,
    min_side: float,
    max_side: float,
    min_area_px: float = 4.0,
) -> List[dict]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for contour in contours:
        if cv2.contourArea(contour) < min_area_px:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        px = moments["m10"] / moments["m00"]
        py = moments["m01"] / moments["m00"]
        box = cv2.boxPoints(cv2.minAreaRect(contour))
        world_box = image_to_world(homography, box)
        side_a = float(np.linalg.norm(world_box[0] - world_box[1]))
        side_b = float(np.linalg.norm(world_box[1] - world_box[2]))
        short, long = min(side_a, side_b), max(side_a, side_b)
        if short < min_side or long > max_side:
            continue
        center = image_to_world(homography, [(px, py)])[0]
        if abs(float(center[0])) > FIELD_HALF + 0.02 or abs(float(center[1])) > FIELD_HALF + 0.05:
            continue
        found.append(
            {
                "xy": (float(center[0]), float(center[1])),
                "px": (float(px), float(py)),
                "side": long,
            }
        )
    return found


def _boundary_y(blue: np.ndarray, yellow: np.ndarray, homography: np.ndarray) -> float:
    """蓝黄分界在当前单应性坐标里的 y。四边形几何中心不一定落在这条线上。"""
    step = 6
    blue_s = blue[::step, ::step] > 0
    yellow_s = yellow[::step, ::step] > 0
    ys, xs = np.where(blue_s | yellow_s)
    if len(xs) < 20:
        return 0.0
    pixels = np.stack((xs * step, ys * step), axis=1).astype(np.float32)
    world = image_to_world(homography, pixels)
    blue_y = world[blue_s[ys, xs], 1]
    yellow_y = world[yellow_s[ys, xs], 1]
    if blue_y.size < 10 or yellow_y.size < 10:
        return 0.0
    # 蓝区靠近分界的一侧，和黄区靠近分界的一侧。
    return float(np.percentile(blue_y, 92) + np.percentile(yellow_y, 8)) / 2.0


def _on_blue(blue: np.ndarray, yellow: np.ndarray, px: float, py: float) -> bool:
    """看红块周围的地面颜色。块本身盖住中心，外圈才是蓝或黄。"""
    x = int(round(px))
    y = int(round(py))
    radius = 16
    y0 = max(0, y - radius)
    y1 = min(blue.shape[0], y + radius + 1)
    x0 = max(0, x - radius)
    x1 = min(blue.shape[1], x + radius + 1)
    patch_b = blue[y0:y1, x0:x1]
    patch_y = yellow[y0:y1, x0:x1]
    if patch_b.size == 0:
        return False
    yy, xx = np.ogrid[y0:y1, x0:x1]
    ring = (xx - x) ** 2 + (yy - y) ** 2 >= 6 ** 2
    blue_count = int(patch_b[ring].sum() / 255) if ring.shape == patch_b.shape else int((patch_b > 0).sum())
    yellow_count = int(patch_y[ring].sum() / 255) if ring.shape == patch_y.shape else int((patch_y > 0).sum())
    return blue_count >= yellow_count


def detect_field(bgr: np.ndarray) -> Dict:
    """从一帧 BGR 图得到红块、车的位姿，以及己方（y<0）剩余块数。"""
    empty = {
        "blocks": [],
        "own_flags": [],
        "obstacles": [],
        "robot": None,
        "own_count": -1,
        "homography": None,
        "field_px": None,
        "block_px": [],
        "robot_px": None,
    }
    if bgr is None or bgr.size == 0:
        return empty

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blue = _mask(hsv, *HSV["blue"])
    yellow = _mask(hsv, *HSV["yellow"])
    field = cv2.bitwise_or(blue, yellow)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
    field = cv2.morphologyEx(field, cv2.MORPH_CLOSE, kernel)

    corners = _field_quad(field)
    blue_xy = _centroid(blue)
    yellow_xy = _centroid(yellow)
    if corners is None or blue_xy is None or yellow_xy is None:
        return empty
    axes = _axes_from_colors(blue_xy, yellow_xy)
    if axes is None:
        return empty
    paired = _pair_corners(corners, *axes)
    if paired is None:
        return empty
    src, dst = paired
    homography = cv2.getPerspectiveTransform(src, dst)

    red = cv2.bitwise_or(_mask(hsv, *HSV["red_lo"]), _mask(hsv, *HSV["red_hi"]))
    magenta = _mask(hsv, *HSV["magenta"])
    cyan = _mask(hsv, *HSV["cyan"])
    small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, small)
    magenta = cv2.morphologyEx(magenta, cv2.MORPH_OPEN, small)

    blocks = _blobs(red, homography, 0.005, 0.08)
    markers = _blobs(magenta, homography, 0.012, 0.12)
    headings = _blobs(cyan, homography, 0.008, 0.12, min_area_px=4.0)
    bias = _boundary_y(blue, yellow, homography)
    for block in blocks:
        x, y = block["xy"]
        y -= bias
        own = _on_blue(blue, yellow, *block["px"])
        # 颜色和坐标打架时信颜色，只翻符号，距离留着。
        if own and y > 0.0:
            y = -abs(y) - 0.005
        elif not own and y < 0.0:
            y = abs(y) + 0.005
        block["xy"] = (x, y)
        block["own"] = own
    for marker in markers:
        x, y = marker["xy"]
        marker["xy"] = (x, y - bias)
    for heading in headings:
        x, y = heading["xy"]
        heading["xy"] = (x, y - bias)

    robot = None
    robot_px = None
    if markers and headings:
        front = _mean_point(markers, "xy")
        rear = _mean_point(headings, "xy")
        rx = (front[0] + rear[0]) / 2.0
        ry = (front[1] + rear[1]) / 2.0
        theta = float(np.arctan2(front[1] - rear[1], front[0] - rear[0]))
        robot = (rx, ry, theta)
        front_px = _mean_point(markers, "px")
        rear_px = _mean_point(headings, "px")
        robot_px = (
            ((front_px[0] + rear_px[0]) / 2.0, (front_px[1] + rear_px[1]) / 2.0),
            front_px,
        )

    own_count = sum(1 for block in blocks if block.get("own"))
    robot_xy = None if robot is None else (robot[0], robot[1])
    obstacles = _obstacles(hsv, homography, src, bias, robot_xy)
    return {
        "blocks": [block["xy"] for block in blocks],
        "own_flags": [bool(block.get("own")) for block in blocks],
        "obstacles": obstacles,
        "robot": robot,
        "own_count": own_count,
        "homography": homography,
        "field_px": src,
        "block_px": [block["px"] for block in blocks],
        "robot_px": robot_px,
    }


def _obstacles(hsv, homography, field_src, bias: float, robot_xy: Optional[Point]) -> List[Point]:
    """场地里比物块大、又不是红块和己方色标的物体。车自己身上的绿色和灰铲不算。"""
    height, width = hsv.shape[:2]
    inside = np.zeros((height, width), np.uint8)
    cv2.fillConvexPoly(inside, np.asarray(field_src, np.int32), 255)
    known = _mask(hsv, *HSV["blue"])
    for name in ("yellow", "red_lo", "red_hi", "magenta", "cyan"):
        known = cv2.bitwise_or(known, _mask(hsv, *HSV[name]))
    other = cv2.bitwise_and(inside, cv2.bitwise_not(known))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    other = cv2.morphologyEx(other, cv2.MORPH_OPEN, kernel)
    points = []
    for blob in _blobs(other, homography, 0.06, 0.40, min_area_px=80.0):
        x, y = blob["xy"]
        y -= bias
        if robot_xy is not None and _dist((x, y), robot_xy) < 0.16:
            continue
        points.append((x, y))
    return points


def _mean_point(blobs: Sequence[dict], key: str) -> Point:
    xs = [blob[key][0] for blob in blobs]
    ys = [blob[key][1] for blob in blobs]
    n = float(len(blobs))
    return sum(xs) / n, sum(ys) / n


def _dist(a: Point, b: Point) -> float:
    return float(np.hypot(a[0] - b[0], a[1] - b[1]))
