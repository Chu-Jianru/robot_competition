"""判断场地在画面里的朝向，不写死旋转角度。

实跑的 `find_seam` 要求画面是“黄上蓝下”，但手机怎么放、场地怎么摆会变：
竖拍时黄区在右、横拍时黄区在上，换个人摆就又不一样。这里用蓝黄地面的重心
关系算出需要转多少度，让调用方不必再猜 `--rotate`。

返回的角度是**逆时针**角度（0/90/180/270），和 `cv2.ROTATE_*` 对应。
"""
from __future__ import annotations

import cv2

ROTATIONS = {0: None, 90: cv2.ROTATE_90_COUNTERCLOCKWISE,
             180: cv2.ROTATE_180, 270: cv2.ROTATE_90_CLOCKWISE}
ANGLES = (0, 90, 180, 270)


def _centroid(mask):
    moments = cv2.moments(mask, binaryImage=True)
    if moments["m00"] < 1.0:
        return None
    return (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])


def centroids(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, (100, 80, 40), (130, 255, 255))
    yellow = cv2.inRange(hsv, (15, 80, 40), (40, 255, 255))
    return _centroid(blue), _centroid(yellow)


def choose_rotation(blue_xy, yellow_xy) -> int | None:
    """Degrees counter-clockwise so the yellow half ends up above the blue half."""
    if blue_xy is None or yellow_xy is None:
        return None
    dx = yellow_xy[0] - blue_xy[0]
    dy = yellow_xy[1] - blue_xy[1]
    if abs(dx) < 4 and abs(dy) < 4:
        return None
    if abs(dx) >= abs(dy):
        return 90 if dx > 0 else 270
    return 0 if dy < 0 else 180


def field_rotation(image) -> int | None:
    return choose_rotation(*centroids(image))


def apply_rotation(image, degrees: int):
    code = ROTATIONS.get(int(degrees) % 360)
    return cv2.rotate(image, code) if code is not None else image


def decide_rotation(rotations) -> int | None:
    """Majority vote over per-frame estimates; None when nothing was measured."""
    votes = {}
    for angle in rotations:
        if angle is None:
            continue
        angle = int(angle) % 360
        votes[angle] = votes.get(angle, 0) + 1
    if not votes:
        return None
    best = max(votes.values())
    winners = sorted(angle for angle, count in votes.items() if count == best)
    # A 50/50 split means the field was moving or half-visible: report nothing
    # rather than silently picking one orientation.
    return winners[0] if len(winners) == 1 else None
