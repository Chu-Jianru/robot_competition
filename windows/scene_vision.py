"""Compose independent pixel detections with optional field calibration."""
import cv2
import numpy as np

from red_blocks import detect_red_blocks


def _field_coverage(image, field):
    from field_vision import HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    perimeter = cv2.convexHull(np.asarray(field["field_px"], np.float32))
    inside = np.zeros(image.shape[:2], np.uint8)
    cv2.fillConvexPoly(inside, np.rint(perimeter).astype(np.int32), 255)
    area = max(cv2.countNonZero(inside), 1)
    return {name: cv2.countNonZero(cv2.bitwise_and(
        cv2.inRange(hsv, np.uint8(HSV[name][0]), np.uint8(HSV[name][1])), inside)) / area
        for name in ("blue", "yellow")}


def locate_red_blocks(image, detections, field):
    """Add calibrated positions without changing independent pixel detections."""
    if field["homography"] is None:
        return []
    from field_vision import FIELD_HALF, HSV, _on_blue, image_to_world
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    ground = {name: cv2.inRange(hsv, np.uint8(HSV[name][0]), np.uint8(HSV[name][1]))
              for name in ("blue", "yellow")}
    homography = field["homography"]
    bias = field["boundary_y"]
    located = []
    for detection in detections:
        box = cv2.boxPoints(cv2.minAreaRect(detection["contour_px"]))
        world_box = image_to_world(homography, box)
        sides = [float(np.linalg.norm(world_box[i] - world_box[(i + 1) % 4]))
                 for i in (0, 1)]
        short, long = min(sides), max(sides)
        single = short >= 0.005 and long <= 0.08
        pile = 0.08 <= long <= 0.45 and detection["area_px"] >= 40.0
        if not (single or pile):
            continue
        x, y = map(float, image_to_world(homography, [detection["center_px"]])[0])
        if abs(x) > FIELD_HALF + 0.02 or abs(y) > FIELD_HALF + 0.05:
            continue
        y -= bias
        own = _on_blue(ground["blue"], ground["yellow"], *detection["center_px"])
        if own and y > 0:
            y = -abs(y) - 0.005
        elif not own and y < 0:
            y = abs(y) + 0.005
        located.append({"xy": (x, y), "own": own,
                        "px": detection["center_px"],
                        "half": tuple(map(float, np.ptp(world_box, axis=0) / 2))})
    return located


def inspect_image(image, field_lock=None, *, red_enabled=True, field_enabled=False,
                  red_config=None):
    """Red detection runs first, regardless of field availability or its switch."""
    detections = detect_red_blocks(image, red_config) if red_enabled else []
    result = {
        "red_enabled": red_enabled, "field_enabled": field_enabled,
        "red_detections": detections,
        "red_candidate_boxes": [d["bbox_px"] for d in detections],
        "blocks": [], "extents": [], "block_px": [], "own_flags": [],
        "own_count": -1, "homography": None, "field_px": None,
        "robot": None, "robot_px": None, "obstacles": [], "obstacle_extents": [],
        "boundary_y": 0.0, "field_rejected": False, "ground_coverage": None,
    }
    if not field_enabled:
        return result
    from field_vision import detect_field
    field = detect_field(image, field_lock)
    if field["homography"] is None:
        return result
    coverage = _field_coverage(image, field)
    result["ground_coverage"] = coverage
    if min(coverage.values()) < 0.10 or sum(coverage.values()) < 0.55:
        result["field_rejected"] = True
        return result
    result.update(field)
    if red_enabled:
        located = locate_red_blocks(image, detections, field)
        result.update(blocks=[b["xy"] for b in located],
                      extents=[b["half"] for b in located],
                      block_px=[b["px"] for b in located],
                      own_flags=[b["own"] for b in located],
                      own_count=sum(b["own"] for b in located))
    return result
