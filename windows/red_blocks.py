"""Independent red-block candidates using color and compact square geometry.

No field or ROS is needed. Geometry suppresses red wheel fragments but cannot
guarantee object identity; calibration and ownership belong upstream.
"""
from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class RedBlockConfig:
    low_hue: tuple = (0, 10)
    high_hue: tuple = (170, 180)
    min_saturation: int = 70
    max_saturation: int = 255
    min_value: int = 40
    min_area_px: float = 20.0
    shape_filter: bool = True
    min_aspect_ratio: float = 0.65
    min_rectangularity: float = 0.80
    min_solidity: float = 0.90
    close_gaps: bool = False
    # Optional second exposure pass. Original calibrated detection stays intact.
    shadow_min_value: int | None = None


def calibrate_red_config(sample, base=None, *, roi=None):
    """Calibrate from the inner red TOP FACE, without background/dark sides.

    Keep the two red hue bands; learn saturation and brightness from actual
    material. Shape can then tolerate minor missing edges without accepting
    the differently colored wheel highlights seen in the supplied images.
    With roi=(x,y,w,h), pass the full image to also estimate a weak pixel-size
    lower bound. Recalibrate when lighting or camera scale changes.
    """
    source = sample
    if roi is not None:
        x, y, width, height = map(int, roi)
        if (source is None or min(x, y) < 0 or min(width, height) <= 0
                or x + width > source.shape[1] or y + height > source.shape[0]):
            raise ValueError("Select a rectangle inside the image.")
        sample = source[y:y + height, x:x + width]
    if sample is None or sample.size == 0:
        raise ValueError("Select the inside of the red block top face.")
    hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    red = (((hue <= 10) | (hue >= 170)) & (saturation >= 70) & (value >= 40))
    pixels = hsv[red]
    if len(pixels) < 9 or len(pixels) < sample.shape[0] * sample.shape[1] * 0.5:
        raise ValueError("Selection must mostly contain the red block top face.")
    s_low, s_high = np.percentile(pixels[:, 1], (5, 95))
    v_low = float(np.percentile(pixels[:, 2], 5))
    config = replace(base or RedBlockConfig(),
                     min_saturation=max(40, int(s_low) - 30),
                     max_saturation=min(255, int(np.ceil(s_high)) + 30),
                     min_value=max(30, int(v_low) - 40),
                     min_rectangularity=0.72, min_solidity=0.80, close_gaps=True)
    if config.shadow_min_value is not None:
        config = replace(config, shadow_min_value=min(config.shadow_min_value, config.min_value))
    if roi is not None:
        # User-selected known block supplies a weak size bound for this camera
        # setup; no field dimensions, fixed location or largest-object rule.
        center = (x + width / 2.0, y + height / 2.0)
        broad = detect_red_blocks(source, RedBlockConfig(shape_filter=False, min_area_px=4))
        seed = next((d for d in broad if cv2.pointPolygonTest(d["contour_px"], center, False) >= 0), None)
        if seed is None:
            raise ValueError("Select inside one visible red block.")
        config = replace(config, min_area_px=max(4.0, seed["area_px"] * 0.10))
    return config


def load_red_config(path):
    path = Path(path)
    if not path.exists():
        return RedBlockConfig()
    values = json.loads(path.read_text(encoding="utf-8"))
    for name in ("low_hue", "high_hue"):
        if name in values:
            values[name] = tuple(values[name])
    config = RedBlockConfig(**values)
    if not (0 <= config.min_saturation <= config.max_saturation <= 255
            and 0 <= config.min_value <= 255 and config.min_area_px > 0):
        raise ValueError("Invalid red-block color profile.")
    if config.shadow_min_value is not None and not 0 <= config.shadow_min_value <= config.min_value:
        raise ValueError("Invalid shadow brightness threshold.")
    return config


def save_red_config(path, config):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(asdict(config), ensure_ascii=False, indent=2) + "\n",
                         encoding="utf-8")
    temporary.replace(path)


def detect_red_blocks(bgr, config=None):
    """Return pixel geometry and shape metrics for compact red candidates.

    Input: BGR image only. Output has no meters, field validity or own/enemy
    flag. Configuring this detector never requires changing field thresholds.
    """
    if bgr is None or bgr.size == 0:
        return []
    config = config or RedBlockConfig()
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    found = _detect_at_value(hsv, config)
    if config.shadow_min_value is not None and config.shadow_min_value < config.min_value:
        # Dim red faces need lower V, with stronger geometry to reject wheel
        # fragments. Do not relax the calibrated hue or saturation bands.
        shadow_config = replace(config, min_value=config.shadow_min_value,
                                min_aspect_ratio=max(config.min_aspect_ratio, 0.80),
                                min_rectangularity=max(config.min_rectangularity, 0.85),
                                min_solidity=max(config.min_solidity, 0.90))
        shadows = _detect_at_value(hsv, shadow_config)
        # A complete dim face replaces its already-detected bright fragments.
        # Keep distinct blocks regardless of their relative sizes or positions.
        found = [candidate for candidate in found if not any(
            cv2.pointPolygonTest(shadow["contour_px"], candidate["center_px"], False) >= 0
            for shadow in shadows)]
        found.extend(shadows)
    return found


def _detect_at_value(hsv, config):
    masks = [cv2.inRange(hsv, (lo, config.min_saturation, config.min_value),
                        (hi, config.max_saturation, 255))
             for lo, hi in (config.low_hue, config.high_hue)]
    red = cv2.bitwise_or(*masks)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    if config.close_gaps:
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < config.min_area_px:
            continue
        (_, _), (width, height), _ = cv2.minAreaRect(contour)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        if min(width, height) <= 0 or hull_area <= 0:
            continue
        aspect = min(width, height) / max(width, height)
        rectangularity = area / (width * height)
        solidity = area / hull_area
        if config.shape_filter and (
                aspect < config.min_aspect_ratio
                or rectangularity < config.min_rectangularity
                or solidity < config.min_solidity):
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        found.append({
            "center_px": (moments["m10"] / moments["m00"],
                          moments["m01"] / moments["m00"]),
            "bbox_px": cv2.boundingRect(contour),
            "area_px": float(area),
            "contour_px": contour,
            "aspect_ratio": float(aspect),
            "rectangularity": float(rectangularity),
            "solidity": float(solidity),
        })
    return found


# 物块标称边长（比赛用 2 cm 木块）。只作为毫米尺度的参考值，不作为身份判据。
BLOCK_MM = 20.0


def equivalent_side_mm(block, mm_per_px):
    """按面积折算的等效正方形边长（mm）。遮挡、透视都会影响它，只用于粗筛。"""
    return float(block["area_px"] ** 0.5) * float(mm_per_px)


def prelabel_boxes(bgr, config=None, min_side: int = 6):
    """把当前帧的颜色候选转成标注用的 [x, y, w, h] 列表（预填，不是真值）。

    给训练集标注当起点：人只需要删误检、补漏检，不用从零画。
    """
    if bgr is None or bgr.size == 0:
        return []
    height, width = bgr.shape[:2]
    boxes = []
    for candidate in detect_red_blocks(bgr, config):
        x, y, w, h = (int(value) for value in candidate["bbox_px"])
        x, y = max(0, x), max(0, y)
        w, h = min(w, width - x), min(h, height - y)
        if w >= min_side and h >= min_side:
            boxes.append([x, y, w, h])
    return boxes


def filter_by_size_mm(blocks, mm_per_px, min_side_mm=12.0, max_side_mm=32.0):
    """用毫米尺度剔除明显不是 2 cm 物块的候选。

    这是分辨率无关的判据：像素面积下限在 640 和 1280 下含义不同，毫米不随分辨率变。
    min 取 12 mm（标称的 60%）给部分遮挡留余量；max 取 32 mm 用来挡合并成一团的块。
    返回 (kept, rejected)；kept 里补 `side_mm` 字段，rejected 是 (block, reason)。
    """
    kept, rejected = [], []
    for block in blocks:
        side = equivalent_side_mm(block, mm_per_px)
        enriched = dict(block, side_mm=side)
        if side < min_side_mm:
            rejected.append((enriched, "too_small"))
        elif side > max_side_mm:
            rejected.append((enriched, "too_large"))
        else:
            kept.append(enriched)
    return kept, rejected
