"""Shared ArUco detector; an ID-specific fallback handles missing white margins.

The fallback proposes geometry from real bright components, then decodes the
original pixels with a temporary *external* white margin. No code bits or black
border pixels are filled in. It requires an explicit expected ID, exact cell
agreement, and a successful image alignment. It does not track old detections.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache

import cv2
import numpy as np

# 车顶 ArUco 码的编号（DICT_4X4_50）。换码只改这一处，其余脚本的默认值都取它。
TAG_ID = 7


@dataclass
class Marker:
    corners: np.ndarray
    tag_id: int
    method: str


def detector_parameters():
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 5
    params.adaptiveThreshWinSizeMax = 45
    params.minMarkerPerimeterRate = 0.02
    params.polygonalApproxAccuracyRate = 0.05
    params.errorCorrectionRate = 0.6
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


@lru_cache(maxsize=2)
def _detector(strict=False):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = detector_parameters()
    if strict:
        params.errorCorrectionRate = 0.0
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return cv2.aruco.ArucoDetector(dictionary, params)


@lru_cache(maxsize=50)
def _template(tag_id):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    image = cv2.aruco.generateImageMarker(dictionary, tag_id, 72)
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Tiny isolated cells cannot constrain a marker's geometry reliably.
    contours = [c for c in contours if cv2.contourArea(c) >= 250
                and min(cv2.minAreaRect(c)[1]) >= 20]
    return image, contours


_SQUARE = np.float32([[0, 0], [71, 0], [71, 71], [0, 71]])


def _valid_quad(quad, shape):
    if not np.isfinite(quad).all() or not cv2.isContourConvex(quad):
        return False
    lengths = np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)
    height, width = shape
    return (min(lengths) >= 18 and max(lengths) / min(lengths) <= 1.8
            and np.all(quad >= 1) and np.all(quad[:, 0] < width - 1)
            and np.all(quad[:, 1] < height - 1))


def _sample(gray, quad):
    return cv2.warpPerspective(gray, cv2.getPerspectiveTransform(quad, _SQUARE), (72, 72))


def _exact_cells(tile, template):
    # Cell centres avoid antialiased boundaries. Check all 16 data + 20 border cells.
    cells = np.array([[np.median(tile[y*12+3:y*12+9, x*12+3:x*12+9])
                       for x in range(6)] for y in range(6)])
    expected = template[6::12, 6::12] > 128
    black, white = cells[~expected], cells[expected]
    contrast = float(np.median(white) - np.median(black))
    threshold = (float(np.median(white)) + float(np.median(black))) / 2
    return contrast >= 30 and np.array_equal(cells > threshold, expected)


def _decode_proposal(gray, quad, tag_id):
    tile = _sample(gray, quad)
    padded = cv2.copyMakeBorder(tile, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=255)
    corners, ids, _ = _detector(True).detectMarkers(padded)
    if ids is None:
        return None
    for points, marker_id in zip(corners, ids.flatten()):
        if int(marker_id) != tag_id:
            continue
        # Use the decoder's canonical corner order, including its rotation.
        to_image = cv2.getPerspectiveTransform(_SQUARE, quad)
        return cv2.perspectiveTransform(points.astype(np.float32) - 12, to_image).reshape(4, 2)
    return None


def _align(gray, quad, tag_id):
    template, _ = _template(tag_id)
    tile = _sample(gray, quad)
    if not _exact_cells(tile, template):
        return None
    mask = np.zeros((72, 72), np.uint8)
    mask[6:66, 6:66] = 255  # Exclude the missing exterior edge during alignment.
    try:
        correlation, warp = cv2.findTransformECC(
            template, tile, np.eye(2, 3, dtype=np.float32), cv2.MOTION_AFFINE,
            (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 25, 1e-4), mask, 3)
    except cv2.error:
        return None
    if correlation < 0.85:
        return None
    aligned = cv2.transform(_SQUARE.reshape(1, 4, 2), warp)
    refined = cv2.perspectiveTransform(aligned, cv2.getPerspectiveTransform(_SQUARE, quad))[0]
    side = float(np.mean(np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)))
    if (not _valid_quad(refined, gray.shape)
            or np.max(np.linalg.norm(refined - quad, axis=1)) > side * 0.15
            or not _exact_cells(_sample(gray, refined), template)):
        return None
    return refined


def _data_square(contour):
    """Return (side, deg, center): smallest square enclosing the bright cells.

    The bright cells fill the marker's 4x4 data area, which is 4/6 of the marker
    side. A minimum-area *rectangle* can rotate on an L- or T-shaped pattern, so
    fit a square instead; the best angle can sit between hull edges, hence the
    bounded sweep.
    """
    hull = cv2.convexHull(contour).reshape(-1, 2).astype(np.float32)
    if len(hull) < 3:
        return None
    best = None
    for deg in range(90):
        rad = math.radians(deg)
        rot = np.float32([[math.cos(rad), -math.sin(rad)],
                          [math.sin(rad), math.cos(rad)]])
        turned = hull @ rot.T
        low, high = turned.min(axis=0), turned.max(axis=0)
        side = float(max(high - low))
        if best is None or side < best[0]:
            best = (side, float(deg), ((low + high) / 2) @ rot)
    return best


def _square_quad(center, side, deg):
    rad = math.radians(deg)
    rot = np.float32([[math.cos(rad), -math.sin(rad)],
                      [math.sin(rad), math.cos(rad)]])
    corners = np.float32([[-.5, -.5], [.5, -.5], [.5, .5], [-.5, .5]]) * float(side)
    # `@ rot` is the inverse of the frame used for the sweep above.
    return (corners @ rot + np.float32(center)).astype(np.float32)


def _fallback(gray, tag_id):
    _, template_contours = _template(tag_id)
    if not template_contours:
        return []
    # Otsu-relative levels cover different exposure; rank and bound the work.
    otsu, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    proposals = []
    for threshold in sorted({int(np.clip(otsu + d, 30, 225)) for d in (-40, -15, 15, 40)}):
        binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)[1]
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if not 35 <= area <= gray.size * 0.08:
                continue
            rect = cv2.minAreaRect(contour)
            if min(rect[1]) < 6:
                continue
            square = None
            for reference in template_contours:
                score = cv2.matchShapes(contour, reference, cv2.CONTOURS_MATCH_I1, 0)
                if score > 0.15:
                    continue
                # Fit the shape only for contours that can plausibly be the tag.
                if square is None:
                    square = _data_square(contour)
                # Existing route: the bright blob's minimum-area rectangle.
                transform = cv2.getPerspectiveTransform(
                    cv2.boxPoints(cv2.minAreaRect(reference)), cv2.boxPoints(rect))
                proposals.append((score, cv2.perspectiveTransform(
                    _SQUARE.reshape(1, 4, 2), transform)[0]))
                # Data-area route: independent of the rectangle's accidental
                # aspect ratio, so patterns such as ID 7 also work at any angle.
                if square is not None:
                    proposals.append((score, _square_quad(
                        square[2], square[0] * 1.5, square[1])))
    proposals.sort(key=lambda item: item[0])
    seen = []
    for _, quad in proposals[:24]:
        if not _valid_quad(quad, gray.shape):
            continue
        if any(np.max(np.linalg.norm(quad - prior, axis=1)) < 1.5 for prior in seen):
            continue
        seen.append(quad)
        for factor in (1.0, 1.05, 0.95):
            candidate = (quad - quad.mean(axis=0)) * factor + quad.mean(axis=0)
            if not _valid_quad(candidate, gray.shape):
                continue
            canonical = _decode_proposal(gray, candidate, tag_id)
            if canonical is None:
                continue
            refined = _align(gray, canonical, tag_id)
            if refined is not None:
                return [Marker(refined, tag_id, "inner_pattern")]
    return []


def detect_markers(image, expected_id=TAG_ID, *, allow_fallback=True):
    """Return current-frame markers, preferring unchanged standard ArUco detection.

    Missing-margin recovery is bounded and requires a specified dictionary ID.
    Strong perspective, occlusion or low contrast can still cause a rejection.
    """
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detector().detectMarkers(gray)
    markers = ([] if ids is None else
               [Marker(c.reshape(4, 2).astype(np.float32), int(i), "standard")
                for c, i in zip(corners, ids.flatten())])
    if expected_id is None or any(m.tag_id == expected_id for m in markers):
        return markers
    if allow_fallback and 0 <= expected_id < 50:
        # Large phone frames can fragment diagonal white-cell connections.
        # Propose at a bounded pyramid level, but validate and align original
        # pixels before returning original-resolution coordinates.
        height, width = gray.shape
        if max(height, width) > 1200:
            ratio = 960.0 / max(height, width)
            small = cv2.resize(gray, (round(width * ratio), round(height * ratio)),
                               interpolation=cv2.INTER_AREA)
            for marker in detect_markers(small, expected_id):
                if marker.tag_id != expected_id:
                    continue
                scale = np.float32([width / small.shape[1], height / small.shape[0]])
                mapped = (marker.corners + 0.5) * scale - 0.5
                refined = _align(gray, mapped, expected_id)
                if refined is not None:
                    markers.append(Marker(refined, expected_id, marker.method + "_pyramid"))
                    return markers
        markers.extend(_fallback(gray, expected_id))
    return markers
