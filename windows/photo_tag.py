"""Offline tag proposal recovery; verify unmodified source pixels strictly.

Exposure/pyramid variants only propose geometry. Returned tag must decode with
zero correction and match all 36 data/border cells on original grayscale pixels.
This deliberately leaves unrecoverable blur/geometry as missing, not a pose.
"""
import cv2
import numpy as np

from tag_detector import (TAG_ID, Marker, detect_markers, _align, _decode_proposal,
                          _exact_cells, _sample, _template, _valid_quad)


def validate_original(gray, quad, expected_id):
    if not _valid_quad(quad, gray.shape):
        return None
    canonical = _decode_proposal(gray, quad, expected_id)
    template, _ = _template(expected_id)
    if canonical is None or not _exact_cells(_sample(gray, canonical), template):
        return None
    refined = _align(gray, canonical, expected_id)
    if refined is not None:
        return refined
    # ECC can move a correct initial corner by <1 pixel and flip a marginal
    # border cell. Retain the already-strict original quad only if the full
    # pattern aligns strongly and the alignment asks for a small displacement.
    mask = np.zeros((72, 72), np.uint8)
    mask[6:66, 6:66] = 255
    try:
        correlation, warp = cv2.findTransformECC(
            template, _sample(gray, canonical), np.eye(2, 3, dtype=np.float32),
            cv2.MOTION_AFFINE, (3, 25, 1e-4), mask, 3)
    except cv2.error:
        return None
    probes = np.float32([[[0, 0], [71, 0], [71, 71], [0, 71]]])
    displacement = np.max(np.linalg.norm(cv2.transform(probes, warp)-probes, axis=2))
    if correlation >= .90 and displacement <= 5:
        return canonical
    return None


def detect_photo_markers(image, expected_id=TAG_ID):
    existing = [m for m in detect_markers(image, expected_id) if m.tag_id == expected_id]
    if existing:
        return existing
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    for ratio in (.5, 1., 1.5):
        small = cv2.resize(gray, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)
        variant = np.uint8(np.clip((small/255.)**1.5*255, 0, 255))
        scale = np.float32([variant.shape[1]/gray.shape[1], variant.shape[0]/gray.shape[0]])
        for marker in detect_markers(variant, expected_id):
            if marker.tag_id != expected_id:
                continue
            mapped = (marker.corners+.5)/scale-.5
            accepted = validate_original(gray, mapped, expected_id)
            if accepted is not None:
                return [Marker(accepted, expected_id, 'exposure_proposal_strict_original')]
    return []
