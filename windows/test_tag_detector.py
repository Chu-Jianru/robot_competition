"""Offline tag regression tests. No camera, serial port or motors are opened."""
import unittest
from pathlib import Path

import cv2
import numpy as np

from bench_check import find_tag
from field_view import FieldCalibration
from pose_tag import detect
from tag_detector import TAG_ID, detect_markers, detector_parameters


PHOTO = Path(__file__).resolve().parent / 'captures_hi/hi_020725_700_720x960.png'
# Historical capture: the car wore the previous ID 0 marker, so photo checks use 0.
PHOTO_TAG_ID = 0
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def synthetic(angle=0, *, margin=False, tag_id=TAG_ID):
    marker = cv2.aruco.generateImageMarker(DICTIONARY, tag_id, 72)
    marker = np.where(marker > 128, 220, 65).astype(np.uint8)
    source = np.float32([[0, 0], [71, 0], [71, 71], [0, 71]])
    transform = cv2.getRotationMatrix2D((35.5, 35.5), angle, .65)
    transform[:, 2] += [204, 144]
    truth = cv2.transform(source[None], transform)[0]
    homography = cv2.getPerspectiveTransform(source, truth)
    if margin:
        marker = cv2.copyMakeBorder(marker, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=245)
        homography = homography @ np.float64([[1, 0, -12], [0, 1, -12], [0, 0, 1]])
    frame = cv2.warpPerspective(marker, homography, (480, 360), borderValue=65)
    return cv2.GaussianBlur(frame, (3, 3), .6), truth


class TagChecks(unittest.TestCase):
    def test_no_margin_rotations_recover_geometry(self):
        for angle in (0, 20, 70, 130, 230, 310):
            with self.subTest(angle=angle):
                frame, truth = synthetic(angle)
                self.assertEqual(detect_markers(frame, allow_fallback=False), [])
                found = detect_markers(frame)
                self.assertEqual([(m.tag_id, m.method) for m in found],
                                 [(TAG_ID, 'inner_pattern')])
                # Canonical corner order matters for heading; centre alone is insufficient.
                self.assertLess(float(np.max(np.linalg.norm(found[0].corners - truth, axis=1))), .75)

    def test_white_margin_preserves_standard_corners(self):
        baseline = cv2.aruco.ArucoDetector(DICTIONARY, detector_parameters())
        for angle in (0, 20, 70, 130, 230, 310):
            with self.subTest(angle=angle):
                frame, _ = synthetic(angle, margin=True)
                corners, ids, _ = baseline.detectMarkers(frame)
                self.assertIsNotNone(ids)
                found = detect_markers(frame)
                self.assertEqual(found[0].method, 'standard')
                np.testing.assert_array_equal(found[0].corners, corners[0].reshape(4, 2))

    @unittest.skipUnless(PHOTO.exists(), 'Local phone regression photo is not present')
    def test_phone_photo_rotations_and_exposure(self):
        frame = cv2.imread(str(PHOTO))
        self.assertEqual(detect_markers(frame, PHOTO_TAG_ID, allow_fallback=False), [])
        for rotation in range(4):
            for gain in (.75, 1., 1.2):
                with self.subTest(rotation=rotation, gain=gain):
                    image = np.clip(np.rot90(frame, rotation).astype(np.float32) * gain,
                                    0, 255).astype(np.uint8)
                    found = detect_markers(image, PHOTO_TAG_ID)
                    self.assertEqual([(m.tag_id, m.method) for m in found],
                                     [(PHOTO_TAG_ID, 'inner_pattern')])
                    self.assertTrue(36 < np.mean(np.linalg.norm(
                        found[0].corners - np.roll(found[0].corners, 1, axis=0), axis=1)) < 44)

    def test_other_id_not_recovered_as_configured_id(self):
        for tag_id in (1, 17, 49):
            for margin in (False, True):
                with self.subTest(tag_id=tag_id, margin=margin):
                    frame, _ = synthetic(20, margin=margin, tag_id=tag_id)
                    self.assertFalse(any(m.tag_id == TAG_ID for m in detect_markers(frame)))
                    self.assertEqual(find_tag(frame), (None, None, None))

    def test_noise_and_existing_negative_photos(self):
        rng = np.random.default_rng(20261103)
        for _ in range(8):
            noise = rng.integers(0, 256, (240, 320), dtype=np.uint8)
            self.assertEqual(detect_markers(noise), [])
        directory = Path(__file__).resolve().parent / 'test_data/red_blocks'
        for path in sorted(directory.glob('*.png')):
            with self.subTest(path=path.name):
                self.assertEqual(find_tag(cv2.imread(str(path))), (None, None, None))

    def test_occlusion_rejected_instead_of_inventing_bits(self):
        frame, truth = synthetic(0)
        center = np.round(truth.mean(axis=0)).astype(int)
        cv2.rectangle(frame, tuple(center - 8), tuple(center + 8), 65, -1)
        self.assertEqual(detect_markers(frame), [])

    def test_pose_path_uses_recovery_and_keeps_size_check(self):
        frame, truth = synthetic(20)
        calib = FieldCalibration(np.float32([[0, 0], [480, 0], [480, 360], [0, 360]]),
                                 size=480, field_mm=480)
        side = float(np.mean(np.linalg.norm(truth - np.roll(truth, 1, axis=0), axis=1)))
        pose, reason = detect(frame, calib, tag_mm=side)
        self.assertEqual(reason, 'ok')
        self.assertLess(float(np.linalg.norm([pose.x_mm - truth[:, 0].mean(),
                                             pose.y_mm - (480 - truth[:, 1].mean())])), .1)
        self.assertEqual(detect(frame, calib, tag_mm=100)[1], 'side_mismatch')


if __name__ == '__main__':
    unittest.main()
