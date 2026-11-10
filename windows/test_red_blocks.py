"""Independent pixel detector checks; no camera or field calibration."""
import sys
from dataclasses import asdict, replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from red_blocks import (RedBlockConfig, calibrate_red_config, detect_red_blocks,
                        load_red_config, save_red_config)


class RedBlockChecks(unittest.TestCase):
    def test_saved_shadow_profile_does_not_add_legacy_photo_candidates(self):
        profile = load_red_config(Path(__file__).with_name('red_block_profile.json'))
        primary = replace(profile, shadow_min_value=None)
        folder = Path(__file__).with_name('test_data') / 'red_blocks'
        # A user's existing calibration can itself admit wheel fragments. The
        # optional dark pass must not add candidates to these historical photos.
        for path in sorted(folder.glob('*.png')):
            with self.subTest(path=path.name):
                image = cv2.imdecode(np.fromfile(str(path), np.uint8), 1)
                self.assertLessEqual(len(detect_red_blocks(image, profile)),
                                     len(detect_red_blocks(image, primary)))

    def test_shadow_pass_finds_multiple_dim_blocks_and_rejects_round_fragment(self):
        hsv = np.full((160, 240, 3), (0, 0, 150), np.uint8)
        hsv[20:40, 20:40] = (4, 200, 70)
        hsv[80:110, 70:100] = (176, 220, 80)
        cv2.circle(hsv, (180, 50), 15, (4, 200, 70), -1)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        config = RedBlockConfig(min_value=103, min_rectangularity=.72)
        self.assertEqual(detect_red_blocks(image, config), [])
        found = detect_red_blocks(image, replace(config, shadow_min_value=40))
        self.assertEqual(len(found), 2)
        for point in ((29.5, 29.5), (84.5, 94.5)):
            self.assertTrue(any(np.linalg.norm(np.array(d['center_px']) - point) < 1 for d in found))

    def test_shadow_pass_deduplicates_bright_part_of_dim_face(self):
        hsv = np.full((100, 120, 3), (0, 0, 150), np.uint8)
        hsv[20:60, 20:60] = (4, 200, 70)
        hsv[25:45, 25:45] = (4, 200, 170)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        config = RedBlockConfig(min_value=103, shadow_min_value=40)
        found = detect_red_blocks(image, config)
        self.assertEqual(len(found), 1)
        self.assertGreater(found[0]['area_px'], 1400)

    def test_shadows_survive_calibration_and_profile_round_trip(self):
        sample = np.full((30, 30, 3), (25, 25, 55), np.uint8)
        config = calibrate_red_config(sample, RedBlockConfig(min_value=103, shadow_min_value=40))
        self.assertLessEqual(config.shadow_min_value, config.min_value)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'profile.json'
            save_red_config(path, config)
            self.assertEqual(load_red_config(path), config)

    def test_both_red_hue_ranges_on_gray_background_without_field(self):
        hsv = np.zeros((120, 200, 3), np.uint8)
        hsv[:, :, 2] = 140
        hsv[20:40, 20:40] = (3, 230, 240)
        hsv[60:80, 80:100] = (176, 220, 230)
        hsv[20:40, 140:160] = (110, 230, 240)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        with patch.dict(sys.modules, {"field_vision": None, "scene_vision": None}):
            detections = detect_red_blocks(image)
        self.assertEqual(len(detections), 2)
        for x, y in ((29.5, 29.5), (89.5, 69.5)):
            self.assertTrue(any(np.linalg.norm(np.array(d["center_px"]) - (x, y)) < 1
                                for d in detections))
        self.assertTrue(all("xy" not in d and "own" not in d for d in detections))

    def test_detector_thresholds_can_be_tuned_without_a_field(self):
        image = np.full((120, 200, 3), 140, np.uint8)
        cv2.rectangle(image, (20, 20), (30, 30), (0, 0, 240), -1)
        self.assertEqual(len(detect_red_blocks(image)), 1)
        self.assertEqual(detect_red_blocks(image, RedBlockConfig(min_area_px=200)), [])

    def test_square_survives_round_wheel_and_fragment_rejection(self):
        image = np.full((220, 320, 3), 140, np.uint8)
        cv2.rectangle(image, (30, 30), (60, 60), (0, 0, 240), -1)
        cv2.circle(image, (160, 55), 25, (0, 0, 240), -1)
        cv2.line(image, (20, 130), (110, 170), (0, 0, 240), 9)
        cv2.rectangle(image, (210, 115), (225, 180), (0, 0, 240), -1)
        cv2.rectangle(image, (185, 140), (250, 155), (0, 0, 240), -1)
        filtered = detect_red_blocks(image)
        self.assertEqual(len(filtered), 1)
        self.assertLess(np.linalg.norm(np.array(filtered[0]["center_px"]) - (45, 45)), 1)
        self.assertEqual(len(detect_red_blocks(image, RedBlockConfig(shape_filter=False))), 4)

    def test_squares_at_different_rotations_scales_and_positions(self):
        for side in (8, 16, 32):
            for angle in (0, 17, 45, 73):
                for center in ((45, 155), (210, 50)):
                    with self.subTest(side=side, angle=angle, center=center):
                        image = np.full((220, 300, 3), 140, np.uint8)
                        corners = cv2.boxPoints((center, (side, side), angle))
                        cv2.fillConvexPoly(image, np.rint(corners).astype(np.int32), (0, 0, 240))
                        found = detect_red_blocks(image)
                        self.assertEqual(len(found), 1)
                        self.assertLess(np.linalg.norm(np.array(found[0]["center_px"]) - center), 1)

    def test_real_photos_keep_only_the_labeled_block(self):
        folder = Path(__file__).with_name("test_data") / "red_blocks"
        for name, block_center in (("block_and_wheels_a.png", (344, 180)),
                                   ("block_and_wheels_b.png", (338, 187)),
                                   ("wheels_only.png", None)):
            with self.subTest(name=name):
                image = cv2.imdecode(np.fromfile(folder / name, dtype=np.uint8), cv2.IMREAD_COLOR)
                found = detect_red_blocks(image)
                self.assertEqual(len(found), int(block_center is not None))
                if block_center is not None:
                    x, y, width, height = found[0]["bbox_px"]
                    self.assertTrue(x <= block_center[0] < x + width)
                    self.assertTrue(y <= block_center[1] < y + height)

    def test_calibration_uses_the_selected_block_not_the_largest_red_object(self):
        image = np.full((180, 260, 3), 150, np.uint8)
        cv2.rectangle(image, (20, 20), (40, 40), (100, 100, 220), -1)
        cv2.rectangle(image, (120, 20), (230, 130), (0, 0, 240), -1)
        with patch.dict(sys.modules, {"field_vision": None}):
            config = calibrate_red_config(image, roi=(25, 25, 10, 10))
        self.assertGreater(config.min_area_px, 20)
        self.assertLess(config.min_area_px, 80)
        self.assertLess(config.max_saturation, 255)
        self.assertTrue(config.close_gaps)

    def test_bad_calibration_selection_is_rejected(self):
        image = np.full((80, 100, 3), 150, np.uint8)
        with self.assertRaises(ValueError):
            calibrate_red_config(image, roi=(10, 10, 20, 20))

    def test_calibration_profile_round_trip(self):
        image = np.full((30, 30, 3), (100, 100, 220), np.uint8)
        config = calibrate_red_config(image)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "实物红块.json"
            save_red_config(path, config)
            self.assertEqual(asdict(load_red_config(path)), asdict(config))

    def test_calibrated_profile_on_both_photo_sessions(self):
        folder = Path(__file__).with_name("test_data") / "red_blocks"
        reference = cv2.imdecode(np.fromfile(folder / "block_and_wheels_a.png", dtype=np.uint8),
                                 cv2.IMREAD_COLOR)
        config = calibrate_red_config(reference, roi=(335, 170, 15, 12))
        cases = [("block_and_wheels_a.png", (344, 180)),
                 ("block_and_wheels_b.png", (338, 187)), ("wheels_only.png", None),
                 ("session_b_01.png", (366, 206)), ("session_b_02.png", (435, 223)),
                 ("session_b_03.png", (409, 228)), ("session_b_04.png", (386, 240)),
                 ("session_b_05.png", (391, 296)), ("session_b_06.png", (410, 220))]
        for name, point in cases:
            with self.subTest(name=name):
                image = cv2.imdecode(np.fromfile(folder / name, dtype=np.uint8), cv2.IMREAD_COLOR)
                result = detect_red_blocks(image, config)
                self.assertEqual(len(result), int(point is not None))
                if point is not None:
                    x, y, width, height = result[0]["bbox_px"]
                    self.assertTrue(x <= point[0] < x + width and y <= point[1] < y + height)


if __name__ == "__main__":
    unittest.main()
