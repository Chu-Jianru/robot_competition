import unittest

import cv2
import numpy as np

from calibrated_blocks import detect_calibrated_blocks
from photo_tag import detect_photo_markers, validate_original
from analyze_field_photos import DATA, read_image
from tag_detector import _exact_cells, _sample, _template


H = np.float64([[.001, 0, -.64], [0, -.001, .64], [0, 0, 1]])


def ground():
    image = np.zeros((1280, 1280, 3), np.uint8)
    image[:640] = (0, 230, 240)
    image[640:] = (240, 70, 0)
    return image


class CalibratedChecks(unittest.TestCase):
    def test_empty_floor_and_missing_calibration_do_not_invent_blocks(self):
        self.assertEqual(detect_calibrated_blocks(ground(), H), [])
        self.assertEqual(detect_calibrated_blocks(ground(), None), [])

    def test_touching_blocks_separate_without_point_labels(self):
        image = ground()
        image[300:320, 300:340] = (15, 20, 170)
        blocks = detect_calibrated_blocks(image, H)
        self.assertEqual(len(blocks), 2)
        self.assertGreater(abs(blocks[0]['center_px'][0]-blocks[1]['center_px'][0]), 12)

    def test_large_connected_pile_is_not_discarded_as_one_oversize_block(self):
        image = ground()
        image[250:310, 300:420] = (15, 20, 170)
        blocks = detect_calibrated_blocks(image, H)
        self.assertGreater(len(blocks), 1)
        self.assertTrue(all(b['split'] for b in blocks))
        # Uniform touching faces have no visible seams: do not assert an exact
        # instance count inferred only from synthetic silhouette area.

    def test_purple_reflection_band_is_admitted_only_near_blue_floor(self):
        color = cv2.cvtColor(np.uint8([[[160, 170, 120]]]), cv2.COLOR_HSV2BGR)[0, 0]
        for y, expected in [(300, 0), (800, 1)]:
            image = ground()
            image[y:y+20, 300:320] = color
            with self.subTest(y=y):
                self.assertEqual(len(detect_calibrated_blocks(image, H)), expected)

    def test_one_block_stays_one_under_rotation_and_dim_light(self):
        image = ground()
        image[300:320, 300:320] = (15, 20, 170)
        for dim in (1., .6):
            source = np.uint8(image*dim)
            for turns in range(4):
                frame = np.ascontiguousarray(np.rot90(source, turns))
                original = np.float32([[0, 0], [1279, 0], [1279, 1279], [0, 1279]])
                rotated = original.copy()
                for _ in range(turns):
                    rotated = np.column_stack([rotated[:, 1], 1279-rotated[:, 0]])
                world = cv2.getPerspectiveTransform(rotated, original)
                with self.subTest(dim=dim, turns=turns):
                    self.assertEqual(len(detect_calibrated_blocks(frame, H@world)), 1)

    def test_visual_chassis_excludes_wheels_but_keeps_nearby_ground_cube(self):
        image = ground()
        image[450:590, 450:630] = (220, 220, 220)
        image[490:550, 500:580] = (50, 140, 30)
        for x in (455, 605):
            for y in (455, 555):
                image[y:y+25, x:x+20] = (10, 15, 160)
        image[500:520, 650:670] = (15, 20, 170)
        blocks = detect_calibrated_blocks(image, H)
        self.assertEqual(len(blocks), 1)
        self.assertGreater(blocks[0]['center_px'][0], 640)

    def test_tag_recovery_does_not_accept_wrong_id_or_uniform_scene(self):
        self.assertEqual(detect_photo_markers(np.full((480, 640, 3), 120, np.uint8)), [])
        tile = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 1, 120)
        image = np.full((240, 240), 255, np.uint8)
        image[60:180, 60:180] = tile
        self.assertEqual(detect_photo_markers(image, 0), [])
        self.assertIsNone(validate_original(image, np.float32([[60, 60], [179, 60], [179, 179], [60, 179]]), 0))

    @unittest.skipUnless((DATA/'field_10.jpg').exists(), 'Local field photos not present')
    def test_photo10_recovery_validates_original_bits(self):
        image = read_image(DATA/'field_10.jpg')
        # The 2026-11-03 capture batch still carries the previous ID 0 marker.
        markers = detect_photo_markers(image, 0)
        self.assertEqual([m.tag_id for m in markers], [0])
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        template, _ = _template(0)
        self.assertTrue(_exact_cells(_sample(gray, markers[0].corners), template))
        self.assertIsNotNone(validate_original(gray, markers[0].corners, 0))

    def test_exposure_proposals_do_not_accept_noise(self):
        rng = np.random.default_rng(37)
        for _ in range(3):
            self.assertEqual(detect_photo_markers(rng.integers(0, 256, (240, 320), dtype=np.uint8)), [])


if __name__ == '__main__':
    unittest.main()
