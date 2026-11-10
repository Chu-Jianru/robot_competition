"""Offline review geometry must respect manual corners and real floor colors."""
import unittest

import cv2
import numpy as np

from analyze_field_photos import analyze, ground_side
from field_vision import image_to_world
from evaluate_field_labels import match_points


class PhotoReviewChecks(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((600, 600, 3), np.uint8)
        self.image[50:300, 50:550] = (0, 220, 240)
        self.image[300:550, 50:550] = (240, 80, 0)

    def test_manual_corners_orient_yellow_positive_even_when_photo_rotated(self):
        for turns in range(4):
            with self.subTest(turns=turns):
                image = np.ascontiguousarray(np.rot90(self.image, turns))
                field, tags, proposals, view = analyze(image, {
                    'field_corners': [[50, 50], [549, 50], [549, 549], [50, 549]]})
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                ys, xs = np.where(cv2.inRange(hsv, (15, 80, 40), (40, 255, 255)) > 0)
                world = image_to_world(field['homography'], [[xs.mean(), ys.mean()]])[0]
                self.assertGreater(world[1], .2)
                self.assertEqual(field['field_source'], 'manual_corners')
                self.assertEqual(view.shape[:2], (1200, 1200))
                self.assertEqual(tags, [])
                self.assertEqual(proposals, [])

    def test_crossed_manual_corners_rejected(self):
        with self.assertRaises(ValueError):
            analyze(self.image, {'field_corners': [[50, 50], [549, 549], [549, 50], [50, 549]]})

    def test_ground_vote_does_not_assign_empty_background_to_blue(self):
        image = np.zeros((100, 100, 3), np.uint8)
        self.assertEqual(ground_side(image, {'bbox_px': [40, 40, 20, 20]})[0], 'unknown')
        image[:] = (240, 80, 0)
        self.assertEqual(ground_side(image, {'bbox_px': [40, 40, 20, 20]})[0], 'blue')

    def test_merged_region_can_match_only_one_block_but_cover_two(self):
        contour = [[0, 0], [40, 0], [40, 20], [0, 20]]
        matches, covered = match_points([contour], [[10, 10], [30, 10]])
        self.assertEqual(len(matches), 1)
        self.assertEqual(covered, [0, 1])

    def test_matching_reassigns_to_avoid_greedy_miss(self):
        wide = [[0, 0], [40, 0], [40, 20], [0, 20]]
        narrow = [[0, 0], [20, 0], [20, 20], [0, 20]]
        matches, _ = match_points([wide, narrow], [[10, 10], [30, 10]], 0)
        self.assertEqual(len(matches), 2)

    def test_match_tolerance_is_original_pixel_distance(self):
        square = [[0, 0], [20, 0], [20, 20], [0, 20]]
        matches, covered = match_points([square], [[24, 10], [24.1, 10]], 4)
        self.assertEqual(len(matches), 1)
        self.assertEqual(covered, [0])


if __name__ == '__main__':
    unittest.main()
