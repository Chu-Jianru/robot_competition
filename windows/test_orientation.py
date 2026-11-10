"""Offline checks for field-orientation detection (no camera)."""
import unittest

import cv2
import numpy as np

from orientation import (apply_rotation, centroids, choose_rotation,
                         decide_rotation, field_rotation)


def field():
    """Yellow on top, blue at the bottom, one red block."""
    image = np.zeros((200, 200, 3), np.uint8)
    image[:100] = (0, 220, 240)      # yellow (BGR)
    image[100:] = (240, 60, 0)       # blue
    image[40:56, 40:56] = (0, 0, 220)
    return image


class ChooseRotationTest(unittest.TestCase):
    def test_yellow_above_needs_no_rotation(self):
        self.assertEqual(choose_rotation((100, 150), (100, 50)), 0)

    def test_yellow_below_needs_half_turn(self):
        self.assertEqual(choose_rotation((100, 50), (100, 150)), 180)

    def test_yellow_right_needs_counter_clockwise(self):
        self.assertEqual(choose_rotation((50, 100), (150, 100)), 90)

    def test_yellow_left_needs_clockwise(self):
        self.assertEqual(choose_rotation((150, 100), (50, 100)), 270)

    def test_unknown_colours_report_nothing(self):
        self.assertIsNone(choose_rotation(None, (10, 10)))
        self.assertIsNone(choose_rotation((10, 10), None))
        self.assertIsNone(choose_rotation((10, 10), (12, 11)))


class FieldRotationTest(unittest.TestCase):
    def test_every_orientation_is_corrected_to_yellow_on_top(self):
        base = field()
        for turns in range(4):
            rotated = np.ascontiguousarray(np.rot90(base, turns))
            degrees = field_rotation(rotated)
            self.assertIsNotNone(degrees, f"turns={turns}")
            fixed = apply_rotation(rotated, degrees)
            blue_xy, yellow_xy = centroids(fixed)
            self.assertLess(yellow_xy[1], blue_xy[1], f"turns={turns}")

    def test_blank_frame_has_no_orientation(self):
        self.assertIsNone(field_rotation(np.full((100, 100, 3), 128, np.uint8)))


class DecideRotationTest(unittest.TestCase):
    def test_majority_wins(self):
        self.assertEqual(decide_rotation([90, 90, 90, 0]), 90)

    def test_tie_reports_nothing(self):
        self.assertIsNone(decide_rotation([90, 270]))

    def test_empty_and_unknown_report_nothing(self):
        self.assertIsNone(decide_rotation([]))
        self.assertIsNone(decide_rotation([None, None]))


if __name__ == "__main__":
    unittest.main()
