"""Offline checks for photo import and orientation canonicalisation."""
import unittest

import cv2
import numpy as np

from import_photos import canonical, centroids, choose_rotation


def field():
    """Yellow on top, blue at the bottom, one red block."""
    image = np.zeros((200, 200, 3), np.uint8)
    image[:100] = (0, 220, 240)      # yellow (BGR)
    image[100:] = (240, 60, 0)       # blue
    image[40:56, 40:56] = (0, 0, 220)
    return image


class RotationTest(unittest.TestCase):
    def test_yellow_above_needs_no_rotation(self):
        self.assertEqual(choose_rotation((100, 150), (100, 50)), 0)

    def test_yellow_below_needs_half_turn(self):
        self.assertEqual(choose_rotation((100, 50), (100, 150)), 180)

    def test_yellow_right_needs_counter_clockwise(self):
        self.assertEqual(choose_rotation((50, 100), (150, 100)), 90)

    def test_yellow_left_needs_clockwise(self):
        self.assertEqual(choose_rotation((150, 100), (50, 100)), 270)

    def test_missing_colour_falls_back_to_no_rotation(self):
        self.assertIsNone(choose_rotation(None, (10, 10)))
        self.assertIsNone(choose_rotation((10, 10), None))

    def test_canonical_puts_yellow_on_top_for_all_four_orientations(self):
        base = field()
        for turns in range(4):
            rotated = np.ascontiguousarray(np.rot90(base, turns))
            fixed, _degrees = canonical(rotated)
            blue_xy, yellow_xy = centroids(fixed)
            self.assertIsNotNone(blue_xy)
            self.assertIsNotNone(yellow_xy)
            self.assertLess(yellow_xy[1], blue_xy[1],
                            f"turns={turns} 之后黄区仍在蓝区下方")


if __name__ == "__main__":
    unittest.main()
