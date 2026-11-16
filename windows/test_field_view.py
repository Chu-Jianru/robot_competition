"""Offline checks for unordered field calibration clicks."""

from itertools import permutations
import unittest

import cv2
import numpy as np

from field_view import FieldCalibration


class FieldCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.corners = np.float32([[18, 27], [161, 14], [182, 125], [29, 143]])
        self.square = np.empty((120, 120, 3), np.uint8)
        self.square[:60] = (10, 230, 240)
        self.square[60:] = (230, 50, 30)

    def test_every_click_order_in_all_four_field_directions(self):
        for turns in range(4):
            expected = np.roll(self.corners, turns, axis=0)
            projection = FieldCalibration(expected, size=120)
            image = cv2.warpPerspective(self.square, np.linalg.inv(projection.homography()), (200, 160))
            for order in permutations(range(4)):
                with self.subTest(turns=turns, order=order):
                    calib = FieldCalibration.from_clicks(image, expected[list(order)], size=120)
                    np.testing.assert_array_equal(calib.corners, expected)
                    self.assertEqual(calib.source_size, (200, 160))
                    self.assertEqual(calib.size, 120)

    def test_rejects_repeated_corner_and_missing_blue_yellow_seam(self):
        image = np.zeros((160, 200, 3), np.uint8)
        with self.assertRaisesRegex(ValueError, "四边形"):
            FieldCalibration.from_clicks(image, self.corners[[0, 1, 2, 2]])
        with self.assertRaisesRegex(ValueError, "蓝黄分界线"):
            FieldCalibration.from_clicks(image, self.corners)


if __name__ == "__main__":
    unittest.main()
