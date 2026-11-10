"""Offline checks for converting ONNX boxes into pipeline detections."""
import unittest

import numpy as np

from onnx_blocks import boxes_to_detections


class BoxesToDetectionsTest(unittest.TestCase):
    def test_centre_bbox_and_contour(self):
        boxes = np.array([[10, 20, 30, 40]], np.float32)
        scores = np.array([0.8], np.float32)
        result = boxes_to_detections(boxes, scores, [0], 100, 100)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["center_px"], (20.0, 30.0))
        self.assertEqual(result[0]["bbox_px"], (10, 20, 20, 20))
        self.assertEqual(result[0]["contour_px"].shape, (4, 1, 2))
        self.assertAlmostEqual(result[0]["score"], 0.8, places=5)

    def test_boxes_are_clamped_to_the_image(self):
        boxes = np.array([[-5, -5, 20, 20]], np.float32)
        result = boxes_to_detections(boxes, np.array([0.9], np.float32), [0], 100, 100)
        self.assertEqual(result[0]["bbox_px"], (0, 0, 20, 20))

    def test_degenerate_boxes_are_skipped(self):
        boxes = np.array([[95, 95, 97, 99]], np.float32)   # 2 px wide
        result = boxes_to_detections(boxes, np.array([0.9], np.float32), [0], 100, 100)
        self.assertEqual(result, [])

    def test_multiple_boxes_keep_their_scores(self):
        boxes = np.array([[0, 0, 10, 10], [50, 50, 60, 60]], np.float32)
        scores = np.array([0.7, 0.6], np.float32)
        result = boxes_to_detections(boxes, scores, [0, 1], 100, 100)
        self.assertEqual([round(item["score"], 2) for item in result], [0.7, 0.6])


if __name__ == "__main__":
    unittest.main()
