"""Offline checks for converting ONNX boxes into pipeline detections."""
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from onnx_blocks import boxes_to_detections, build_detector


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

    def test_area_px_is_present_for_the_millimetre_filter(self):
        # Regression: filter_by_size_mm/bench_check read `area_px`; without it the
        # bench preview crashed with KeyError as soon as ONNX was selected.
        from red_blocks import filter_by_size_mm
        boxes = np.array([[10, 20, 30, 40]], np.float32)
        result = boxes_to_detections(boxes, np.array([0.8], np.float32), [0], 100, 100)
        self.assertEqual(result[0]["area_px"], 400.0)
        kept, rejected = filter_by_size_mm(result, 0.05, 12, 32)
        self.assertEqual(len(kept) + len(rejected), 1)
        self.assertIn("side_mm", kept[0] if kept else rejected[0][0])


class BuildDetectorTest(unittest.TestCase):
    """The bench preview and the competition run must pick the same detector."""

    def test_auto_uses_hsv_when_no_model_exists(self):
        missing = Path("no_such_model.onnx")
        detector, label = build_detector("auto", missing, quiet=True)
        self.assertIsNone(detector)
        self.assertEqual(label, "HSV")

    def test_explicit_onnx_without_model_raises(self):
        with self.assertRaises(RuntimeError):
            build_detector("onnx", Path("no_such_model.onnx"), quiet=True)

    def test_explicit_hsv_wins_even_with_a_model(self):
        with mock.patch("onnx_blocks.OnnxBlockDetector") as fake:
            detector, label = build_detector("hsv", Path("whatever.onnx"), quiet=True)
        self.assertIsNone(detector)
        self.assertEqual(label, "HSV")
        fake.assert_not_called()

    def test_auto_builds_onnx_when_the_model_exists(self):
        with mock.patch("onnx_blocks.Path.exists", return_value=True), \
                mock.patch("onnx_blocks.OnnxBlockDetector") as fake:
            fake.return_value.provider = "CPUExecutionProvider"
            detector, label = build_detector("auto", Path("model.onnx"), quiet=True)
        fake.assert_called_once()
        self.assertIsNotNone(detector)
        self.assertIn("ONNX", label)


if __name__ == "__main__":
    unittest.main()
