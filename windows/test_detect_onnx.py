"""Offline checks for the ONNX inference helpers (no model needed)."""
import unittest

import numpy as np

from detect_onnx import (decode, letterbox, nms, restore, to_input,
                         xywh_to_xyxy)


class LetterboxTest(unittest.TestCase):
    def test_aspect_is_kept_and_padding_is_centred(self):
        image = np.zeros((50, 100, 3), np.uint8)
        padded, ratio, (pad_x, pad_y) = letterbox(image, (200, 200))
        self.assertEqual(padded.shape[:2], (200, 200))
        self.assertAlmostEqual(ratio, 2.0)
        self.assertEqual(pad_x, 0)
        self.assertEqual(pad_y, 50)

    def test_input_tensor_is_rgb_nchw_normalised(self):
        image = np.full((10, 20, 3), 255, np.uint8)
        tensor = to_input(image)
        self.assertEqual(tensor.shape, (1, 3, 10, 20))
        self.assertAlmostEqual(float(tensor.max()), 1.0)


class DecodeTest(unittest.TestCase):
    def test_low_confidence_rows_are_dropped(self):
        prediction = np.zeros((1, 5, 10), np.float32)   # 4 box + 1 class, 10 anchors
        prediction[0, :4, 0] = [50, 50, 20, 20]
        prediction[0, 4, 0] = 0.9
        prediction[0, :4, 1] = [10, 10, 20, 20]
        prediction[0, 4, 1] = 0.1
        boxes, scores = decode(prediction, 0.5)
        self.assertEqual(len(boxes), 1)
        self.assertAlmostEqual(float(scores[0]), 0.9, places=4)

    def test_anchors_first_layout_is_handled(self):
        prediction = np.zeros((1, 10, 5), np.float32)
        prediction[0, 0, :4] = [50, 50, 20, 20]
        prediction[0, 0, 4] = 0.8
        boxes, scores = decode(prediction, 0.5)
        self.assertEqual(len(boxes), 1)
        self.assertAlmostEqual(float(boxes[0, 0]), 50.0)

    def test_empty_prediction_returns_nothing(self):
        boxes, scores = decode(np.zeros((1, 5, 0), np.float32), 0.5)
        self.assertEqual(len(boxes), 0)
        self.assertEqual(len(scores), 0)

    def test_xywh_becomes_xyxy(self):
        out = xywh_to_xyxy(np.array([[50, 50, 20, 20]], np.float32))
        self.assertEqual(out.tolist(), [[40, 40, 60, 60]])

    def test_restore_only_shifts_the_centre(self):
        # cx, cy, w, h in padded pixels; scale 2, pad (20, 10).
        boxes = np.array([[140, 90, 40, 20]], np.float32)
        out = restore(boxes, 2.0, (20, 10))
        self.assertAlmostEqual(out[0, 0], 60.0)
        self.assertAlmostEqual(out[0, 1], 40.0)
        self.assertAlmostEqual(out[0, 2], 20.0)   # width must not lose the pad
        self.assertAlmostEqual(out[0, 3], 10.0)


class NmsTest(unittest.TestCase):
    def test_overlapping_boxes_collapse(self):
        boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11]], np.float32)
        scores = np.array([0.9, 0.8], np.float32)
        kept = nms(boxes, scores, 0.5)
        self.assertEqual(len(kept), 1)

    def test_separate_boxes_survive(self):
        boxes = np.array([[0, 0, 10, 10], [100, 100, 110, 110]], np.float32)
        scores = np.array([0.9, 0.8], np.float32)
        self.assertEqual(len(nms(boxes, scores, 0.5)), 2)

    def test_empty_input(self):
        self.assertEqual(nms(np.zeros((0, 4)), np.zeros((0,)), 0.5), [])


if __name__ == "__main__":
    unittest.main()
