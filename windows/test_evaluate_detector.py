"""Offline checks for detector evaluation against hand labels."""
import unittest

import numpy as np

from evaluate_detector import iou, match_boxes, summary


class IouTest(unittest.TestCase):
    def test_identical_boxes(self):
        self.assertAlmostEqual(iou([0, 0, 10, 10], [0, 0, 10, 10]), 1.0)

    def test_disjoint_boxes(self):
        self.assertEqual(iou([0, 0, 10, 10], [20, 20, 10, 10]), 0.0)

    def test_partial_overlap(self):
        self.assertAlmostEqual(iou([0, 0, 10, 10], [5, 0, 10, 10]), 1 / 3, places=4)


class MatchTest(unittest.TestCase):
    def test_perfect_match(self):
        truth = [[0, 0, 10, 10], [50, 50, 10, 10]]
        tp, fp, fn, matched = match_boxes(truth, truth, 0.3)
        self.assertEqual((tp, fp, fn), (2, 0, 0))
        self.assertEqual(len(matched), 2)

    def test_extra_prediction_counts_as_false_positive(self):
        truth = [[0, 0, 10, 10]]
        predicted = [[0, 0, 10, 10], [100, 100, 10, 10]]
        self.assertEqual(match_boxes(predicted, truth, 0.3)[:3], (1, 1, 0))

    def test_missing_detection_counts_as_false_negative(self):
        truth = [[0, 0, 10, 10], [50, 50, 10, 10]]
        predicted = [[0, 0, 10, 10]]
        self.assertEqual(match_boxes(predicted, truth, 0.3)[:3], (1, 0, 1))

    def test_one_to_one_only(self):
        # Two predictions on the same block: one TP, one FP.
        truth = [[0, 0, 10, 10]]
        predicted = [[0, 0, 10, 10], [1, 1, 10, 10]]
        self.assertEqual(match_boxes(predicted, truth, 0.3)[:3], (1, 1, 0))

    def test_low_overlap_is_rejected(self):
        truth = [[0, 0, 10, 10]]
        predicted = [[9, 9, 10, 10]]
        self.assertEqual(match_boxes(predicted, truth, 0.3)[:3], (0, 1, 1))


class SummaryTest(unittest.TestCase):
    def test_metrics(self):
        precision, recall, f1 = summary({"tp": 8, "fp": 2, "fn": 4})
        self.assertAlmostEqual(precision, 0.8)
        self.assertAlmostEqual(recall, 8 / 12)
        self.assertAlmostEqual(f1, 2 * 0.8 * (8 / 12) / (0.8 + 8 / 12), places=6)

    def test_empty_counts_do_not_divide_by_zero(self):
        self.assertEqual(summary({"tp": 0, "fp": 0, "fn": 0}), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
