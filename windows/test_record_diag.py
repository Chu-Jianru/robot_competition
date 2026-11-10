"""Offline checks for the record_diag instrumentation.

No camera, no chassis: synthetic field frames, synthetic rows, and a tag-free
bench frame that must not report a block count when the millimetre scale is
unavailable.
"""
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from field_view import FieldCalibration
from record_diag import (bench_frame, field_frame, match_jitter, summarize,
                         write_outputs)
from tag_detector import TAG_ID


def field_image():
    image = np.zeros((1000, 1000, 3), np.uint8)
    image[20:512, 20:980] = (10, 230, 240)          # yellow (BGR) north half
    image[512:980, 20:980] = (230, 50, 30)          # blue south half
    image[740:800, 460:520] = (255, 255, 255)       # tag backing
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), TAG_ID, 40)
    image[750:790, 470:510] = marker[:, :, None]
    image[300:316, 460:476] = (0, 0, 220)           # red block, yellow half
    image[700:716, 460:476] = (0, 0, 220)           # red block, blue half
    return image


class FieldFrameTest(unittest.TestCase):
    def setUp(self):
        self.calib = FieldCalibration(
            np.float32([[20, 20], [980, 20], [980, 980], [20, 980]]),
            source_size=(1000, 1000),
        )

    def test_counters_and_blocks_are_exposed(self):
        result = field_frame(field_image(), self.calib, 4.0)
        self.assertTrue(result["valid"], result["label"])
        self.assertTrue(result["tag_ok"])
        self.assertTrue(result["seam_ok"])
        self.assertEqual(result["blocks"], len(result["blocks_json"]))
        self.assertGreaterEqual(result["blocks"], 2)
        self.assertEqual(result["own"], sum(1 for block in result["blocks_json"] if block[4]))

    def test_missing_seam_is_reported_as_a_whole_frame_failure(self):
        image = field_image()
        image[20:980, 20:980] = (10, 230, 240)
        result = field_frame(image, self.calib, 4.0)
        self.assertFalse(result["valid"])
        self.assertEqual(result["blocks"], 0)
        self.assertIn("seam", result["label"])

    def test_missing_tag_is_reported_without_inventing_blocks(self):
        image = field_image()
        image[740:800, 460:520] = (230, 50, 30)
        result = field_frame(image, self.calib, 4.0)
        self.assertFalse(result["valid"])
        self.assertFalse(result["tag_ok"])
        self.assertTrue(result["label"].startswith(f"ID {TAG_ID}:"))


class BenchFrameTest(unittest.TestCase):
    def test_no_tag_means_no_millimetre_scale_and_no_count(self):
        image = np.full((480, 640, 3), 120, np.uint8)
        image[300:320, 300:320] = (0, 0, 220)
        result = bench_frame(image, None, 50.0, 0, 12.0, 32.0)
        self.assertFalse(result["valid"])
        self.assertFalse(result["tag_ok"])
        self.assertEqual(result["blocks"], 0)
        self.assertEqual(result["blocks_json"], [])
        self.assertIn("tag", result["label"])


class JitterTest(unittest.TestCase):
    def test_matches_within_limit_and_counts_the_rest(self):
        previous = [[0.0, 0.0, 0, 0, True], [0.10, 0.0, 0, 0, True]]
        current = [[0.004, 0.0, 0, 0, True], [0.40, 0.0, 0, 0, True]]
        distances, unmatched = match_jitter(previous, current)
        self.assertEqual(len(distances), 1)
        self.assertAlmostEqual(distances[0], 0.004)
        self.assertEqual(unmatched, 2)


class SummaryTest(unittest.TestCase):
    @staticmethod
    def rows():
        def row(wall, valid, blocks, positions, tag_ok=True, label="vision ready"):
            return {"wall": wall, "valid": valid, "tag_ok": tag_ok, "seam_ok": True,
                    "blocks": blocks, "label": label, "candidates": blocks,
                    "instance_area": 460.0,
                    "blocks_json": [[x, y, 0.0, 0.0, True] for x, y in positions]}
        return [
            row(0.0, True, 2, [(0.0, 0.0), (0.05, 0.0)]),
            row(0.1, True, 3, [(0.001, 0.0), (0.05, 0.0), (0.10, 0.0)]),
            row(0.2, False, 0, [], tag_ok=False, label=f"ID {TAG_ID}: not found"),
            row(0.3, True, 2, [(0.002, 0.0), (0.05, 0.0)]),
        ]

    def test_reports_swing_jitter_and_reasons(self):
        result = summarize(self.rows(), "field")
        self.assertEqual(result["frames"], 4)
        self.assertAlmostEqual(result["valid_pct"], 75.0)
        self.assertAlmostEqual(result["seam_ok_pct"], 100.0)
        self.assertEqual(result["blocks_median"], 2)
        self.assertEqual(result["blocks_max"], 3)
        self.assertEqual(result["reasons"], {f"ID {TAG_ID}: not found": 1})
        self.assertEqual(result["count_swing_median"], 1.0)
        # The rejected frame breaks the chain, so only 0->1 and 3->4 are compared.
        self.assertGreater(result["jitter_matched"], 0)
        self.assertLess(result["jitter_mm_p90"], 5.0)
        self.assertEqual(result["jitter_unmatched_total"], 2)

    def test_bench_mode_summary_has_no_seam_column(self):
        result = summarize(self.rows(), "bench")
        self.assertNotIn("seam_ok_pct", result)
        self.assertIn("jitter_mm_median", result)

    def test_empty_rows_do_not_crash(self):
        self.assertEqual(summarize([], "field"), {"mode": "field", "frames": 0})

    def test_outputs_round_trip(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)
            result = write_outputs(out, self.rows(), "field", {"secs": 1.0},
                                   {"saved_frames": 0})
            self.assertTrue(result["saved_frames"] == 0)
            written = json.loads((out / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(written["frames"], 4)
            self.assertEqual(written["params"], {"secs": 1.0})
            lines = (out / "frames.csv").read_text(encoding="utf-8-sig").splitlines()
            self.assertEqual(len(lines), 5)
            header = lines[0].split(",")
            self.assertIn("wall", header)
            self.assertIn("blocks_json", header)
            self.assertEqual(header[-1], "blocks_json")


if __name__ == "__main__":
    unittest.main()
