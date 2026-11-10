"""Offline checks for converting centre-point labels into boxes."""
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from import_points import (box_for_point, clamp, homography, import_batch,
                           local_mm_per_px, rotate_box)


def square_scene():
    """1200x1200 field: yellow top, blue bottom, one red block at (600, 600)."""
    image = np.zeros((1200, 1200, 3), np.uint8)
    image[:600] = (0, 220, 240)
    image[600:] = (240, 60, 0)
    image[590:610, 590:610] = (0, 0, 220)
    return image


class GeometryTest(unittest.TestCase):
    def test_full_frame_corners_give_one_mm_per_pixel(self):
        matrix = homography([[0, 0], [1200, 0], [1200, 1200], [0, 1200]])
        self.assertIsNotNone(matrix)
        self.assertAlmostEqual(local_mm_per_px(matrix, (600, 600))[0], 1.0, delta=0.01)

    def test_invalid_corners_are_rejected(self):
        self.assertIsNone(homography([[0, 0], [10, 10], [0, 10]]))
        self.assertIsNone(homography([[0, 0], [0, 0], [0, 0], [0, 0]]))

    def test_box_is_centred_and_scaled(self):
        box = box_for_point((100, 200), (1.0, 1.0))
        self.assertAlmostEqual(box[0], 100 - 12.5)
        self.assertAlmostEqual(box[2], 25.0)

    def test_rotation_moves_the_box_with_the_image(self):
        # 100x50 image rotated 90° counter-clockwise: (x, y) -> (y, 100 - x)
        rotated = rotate_box([10, 10, 20, 20], 90, 100, 50)
        self.assertEqual(rotated, [10, 70, 20, 20])

    def test_clamp_keeps_boxes_inside(self):
        self.assertEqual(clamp([-10, -10, 30, 30], 100, 100), [0, 0, 20, 20])
        self.assertIsNone(clamp([200, 200, 10, 10], 100, 100))


class ImportBatchTest(unittest.TestCase):
    def test_points_become_boxes_and_entries_are_not_reviewed(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            batch = root / "batch"
            batch.mkdir()
            cv2.imwrite(str(batch / "a.jpg"), square_scene())
            annotations = {"a.jpg": {
                "blocks": [[600, 600]],
                "field_corners": [[0, 0], [1200, 0], [1200, 1200], [0, 1200]],
            }}
            (batch / "annotations.json").write_text(json.dumps(annotations), encoding="utf-8")
            target = root / "train"

            result = import_batch(batch, target, "field")
            self.assertEqual(result["images"], 1)
            self.assertEqual(result["boxes"], 1)
            store = json.loads((target / "frames" / "labels.json").read_text(encoding="utf-8"))
            entry = store["images"]["field_001.jpg"]
            self.assertFalse(entry["reviewed"])
            self.assertTrue(entry["from_points"])
            x, y, w, h = entry["boxes"][0]
            self.assertLessEqual(abs(x + w / 2 - 600), 2)
            self.assertLessEqual(abs(y + h / 2 - 600), 2)
            self.assertLessEqual(abs(w - 25), 2)


if __name__ == "__main__":
    unittest.main()
