"""Offline checks for frame capture helpers: numbering and HSV pre-labelling."""
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from collect_red_frames import next_frame_index, prelabel_boxes, save_frame
from red_blocks import RedBlockConfig


def scene():
    image = np.zeros((240, 320, 3), np.uint8)
    image[:] = (240, 60, 0)          # blue floor (BGR)
    image[100:122, 150:172] = (0, 0, 220)   # a 22 px red block
    return image


class PrelabelTest(unittest.TestCase):
    def test_red_block_is_pre_filled_as_a_box(self):
        boxes = prelabel_boxes(scene(), RedBlockConfig())
        self.assertEqual(len(boxes), 1)
        x, y, w, h = boxes[0]
        self.assertTrue(abs(x - 150) <= 3 and abs(y - 100) <= 3, boxes)
        self.assertTrue(abs(w - 22) <= 4 and abs(h - 22) <= 4, boxes)

    def test_blue_floor_alone_gives_no_boxes(self):
        image = scene()
        image[100:122, 150:172] = (240, 60, 0)
        self.assertEqual(prelabel_boxes(image, RedBlockConfig()), [])

    def test_default_config_is_used_when_none_is_given(self):
        # Passing None means "use the default colour profile"; disabling prefill
        # is the caller's decision, not this helper's.
        self.assertEqual(len(prelabel_boxes(scene(), None)), 1)


class NumberingTest(unittest.TestCase):
    def test_empty_folder_starts_at_zero(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(next_frame_index(Path(folder)), 0)

    def test_existing_frames_are_not_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for index in (0, 1, 7):
                (root / f"frame_{index:05d}.jpg").write_bytes(b"x")
            self.assertEqual(next_frame_index(root), 8)

    def test_save_frame_continues_the_sequence(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            rows = []
            name = save_frame(scene(), 100, root, rows, start_index=8)
            self.assertEqual(name, "frame_00008.jpg")
            self.assertTrue((root / name).exists())
            self.assertEqual(rows[0]["file"], "frame_00008.jpg")


if __name__ == "__main__":
    unittest.main()
