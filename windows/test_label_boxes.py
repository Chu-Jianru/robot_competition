"""Offline checks for the bounding-box annotator and YOLO export."""
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from export_yolo import export, is_validation, normalized
from label_boxes import (box_index_at, clamp_box, entry_of, fit_scale,
                         load_store, prefill_entries, render, save_store,
                         summarize_store)
from red_blocks import RedBlockConfig


class BoxMathTest(unittest.TestCase):
    def test_clamp_keeps_inside_the_image(self):
        self.assertEqual(clamp_box((10, 20, 50, 60), 100, 100), [10, 20, 40, 40])

    def test_clamp_swaps_reversed_drag(self):
        self.assertEqual(clamp_box((60, 70, 20, 30), 100, 100), [20, 30, 40, 40])

    def test_clamp_rejects_tiny_or_outside_boxes(self):
        self.assertIsNone(clamp_box((0, 0, 3, 3), 100, 100))
        self.assertIsNone(clamp_box((200, 200, 260, 260), 100, 100))

    def test_hit_test_picks_the_topmost_box(self):
        boxes = [[0, 0, 20, 20], [10, 10, 20, 20]]
        self.assertEqual(box_index_at(boxes, 12, 12), 1)
        self.assertEqual(box_index_at(boxes, 5, 5), 0)
        self.assertIsNone(box_index_at(boxes, 90, 90))


class DisplayTest(unittest.TestCase):
    def test_scale_fills_the_screen_without_exceeding_it(self):
        scale = fit_scale(960, 1280, 1920, 1080)
        self.assertGreater(scale, 0.4)
        self.assertLessEqual(1280 * scale, 1080)
        self.assertLessEqual(960 * scale, 1920)

    def test_render_matches_the_scaled_size_and_keeps_the_image_intact(self):
        image = np.zeros((100, 200, 3), np.uint8)
        canvas = render(image, [[10, 10, 20, 20]], (0, 0, 30, 30), "status", 0.5)
        self.assertEqual(canvas.shape[1], 100)
        self.assertEqual(canvas.shape[0], 50)
        self.assertEqual(image.shape, (100, 200, 3))

    def test_float_drag_coordinates_are_accepted_and_drawn(self):
        # Regression: cv2.rectangle rejects float tuples, and the drag preview
        # used to be drawn without scaling.
        image = np.zeros((100, 200, 3), np.uint8)
        plain = render(image, [], None, "status", 0.5)
        dragged = render(image, [], (10.5, 10.25, 30.75, 40.0), "status", 0.5)
        self.assertFalse(np.array_equal(plain, dragged))

    def test_mouse_pixels_map_back_to_image_pixels(self):
        # A drag drawn at display (100,50) with scale 0.5 is image (200,100).
        scale = 0.5
        box = clamp_box((100 / scale, 50 / scale, 140 / scale, 90 / scale), 960, 1280)
        self.assertEqual(box, [200, 100, 80, 80])


class PrefillTest(unittest.TestCase):
    def test_only_empty_unreviewed_frames_are_filled(self):
        with tempfile.TemporaryDirectory() as folder:
            frames = Path(folder) / "frames"
            frames.mkdir()
            image = np.zeros((240, 320, 3), np.uint8)
            image[:] = (240, 60, 0)
            image[100:122, 150:172] = (0, 0, 220)
            for name in ("a.jpg", "b.jpg"):
                cv2.imwrite(str(frames / name), image)
            store = {"images": {"b.jpg": {"boxes": [], "reviewed": True}}}
            filled = prefill_entries(store, sorted(frames.glob("*.jpg")),
                                     RedBlockConfig())
            self.assertEqual(filled, 1)
            self.assertEqual(len(store["images"]["a.jpg"]["boxes"]), 1)
            self.assertEqual(store["images"]["b.jpg"]["boxes"], [])


class StoreTest(unittest.TestCase):
    def test_missing_file_gives_an_empty_store(self):
        with tempfile.TemporaryDirectory() as folder:
            store = load_store(Path(folder) / "labels.json")
            self.assertEqual(store["images"], {})
            self.assertEqual(store["class_name"], "red_block")

    def test_round_trip_keeps_boxes_and_review_flag(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "labels.json"
            store = load_store(path)
            entry = entry_of(store, "frame_00000.jpg")
            entry["boxes"].append([1, 2, 30, 40])
            entry["reviewed"] = True
            save_store(path, store)
            again = load_store(path)
            self.assertEqual(again["images"]["frame_00000.jpg"]["boxes"], [[1, 2, 30, 40]])
            self.assertTrue(again["images"]["frame_00000.jpg"]["reviewed"])

    def test_summary_counts_reviewed_and_negatives(self):
        store = {"images": {
            "a.jpg": {"boxes": [[0, 0, 10, 10], [20, 20, 10, 10]], "reviewed": True},
            "b.jpg": {"boxes": [], "reviewed": True},
            "c.jpg": {"boxes": [], "reviewed": False},
        }}
        self.assertIn("已核对 2 张", summarize_store(store))
        self.assertIn("2 个框", summarize_store(store))
        self.assertIn("空帧负样本 1 张", summarize_store(store))


class YoloMathTest(unittest.TestCase):
    def test_normalizes_to_centre_and_size(self):
        values = normalized([10, 20, 20, 40], 100, 200)
        self.assertAlmostEqual(values[0], 0.20)
        self.assertAlmostEqual(values[1], 0.20)
        self.assertAlmostEqual(values[2], 0.20)
        self.assertAlmostEqual(values[3], 0.20)

    def test_degenerate_and_outside_boxes_are_rejected(self):
        self.assertIsNone(normalized([0, 0, 0, 0], 100, 100))
        self.assertIsNone(normalized([200, 0, 10, 10], 100, 100))

    def test_validation_split_is_stable(self):
        first = [is_validation(f"frame_{index:05d}.jpg", 5) for index in range(20)]
        second = [is_validation(f"frame_{index:05d}.jpg", 5) for index in range(20)]
        self.assertEqual(first, second)
        self.assertEqual(sum(first), 4)


class ExportTest(unittest.TestCase):
    def test_export_writes_labels_only_for_reviewed_images(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            frames = root / "frames"
            frames.mkdir()
            for name in ("frame_00001.jpg", "frame_00002.jpg", "frame_00003.jpg"):
                cv2.imwrite(str(frames / name), np.zeros((100, 100, 3), np.uint8))
            store = {"version": 1, "class_name": "red_block", "images": {
                "frame_00001.jpg": {"boxes": [[10, 20, 20, 20]], "reviewed": True,
                                    "width": 100, "height": 100},
                "frame_00002.jpg": {"boxes": [], "reviewed": True,
                                    "width": 100, "height": 100},
                "frame_00003.jpg": {"boxes": [[5, 5, 10, 10]], "reviewed": False,
                                    "width": 100, "height": 100},
            }}
            (root / "labels.json").write_text(json.dumps(store), encoding="utf-8")
            stats = export(root, root / "yolo", val_every=5)
            self.assertEqual(stats["images_train"] + stats["images_val"], 2)
            self.assertEqual(stats["skipped_unreviewed"], 1)
            self.assertEqual(stats["boxes_train"] + stats["boxes_val"], 1)
            self.assertTrue((root / "yolo" / "data.yaml").exists())
            text = (root / "yolo" / "labels" / "train" / "frame_00001.txt").read_text()
            self.assertTrue(text.startswith("0 0.200000 0.300000 0.200000 0.200000"))
            negative = (root / "yolo" / "labels" / "train" / "frame_00002.txt").read_text()
            self.assertEqual(negative, "")

    def test_labels_next_to_the_images_are_found(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "frames").mkdir()
            for name in ("frame_00001.jpg", "frame_00002.jpg"):
                cv2.imwrite(str(root / "frames" / name), np.zeros((100, 100, 3), np.uint8))
            store = {"version": 1, "class_name": "red_block", "images": {
                "frame_00001.jpg": {"boxes": [[10, 20, 20, 20]], "reviewed": True,
                                    "width": 100, "height": 100},
            }}
            (root / "frames" / "labels.json").write_text(json.dumps(store), encoding="utf-8")
            stats = export(root, root / "yolo", val_every=5)
            self.assertEqual(stats["images_train"] + stats["images_val"], 1)
            self.assertEqual(stats["boxes_train"] + stats["boxes_val"], 1)


if __name__ == "__main__":
    unittest.main()
