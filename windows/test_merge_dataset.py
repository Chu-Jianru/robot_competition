"""Offline checks for merging an extra image batch into a dataset."""
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from merge_dataset import image_folder, merge, next_number


def scene():
    image = np.zeros((240, 320, 3), np.uint8)
    image[:] = (240, 60, 0)
    image[100:122, 150:172] = (0, 0, 220)
    return image


class MergeTest(unittest.TestCase):
    def test_images_and_labels_are_merged_without_touching_existing_entries(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            target = root / "train"
            (target / "frames").mkdir(parents=True)
            cv2.imwrite(str(target / "frames" / "frame_00000.jpg"), scene())
            (target / "frames" / "labels.json").write_text(json.dumps({
                "version": 1, "class_name": "red_block",
                "images": {"frame_00000.jpg": {"boxes": [[1, 2, 3, 4]], "reviewed": True}},
            }), encoding="utf-8")
            source = root / "photos" / "frames"
            source.mkdir(parents=True)
            for index in range(2):
                cv2.imwrite(str(source / f"photo_{index + 1:03d}.jpg"), scene())

            result = merge(target, root / "photos", "photo", prefill=False)
            self.assertEqual(result["added"], 2)
            self.assertEqual(result["total_images"], 3)
            store = json.loads((target / "frames" / "labels.json").read_text(encoding="utf-8"))
            self.assertEqual(store["images"]["frame_00000.jpg"]["boxes"], [[1, 2, 3, 4]])
            self.assertTrue(store["images"]["photo_001.jpg"]["boxes"] == [])
            self.assertTrue((target / "sources.json").exists())

    def test_running_twice_does_not_overwrite_the_first_batch(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            target = root / "train"
            source = root / "photos"
            source.mkdir(parents=True)
            for index in range(2):
                cv2.imwrite(str(source / f"p{index}.jpg"), scene())
            merge(target, source, "photo", prefill=False)
            second = merge(target, source, "photo", prefill=False)
            self.assertEqual(second["added"], 2)
            self.assertEqual(second["total_images"], 4)

    def test_source_can_be_a_bare_image_folder(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.assertEqual(image_folder(root), root)
            (root / "frames").mkdir()
            self.assertEqual(image_folder(root), root / "frames")

    def test_numbering_continues_after_existing_prefix(self):
        with tempfile.TemporaryDirectory() as folder:
            frames = Path(folder)
            for number in (1, 5):
                (frames / f"photo_{number:03d}.jpg").write_bytes(b"x")
            self.assertEqual(next_number(frames, "photo"), 6)


if __name__ == "__main__":
    unittest.main()
