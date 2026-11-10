"""Regression against seven local real photos; all expectations are test labels.

The detector has no knowledge of image names, labelled positions or this folder.
"""
import unittest
from pathlib import Path

import cv2
import numpy as np

from red_blocks import detect_red_blocks, filter_by_size_mm, load_red_config
from tag_detector import detect_markers

ROOT = Path(__file__).resolve().parent.parent
FOLDER = ROOT / 'windows/test_data/phone_tag_blocks'
CENTRES = {1: (931, 1061), 2: (705, 1205), 3: (756, 1265),
           4: (508, 1093), 5: (564, 1202), 6: (1118, 609), 7: (668, 1195)}


@unittest.skipUnless(all((FOLDER / f'phone_tag_block_{i:02d}.jpg').exists() for i in CENTRES),
                     'Local seven-photo collection is not present')
class PhonePhotoChecks(unittest.TestCase):
    def test_tag_and_exactly_one_labelled_ground_block_in_all_seven_photos(self):
        config = load_red_config(ROOT / 'windows/red_block_profile.json')
        for number, centre in CENTRES.items():
            original = cv2.imdecode(np.fromfile(str(FOLDER / f'phone_tag_block_{number:02d}.jpg'), np.uint8), 1)
            for rotation in (False, True):
                with self.subTest(number=number, rotation=rotation):
                    image = cv2.rotate(original, cv2.ROTATE_90_CLOCKWISE) if rotation else original
                    # These photos predate the ID change: the car still wore ID 0.
                    target = [m for m in detect_markers(image, 0) if m.tag_id == 0]
                    self.assertEqual(len(target), 1)
                    points = target[0].corners
                    side = float(np.mean(np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1)))
                    blocks, _ = filter_by_size_mm(detect_red_blocks(image, config), 50 / side)
                    self.assertEqual(len(blocks), 1)
                    expected = (original.shape[0] - 1 - centre[1], centre[0]) if rotation else centre
                    self.assertLess(float(np.linalg.norm(np.array(blocks[0]['center_px']) - expected)), 3)
                    self.assertTrue(17 < blocks[0]['side_mm'] < 23)


if __name__ == '__main__':
    unittest.main()
