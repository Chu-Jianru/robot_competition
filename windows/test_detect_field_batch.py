import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from detect_field_batch import run


class BatchMetadataChecks(unittest.TestCase):
    def test_absence_metadata_does_not_suppress_false_tag_or_body_and_labels_are_not_detector_input(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root/'photos'
            source.mkdir()
            cv2.imencode('.jpg', np.zeros((100, 100, 3), np.uint8))[1].tofile(source/'sample.jpg')
            (source/'annotations.json').write_text(json.dumps({'sample.jpg': {
                'blocks': [[10, 10], [30, 30]],
                'field_corners': [[5, 5], [95, 95], [95, 5], [5, 95]]}}), encoding='utf-8')
            tag = {'id': 0, 'corners_px': [[10, 10], [20, 10], [20, 20], [10, 20]]}
            field = {'homography': np.eye(3), 'field_px': None, 'field_source': 'automatic_unverified'}
            with patch('detect_field_batch.analyze', return_value=(field, [tag], [], None)) as inspect, \
                 patch('detect_field_batch.detect_calibrated_blocks', return_value=[]) as detector, \
                 patch('detect_field_batch.robot_regions', return_value=[np.zeros((4, 1, 2), np.float32)]):
                report = run(source, root/'out', expect_no_robot=True)
            self.assertIs(report['expected_robot_present'], False)
            self.assertEqual(report['photos'][0]['tags'], [tag])
            self.assertEqual(report['photos'][0]['robot_region_count'], 1)
            self.assertEqual(report['photos'][0]['saved_block_point_count'], 2)
            self.assertIsNone(report['photos'][0]['accuracy'])
            self.assertIn('invalid_manual_corners', report['photos'][0]['calibration_warning'])
            self.assertEqual(inspect.call_args.args[1], {'field_corners': []})
            self.assertEqual(len(detector.call_args.args), 3)
            self.assertEqual(detector.call_args.args[2], [tag])


if __name__ == '__main__':
    unittest.main()
