"""Run the current offline calibrated detector on an arbitrary photo folder.

Without point labels this reports proposals only, never accuracy. Manual floor
corners may be read from annotations.json; block points never enter detection.
--expect-no-robot records ground-truth metadata and does not disable tag/body
detection, so this mode can expose false tag or chassis-region detections.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np

from analyze_field_photos import ROOT, analyze, read_image, save_image
from calibrated_blocks import detect_calibrated_blocks, robot_regions
from optimize_field_photos import contact, thumbnail


def run(input_folder, output, *, ignore_labels=False, expect_no_robot=False):
    files = sorted(input_folder.glob('*.jpg'))
    if not files:
        raise ValueError('No JPG photos found')
    path = input_folder/'annotations.json'
    label_bytes = path.read_bytes() if path.exists() and not ignore_labels else None
    labels = json.loads(label_bytes.decode('utf-8')) if label_bytes is not None else {}
    modules = ['calibrated_blocks.py', 'photo_tag.py', 'tag_detector.py',
               'field_vision.py', 'analyze_field_photos.py', 'scene_vision.py',
               'red_blocks.py', 'red_block_profile.json', 'detect_field_batch.py']
    hashes = {name: hashlib.sha256((ROOT/'windows'/name).read_bytes()).hexdigest() for name in modules}
    output.mkdir(parents=True, exist_ok=True)
    if label_bytes is not None:
        (output/'annotations_used.json').write_bytes(label_bytes)
    rows, thumbnails = [], []
    for file in files:
        image = read_image(file)
        label = labels.get(file.name, {})
        supplied_corners = label.get('field_corners', [])
        calibration_warning = None
        usable_corners = supplied_corners
        if len(supplied_corners) == 4:
            quad = np.float32(supplied_corners)
            if not cv2.isContourConvex(quad) or cv2.contourArea(quad) < 100:
                calibration_warning = 'invalid_manual_corners_ignored; automatic_scale_unverified'
                usable_corners = []
        elif supplied_corners:
            calibration_warning = 'incomplete_manual_corners_ignored'
            usable_corners = []
        started = time.perf_counter()
        field, tags, _, _ = analyze(image, {'field_corners': usable_corners}, enhanced_tag=True)
        detections = detect_calibrated_blocks(image, field['homography'], tags)
        bodies = []
        if field['homography'] is not None:
            transform = np.float64([[1000, 0, 640], [0, -1000, 640], [0, 0, 1]]) @ field['homography']
            view = cv2.warpPerspective(image, transform, (1280, 1280))
            bodies = robot_regions(view, tags, transform)
        elapsed = (time.perf_counter()-started)*1000
        corners = field['field_px']
        h, w = image.shape[:2]
        edge_contact = bool(corners is not None and np.any(
            (corners[:, 0] <= 3) | (corners[:, 0] >= w-4) |
            (corners[:, 1] <= 3) | (corners[:, 1] >= h-4)))
        row = {'file': file.name, 'source_sha256': hashlib.sha256(file.read_bytes()).hexdigest(),
               'image_size': [w, h], 'predictions': len(detections), 'tags': tags,
               'robot_region_count': len(bodies), 'field_source': field['field_source'],
               'calibration_warning': calibration_warning,
               'field_edge_contact': edge_contact,
               'field_corners_px': None if corners is None else corners.tolist(),
               'saved_block_point_count': len(label['blocks']) if 'blocks' in label else None,
               'accuracy': None, 'elapsed_ms': elapsed,
               'detections': [{**d, 'contour_px': d['contour_px'].reshape(-1, 2).tolist()} for d in detections]}
        rows.append(row)
        marked = image.copy()
        if corners is not None:
            cv2.polylines(marked, [cv2.convexHull(np.int32(corners))], True, (255, 200, 0), 3)
        for d in detections:
            cv2.polylines(marked, [np.int32(d['contour_px'])], True, (40, 220, 40), 2)
        for tag in tags:
            cv2.polylines(marked, [np.int32(tag['corners_px'])], True, (0, 230, 255), 3)
        title = f'{file.stem} regions={len(detections)} ID0={bool(tags)}'
        cv2.rectangle(marked, (0, 0), (w, 76), (25, 25, 25), -1)
        cv2.putText(marked, title, (10, 28), 0, .65, (255, 255, 255), 2)
        subtitle = 'Proposals only | field scale needs review' + (' | IMAGE EDGE' if edge_contact else '')
        cv2.putText(marked, subtitle, (10, 59), 0, .57, (240, 240, 240), 1)
        save_image(output/f'{file.stem}_detections.jpg', marked)
        thumbnails.append(thumbnail(marked, title))
        print(title, f"field={field['field_source']} body_regions={len(bodies)} edge={edge_contact}")
    contact(thumbnails, output/'detections_contact.jpg')
    report = {'detector_hashes': hashes, 'expected_robot_present': False if expect_no_robot else None,
              'annotation_sha256': hashlib.sha256(label_bytes).hexdigest() if label_bytes is not None else None,
              'limits': 'Unverified proposals. Automatic or partial-field calibration can distort scale. '
                        'No object accuracy is inferred from counts. Absence metadata does not disable detection. '
                        'No camera, serial or motor access.', 'photos': rows}
    (output/'detections.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ignore-labels', action='store_true')
    parser.add_argument('--expect-no-robot', action='store_true')
    args = parser.parse_args()
    run(args.input, args.output, ignore_labels=args.ignore_labels, expect_no_robot=args.expect_no_robot)


if __name__ == '__main__':
    main()
