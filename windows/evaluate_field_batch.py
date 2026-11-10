"""Score saved predictions against point labels without rerunning/tuning detection."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from analyze_field_photos import read_image, save_image
from evaluate_field_labels import match_points
from optimize_field_photos import contact, thumbnail


def evaluate(prediction_path, folder, output):
    prediction_bytes = prediction_path.read_bytes()
    prediction = json.loads(prediction_bytes.decode('utf-8'))
    label_bytes = (folder/'annotations.json').read_bytes()
    labels = json.loads(label_bytes.decode('utf-8'))
    output.mkdir(parents=True, exist_ok=True)
    (output/'annotations_evaluated.json').write_bytes(label_bytes)
    rows, thumbs = [], []
    for photo in prediction['photos']:
        name = photo['file']
        points = labels[name]['blocks']
        detections = photo['detections']
        matched, _ = match_points([d['contour_px'] for d in detections], points)
        used = set(matched.values())
        row = {'file': name, 'labels': len(points), 'predictions': len(detections),
               'matched': len(matched), 'missed': len(points)-len(matched),
               'extra': len(detections)-len(matched),
               'field_source': photo['field_source'],
               'calibration_warning': photo.get('calibration_warning'),
               'missed_label_ids': [i+1 for i in range(len(points)) if i not in matched],
               'extra_prediction_ids': [i+1 for i in range(len(detections)) if i not in used]}
        rows.append(row)
        image = read_image(folder/name)
        for i, d in enumerate(detections):
            color = (40, 200, 40) if i in used else (255, 80, 255)
            cv2.polylines(image, [np.int32(d['contour_px'])], True, color, 2)
        for i, point in enumerate(points):
            xy = tuple(map(round, point))
            color = (40, 200, 40) if i in matched else (0, 0, 255)
            cv2.drawMarker(image, xy, color, cv2.MARKER_CROSS, 18, 2)
            cv2.putText(image, str(i+1), (xy[0]+7, xy[1]-7), 0, .55, color, 2)
        title = f"{Path(name).stem} hit={len(matched)}/{len(points)} extra={row['extra']}"
        cv2.rectangle(image, (0, 0), (image.shape[1], 72), (25, 25, 25), -1)
        cv2.putText(image, title, (10, 28), 0, .65, (255, 255, 255), 2)
        cv2.putText(image, 'Green=matched Red=missed Magenta=extra | pixel comparison only',
                    (10, 57), 0, .52, (255, 255, 255), 1)
        save_image(output/f'{Path(name).stem}_errors.jpg', image)
        thumbs.append(thumbnail(image, title))
        print(title, f"field={row['field_source']}")
    def totals(selected):
        return {k: sum(row[k] for row in selected) for k in ('labels', 'predictions', 'matched', 'missed', 'extra')}
    report = {'prediction_sha256': hashlib.sha256(prediction_bytes).hexdigest(),
              'annotations_sha256': hashlib.sha256(label_bytes).hexdigest(),
              'protocol': 'Maximum one-to-one contour matching; label inside or <=4 original pixels outside contour. '
                          'Does not run detector or infer metric geometry. Saved label errors/omissions affect results.',
              'totals': totals(rows),
              'manual_geometry_subset': totals([r for r in rows if r['field_source'] == 'manual_corners']),
              'unverified_geometry_subset': totals([r for r in rows if r['field_source'] != 'manual_corners']),
              'photos': rows}
    (output/'evaluation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    contact(thumbs, output/'errors_contact.jpg')
    print('TOTAL', report['totals'])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--predictions', type=Path, required=True)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    evaluate(args.predictions, args.input, args.output)


if __name__ == '__main__':
    main()
