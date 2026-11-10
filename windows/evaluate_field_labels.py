"""Evaluate strict candidates against saved user points, without changing labels.

A hit means a point is inside or <=4 original pixels outside the contour.
Maximum one-to-one matching prevents a merged pile counting as several blocks.
Broad region coverage is a separate many-to-one diagnostic, not recall.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from analyze_field_photos import DATA, OUT, analyze, read_image, save_image


def match_points(contours, points, tolerance=4.):
    adjacency = [[j for j, p in enumerate(points) if cv2.pointPolygonTest(
        np.float32(c).reshape(-1, 1, 2), tuple(map(float, p)), True) >= -tolerance]
        for c in contours]
    owners = {}
    def augment(i, seen):
        for j in adjacency[i]:
            if j in seen:
                continue
            seen.add(j)
            if j not in owners or augment(owners[j], seen):
                owners[j] = i
                return True
        return False
    for i in range(len(contours)):
        augment(i, set())
    return owners, sorted({j for neighbours in adjacency for j in neighbours})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=DATA)
    parser.add_argument('--output', type=Path, default=OUT/'manual_review')
    args = parser.parse_args()
    source = (args.input/'annotations.json').read_bytes()
    labels = json.loads(source.decode('utf-8'))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'annotations_evaluated.json').write_bytes(source)
    rows, thumbnails = [], []
    for name, label in sorted(labels.items()):
        image = read_image(args.input/name)
        field, tags, proposals, view = analyze(image, label)
        points = label['blocks']
        strict = field['red_detections']
        matched, _ = match_points([d['contour_px'] for d in strict], points)
        _, covered = match_points([d['contour_px'] for d in proposals], points)
        used = set(matched.values())
        missed = [i for i in range(len(points)) if i not in matched]
        unmatched = [i for i in range(len(strict)) if i not in used]
        row = {'file': name, 'labels': len(points), 'strict_predictions': len(strict),
               'matched': len(matched), 'missed': len(missed), 'unmatched_predictions': len(unmatched),
               'missed_label_ids': [i+1 for i in missed],
               'unmatched_prediction_ids': [i+1 for i in unmatched],
               'broad_covered_labels': len(covered),
               'broad_uncovered_label_ids': [i+1 for i in range(len(points)) if i not in covered],
               'manual_corner_count': len(label.get('field_corners', [])),
               'reviewed_key_flag': label.get('reviewed', False),
               'tag_detected': bool(tags)}
        rows.append(row)
        out = image.copy()
        for i, d in enumerate(strict):
            color = (40, 200, 40) if i in used else (255, 80, 255)
            cv2.polylines(out, [d['contour_px']], True, color, 2)
        for i, point in enumerate(points):
            xy = tuple(map(round, point))
            color = (40, 200, 40) if i in matched else (0, 0, 255)
            cv2.drawMarker(out, xy, color, cv2.MARKER_CROSS, 20, 2)
            cv2.putText(out, str(i+1), (xy[0]+8, xy[1]-6), 0, .55, color, 2)
        cv2.rectangle(out, (0, 0), (out.shape[1], 44), (30, 30, 30), -1)
        cv2.putText(out, f"Labels={len(points)} hit={len(matched)} missed={len(missed)} extra={len(unmatched)} | red=miss magenta=extra",
                    (10, 29), 0, .65, (255, 255, 255), 2)
        save_image(args.output/(Path(name).stem+'_errors.jpg'), out)
        thumb = np.full((355, 350, 3), 235, np.uint8)
        ratio = min(350/out.shape[1], 325/out.shape[0])
        small = cv2.resize(out, None, fx=ratio, fy=ratio)
        thumb[30:30+small.shape[0], :small.shape[1]] = small
        cv2.putText(thumb, f"{Path(name).stem} hit={len(matched)}/{len(points)} extra={len(unmatched)}",
                    (5, 22), 0, .49, (20, 20, 20), 1)
        thumbnails.append(thumb)
        print(f"{name}: labels={len(points)} hit={len(matched)} miss={len(missed)} extra={len(unmatched)} covered={len(covered)}")
    totals = {key: sum(r[key] for r in rows) for key in
              ('labels', 'strict_predictions', 'matched', 'missed', 'unmatched_predictions', 'broad_covered_labels')}
    report = {'label_sha256': hashlib.sha256(source).hexdigest(),
              'protocol': 'Maximum one-to-one matching; label within strict contour or <=4 original pixels from it. '
                          'Broad coverage permits several labels per region and is NOT object recall. '
                          'Results compare with saved labels, which may contain omissions/errors; no held-out validation.',
              'totals': totals, 'photos': rows}
    (args.output/'evaluation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    while len(thumbnails) % 4:
        thumbnails.append(np.full_like(thumbnails[0], 235))
    save_image(args.output/'errors_contact.jpg', np.vstack([
        np.hstack(thumbnails[i:i+4]) for i in range(0, len(thumbnails), 4)]))
    print('TOTAL', totals)


if __name__ == '__main__':
    main()
