"""Reproduce the offline baseline/experimental comparison on saved labels.

Labels enter evaluation only. The detector receives BGR pixels, the four-corner
homography, and decoded tags; no point labels, filenames or expected counts.
"""
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np

from analyze_field_photos import DATA, OUT, analyze, read_image, save_image
from calibrated_blocks import detect_calibrated_blocks
from evaluate_field_labels import match_points
from field_vision import image_to_world


def center_matches(detections, points, homography, tolerance_mm=12.):
    if not detections or not points:
        return 0
    centers = image_to_world(homography, [d['center_px'] for d in detections])*1000
    labels = image_to_world(homography, points)*1000
    owners = {}
    edges = [np.flatnonzero(np.linalg.norm(labels-center, axis=1) <= tolerance_mm).tolist()
             for center in centers]
    def augment(i, seen):
        for j in edges[i]:
            if j in seen:
                continue
            seen.add(j)
            if j not in owners or augment(owners[j], seen):
                owners[j] = i
                return True
        return False
    for i in range(len(edges)):
        augment(i, set())
    return len(owners)


def score(detections, points, homography):
    matched, _ = match_points([d['contour_px'] for d in detections], points)
    return {'predictions': len(detections), 'matched': len(matched),
            'missed': len(points)-len(matched), 'extra': len(detections)-len(matched),
            'center_matched_12mm': center_matches(detections, points, homography)}, matched


def contact(thumbnails, path):
    while len(thumbnails) % 4:
        thumbnails.append(np.full_like(thumbnails[0], 235))
    save_image(path, np.vstack([np.hstack(thumbnails[i:i+4]) for i in range(0, len(thumbnails), 4)]))


def thumbnail(image, title):
    canvas = np.full((355, 350, 3), 235, np.uint8)
    ratio = min(350/image.shape[1], 325/image.shape[0])
    small = cv2.resize(image, None, fx=ratio, fy=ratio)
    canvas[30:30+small.shape[0], :small.shape[1]] = small
    cv2.putText(canvas, title, (5, 22), 0, .47, (20, 20, 20), 1)
    return canvas


def main():
    output = OUT/'optimized'
    output.mkdir(parents=True, exist_ok=True)
    source = (DATA/'annotations.json').read_bytes()
    labels = json.loads(source.decode('utf-8'))
    (output/'annotations_evaluated.json').write_bytes(source)
    rows, detection_thumbs, error_thumbs = [], [], []
    for name, label in sorted(labels.items()):
        image = read_image(DATA/name)
        start = time.perf_counter()
        field, tags, _, _ = analyze(image, {'field_corners': label['field_corners']}, enhanced_tag=True)
        detections = detect_calibrated_blocks(image, field['homography'], tags)
        elapsed = (time.perf_counter()-start)*1000
        baseline, _ = score(field['red_detections'], label['blocks'], field['homography'])
        improved, matched = score(detections, label['blocks'], field['homography'])
        row = {'file': name, 'labels': len(label['blocks']), 'baseline': baseline,
               'improved': improved, 'tags': tags, 'offline_total_ms': elapsed,
               'missed_label_ids': [i+1 for i in range(len(label['blocks'])) if i not in matched],
               'detections': [{**d, 'contour_px': d['contour_px'].reshape(-1, 2).tolist()}
                              for d in detections]}
        rows.append(row)
        detected = image.copy()
        errors = image.copy()
        for d in detections:
            cv2.polylines(detected, [np.int32(d['contour_px'])], True, (40, 220, 40), 2)
        for tag in tags:
            cv2.polylines(detected, [np.int32(tag['corners_px'])], True, (0, 230, 255), 3)
        used = set(matched.values())
        for i, d in enumerate(detections):
            cv2.polylines(errors, [np.int32(d['contour_px'])], True,
                          (40, 200, 40) if i in used else (255, 80, 255), 2)
        for i, point in enumerate(label['blocks']):
            xy = tuple(map(round, point))
            color = (40, 200, 40) if i in matched else (0, 0, 255)
            cv2.drawMarker(errors, xy, color, cv2.MARKER_CROSS, 18, 2)
            cv2.putText(errors, str(i+1), (xy[0]+7, xy[1]-7), 0, .55, color, 2)
        for target, message in [(detected, f"Detected regions={len(detections)} | green=detection yellow=ID0"),
                                (errors, f"Hit={improved['matched']}/{len(label['blocks'])} extra={improved['extra']} | red=miss magenta=extra")]:
            cv2.rectangle(target, (0, 0), (target.shape[1], 44), (25, 25, 25), -1)
            cv2.putText(target, message, (10, 30), 0, .65, (255, 255, 255), 2)
        stem = Path(name).stem
        save_image(output/f'{stem}_detections.jpg', detected)
        save_image(output/f'{stem}_errors.jpg', errors)
        title = f"{stem} {baseline['matched']}->{improved['matched']}/{len(label['blocks'])} extra={improved['extra']}"
        detection_thumbs.append(thumbnail(detected, f'{stem} regions={len(detections)} ID0={bool(tags)}'))
        error_thumbs.append(thumbnail(errors, title))
        print(title)
    contact(detection_thumbs, output/'detections_contact.jpg')
    contact(error_thumbs, output/'errors_contact.jpg')
    totals = {version: {key: sum(r[version][key] for r in rows) for key in rows[0][version]}
              for version in ('baseline', 'improved')}
    totals['labels'] = sum(r['labels'] for r in rows)
    totals['tag_photos'] = sum(bool(r['tags']) for r in rows)
    report = {'label_sha256': hashlib.sha256(source).hexdigest(),
              'limits': 'Development photos, not held-out validation. Uses manually marked field corners. '
                        'Contour matching uses original-pixel 4px tolerance; center matching uses 12mm '
                        'in the same approximate field projection. Size-inferred splits remain estimates. '
                        'No tag pose truth. Not connected to real-time camera or control.',
              'totals': totals, 'photos': rows}
    (output/'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(totals)


if __name__ == '__main__':
    main()
