"""Offline photo review only. Never opens a camera or sends motion commands.

Strict detections use the current production profile. Broad connected components
are review proposals, NOT a block counter. Manual labels never enter detection.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from field_vision import (HSV, image_to_world, _axes_from_colors, _pair_corners,
                          _centroid, _boundary_y)
from red_blocks import RedBlockConfig, detect_red_blocks, load_red_config
from scene_vision import inspect_image
from tag_detector import detect_markers

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / 'windows/test_data/field_photos_20261103'
OUT = ROOT / 'log/field_photos_20261103'
# Offline review of the 2026-11-03 capture batch: that car still wore the old ID 0
# marker. The live pipeline uses tag_detector.TAG_ID (currently 7).
BATCH_TAG_ID = 0


def read_image(path):
    image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'Cannot read {path}')
    return image


def save_image(path, image):
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise ValueError(f'Cannot encode {path}')
    encoded.tofile(path)


def ground_side(image, detection):
    """Count actual blue/yellow floor around each component, not geometric y."""
    x, y, w, h = detection['bbox_px']
    pad = max(12, int(max(w, h) * .6))
    x0, y0 = max(0, x-pad), max(0, y-pad)
    patch = image[y0:min(image.shape[0], y+h+pad),
                  x0:min(image.shape[1], x+w+pad)]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    counts = {name: int(cv2.countNonZero(cv2.inRange(
        hsv, np.uint8(HSV[name][0]), np.uint8(HSV[name][1]))))
        for name in ('blue', 'yellow')}
    side = ('unknown' if sum(counts.values()) < 20 or counts['blue'] == counts['yellow']
            else 'blue' if counts['blue'] > counts['yellow'] else 'yellow')
    return side, counts


def analyze(image, label=None, *, enhanced_tag=False):
    baseline = inspect_image(image, field_enabled=True,
                             red_config=load_red_config(ROOT/'windows/red_block_profile.json'))
    baseline['automatic_field_px'] = baseline['field_px']
    baseline['field_source'] = 'automatic_unverified'
    corners = (label or {}).get('field_corners', [])
    if len(corners) == 4:
        quad = np.float32(corners)
        if not cv2.isContourConvex(quad) or cv2.contourArea(quad) < 100:
            raise ValueError('Manual field corners must form a convex quadrilateral in cyclic order')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        inside = np.zeros(image.shape[:2], np.uint8)
        cv2.fillConvexPoly(inside, np.int32(quad), 255)
        masks = [cv2.bitwise_and(cv2.inRange(hsv, np.uint8(HSV[n][0]),
                 np.uint8(HSV[n][1])), inside) for n in ('blue', 'yellow')]
        centers = [_centroid(m) for m in masks]
        axes = _axes_from_colors(*centers) if all(c is not None for c in centers) else None
        paired = _pair_corners(quad, *axes) if axes is not None else None
        if paired is None:
            raise ValueError('Cannot orient manual corners from blue/yellow floor')
        src, dst = paired
        baseline['field_px'] = src
        baseline['homography'] = cv2.getPerspectiveTransform(src, dst)
        baseline['boundary_y'] = _boundary_y(*masks, baseline['homography'])
        baseline['field_source'] = 'manual_corners'
    h = baseline['homography']
    if enhanced_tag:
        from photo_tag import detect_photo_markers
        detector = detect_photo_markers
    else:
        detector = detect_markers
    markers = [m for m in detector(image, BATCH_TAG_ID) if m.tag_id == BATCH_TAG_ID]
    tags = [{'id': m.tag_id, 'corners_px': m.corners.tolist(),
             'method': 'original/' + m.method} for m in markers]
    # The field plane can help decode foreshortened tags. This is a separate
    # offline experiment, not a silent change to live pose acceptance.
    view = None
    if h is not None:
        transform = np.float64([[1000, 0, 600], [0, -1000, 600], [0, 0, 1]]) @ h
        view = cv2.warpPerspective(image, transform, (1200, 1200))
        if not tags:
            for m in detector(view, BATCH_TAG_ID):
                if m.tag_id != BATCH_TAG_ID:
                    continue
                corners = image_to_world(np.linalg.inv(transform), m.corners)
                tags.append({'id': m.tag_id, 'corners_px': corners.tolist(),
                             'method': 'field_warp/' + m.method})
    proposals = []
    broad = detect_red_blocks(image, RedBlockConfig(shape_filter=False, min_area_px=12))
    for d in broad:
        if h is None:
            continue
        center = image_to_world(h, [d['center_px']])[0]
        if np.max(np.abs(center)) > .625:
            continue
        contour_mm = image_to_world(h, d['contour_px']) * 1000
        area_mm2 = float(cv2.contourArea(contour_mm))
        short, long = sorted(cv2.minAreaRect(contour_mm)[1])
        if area_mm2 < 55 or area_mm2 > 4500 or short < 5 or long > 150:
            continue
        near_tag = False
        for tag in tags:
            quad = np.float32(tag['corners_px'])
            tag_side = float(np.mean(np.linalg.norm(quad-np.roll(quad, 1, axis=0), axis=1)))
            # Only a review flag: actual body footprint/offset is not calibrated.
            near_tag |= float(np.linalg.norm(np.array(d['center_px'])-quad.mean(axis=0))) < 2.8*tag_side
        side, votes = ground_side(image, d)
        status = ('near_robot_review' if near_tag else 'group_or_shape_review'
                  if area_mm2 > 800 or d['aspect_ratio'] < .55 or d['solidity'] < .8
                  else 'single_candidate')
        proposals.append({'center_px': list(d['center_px']), 'bbox_px': list(d['bbox_px']),
                          'contour_px': d['contour_px'].reshape(-1, 2).tolist(),
                          'area_mm2_approx': area_mm2, 'side': side, 'floor_votes': votes,
                          'status': status})
    return baseline, tags, proposals, view


def render(image, baseline, tags, proposals, label):
    out = image.copy()
    if baseline['field_px'] is not None:
        quad = cv2.convexHull(np.int32(baseline['field_px']))
        cv2.polylines(out, [quad], True, (255, 200, 0), 4)
    for key, color in [('robot_center', (255, 255, 255)), ('robot_front', (0, 255, 255))]:
        for point in label.get(key, []):
            cv2.drawMarker(out, tuple(map(round, point)), color, cv2.MARKER_CROSS, 25, 3)
    for d in baseline['red_detections']:
        x, y, w, h = d['bbox_px']
        cv2.rectangle(out, (x-3, y-3), (x+w+3, y+h+3), (180, 180, 180), 1)
    for i, d in enumerate(proposals, 1):
        color = {'single_candidate': (50, 220, 50), 'group_or_shape_review': (0, 170, 255),
                 'near_robot_review': (255, 80, 255)}[d['status']]
        cv2.polylines(out, [np.int32(d['contour_px'])], True, color, 2)
        x, y = map(round, d['center_px'])
        cv2.putText(out, str(i), (x+9, y-8), 0, .6, color, 2)
    for tag in tags:
        quad = np.int32(tag['corners_px'])
        cv2.polylines(out, [quad], True, (255, 255, 0), 3)
        cv2.putText(out, f"ID {tag['id']}", tuple(quad[0]), 0, .65, (255, 255, 0), 2)
    for i, point in enumerate(label.get('blocks', []), 1):
        x, y = map(round, point)
        cv2.drawMarker(out, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(out, f'M{i}', (x+8, y+20), 0, .55, (255, 255, 255), 2)
    cv2.rectangle(out, (0, 0), (out.shape[1], 75), (30, 30, 30), -1)
    cv2.putText(out, f"Strict: {len(baseline['red_detections'])}  Review regions: {len(proposals)}  ID0: {bool(tags)}",
                (15, 29), 0, .75, (255, 255, 255), 2)
    cv2.putText(out, 'Green: candidate | Orange: group/shape | Magenta: near tag | Gray: strict',
                (15, 59), 0, .56, (240, 240, 240), 1)
    return out


def annotate(files, path):
    """Original-pixel labels, separate from predictions. No inferred block count."""
    labels = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
    window = 'Labels: B blocks / C robot center / F front / K field corners'
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    index, mode = 0, 'blocks'
    try:
        while index < len(files):
            p = files[index]
            image = read_image(p)
            scale = min(1100/image.shape[1], 800/image.shape[0], 1.)
            label = labels.setdefault(p.name, {'blocks': [], 'robot_center': [],
                                               'robot_front': [], 'reviewed': False})
            label.setdefault('field_corners', [])
            def mouse(event, x, y, flags, param):
                if y < 55:
                    return
                if event == cv2.EVENT_LBUTTONDOWN:
                    point = [round(x/scale, 2), round((y-55)/scale, 2)]
                    if not (0 <= point[0] < image.shape[1] and 0 <= point[1] < image.shape[0]):
                        return
                    if mode == 'field_corners' and len(label[mode]) >= 4:
                        return
                    if mode in ('robot_center', 'robot_front'):
                        label[mode] = []
                    label[mode].append(point)
                    label['reviewed'] = False
                elif event == cv2.EVENT_RBUTTONDOWN and label[mode]:
                    label[mode].pop()
                    label['reviewed'] = False
            cv2.setMouseCallback(window, mouse)
            while True:
                preview = cv2.resize(image, None, fx=scale, fy=scale)
                for kind, color in [('blocks', (0, 0, 255)), ('robot_center', (0, 255, 0)),
                                    ('robot_front', (0, 255, 255)), ('field_corners', (255, 255, 0))]:
                    for i, point in enumerate(label[kind], 1):
                        xy = tuple(np.rint(np.array(point)*scale).astype(int))
                        cv2.circle(preview, xy, 6, color, 2)
                        cv2.putText(preview, str(i), xy, 0, .5, color, 1)
                banner = np.full((55, preview.shape[1], 3), 30, np.uint8)
                text = f"{p.stem} {mode}={len(label[mode])}  V: reviewed  S: save  N/P: next/prev  Q: save+quit"
                cv2.putText(banner, text, (5, 30), 0, .46, (255, 255, 255), 1)
                cv2.imshow(window, np.vstack([banner, preview]))
                key = cv2.waitKey(30) & 255
                if key in (ord('b'), ord('c'), ord('f'), ord('k')):
                    mode = {ord('b'): 'blocks', ord('c'): 'robot_center',
                            ord('f'): 'robot_front', ord('k'): 'field_corners'}[key]
                if key == ord('v'):
                    label['reviewed'] = True
                if key in (ord('s'), ord('n'), ord('p'), ord('q'), 27):
                    path.write_text(json.dumps(labels, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
                if key in (ord('q'), 27):
                    return
                if key in (ord('n'), ord('p')):
                    index = min(len(files), index+1) if key == ord('n') else max(0, index-1)
                    break
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    path.write_text(json.dumps(labels, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
                    return
    finally:
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=DATA)
    parser.add_argument('--output', type=Path, default=OUT)
    parser.add_argument('--annotate', action='store_true')
    args = parser.parse_args()
    files = sorted(args.input.glob('*.jpg'))
    if not files:
        raise SystemExit('No JPG photos found')
    label_path = args.input/'annotations.json'
    if args.annotate:
        annotate(files, label_path)
        return
    labels = json.loads(label_path.read_text(encoding='utf-8')) if label_path.exists() else {}
    args.output.mkdir(parents=True, exist_ok=True)
    rows, thumbnails = [], []
    for p in files:
        image = read_image(p)
        label = labels.get(p.name, {})
        baseline, tags, proposals, view = analyze(image, label)
        row = {'file': p.name, 'image_size': [image.shape[1], image.shape[0]],
               'field_detected': baseline['homography'] is not None,
               'field_source': baseline['field_source'],
               'automatic_field_corners_px': None if baseline['automatic_field_px'] is None else baseline['automatic_field_px'].tolist(),
               'field_corners_px': None if baseline['field_px'] is None else baseline['field_px'].tolist(),
               'boundary_y_m': baseline['boundary_y'],
               'strict_pixel_count': len(baseline['red_detections']),
               'strict_located_count': len(baseline['blocks']),
               'strict_blue_count': baseline['own_count'],
               'tags': tags, 'review_regions': proposals,
               'manual_reviewed': label.get('reviewed', False),
               'manual_saved_block_points': len(label.get('blocks', [])),
               'manual_visible_blocks': len(label.get('blocks', [])) if label.get('reviewed') else None}
        rows.append(row)
        marked = render(image, baseline, tags, proposals, label)
        save_image(args.output/f'{p.stem}_marked.jpg', marked)
        if view is not None:
            save_image(args.output/f'{p.stem}_field.jpg', view)
        thumb = np.full((355, 350, 3), 235, np.uint8)
        ratio = min(350/marked.shape[1], 325/marked.shape[0])
        small = cv2.resize(marked, None, fx=ratio, fy=ratio)
        thumb[30:30+small.shape[0], :small.shape[1]] = small
        cv2.putText(thumb, f"{p.stem} strict={row['strict_pixel_count']} tag={bool(tags)}",
                    (5, 22), 0, .5, (20, 20, 20), 1)
        thumbnails.append(thumb)
        print(f"{p.name}: strict={row['strict_pixel_count']} regions={len(proposals)} tag={bool(tags)}")
    while len(thumbnails) % 4:
        thumbnails.append(np.full_like(thumbnails[0], 235))
    sheet = np.vstack([np.hstack(thumbnails[i:i+4]) for i in range(0, len(thumbnails), 4)])
    save_image(args.output/'contact_sheet.jpg', sheet)
    report = {'warning': 'Offline proposals only; connected regions are not block counts. '
                        'Different photos may contain different numbers of blocks (user confirmed). '
                        'Field fit and tag decode do not validate metric accuracy or vehicle heading.',
              'photos': rows}
    (args.output/'detections.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
