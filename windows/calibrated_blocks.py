"""Experimental HSV + calibrated-size instance segmentation, with no labels.

Requires image-to-field homography in metres. The extra 40 mm margin preserves
wall-side blocks. Segments are image evidence/size estimates, not exact counts.
Used by the offline photo tools and the Windows task runner.
"""
import cv2
import numpy as np

from field_vision import image_to_world


def robot_regions(view, tags, transform):
    """Conservative visual chassis hulls, not a radius that erases nearby blocks."""
    hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)
    green = cv2.inRange(hsv, (40, 65, 35), (90, 255, 255))
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, stats, centers = cv2.connectedComponentsWithStats(green)
    seeds = [centers[i] for i in range(1, n) if 60 <= stats[i, 4] <= 5000
             and min(stats[i, 2:4]) >= 5 and 20 < centers[i, 0] < 1260 and 20 < centers[i, 1] < 1260]
    for tag in tags:
        if tag['id'] == 0:
            seeds.append(image_to_world(transform, tag['corners_px']).mean(axis=0))
    # Green PCB identifies a body region even if the tag cannot be decoded.
    feature = ((hsv[:, :, 1] < 65) & (hsv[:, :, 2] > 100)) | (green > 0)
    hulls = []
    used = []
    for seed in seeds:
        if any(np.linalg.norm(seed-old) < 150 for old in used):
            continue
        # Only pixels in the seed's local box can be within the same radius.
        sx, sy = seed
        x0, x1 = max(0, int(sx)-150), min(feature.shape[1], int(sx)+151)
        y0, y1 = max(0, int(sy)-150), min(feature.shape[0], int(sy)+151)
        yy, xx = np.where(feature[y0:y1, x0:x1])
        pixels = np.column_stack([xx + x0, yy + y0])
        selected = pixels[(pixels[:, 0]-sx)**2 + (pixels[:, 1]-sy)**2 < 150**2]
        if len(selected) < 200:
            continue
        hull = cv2.convexHull(np.float32(selected))
        if 1500 < cv2.contourArea(hull) < 50000:
            hulls.append(hull)
            used.append(seed)
    return hulls


def detect_calibrated_blocks(image, homography, tags=(), stats=None):
    """Return block instances in original-image pixels.

    `stats` is an optional dict that receives this frame's decision counters
    (components, per-frame instance-area estimate, splits, drops) for
    diagnostics. It never changes the returned detections.
    """
    if stats is not None:
        stats.update(reason='', components=0, instance_area=None, splits=0,
                     seeds=0, kept=0, dropped=0)
    if homography is None:
        if stats is not None:
            stats['reason'] = 'no_homography'
        return []
    transform = np.float64([[1000, 0, 640], [0, -1000, 640], [0, 0, 1]]) @ homography
    view = cv2.warpPerspective(image, transform, (1280, 1280))
    hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)
    red = (cv2.inRange(hsv, (0, 70, 35), (12, 255, 255)) |
           cv2.inRange(hsv, (165, 70, 35), (180, 255, 255)))
    # Blue-floor reflections can shift red faces toward purple (H=155..164).
    # Admit that supplementary band only near observed blue ground, retaining
    # saturation/brightness and calibrated instance checks below.
    blue = cv2.inRange(hsv, (100, 80, 40), (130, 255, 255))
    near_blue = cv2.dilate(blue, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (45, 45)))
    reflected = cv2.inRange(hsv, (155, 85, 35), (164, 255, 255))
    red |= cv2.bitwise_and(reflected, near_blue)
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    bodies = robot_regions(view, tags, transform)
    count, components, component_stats, _ = cv2.connectedComponentsWithStats(red)
    inverse = np.linalg.inv(transform)
    detections = []
    targets_total = 0
    # Estimate visible face+side area from compact isolated candidates in this
    # frame, since camera tilt changes the apparent cube silhouette area.
    samples = []
    for x, y, w, h, area in component_stats[1:]:
        center = (float(x+w/2), float(y+h/2))
        if (200 < area < 850 and min(w, h) > 12 and max(w, h) < 38
                and min(w, h)/max(w, h) > .6
                and not any(cv2.pointPolygonTest(b, center, True) > 0 for b in bodies)):
            samples.append(area)
    instance_area = float(np.clip(np.median(samples), 260, 600))*1.15 if samples else 460.
    if stats is not None:
        stats['instance_area'] = round(instance_area, 1)
    for component in range(1, count):
        x, y, w, h, area = component_stats[component]
        # A dense wall pile can contain most of the twenty cubes. Rejecting
        # it as an oversized single object loses the entire connected group.
        # Keep a bounded group budget, then apply single-instance geometry
        # after partitioning below.
        if not 70 <= area <= 30000 or min(w, h) < 8 or max(w, h) > 400:
            continue
        if stats is not None:
            stats['components'] += 1
        wall = x < 70 or y < 70 or x+w > 1210 or y+h > 1210
        mask = np.uint8(components[y:y+h, x:x+w] == component)*255
        padded = cv2.copyMakeBorder(mask, 2, 2, 2, 2, cv2.BORDER_CONSTANT)
        distance = cv2.distanceTransform(padded, cv2.DIST_L2, 5)
        distance = cv2.GaussianBlur(distance, (3, 3), .7)
        maxima = np.uint8((distance >= cv2.dilate(distance, np.ones((9, 9), np.uint8))-.01) & (distance >= 4))*255
        n, _, _, centers = cv2.connectedComponentsWithStats(maxima)
        seeds = []
        ranked = sorted(centers[1:], key=lambda p: -distance[round(p[1]), round(p[0])])
        for point in ranked:
            if all(np.linalg.norm(point-old) >= 14 for old in seeds):
                seeds.append(point)
        if not seeds:
            continue
        if stats is not None:
            stats['seeds'] += len(seeds)
        # A broad flat distance ridge cannot reveal all touching cube seams.
        # Use a physical size estimate only when it supplies a missing seed.
        estimated = 1 if wall else max(1, round(area/instance_area))
        target = max(len(seeds), estimated)
        target = min(target, max(1, round(area/140.)), 32)
        targets_total += target
        yy, xx = np.where(padded > 0)
        pixels = np.float32(np.column_stack([xx, yy]))
        if target == 1:
            assignments = np.zeros(len(pixels), np.int32)
        else:
            # Deterministic farthest-point fill; Lloyd partition is independent
            # of manual coordinates and has no filename-specific decisions.
            centers = list(seeds[:target])
            while len(centers) < target:
                d = np.min(np.sum((pixels[:, None]-np.array(centers)[None])**2, axis=2), axis=1)
                centers.append(pixels[np.argmax(d)])
            centers = np.float32(centers)
            for _ in range(20):
                assignments = np.argmin(np.sum((pixels[:, None]-centers[None])**2, axis=2), axis=1)
                updated = np.float32([pixels[assignments == i].mean(axis=0)
                                     if np.any(assignments == i) else centers[i] for i in range(target)])
                if np.max(np.linalg.norm(updated-centers, axis=1)) < .1:
                    break
                centers = updated
        for i in range(target):
            part = np.zeros_like(padded)
            part[yy[assignments == i], xx[assignments == i]] = 255
            cs, _ = cv2.findContours(part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cs:
                continue
            contour = max(cs, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            sides = sorted(cv2.minAreaRect(contour)[1])
            hull = cv2.contourArea(cv2.convexHull(contour))
            if area < 65 or sides[0] < 7 or sides[1] > 43 or sides[0]/max(sides[1], 1) < .38 or area/max(hull, 1) < .65:
                continue
            contour_view = contour + np.int32([[[x-2, y-2]]])
            center_view = contour_view.reshape(-1, 2).mean(axis=0)
            if any(cv2.pointPolygonTest(body, tuple(map(float, center_view)), True) > 2 for body in bodies):
                continue
            original = image_to_world(inverse, contour_view).reshape(-1, 1, 2)
            moment = cv2.moments(original)
            if moment['m00'] <= 0:
                continue
            center = (moment['m10']/moment['m00'], moment['m01']/moment['m00'])
            detections.append({'center_px': center, 'contour_px': original,
                               'bbox_px': cv2.boundingRect(original), 'area_mm2': area,
                               'split': target > 1, 'size_inferred_seed': target > len(seeds)})
    if stats is not None:
        stats.update(kept=len(detections), splits=sum(1 for d in detections if d['split']),
                     dropped=max(0, targets_total - len(detections)))
    return detections
