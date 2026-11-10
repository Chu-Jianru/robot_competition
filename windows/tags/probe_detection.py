"""离线探针：在 640x480 俯视尺度下，测 4x4_50 tag 的检出率与位姿精度。

不连接相机、不写文件；只回答一个问题：
"5 cm 的 tag 在每模块 3.5 px 左右、有模糊和噪声时，还能不能稳定读出来，误差多大？"

用法：
    python windows/tags/probe_detection.py
"""

from __future__ import annotations

import math

import cv2
import numpy as np

FIELD_PX = 500.0          # 1.2 m 场地占 500 px -> 2.4 mm/px
FIELD_MM = 1200.0
MM_PER_PX = FIELD_MM / FIELD_PX
CANVAS = (640, 480)
FIELD_ORIGIN = (70, 20)   # 场地左上角在画布中的像素位置
TRIALS = 24
SIDES_PX = [21, 25, 27, 33, 41]      # 对应 tag 边长 50/60/65/79/98 mm
BLURS = [0.6, 1.0, 1.4]              # 高斯模糊 sigma（px）


def make_tag(side_px: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    big = cv2.aruco.generateImageMarker(dictionary, 0, 600)
    return cv2.resize(big, (side_px, side_px), interpolation=cv2.INTER_AREA)


def make_background(rng: np.random.Generator) -> np.ndarray:
    img = np.full((CANVAS[1], CANVAS[0], 3), 110, np.uint8)
    x0, y0 = FIELD_ORIGIN
    # 蓝黄各半的场地底色
    cv2.rectangle(img, (x0, y0), (x0 + 250, y0 + 500), (150, 90, 60), -1)
    cv2.rectangle(img, (x0 + 250, y0), (x0 + 500, y0 + 500), (70, 160, 190), -1)
    # 20 个 2 cm 红块（约 8 px）
    for _ in range(20):
        cx = int(rng.integers(x0 + 15, x0 + 485))
        cy = int(rng.integers(y0 + 15, y0 + 485))
        cv2.rectangle(img, (cx - 4, cy - 4), (cx + 4, cy + 4), (40, 40, 190), -1)
    # 轻微亮度梯度 + 噪声
    grad = np.linspace(-8, 8, CANVAS[1])[:, None, None]
    img = np.clip(img.astype(np.float32) + grad, 0, 255)
    img += rng.normal(0, 3.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def paste_tag(img: np.ndarray, tag: np.ndarray, cx: float, cy: float, angle_deg: float) -> None:
    side = tag.shape[0]
    pad = max(2, side // 6)                      # 1 模块白边
    canvas = np.full((side + 2 * pad, side + 2 * pad, 3), 245, np.uint8)
    canvas[pad:pad + side, pad:pad + side] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
    h, w = canvas.shape[:2]
    m = cv2.getRotationMatrix2D((w / 2, h / 2), angle_deg, 1.0)
    m[0, 2] += cx - w / 2
    m[1, 2] += cy - h / 2
    warped = cv2.warpAffine(canvas, m, (img.shape[1], img.shape[0]),
                            flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
    mask = cv2.warpAffine(np.full((h, w), 255, np.uint8), m, (img.shape[1], img.shape[0]),
                          flags=cv2.INTER_NEAREST)
    img[:] = np.where(mask[..., None] > 0, warped, img)


def detect(gray: np.ndarray):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 5
    params.adaptiveThreshWinSizeMax = 45
    params.minMarkerPerimeterRate = 0.02
    params.polygonalApproxAccuracyRate = 0.05
    params.errorCorrectionRate = 0.6
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    corners, ids, _ = detector.detectMarkers(gray)
    return corners, ids


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--jpeg", type=int, default=0, help="附加 JPEG 压缩质量，模拟 scrcpy 的编码失真")
    args = parser.parse_args()
    rng = np.random.default_rng(20261102)
    x0, y0 = FIELD_ORIGIN
    h_src = np.array([[x0, y0], [x0 + FIELD_PX, y0],
                      [x0 + FIELD_PX, y0 + FIELD_PX], [x0, y0 + FIELD_PX]], np.float32)
    h_dst = np.array([[0, 0], [FIELD_MM, 0], [FIELD_MM, FIELD_MM], [0, FIELD_MM]], np.float32)
    homography = cv2.getPerspectiveTransform(h_src, h_dst)

    print(f"{'tag_px':>7} {'blur':>5} {'detect':>7} {'pos_err_mm(P50/P95)':>22} {'yaw_err_deg(P50/P95)':>22}")
    for side in SIDES_PX:
        tag = make_tag(side)
        for blur in BLURS:
            pos_errs, yaw_errs, hits = [], [], 0
            for _ in range(TRIALS):
                img = make_background(rng)
                cx = float(rng.integers(int(x0 + 40), int(x0 + 460)))
                cy = float(rng.integers(int(y0 + 40), int(y0 + 460)))
                angle = float(rng.uniform(0, 360))
                paste_tag(img, tag, cx, cy, angle)
                if blur > 0:
                    img = cv2.GaussianBlur(img, (0, 0), blur)
                if args.jpeg:
                    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg])
                    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                corners, ids = detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
                if ids is None or len(ids) == 0:
                    continue
                hits += 1
                pts = corners[0].reshape(4, 2).astype(np.float32)
                mapped = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), homography).reshape(4, 2)
                center = mapped.mean(axis=0)
                # 真值：tag 中心与 yaw
                truth = cv2.perspectiveTransform(
                    np.array([[[cx, cy]]], np.float32), homography).reshape(2)
                pos_errs.append(float(np.linalg.norm(center - truth)))
                # 取上面那条边（角点 0-1）的方向，换算回场地角度
                edge = mapped[1] - mapped[0]
                measured = math.degrees(math.atan2(edge[1], edge[0])) % 360
                diff = (measured - (-angle)) % 360
                yaw_errs.append(min(diff, 360 - diff))
            rate = hits / TRIALS * 100
            if pos_errs:
                p50 = np.percentile(pos_errs, 50)
                p95 = np.percentile(pos_errs, 95)
                y50 = np.percentile(yaw_errs, 50)
                y95 = np.percentile(yaw_errs, 95)
                print(f"{side:>7} {blur:>5.1f} {rate:>6.0f}% {p50:>9.2f} /{p95:>9.2f}      {y50:>9.2f} /{y95:>9.2f}")
            else:
                print(f"{side:>7} {blur:>5.1f} {rate:>6.0f}% {'--':>22} {'--':>22}")


if __name__ == "__main__":
    main()
