"""场地四角标定与透视校正。

作用：把高分辨率俯视画面（如 1920x1080）按场地四角裁剪并透视校正到正方形
"场地视图"，之后所有几何量都以场地毫米为基准，不需要相机内参。

用法：
    # 1) 用一张拍好的原图点四个地面内角，顺序不限
    python windows/field_view.py calibrate windows/captures/xxx_raw.png --out windows/field_calib.json

    # 2) 用已保存的标定预览效果
    python windows/field_view.py preview windows/captures/xxx_raw.png --config windows/field_calib.json

按蓝黄地面分界确定场地方向，校正图为黄上蓝下。场地坐标 x 向右（东），y 向上（北）。
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from daily_tools import menu_text
from field_vision import HSV, find_seam

FIELD_MM = 1200.0
DEFAULT_SIZE = 960          # 校正后边长（像素）；1.25 mm/px


@dataclass
class FieldCalibration:
    corners: np.ndarray      # 原图像素，依次对应校正图的左上、右上、右下、左下
    size: int = DEFAULT_SIZE
    field_mm: float = FIELD_MM
    source_size: tuple[int, int] | None = None   # 标定所用原图 (w, h)

    @classmethod
    def from_clicks(cls, image, clicks, *, size=DEFAULT_SIZE, field_mm=FIELD_MM):
        """Order four ground corners and orient the square using the observed seam."""
        points = np.asarray(clicks, dtype=np.float32)
        if points.shape != (4, 2) or not np.isfinite(points).all():
            raise ValueError("请点四个有效的地面内角。")
        corners = cv2.convexHull(points, clockwise=False).reshape(-1, 2)
        if len(corners) != 4 or cv2.contourArea(corners, oriented=True) < 100:
            raise ValueError("四个点未围成场地四边形。请撤销点错的角或重合的点。")

        height, width = image.shape[:2]
        if np.any(corners < 2) or np.any(corners >= np.array([width, height]) - 2):
            raise ValueError("请用四个地面内角都完整入镜的原图。")
        # The hull has the right winding. Only its starting corner is unknown.
        hsv = cv2.cvtColor(cls(corners).warp(image), cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(hsv, *HSV["blue"])
        yellow = cv2.inRange(hsv, *HSV["yellow"])
        matching_turns = [turns for turns in range(4)
                          if find_seam(np.rot90(blue, turns), np.rot90(yellow, turns)) is not None]
        if len(matching_turns) != 1:
            raise ValueError("无法确认蓝黄分界线。请核对四个地面内角，或重拍清晰的全场原图。")
        ordered = np.roll(corners, -matching_turns[0], axis=0)
        return cls(ordered, size=size, field_mm=field_mm, source_size=(width, height))

    @property
    def mm_per_px(self) -> float:
        return self.field_mm / self.size

    def homography(self) -> np.ndarray:
        dst = np.array([[0, 0], [self.size, 0], [self.size, self.size], [0, self.size]], np.float32)
        return cv2.getPerspectiveTransform(self.corners.astype(np.float32), dst)

    def warp(self, frame: np.ndarray) -> np.ndarray:
        return cv2.warpPerspective(frame, self.homography(), (self.size, self.size),
                                   flags=cv2.INTER_LINEAR)

    def px_to_mm(self, points: np.ndarray) -> np.ndarray:
        """场地视图像素 -> 场地毫米，y 轴翻转成北为正。"""
        pts = np.asarray(points, np.float32).reshape(-1, 2)
        mm = pts * self.mm_per_px
        mm[:, 1] = self.field_mm - mm[:, 1]
        return mm

    def to_dict(self) -> dict:
        return {
            "corners": self.corners.tolist(),
            "size": self.size,
            "field_mm": self.field_mm,
            "source_size": list(self.source_size) if self.source_size else None,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "FieldCalibration":
        return cls(
            corners=np.array(data["corners"], np.float32),
            size=int(data.get("size", DEFAULT_SIZE)),
            field_mm=float(data.get("field_mm", FIELD_MM)),
            source_size=tuple(data["source_size"]) if data.get("source_size") else None,
        )

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> "FieldCalibration":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def residuals(self) -> float:
        """四角校正残差（像素，场地视图下），用于判断点位是否点歪。"""
        h = self.homography()
        src = self.corners.astype(np.float32).reshape(-1, 1, 2)
        projected = cv2.perspectiveTransform(src, h).reshape(-1, 2)
        target = np.array([[0, 0], [self.size, 0], [self.size, self.size], [0, self.size]], np.float32)
        return float(np.abs(projected - target).max())


def draw_overlay(view: np.ndarray, calib: FieldCalibration) -> np.ndarray:
    out = view.copy()
    size = calib.size
    step = size // 6
    for i in range(1, 6):
        cv2.line(out, (i * step, 0), (i * step, size), (0, 200, 0), 1)
        cv2.line(out, (0, i * step), (size, i * step), (0, 200, 0), 1)
    cv2.line(out, (0, size // 2), (size, size // 2), (0, 0, 255), 2)
    cv2.line(out, (size // 2, 0), (size // 2, size), (255, 0, 0), 2)
    cv2.putText(out, "N", (size // 2 + 8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.putText(out, f"{calib.mm_per_px:.3f} mm/px", (8, size - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return out


def calibrate_image(img: np.ndarray, out_path: Path, window: str,
                    *, size: int = DEFAULT_SIZE, field_mm: float = FIELD_MM) -> FieldCalibration | None:
    print("左键点四个地面内角，顺序不限。右键撤销，Q/Esc 返回。")

    clicks: list[tuple[int, int]] = []
    calib = None

    def on_mouse(event, x, y, _flags, _param):
        nonlocal calib
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))
            if len(clicks) == 4:
                try:
                    calib = FieldCalibration.from_clicks(img, clicks, size=size, field_mm=field_mm)
                    print(menu_text("已确认蓝黄分界线。回车保存，右键撤销。", 32))
                except ValueError as error:
                    print(menu_text(str(error), 31))
        elif event == cv2.EVENT_RBUTTONDOWN and clicks:
            clicks.pop()
            calib = None

    cv2.setMouseCallback(window, on_mouse)
    try:
        while True:
            preview = img.copy()
            for x, y in clicks:
                cv2.circle(preview, (x, y), 6, (0, 255, 0), 2)
            if calib is not None:
                cv2.polylines(preview, [calib.corners.astype(np.int32)], True, (0, 255, 255), 2)
            cv2.imshow(window, preview)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                return None
            if key in (13, 10) and calib is not None:
                calib.save(out_path)
                print(menu_text(f"标定已保存到 {out_path}。", 32))
                return calib
    finally:
        if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) >= 1:
            cv2.setMouseCallback(window, lambda *_: None)


def calibrate(image_path: Path, out_path: Path, size: int, field_mm: float) -> None:
    img = cv2.imread(str(image_path))
    if img is None:
        raise SystemExit(f"读不到图片：{image_path}")
    window = "field calibration"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        if calibrate_image(img, out_path, window, size=size, field_mm=field_mm) is None:
            print("已取消四角标定")
            raise SystemExit(130)
    finally:
        cv2.destroyAllWindows()


def save_preview(image: np.ndarray, calib: FieldCalibration, config_path: Path) -> np.ndarray:
    view = draw_overlay(calib.warp(image), calib)
    out = config_path.with_name(config_path.stem + "_preview.png")
    if not cv2.imwrite(str(out), view):
        print(menu_text(f"无法保存校正预览图：{out}", 31))
    return view


def preview(image_path: Path, config_path: Path) -> None:
    calib = FieldCalibration.load(config_path)
    img = cv2.imread(str(image_path))
    if img is None:
        raise SystemExit(f"读不到图片：{image_path}")
    if calib.source_size and (img.shape[1], img.shape[0]) != tuple(calib.source_size):
        print(f"警告：标定基于 {calib.source_size}，当前图 {img.shape[1]}x{img.shape[0]}，点位可能不匹配")
    cv2.imshow("warped field", save_preview(img, calib, config_path))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="场地四角标定与透视校正")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_cal = sub.add_parser("calibrate", help="在图片上点四个角并保存标定")
    p_cal.add_argument("image", type=Path)
    p_cal.add_argument("--out", type=Path, default=Path("windows/field_calib.json"))
    p_cal.add_argument("--size", type=int, default=DEFAULT_SIZE)
    p_cal.add_argument("--field-mm", type=float, default=FIELD_MM)

    p_prev = sub.add_parser("preview", help="用标定预览校正效果")
    p_prev.add_argument("image", type=Path)
    p_prev.add_argument("--config", type=Path, default=Path("windows/field_calib.json"))

    args = parser.parse_args()
    if args.cmd == "calibrate":
        calibrate(args.image, args.out, args.size, args.field_mm)
    else:
        preview(args.image, args.config)


if __name__ == "__main__":
    main()
