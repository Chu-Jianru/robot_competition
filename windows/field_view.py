"""场地四角标定与透视校正。

作用：把高分辨率俯视画面（如 1920x1080）按场地四角裁剪并透视校正到正方形
"场地视图"，之后所有几何量都以场地毫米为基准，不需要相机内参。

用法：
    # 1) 用一张拍好的原图点四个角（顺序：左上、右上、右下、左下，按场地实际方位）
    python windows/field_view.py calibrate windows/captures/xxx_raw.png --out windows/field_calib.json

    # 2) 用已保存的标定预览效果
    python windows/field_view.py preview windows/captures/xxx_raw.png --config windows/field_calib.json

角度约定：场地坐标 x 向右（东），y 向上（北为正，与比赛约定一致）。

注意：俯视正对场地时四角顺序为 场地左上→右上→右下→左下。若相机方向旋转，
按"画面上看到的顺序"点，脚本会把图像坐标换算到场地坐标时保持一致。
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

FIELD_MM = 1200.0
DEFAULT_SIZE = 960          # 校正后边长（像素）；1.25 mm/px
CORNER_ORDER = ("左上", "右上", "右下", "左下")


@dataclass
class FieldCalibration:
    corners: np.ndarray      # (4, 2) 原始画面中的像素坐标，顺序 左上/右上/右下/左下
    size: int = DEFAULT_SIZE
    field_mm: float = FIELD_MM
    source_size: tuple[int, int] | None = None   # 标定所用原图 (w, h)

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


def calibrate(image_path: Path, out_path: Path, size: int, field_mm: float) -> None:
    img = cv2.imread(str(image_path))
    if img is None:
        raise SystemExit(f"读不到图片：{image_path}")
    print(f"原图 {img.shape[1]}x{img.shape[0]}")
    print(f"依次点击场地四角：{', '.join(CORNER_ORDER)}（按画面看到的顺序，左键点选，右键撤销，回车确认）")

    clicks: list[tuple[int, int]] = []
    window = "field calibration"

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN and clicks:
            clicks.pop()

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    while True:
        preview = img.copy()
        for index, (x, y) in enumerate(clicks):
            cv2.circle(preview, (x, y), 6, (0, 255, 0), 2)
            cv2.putText(preview, CORNER_ORDER[index], (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if len(clicks) >= 2:
            cv2.polylines(preview, [np.array(clicks, np.int32)], len(clicks) == 4, (0, 255, 255), 1)
        cv2.imshow(window, preview)
        key = cv2.waitKey(20) & 0xFF
        if key in (13, 10) and len(clicks) == 4:
            break
        if key == 27:
            cv2.destroyAllWindows()
            print("已取消四角标定")
            raise SystemExit(130)
    cv2.destroyAllWindows()

    calib = FieldCalibration(np.array(clicks, np.float32), size=size, field_mm=field_mm,
                             source_size=(img.shape[1], img.shape[0]))
    calib.save(out_path)
    print(f"已保存 {out_path}；四角残差 {calib.residuals():.2f} px（应接近 0）")
    print(f"校正后 {size}x{size}，{calib.mm_per_px:.3f} mm/px；"
          f"50 mm tag 约 {50 / calib.mm_per_px:.1f} px，20 mm 红块约 {20 / calib.mm_per_px:.1f} px")


def preview(image_path: Path, config_path: Path) -> None:
    calib = FieldCalibration.load(config_path)
    img = cv2.imread(str(image_path))
    if img is None:
        raise SystemExit(f"读不到图片：{image_path}")
    if calib.source_size and (img.shape[1], img.shape[0]) != tuple(calib.source_size):
        print(f"警告：标定基于 {calib.source_size}，当前图 {img.shape[1]}x{img.shape[0]}，点位可能不匹配")
    view = draw_overlay(calib.warp(img), calib)
    out = config_path.with_name(config_path.stem + "_preview.png")
    cv2.imwrite(str(out), view)
    cv2.imshow("warped field", view)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print(f"预览已保存 {out}")


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
