"""在透视校正后的场地视图里检测车顶 ArUco tag，输出场地毫米坐标与朝向。

设计要点：
- 输入是 field_view 校正过的正方形场地视图，因此检测器只需处理"正俯视"图像。
- 不需要相机内参：四角已由标定确定，tag 四角直接换算成场地毫米。
- 三道自检：边长与标称一致（±15%）、ID 白名单、连续两帧一致。

角度约定：heading = tag 的"上边"在场地坐标中的方向，0° 指东（+x），逆时针为正。
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from field_view import FieldCalibration
from tag_detector import TAG_ID, detect_markers


@dataclass
class TagPose:
    x_mm: float
    y_mm: float
    heading_deg: float
    side_mm: float
    tag_id: int
    center_px: tuple[float, float]
    corners_px: np.ndarray | None = None

    def __str__(self) -> str:
        return (f"id={self.tag_id} x={self.x_mm:7.1f} y={self.y_mm:7.1f} "
                f"heading={self.heading_deg:7.1f} side={self.side_mm:5.1f}mm")


def detect(
    view_bgr: np.ndarray,
    calib: FieldCalibration,
    tag_mm: float = 50.0,
    expected_id: int | None = TAG_ID,
    side_tol: float = 0.15,
    *,
    unique: bool = False,
) -> tuple[TagPose | None, str]:
    """返回 (位姿或 None, 原因)。原因用于日志："ok" / "no_marker" / "id_mismatch" / "side_mismatch"。"""
    gray = view_bgr if view_bgr.ndim == 2 else cv2.cvtColor(view_bgr, cv2.COLOR_BGR2GRAY)
    markers = detect_markers(gray, expected_id)
    if not markers:
        return None, "no_marker"
    matching = [m for m in markers if expected_id is None or m.tag_id == expected_id]
    if not matching:
        return None, "id_mismatch"
    if unique and len(matching) != 1:
        return None, "multiple_markers"

    candidates = []
    for marker in matching:
        pts, marker_id = marker.corners, marker.tag_id
        edges = [float(np.linalg.norm(pts[i] - pts[(i + 1) % 4])) for i in range(4)]
        side_px = float(np.mean(edges))
        side_mm = side_px * calib.mm_per_px
        candidates.append((abs(side_mm - tag_mm), pts, side_mm, int(marker_id)))

    candidates.sort(key=lambda item: item[0])
    err, pts, side_mm, marker_id = candidates[0]

    if expected_id is not None and marker_id != expected_id:
        return None, "id_mismatch"
    if abs(side_mm - tag_mm) / tag_mm > side_tol:
        return None, "side_mismatch"

    mm = calib.px_to_mm(pts)          # (4,2) 场地毫米，y 北为正
    up = mm[0] - mm[3]                # 左上 - 左下 = "上边"方向（tag 自带朝向）
    heading = math.degrees(math.atan2(float(up[1]), float(up[0]))) % 360.0
    center_mm = mm.mean(axis=0)
    center_px = pts.mean(axis=0)
    return TagPose(
        x_mm=float(center_mm[0]),
        y_mm=float(center_mm[1]),
        heading_deg=heading,
        side_mm=side_mm,
        tag_id=marker_id,
        center_px=(float(center_px[0]), float(center_px[1])),
        corners_px=pts.astype(np.float32),
    ), "ok"


class PoseFilter:
    """连续两帧一致才认定位姿有效；单帧跳变视为误检。"""

    def __init__(self, pos_tol_mm: float = 10.0, heading_tol_deg: float = 3.0) -> None:
        self.pos_tol_mm = pos_tol_mm
        self.heading_tol_deg = heading_tol_deg
        self.last: TagPose | None = None

    def reset(self) -> None:
        self.last = None

    def update(self, pose: TagPose | None) -> TagPose | None:
        if pose is None:
            self.last = None
            return None
        if self.last is None:
            self.last = pose
            return pose
        dist = math.hypot(pose.x_mm - self.last.x_mm, pose.y_mm - self.last.y_mm)
        diff = abs(pose.heading_deg - self.last.heading_deg) % 360.0
        diff = min(diff, 360.0 - diff)
        if dist <= self.pos_tol_mm and diff <= self.heading_tol_deg:
            self.last = pose
            return pose
        self.last = pose
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description="单张图片测 tag 位姿")
    parser.add_argument("image", type=Path)
    parser.add_argument("--config", type=Path, default=Path("windows/field_calib.json"))
    parser.add_argument("--tag-mm", type=float, default=50.0)
    parser.add_argument("--id", type=int, default=TAG_ID)
    args = parser.parse_args()

    calib = FieldCalibration.load(args.config)
    img = cv2.imread(str(args.image))
    if img is None:
        raise SystemExit(f"读不到图片：{args.image}")
    pose, reason = detect(calib.warp(img), calib, args.tag_mm, args.id)
    print(f"{args.image.name}: {reason}")
    if pose:
        print("  " + str(pose))


if __name__ == "__main__":
    main()
