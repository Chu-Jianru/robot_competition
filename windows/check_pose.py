"""离线批量验收：跑一整个目录的图，输出检出率与位姿抖动。

用法：
    python windows/check_pose.py windows/captures --config windows/field_calib.json
    python windows/check_pose.py windows/captures --config windows/field_calib.json \\
        --truth windows/captures/truth.csv --csv out.csv

truth.csv 可选，格式（表头必需）：
    file,x_mm,y_mm,heading_deg
    A_center_raw.png,600,600,90

抖动按"同一位姿多帧"统计：默认把全部帧当作静止样本来算标准差；
给了 truth.csv 时，同一真值的帧才归为一组。
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

import cv2

from field_view import FieldCalibration
from pose_tag import PoseFilter, detect
from tag_detector import TAG_ID

IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".bmp")


def load_truth(path: Path) -> dict[str, tuple[float, float, float]]:
    truth = {}
    with path.open(newline="", encoding="utf-8-sig") as fh:
        for row in csv.DictReader(fh):
            truth[row["file"]] = (float(row["x_mm"]), float(row["y_mm"]), float(row["heading_deg"]))
    return truth


def angle_diff(a: float, b: float) -> float:
    diff = abs(a - b) % 360.0
    return min(diff, 360.0 - diff)


def summarize(values: list[float]) -> str:
    if not values:
        return "--"
    if len(values) == 1:
        return f"{values[0]:.2f}"
    return f"P50 {statistics.median(values):.2f} / max {max(values):.2f}"


def main() -> None:
    parser = argparse.ArgumentParser(description="批量测 tag 位姿并统计")
    parser.add_argument("images", type=Path, help="图片目录")
    parser.add_argument("--config", type=Path, default=Path("windows/field_calib.json"))
    parser.add_argument("--tag-mm", type=float, default=50.0)
    parser.add_argument("--id", type=int, default=TAG_ID)
    parser.add_argument("--truth", type=Path)
    parser.add_argument("--csv", type=Path, help="把逐帧结果写到这个 CSV")
    parser.add_argument("--filter", action="store_true", help="启用连续两帧一致性过滤")
    args = parser.parse_args()

    calib = FieldCalibration.load(args.config)
    truth = load_truth(args.truth) if args.truth else {}
    files = sorted(p for p in args.images.rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES)
    files = [p for p in files if "_annotated" not in p.name]

    pose_filter = PoseFilter()
    rows, reasons = [], defaultdict(int)
    pos_err, head_err = [], []
    groups: dict[tuple[float, float, float], list[tuple[float, float, float]]] = defaultdict(list)

    for path in files:
        img = cv2.imread(str(path))
        if img is None:
            reasons["unreadable"] += 1
            continue
        pose, reason = detect(calib.warp(img), calib, args.tag_mm, args.id)
        if args.filter:
            pose = pose_filter.update(pose)
            if pose is None and reason == "ok":
                reason = "filtered"
        reasons[reason] += 1

        row = {"file": path.name, "reason": reason}
        if pose is not None:
            row.update(x_mm=f"{pose.x_mm:.2f}", y_mm=f"{pose.y_mm:.2f}",
                       heading_deg=f"{pose.heading_deg:.2f}", side_mm=f"{pose.side_mm:.2f}")
            ref = truth.get(path.name)
            if ref is not None:
                dx, dy, dh = pose.x_mm - ref[0], pose.y_mm - ref[1], angle_diff(pose.heading_deg, ref[2])
                row.update(err_mm=f"{math.hypot(dx, dy):.2f}", err_deg=f"{dh:.2f}")
                pos_err.append(math.hypot(dx, dy))
                head_err.append(dh)
                groups[ref].append((pose.x_mm, pose.y_mm, pose.heading_deg))
        rows.append(row)
        print(f"{path.name:40s} {reason:14s} "
              + (f"x={pose.x_mm:7.1f} y={pose.y_mm:7.1f} h={pose.heading_deg:6.1f}" if pose else ""))

    if args.csv:
        with args.csv.open("w", newline="", encoding="utf-8-sig") as fh:
            writer = csv.DictWriter(fh, fieldnames=["file", "reason", "x_mm", "y_mm",
                                                    "heading_deg", "side_mm", "err_mm", "err_deg"])
            writer.writeheader()
            for row in rows:
                writer.writerow(row)
        print(f"\n逐帧结果已写入 {args.csv}")

    total = len(files)
    ok = reasons.get("ok", 0)
    print("\n=== 汇总 ===")
    print(f"图片 {total} 张，有效 {ok} 张，检出率 {ok / total * 100:.1f}%" if total else "没有图片")
    for reason, count in sorted(reasons.items()):
        if reason != "ok":
            print(f"  未给出位姿：{reason} = {count}")
    if pos_err:
        print(f"位置误差 mm：{summarize(pos_err)}")
        print(f"朝向误差 deg：{summarize(head_err)}")

    jitters_pos, jitters_head = [], []
    for _key, samples in groups.items():
        if len(samples) < 2:
            continue
        xs = [s[0] for s in samples]
        ys = [s[1] for s in samples]
        jitters_pos.append(math.hypot(statistics.pstdev(xs), statistics.pstdev(ys)))
        base = samples[0][2]
        jitters_head.append(max(angle_diff(s[2], base) for s in samples))
    if jitters_pos:
        print(f"同一位置多帧抖动（位置）：{summarize(jitters_pos)} mm")
        print(f"同一位置多帧抖动（朝向）：{summarize(jitters_head)} deg")


if __name__ == "__main__":
    main()
