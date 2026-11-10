"""Record continuous phone frames and log the per-frame vision decision.

Answers "why does the block list jump" without labelling anything: every analysed
frame reports whether the tag decoded, whether the field seam was found, how many
red components were seen, what per-frame instance area split them, and where each
block ended up. The summary turns those rows into the three numbers that matter:

  * whole-frame rejection  (% frames with no block list at all)
  * block-count swing      (median / P90 of |count(frame i+1) - count(frame i)|)
  * position jitter        (nearest-neighbour displacement of matched blocks)

Two modes, matching the two live paths:
  field (needs windows/field_calib.json): identical calls to run_task.observe.
  bench (no calibration): tag 50 mm gives mm/px, as in capture_hi --bench.

Read-only: no chassis command, no window unless --show is given.

Usage:
    & 'E:\\scrcpy\\scrcpy-win64-v5.0.1\\.venv\\Scripts\\python.exe' windows\\record_diag.py ^
        --scrcpy-dir 'E:\\scrcpy\\scrcpy-win64-v5.0.1' --secs 15 --save-frames

    # after calibrating the field
    ... windows\\record_diag.py --config windows\\field_calib.json --secs 20 --save-frames
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import queue
import statistics
import sys
import threading
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from capture_hi import open_camera, rotate                             # noqa: E402
from tag_detector import TAG_ID                                       # noqa: E402

HERE = Path(__file__).resolve().parent
DEFAULT_SCRCPY = Path(r"E:\scrcpy\scrcpy-win64-v5.0.1")
MATCH_LIMIT_M = 0.03


class FrameWriter:
    """Save analysed frames off-thread so encoding does not distort timing."""

    def __init__(self, folder: Path, quality: int = 90, depth: int = 12):
        self.folder = folder
        self.quality = quality
        self.saved = 0
        self.dropped = 0
        self._queue: queue.Queue = queue.Queue(maxsize=depth)
        self._thread = threading.Thread(target=self._run, daemon=True, name="diag-frames")
        self._thread.start()

    def put(self, index: int, image: np.ndarray) -> None:
        try:
            self._queue.put_nowait((index, image.copy()))
        except queue.Full:
            self.dropped += 1

    def close(self) -> None:
        self._queue.put(None)
        self._thread.join(timeout=10)

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return
            index, image = item
            ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                buffer.tofile(str(self.folder / f"frame_{index:05d}.jpg"))
                self.saved += 1


def field_frame(image, calib, stamp):
    """Same call sequence as run_task.observe, keeping its counters."""
    from run_task import observe
    stats: dict = {}
    began = time.perf_counter()
    scene, marked, label = observe(image, calib, stamp, stats)
    elapsed = (time.perf_counter() - began) * 1000
    blocks = [[round(float(b[0]), 4), round(float(b[1]), 4), round(float(b[2]), 4),
               round(float(b[3]), 4), bool(b[4])] for b in scene["blocks"]]
    return {
        "valid": bool(scene["valid"]),
        "label": label,
        "tag_ok": bool(stats.get("tag_ok")),
        "tag_id": TAG_ID if stats.get("tag_ok") else -1,
        "tag_side_px": "",
        "mm_per_px": "",
        "heading_deg": "" if scene["robot"] is None else round(math.degrees(scene["robot"][2]), 1),
        "seam_ok": stats.get("seam_px") is not None,
        "seam_px": "" if stats.get("seam_px") is None else round(stats["seam_px"], 1),
        "bias_m": "" if stats.get("bias") is None else round(stats["bias"], 3),
        "candidates": stats.get("candidates", 0),
        "instance_area": stats.get("instance_area") or "",
        "splits": stats.get("splits", 0),
        "dropped": stats.get("dropped", 0),
        "blocks": len(blocks),
        "own": sum(1 for block in blocks if block[4]),
        "ms": round(elapsed, 1),
        "blocks_json": blocks,
        "marked": marked,
    }


def bench_frame(image, red_config, tag_mm, tag_id, min_mm, max_mm):
    """Same call sequence as capture_hi --bench, keeping its counters."""
    from bench_check import find_tag
    from red_blocks import detect_red_blocks, filter_by_size_mm
    began = time.perf_counter()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    tag_pts, found_id, side_px = find_tag(gray, tag_id)
    scale = tag_mm / side_px if tag_pts is not None and side_px else None
    candidates = detect_red_blocks(image, red_config)
    kept: list = []
    rejected: list = []
    if scale is not None:
        kept, rejected = filter_by_size_mm(candidates, scale, min_mm, max_mm)
    elapsed = (time.perf_counter() - began) * 1000
    blocks = []
    if scale is not None:
        center = tag_pts.mean(axis=0)
        for block in kept:
            x = (block["center_px"][0] - center[0]) * scale / 1000.0
            y = (block["center_px"][1] - center[1]) * scale / 1000.0
            blocks.append([round(x, 4), round(y, 4),
                           round(block["side_mm"], 2), round(block["side_mm"], 2), True])
    return {
        "valid": scale is not None,
        "label": "bench ready" if scale is not None else "tag missing: mm screen unavailable",
        "tag_ok": tag_pts is not None,
        "tag_id": -1 if found_id is None else int(found_id),
        "tag_side_px": "" if side_px is None else round(float(side_px), 1),
        "mm_per_px": "" if scale is None else round(float(scale), 4),
        "heading_deg": "",
        "seam_ok": "",
        "seam_px": "",
        "bias_m": "",
        "candidates": len(candidates),
        "instance_area": "",
        "splits": "",
        "dropped": len(rejected),
        "blocks": len(blocks),
        "own": len(blocks),
        "ms": round(elapsed, 1),
        "blocks_json": blocks,
        "marked": None,
    }


def _percentile(values, q):
    return float(np.percentile(values, q)) if values else float("nan")


def match_jitter(previous, current, limit=MATCH_LIMIT_M):
    """Greedy nearest-neighbour matching of the same physical block in two frames."""
    used = set()
    distances = []
    for point in previous:
        best, best_index = None, None
        for index, candidate in enumerate(current):
            if index in used:
                continue
            distance = math.hypot(point[0] - candidate[0], point[1] - candidate[1])
            if best is None or distance < best:
                best, best_index = distance, index
        if best is not None and best <= limit:
            used.add(best_index)
            distances.append(best)
    return distances, len(previous) + len(current) - 2 * len(distances)


def summarize(rows, mode):
    result = {"mode": mode, "frames": len(rows)}
    if not rows:
        return result
    result["duration_s"] = round(rows[-1]["wall"] - rows[0]["wall"], 2)
    result["analysis_fps"] = round(len(rows) / max(1e-6, result["duration_s"]), 2)
    result["tag_ok_pct"] = round(100.0 * sum(1 for r in rows if r["tag_ok"]) / len(rows), 1)
    result["valid_pct"] = round(100.0 * sum(1 for r in rows if r["valid"]) / len(rows), 1)
    if mode == "field":
        result["seam_ok_pct"] = round(100.0 * sum(1 for r in rows if r["seam_ok"]) / len(rows), 1)
    full = [r for r in rows if r["valid"]]
    counts = [r["blocks"] for r in full]
    if counts:
        swings = [abs(b - a) for a, b in zip(counts, counts[1:])]
        result.update(blocks_median=statistics.median(counts), blocks_min=min(counts),
                      blocks_max=max(counts),
                      blocks_zero_pct=round(100.0 * counts.count(0) / len(counts), 1),
                      count_swing_median=float(statistics.median(swings)) if swings else 0.0,
                      count_swing_p90=round(_percentile(swings, 90), 2) if swings else 0.0)
    distances = []
    unmatched = 0
    for previous, current in zip(full, full[1:]):
        pair, missed = match_jitter(previous["blocks_json"], current["blocks_json"])
        distances.extend(pair)
        unmatched += missed
    if distances:
        result.update(jitter_mm_median=round(1000.0 * statistics.median(distances), 1),
                      jitter_mm_p90=round(1000.0 * _percentile(distances, 90), 1),
                      jitter_matched=len(distances), jitter_unmatched_total=unmatched,
                      jitter_unmatched_per_pair=round(unmatched / max(1, len(full) - 1), 2))
    tracked = [r.get("blocks_tracked") for r in full]
    tracked = [value for value in tracked if isinstance(value, int)]
    if len(tracked) == len(full) and len(tracked) > 1:
        swings = [abs(b - a) for a, b in zip(tracked, tracked[1:])]
        result.update(tracked_blocks_median=statistics.median(tracked),
                      tracked_blocks_min=min(tracked), tracked_blocks_max=max(tracked),
                      tracked_swing_median=float(statistics.median(swings)) if swings else 0.0,
                      tracked_swing_p90=round(_percentile(swings, 90), 2) if swings else 0.0)
    result["reasons"] = dict(Counter(r["label"] for r in rows if not r["valid"]))
    if mode == "field":
        candidates = [r["candidates"] for r in rows]
        if candidates:
            result["candidates_median"] = statistics.median(candidates)
        areas = [r["instance_area"] for r in rows if r["instance_area"] != ""]
        if areas:
            result["instance_area_p5_p95"] = [round(_percentile(areas, 5), 1),
                                              round(_percentile(areas, 95), 1)]
    return result


def format_summary(result):
    lines = [f"模式：{result['mode']}；分析 {result.get('frames', 0)} 帧 / "
             f"{result.get('duration_s', 0.0):.1f} s（{result.get('analysis_fps', 0.0):.1f} 帧/s）",
             f"tag 解码 {result.get('tag_ok_pct', 0.0):.1f}% ；"
             f"完整可用帧 {result.get('valid_pct', 0.0):.1f}%"
             + (f" ；分界线可用 {result.get('seam_ok_pct', 0.0):.1f}%"
                if "seam_ok_pct" in result else "")]
    if "blocks_median" in result:
        lines.append(f"块数 中位 {result['blocks_median']}，范围 {result['blocks_min']}–"
                     f"{result['blocks_max']}，块数为 0 的帧占 {result['blocks_zero_pct']:.1f}%")
        lines.append(f"相邻帧块数变化：中位 {result['count_swing_median']:.1f}，"
                     f"P90 {result['count_swing_p90']:.1f}")
    if "jitter_mm_median" in result:
        lines.append(f"同块位置抖动：中位 {result['jitter_mm_median']:.1f} mm，"
                     f"P90 {result['jitter_mm_p90']:.1f} mm"
                     f"（每对帧平均 {result['jitter_unmatched_per_pair']:.2f} 个块无对应）")
    if "tracked_blocks_median" in result:
        lines.append(f"跟踪后（合并+确认+平滑）：块数 中位 {result['tracked_blocks_median']}，"
                     f"范围 {result['tracked_blocks_min']}–{result['tracked_blocks_max']}，"
                     f"相邻帧变化 中位 {result['tracked_swing_median']:.1f} / "
                     f"P90 {result['tracked_swing_p90']:.1f}")
    if "candidates_median" in result:
        lines.append(f"红候选连通域中位 {result['candidates_median']}；"
                     f"单块面积估计 P5–P95 {result['instance_area_p5_p95']} mm²")
    if result.get("reasons"):
        lines.append("失效原因：" + "；".join(
            f"{reason} x{count}" for reason, count in
            sorted(result["reasons"].items(), key=lambda item: -item[1])))
    return "\n".join(lines)


def write_outputs(out_dir, rows, mode, params, extra=None):
    """Write frames.csv and summary.json, returning the summary dict."""
    fields = [key for key in rows[0] if key != "blocks_json"] + ["blocks_json"]
    with (out_dir / "frames.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        csv_writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        csv_writer.writeheader()
        for row in rows:
            csv_writer.writerow(dict(row, blocks_json=json.dumps(row["blocks_json"],
                                                                 ensure_ascii=False)))
    result = summarize(rows, mode)
    result["params"] = {key: (str(value) if isinstance(value, Path) else value)
                        for key, value in params.items()}
    result.update(extra or {})
    (out_dir / "summary.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scrcpy-dir", type=Path, default=DEFAULT_SCRCPY)
    parser.add_argument("--serial")
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=90)
    parser.add_argument("--max-size", type=int, default=1280)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--secs", type=float, default=15.0)
    parser.add_argument("--detect-hz", type=float, default=12.0, help="分析频率")
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--config", type=Path, default=HERE / "field_calib.json")
    parser.add_argument("--bench", action="store_true", help="强制台架模式（不要求场地标定）")
    parser.add_argument("--profile", type=Path, default=HERE / "red_block_profile.json")
    parser.add_argument("--tag-mm", type=float, default=50.0)
    parser.add_argument("--tag-id", type=int, default=TAG_ID)
    parser.add_argument("--min-mm", type=float, default=12.0)
    parser.add_argument("--max-mm", type=float, default=32.0)
    parser.add_argument("--save-frames", action="store_true", help="保存每帧 JPEG 供回放")
    parser.add_argument("--save-every", type=int, default=1)
    parser.add_argument("--track", action="store_true",
                        help="同时记录合并/确认/平滑后的块数，用于和逐帧原始值对比")
    parser.add_argument("--show", action="store_true", help="显示预览窗口（会计入耗时）")
    parser.add_argument("--out", type=Path, help="默认 windows/captures_hi/diag_<时间>")
    args = parser.parse_args()

    mode = "bench" if args.bench or not args.config.exists() else "field"
    if mode == "bench" and not args.bench:
        print(f"没有场地标定 {args.config}，改用台架模式（tag 尺寸换算 mm/px）。")

    out_dir = args.out or (HERE / "captures_hi" / time.strftime("diag_%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"模式：{mode}；输出：{out_dir}；时长 {args.secs:.0f} s，分析 {args.detect_hz:.0f} Hz")

    calib = None
    red_config = None
    if mode == "field":
        from field_view import FieldCalibration
        calib = FieldCalibration.load(args.config)
        print(f"场地标定 {args.config} -> {calib.size}x{calib.size}，{calib.mm_per_px:.3f} mm/px")
    else:
        from red_blocks import load_red_config
        red_config = load_red_config(args.profile) if args.profile.exists() else None
        print(f"台架：红块配置 {args.profile} (exists={args.profile.exists()})；"
              f"tag {args.tag_mm:.0f} mm；毫米筛 {args.min_mm:.0f}–{args.max_mm:.0f} mm")

    camera, _used_size = open_camera(args.scrcpy_dir, args.serial, [args.max_size],
                                     args.fps, args.timeout)
    if camera is None:
        return 2

    rows: list = []
    writer = None
    stabilizer = None
    if args.track:
        from block_tracks import BlockStabilizer
        stabilizer = BlockStabilizer()
    last_sequence = 0
    last_detect = 0.0
    started = time.monotonic()
    min_dt = 1.0 / max(1.0, args.detect_hz)
    try:
        while time.monotonic() - started < args.secs:
            now = time.monotonic()
            if camera.error:
                raise RuntimeError(camera.error)
            frame, received_at, sequence = camera.latest.get()
            if frame is None or sequence == last_sequence:
                time.sleep(0.005)
                continue
            last_sequence = sequence
            if now - last_detect < min_dt:
                continue
            last_detect = now
            image = rotate(frame.to_ndarray(format="bgr24"), args.rotate)
            index = len(rows)
            if mode == "field":
                result = field_frame(image, calib, received_at)
            else:
                result = bench_frame(image, red_config, args.tag_mm, args.tag_id,
                                     args.min_mm, args.max_mm)
            if stabilizer is not None:
                # Same rule as run_task: an invalid frame does not advance the
                # tracker, so a short vision gap cannot erase live blocks.
                tracked = (stabilizer.update(result["blocks_json"], received_at)
                           if result["valid"] else None)
                result["blocks_tracked"] = "" if tracked is None else len(tracked)
            else:
                result["blocks_tracked"] = ""
            if args.save_frames and index % max(1, args.save_every) == 0:
                if writer is None:
                    writer = FrameWriter(out_dir)
                writer.put(index, image)
            row = {"frame": index, "seq": sequence, "wall": round(time.time(), 3),
                   "age_ms": round((time.monotonic() - received_at) * 1000)}
            row.update({key: value for key, value in result.items()
                        if key not in ("marked", "blocks_json")})
            row["blocks_json"] = result["blocks_json"]
            rows.append(row)
            if args.show and result["marked"] is not None:
                cv2.imshow("record_diag", cv2.resize(result["marked"], (720, 720)))
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    break
            if index % 10 == 0:
                print(f"  帧 {index:4d} | tag={'Y' if result['tag_ok'] else 'N'} "
                      f"| {result['label']} | blocks={result['blocks']} | {result['ms']:.0f} ms")
    except KeyboardInterrupt:
        print("手动停止，仍写出已记录的部分。")
    finally:
        if writer is not None:
            writer.close()
        camera.close()
        cv2.destroyAllWindows()

    if not rows:
        print("没有记录到任何帧。", file=sys.stderr)
        return 1
    result = write_outputs(
        out_dir, rows, mode, vars(args),
        {"saved_frames": getattr(writer, "saved", 0),
         "dropped_frames": getattr(writer, "dropped", 0)})
    print("\n=== 诊断结果 ===")
    print(format_summary(result))
    print(f"\n逐帧明细：{out_dir / 'frames.csv'}")
    if args.save_frames:
        print(f"已保存 {result['saved_frames']} 帧 JPEG（丢弃 {result['dropped_frames']} 帧）")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
