"""生成可直接打印的 ArUco 车顶定位 tag（A4，一页 6 枚）。

用法（需要 opencv + Pillow）：
    python windows/tags/make_tags.py

输出：
    windows/tags/aruco_4x4_50_50mm_A4.pdf
    windows/tags/aruco_4x4_50_50mm_A4.png

规格：DICT_4X4_50，黑色方块边长 50.0 mm，四边各留 12.5 mm 白边（1 模块），
一页排 6 枚，**全部为同一个 ID（取 windows/tag_detector.py 的 TAG_ID，当前 7）**，
互为备用件；底部附 100 mm 标尺，用于卡尺核对打印比例。
打印时必须选“实际大小 / 100%”，禁用“适应页面 / 缩放”。
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tag_detector import TAG_ID  # noqa: E402

DPI = 600.0
PX_PER_MM = DPI / 25.4
A4_MM = (210.0, 297.0)

TAG_MM = 50.0
QUIET_MM = 12.5
CELL_MM = TAG_MM + 2 * QUIET_MM  # 75 mm
COLS, ROWS = 2, 3
GAP_X_MM, GAP_Y_MM = 20.0, 12.0
TOP_MM = 18.0
# 一台车只用 1 个 ID；同页多枚是同一 ID 的备用件，不是不同编号。
COUNT = COLS * ROWS
IDS = [TAG_ID] * COUNT

FONT = "C:/Windows/Fonts/msyh.ttc"
FONT_MONO = "C:/Windows/Fonts/consola.ttf"


def mm(value: float) -> int:
    return int(round(value * PX_PER_MM))


def load_font(size_mm: float) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT, mm(size_mm))


def make_marker(marker_id: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    side = mm(TAG_MM)
    if hasattr(cv2.aruco, "generateImageMarker"):
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    else:  # OpenCV < 4.7 兼容
        img = cv2.aruco.drawMarker(dictionary, marker_id, side)
    return img


def build_page() -> Image.Image:
    page_w, page_h = mm(A4_MM[0]), mm(A4_MM[1])
    page = Image.new("RGB", (page_w, page_h), "white")
    draw = ImageDraw.Draw(page)

    title_font = load_font(5.0)
    note_font = load_font(3.2)
    id_font = load_font(3.6)

    draw.text((mm(20), mm(6)), f"ArUco DICT_4X4_50   黑边 50.0 mm   四边留白 12.5 mm   全部 ID {TAG_ID}（备用件）", font=title_font, fill="black")
    draw.text(
        (mm(20), mm(12.5)),
        "打印：选“实际大小 / 100%”，禁用“适应页面”。打完用卡尺量黑方块边长，应为 50.0 mm。车上只贴一枚，其余备用。",
        font=note_font,
        fill=(60, 60, 60),
    )

    left = (A4_MM[0] - (COLS * CELL_MM + (COLS - 1) * GAP_X_MM)) / 2
    for index, marker_id in enumerate(IDS):
        col, row = index % COLS, index // COLS
        cell_x = left + col * (CELL_MM + GAP_X_MM)
        cell_y = TOP_MM + row * (CELL_MM + GAP_Y_MM)

        marker = make_marker(marker_id)
        marker_rgb = Image.fromarray(cv2.cvtColor(marker, cv2.COLOR_GRAY2RGB))
        page.paste(marker_rgb, (mm(cell_x + QUIET_MM), mm(cell_y + QUIET_MM)))

        # 75 mm 裁切角标，放在留白之外，浅灰不会干扰解码
        tick = mm(3.0)
        x0, y0 = mm(cell_x), mm(cell_y)
        x1, y1 = mm(cell_x + CELL_MM), mm(cell_y + CELL_MM)
        for (px, py, dx, dy) in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
            draw.line([(px, py), (px + dx * tick, py)], fill=(200, 200, 200), width=mm(0.4))
            draw.line([(px, py), (px, py + dy * tick)], fill=(200, 200, 200), width=mm(0.4))

        draw.text(
            (mm(cell_x), mm(cell_y + CELL_MM + 2.0)),
            f"ID {marker_id}  备用 {index + 1}/{COUNT}",
            font=id_font,
            fill=(40, 40, 40),
        )

    # 100 mm 标尺：每 10 mm 一格，用于核对打印机是否缩放
    bar_y = A4_MM[1] - 22.0
    draw.line([(mm(20), mm(bar_y)), (mm(120), mm(bar_y))], fill="black", width=mm(0.5))
    for i in range(11):
        x = 20.0 + i * 10.0
        height = 4.0 if i % 5 == 0 else 2.5
        draw.line([(mm(x), mm(bar_y)), (mm(x), mm(bar_y - height))], fill="black", width=mm(0.4))
    draw.text((mm(20), mm(bar_y + 1.5)), "0", font=note_font, fill="black")
    draw.text((mm(116), mm(bar_y + 1.5)), "100 mm", font=note_font, fill="black")
    draw.text(
        (mm(20), mm(bar_y + 8.0)),
        "标尺总长应为 100.0 mm；不一致说明打印被缩放，请按 100% 重打。",
        font=note_font,
        fill=(60, 60, 60),
    )
    return page


def verify(page: Image.Image) -> None:
    gray = cv2.cvtColor(np.array(page), cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    found = [] if ids is None else [int(i) for i in ids.flatten()]
    print(f"detected {len(found)} markers: {sorted(found)}")
    if corners:
        edges = []
        for c in corners:
            pts = c.reshape(4, 2)
            side_px = np.linalg.norm(pts[0] - pts[1])
            edges.append(side_px / PX_PER_MM)
        print("edge length mm: min %.2f max %.2f" % (min(edges), max(edges)))


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=None, help="输出目录，默认与本脚本同目录")
    args = parser.parse_args()
    out_dir = Path(args.out) if args.out else Path(__file__).resolve().parent
    out_dir.mkdir(parents=True, exist_ok=True)
    page = build_page()
    png_path = out_dir / "aruco_4x4_50_50mm_A4.png"
    pdf_path = out_dir / "aruco_4x4_50_50mm_A4.pdf"
    page.save(png_path, dpi=(DPI, DPI))
    page.save(pdf_path, "PDF", resolution=DPI)
    verify(page)
    print(f"page px: {page.size[0]}x{page.size[1]} @ {DPI:.0f} dpi")
    print(f"wrote {png_path}")
    print(f"wrote {pdf_path}")


if __name__ == "__main__":
    main()
