"""逐帧框选红块，保存 JSON 标注，供检测训练集使用。

只读图片、只写 `labels.json`：不改原图，不启动相机，不发任何控制指令。

用法：
    & 'E:\\scrcpy\\scrcpy-win64-v5.0.1\\.venv\\Scripts\\python.exe' windows\\label_boxes.py ^
        windows\\test_data\\red_train_20261110\\frames

操作：
- 左键拖出矩形：新增一个红块框，块与块挨着就分别框，不要框成一个。
- 右键点在框内：删除该框。
- Z 撤销最后一个框；D 清空当前帧所有框（误点多了用）。
- N / 空格 / V / 回车：**确认这张并下一张**（没框的帧也确认，就是负样本）。
- U：取消这张的“已核对”，退回未完成状态。
- P：上一张。确认时自动保存。
- S：保存；Q / Esc：保存并退出。

采帧时已经用 HSV 检测器预填了框（记在该帧的 `auto` 字段），这里主要是**删掉误检、
补上漏检、微调位置**，不用从零画。

只框**看得见的红块**：被车身挡住、只剩一条边或看不清个数的，不猜。
车轮、红色支架、反光不算红块；这类帧留空并按 V，就是负样本。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")
MIN_SIDE = 6
HERE = Path(__file__).resolve().parent


def load_store(path: Path) -> dict:
    if path.exists():
        data = json.loads(path.read_text(encoding="utf-8"))
        data.setdefault("images", {})
        data.setdefault("class_name", "red_block")
        data.setdefault("version", 1)
        return data
    return {"version": 1, "class_name": "red_block", "created": time.strftime("%Y-%m-%d %H:%M:%S"),
            "images": {}}


def save_store(path: Path, store: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(store, ensure_ascii=False, indent=1) + "\n",
                         encoding="utf-8")
    temporary.replace(path)


def entry_of(store: dict, name: str) -> dict:
    return store["images"].setdefault(name, {"boxes": [], "reviewed": False})


def clamp_box(box, width: int, height: int, min_side: int = MIN_SIDE):
    """Return [x, y, w, h] inside the image, or None when too small/outside."""
    x0, y0, x1, y1 = (int(round(value)) for value in box)
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    x0, x1 = max(0, x0), min(width, x1)
    y0, y1 = max(0, y0), min(height, y1)
    if x1 - x0 < min_side or y1 - y0 < min_side:
        return None
    return [x0, y0, x1 - x0, y1 - y0]


def box_index_at(boxes, x: float, y: float):
    for index in range(len(boxes) - 1, -1, -1):
        bx, by, bw, bh = boxes[index]
        if bx <= x <= bx + bw and by <= y <= by + bh:
            return index
    return None


def read_image(path: Path):
    encoded = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR) if encoded.size else None


def summarize_store(store: dict) -> str:
    images = store["images"]
    reviewed = sum(1 for entry in images.values() if entry.get("reviewed"))
    boxes = sum(len(entry.get("boxes", [])) for entry in images.values())
    negatives = sum(1 for entry in images.values()
                    if entry.get("reviewed") and not entry.get("boxes"))
    return (f"已核对 {reviewed} 张，共 {boxes} 个框，其中空帧负样本 {negatives} 张")


def screen_size():
    try:
        import ctypes
        user32 = ctypes.windll.user32
        return int(user32.GetSystemMetrics(0)), int(user32.GetSystemMetrics(1))
    except Exception:                                          # noqa: BLE001
        return 1600, 900


def fit_scale(width: int, height: int, screen_w: int, screen_h: int,
              fraction: float = 0.92) -> float:
    """Largest scale that keeps the whole frame on screen (may upscale)."""
    usable_w = max(320.0, screen_w * fraction)
    usable_h = max(240.0, (screen_h - 80) * fraction)
    return min(usable_w / width, usable_h / height, 2.0)


def render(image, boxes, dragging, status: str, scale: float):
    """Scale the frame once, draw everything in display pixels."""
    width, height = image.shape[1], image.shape[0]
    size = (max(1, int(width * scale)), max(1, int(height * scale)))
    canvas = cv2.resize(image, size, interpolation=cv2.INTER_AREA if scale < 1 else
                        cv2.INTER_LINEAR)
    for box in boxes:
        x, y, w, h = (int(round(value * scale)) for value in box)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 210, 0), 2)
    if dragging is not None:
        # Dragging is tracked in image pixels; draw it in display pixels, and
        # give OpenCV integers (float tuples are rejected by cv2.rectangle).
        p1 = (int(round(dragging[0] * scale)), int(round(dragging[1] * scale)))
        p2 = (int(round(dragging[2] * scale)), int(round(dragging[3] * scale)))
        cv2.rectangle(canvas, p1, p2, (0, 255, 255), 2)
    bar = canvas.copy()
    cv2.rectangle(bar, (0, 0), (canvas.shape[1], 30), (20, 20, 20), -1)
    cv2.addWeighted(bar, 0.75, canvas, 0.25, 0, canvas)
    cv2.putText(canvas, status, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (235, 235, 235), 2)
    return canvas


def prefill_entries(store: dict, files, config) -> int:
    """Pre-fill empty, un-reviewed frames with HSV boxes (existing photos too)."""
    from red_blocks import prelabel_boxes
    filled = 0
    for path in files:
        entry = store["images"].setdefault(path.name, {"boxes": [], "reviewed": False})
        if entry.get("reviewed") or entry.get("boxes"):
            continue
        image = read_image(path)
        if image is None:
            continue
        entry["boxes"] = prelabel_boxes(image, config)
        entry["auto"] = True
        entry["width"], entry["height"] = int(image.shape[1]), int(image.shape[0])
        filled += 1
    return filled


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", type=Path, help="截帧目录（含 frame_*.jpg）")
    parser.add_argument("--labels", type=Path, help="默认 <folder>/labels.json")
    parser.add_argument("--start", type=int, help="从第几张开始（0 起）")
    parser.add_argument("--zoom", type=float, default=1.0,
                        help="在铺满屏幕的基础上再放大/缩小（0.5–2）")
    parser.add_argument("--prefill", action="store_true",
                        help="先用 HSV 检测预填空帧的框（已采的帧也能补）")
    args = parser.parse_args()

    folder = args.folder
    files = sorted(path for path in folder.iterdir()
                   if path.suffix.lower() in IMAGE_SUFFIXES) if folder.is_dir() else []
    if not files:
        print(f"{folder} 里没有图片", file=sys.stderr)
        return 2
    labels_path = args.labels or (folder / "labels.json")
    store = load_store(labels_path)

    if args.prefill:
        from red_blocks import load_red_config
        profile = HERE / "red_block_profile.json"
        config = load_red_config(profile) if profile.exists() else None
        filled = prefill_entries(store, files, config)
        save_store(labels_path, store)
        print(f"预填 {filled} 张（黑框来自 HSV 检测，需要你删误检、补漏检）")

    index = 0 if args.start is None else max(0, min(args.start, len(files) - 1))
    if args.start is None:
        for position, path in enumerate(files):
            if not store["images"].get(path.name, {}).get("reviewed"):
                index = position
                break
    print(f"{len(files)} 张图，标注文件 {labels_path}")
    print("左键拖框 · 右键删框 · Z 撤销 · D 清空 · N/空格/V/回车 确认并下一张 · "
          "U 取消确认 · P 上一张 · Q 退出")

    dragging = None
    dirty = False
    window = "label_boxes"
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    screen_w, screen_h = screen_size()
    zoom = args.zoom if 0.3 <= args.zoom <= 2.0 else 1.0
    current = {"name": files[index].name, "width": 1, "height": 1, "scale": 1.0}

    def on_mouse(event, x, y, _flags, _param):
        nonlocal dragging, dirty
        # The window shows a scaled frame; store everything in image pixels.
        ix = x / current["scale"]
        iy = y / current["scale"]
        entry = entry_of(store, current["name"])
        if event == cv2.EVENT_LBUTTONDOWN:
            dragging = (ix, iy, ix, iy)
        elif event == cv2.EVENT_MOUSEMOVE and dragging is not None:
            dragging = (dragging[0], dragging[1], ix, iy)
        elif event == cv2.EVENT_LBUTTONUP and dragging is not None:
            box = clamp_box((dragging[0], dragging[1], ix, iy),
                            current["width"], current["height"])
            dragging = None
            if box:
                entry["boxes"].append(box)
                dirty = True
        elif event == cv2.EVENT_RBUTTONDOWN:
            hit = box_index_at(entry["boxes"], ix, iy)
            if hit is not None:
                entry["boxes"].pop(hit)
                dirty = True

    cv2.setMouseCallback(window, on_mouse)

    def persist(force: bool = False) -> None:
        nonlocal dirty
        if dirty or force:
            save_store(labels_path, store)
            dirty = False

    while True:
        path = files[index]
        image = read_image(path)
        if image is None:
            print(f"读不到 {path.name}，跳过")
            index = (index + 1) % len(files)
            continue
        height, width = image.shape[:2]
        entry = store["images"].setdefault(
            path.name, {"boxes": [], "reviewed": False, "width": width, "height": height})
        entry["width"], entry["height"] = width, height
        scale = fit_scale(width, height, screen_w, screen_h) * zoom
        current.update(name=path.name, width=width, height=height, scale=scale)

        status = (f"{index + 1}/{len(files)}  {path.name}  boxes={len(entry['boxes'])}"
                  f"  {round(scale * 100)}%  {'[reviewed]' if entry['reviewed'] else ''}")
        canvas = render(image, entry["boxes"], dragging, status, scale)
        cv2.imshow(window, canvas)

        key = cv2.waitKey(20) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            persist(force=True)
            break
        if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            persist(force=True)
            break
        if key in (ord("n"), 32, 13, 83, ord("v"), ord("V")):
            # One key does "confirm and advance": N, space, V and Enter are the
            # same action. A frame with no boxes becomes a negative sample.
            entry["reviewed"] = True
            dirty = True
            persist(force=True)
            index = (index + 1) % len(files)
        elif key in (ord("u"), ord("U")):
            entry["reviewed"] = False
            dirty = True
        elif key in (ord("p"), 81):
            persist()
            index = (index - 1) % len(files)
        elif key in (ord("s"), ord("S")):
            persist(force=True)
            print("已保存")
        elif key in (ord("z"), ord("Z")):
            if entry["boxes"]:
                entry["boxes"].pop()
                dirty = True
        elif key in (ord("d"), ord("D")):
            entry["boxes"].clear()
            dirty = True
        elif key in (ord("+"), ord("=")):
            zoom = min(2.0, zoom + 0.1)
        elif key in (ord("-"), ord("_")):
            zoom = max(0.3, zoom - 0.1)

    cv2.destroyAllWindows()
    missing = [name for name in (p.name for p in files)
               if not store["images"].get(name, {}).get("reviewed")]
    print(summarize_store(store))
    if missing:
        print(f"还有 {len(missing)} 张未按 V 核对")
    print(f"标注文件：{labels_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
