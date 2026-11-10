"""用训练好的 ONNX 模型在本机跑推理：延迟、检出数、可视化。

训练在 Colab 做（见 `training/colab_train.ipynb`），这里只做导入侧的事：
读帧、letterbox、推理、还原框、画图、报耗时。

依赖：`pip install onnxruntime`（本机没装时会给出提示）。不发送任何控制指令。

用法：
    python windows\\detect_onnx.py --model windows\\models\\red_block.onnx ^
        --frames windows\\test_data\\red_train_20261110\\frames --limit 20
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp")
PAD_VALUE = 114
ROOT = Path(__file__).resolve().parent.parent


def cuda_library_dirs():
    """Where the CUDA/cuDNN DLLs that onnxruntime-gpu needs actually live.

    This machine has no system-wide CUDA toolkit: PyTorch ships the CUDA 12 and
    cuDNN 9 runtimes inside the conda env. Adding those directories to the DLL
    search path lets onnxruntime-gpu use them.
    """
    dirs = []
    cuda = os.environ.get("CUDA_PATH")
    if cuda:
        dirs.append(Path(cuda) / "bin")
    override = os.environ.get("ROBOT_TORCH_LIB")
    if override:
        dirs.append(Path(override))
    for base in (Path.home() / "anaconda3" / "envs",
                 Path.home() / "miniconda3" / "envs"):
        if base.is_dir():
            try:
                dirs.extend(base.glob("*/Lib/site-packages/torch/lib"))
            except OSError:
                pass
    return [path for path in dirs if path.is_dir()]


def ensure_runtime_paths() -> list:
    """Make repo-local deps importable and expose the CUDA DLLs to ONNX Runtime.

    `.deps-gpu` must win over `.deps`: both contain an `onnxruntime` package, and
    the CPU one would silently shadow the GPU build.
    """
    order = [folder for folder in (ROOT / ".deps-gpu", ROOT / ".deps")
             if folder.is_dir()]
    missing = [str(folder) for folder in order if str(folder) not in sys.path]
    sys.path[:0] = missing
    added = []
    for path in cuda_library_dirs():
        try:
            if hasattr(os, "add_dll_directory"):
                os.add_dll_directory(str(path))
            added.append(path)
        except OSError:
            continue
    # ONNX Runtime loads its provider DLL with plain LoadLibrary, which resolves
    # dependencies through PATH, not through add_dll_directory. Put the CUDA
    # runtime directories on PATH as well or cuDNN/cuBLAS are "not found".
    if added:
        current = os.environ.get("PATH", "")
        prefix = os.pathsep.join(str(path) for path in added)
        if prefix not in current:
            os.environ["PATH"] = prefix + (os.pathsep + current if current else "")
    return added


def letterbox(image, size):
    """Resize keeping ratio and pad to `size` (w, h); returns (image, ratio, pad)."""
    target_w, target_h = size
    height, width = image.shape[:2]
    ratio = min(target_w / width, target_h / height)
    new_w, new_h = max(1, int(round(width * ratio))), max(1, int(round(height * ratio)))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((target_h, target_w, 3), PAD_VALUE, np.uint8)
    pad_x = (target_w - new_w) // 2
    pad_y = (target_h - new_h) // 2
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return canvas, ratio, (pad_x, pad_y)


def to_input(padded):
    tensor = padded[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
    return tensor[None, ...]


def decode(prediction, conf_thres: float, num_classes: int = 1):
    """YOLOv8 head without NMS -> boxes xywh + scores.

    The exported tensor is either (1, 4+classes, anchors) or (1, anchors,
    4+classes); the channel count tells them apart, so no guessing by size.
    """
    data = np.asarray(prediction, dtype=np.float32)
    if data.ndim == 3:
        data = data[0]
    if data.size == 0:
        return np.zeros((0, 4), np.float32), np.zeros((0,), np.float32)
    channels = 4 + num_classes
    if data.shape[0] == channels:
        data = data.T
    elif data.shape[1] != channels and data.shape[0] < data.shape[1]:
        data = data.T
    boxes = data[:, :4]
    scores = data[:, 4:]
    best = scores.max(axis=1)
    keep = best >= conf_thres
    return boxes[keep], best[keep]


def restore(boxes, ratio, pad):
    """Map cx, cy, w, h from the padded input back to original image pixels.

    Only the centre is shifted by the padding; width and height must merely be
    divided by the scale. Shifting them too turns every box into a negative
    width as soon as the letterbox actually pads (aspect ratios other than the
    model input).
    """
    moved = boxes.copy().astype(np.float32)
    moved[:, 0] = (moved[:, 0] - pad[0]) / max(ratio, 1e-6)
    moved[:, 1] = (moved[:, 1] - pad[1]) / max(ratio, 1e-6)
    moved[:, 2] = moved[:, 2] / max(ratio, 1e-6)
    moved[:, 3] = moved[:, 3] / max(ratio, 1e-6)
    return moved


def xywh_to_xyxy(boxes):
    out = boxes.copy().astype(np.float32)
    out[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    out[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    out[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    out[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    return out


def nms(boxes, scores, iou_thres: float):
    if len(boxes) == 0:
        return []
    rects = [[float(x1), float(y1), float(x2 - x1), float(y2 - y1)] for x1, y1, x2, y2 in boxes]
    return list(cv2.dnn.NMSBoxes(rects, [float(s) for s in scores], 0.0, iou_thres))


def read_image(path: Path):
    encoded = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR) if encoded.size else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--frames", type=Path, help="图片目录，默认取数据集 frames")
    parser.add_argument("--limit", type=int, default=20)
    parser.add_argument("--imgsz", default="1280x960", help="模型输入，宽x高")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--device", choices=("auto", "gpu", "cpu"), default="auto",
                        help="auto 有 CUDA 就用 GPU，否则 CPU")
    parser.add_argument("--save", type=Path, help="画框图输出目录")
    args = parser.parse_args()

    try:
        ensure_runtime_paths()
        import onnxruntime as ort
    except ImportError:
        print("需要 onnxruntime：pip install onnxruntime", file=sys.stderr)
        return 2
    if not args.model.exists():
        print(f"找不到模型 {args.model}", file=sys.stderr)
        return 2
    width, height = (int(v) for v in args.imgsz.lower().split("x"))
    folder = args.frames or Path(__file__).resolve().parent / "test_data" / "red_train_20261110" / "frames"
    files = sorted(p for p in folder.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)[:args.limit]
    if not files:
        print(f"{folder} 里没有图片", file=sys.stderr)
        return 2

    providers = (["CUDAExecutionProvider", "CPUExecutionProvider"]
                 if args.device in ("auto", "gpu") else ["CPUExecutionProvider"])
    session = ort.InferenceSession(str(args.model), providers=providers)
    active = session.get_providers()
    print(f"推理后端：{active[0] if active else 'unknown'}（请求 {args.device}）")
    if args.device == "gpu" and "CUDAExecutionProvider" not in active:
        print("警告：CUDA 不可用，已退回 CPU；需要 onnxruntime-gpu + CUDA/cuDNN 匹配")
    name = session.get_inputs()[0].name
    dummy = np.zeros((1, 3, height, width), np.float32)
    first = None
    for index in range(3):
        began = time.perf_counter()
        session.run(None, {name: dummy})
        elapsed = (time.perf_counter() - began) * 1000
        if index == 0:
            first = elapsed
    print(f"首次推理 {first:.0f} ms（含 CUDA 初始化，只发生在启动时）")

    times, counts = [], []
    for path in files:
        image = read_image(path)
        if image is None:
            continue
        padded, ratio, pad = letterbox(image, (width, height))
        tensor = to_input(padded)
        began = time.perf_counter()
        output = session.run(None, {name: tensor})[0]
        times.append((time.perf_counter() - began) * 1000)
        boxes, scores = decode(output, args.conf)
        boxes = xywh_to_xyxy(restore(boxes, ratio, pad))
        picked = nms(boxes, scores, args.iou)
        kept = [boxes[i] for i in picked]
        counts.append(len(kept))
        if args.save:
            args.save.mkdir(parents=True, exist_ok=True)
            marked = image.copy()
            for x1, y1, x2, y2 in kept:
                cv2.rectangle(marked, (int(x1), int(y1)), (int(x2), int(y2)), (0, 220, 0), 2)
            ok, buffer = cv2.imencode(".jpg", marked, [cv2.IMWRITE_JPEG_QUALITY, 88])
            if ok:
                buffer.tofile(str(args.save / path.name))

    if times:
        times = np.array(times)
        print(f"帧数 {len(times)}；单帧推理 ms：中位 {np.median(times):.1f}  "
              f"P95 {np.percentile(times, 95):.1f}  max {times.max():.1f}")
        print(f"检出数：中位 {int(np.median(counts))} 范围 {min(counts)}–{max(counts)}")
        print(f"目标 < 100 ms（10 Hz）；超过就考虑降分辨率或 4 Hz 检测 + 跟踪")
    if args.save:
        print(f"标注图：{args.save}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:                                   # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
