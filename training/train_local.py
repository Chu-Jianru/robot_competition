"""在本机 GPU 上训练红块检测模型，并导出 ONNX 到 windows/models/。

默认参数针对本数据集（61 训练 / 13 验证，红块 9–25 px）：
`imgsz=1280`、`yolov8n`、120 轮、早停 30。

用法：
    python training\\train_local.py
    python training\\train_local.py --epochs 40 --imgsz 960      # 想快点看趋势
    python training\\train_local.py --device cpu                 # 没有可用 GPU 时

训练产物：
    training/runs/red_block/weights/best.pt      继续训练用
    windows/models/red_block.onnx                给车上推理用
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DEFAULT_DATA = ROOT / "windows" / "test_data" / "red_train_20261110" / "yolo" / "data.yaml"
DEFAULT_MODEL = ROOT / "windows" / "models" / "red_block.onnx"


def check_torch():
    try:
        import torch
    except ImportError:
        print("没有安装 PyTorch。先运行 windows\\训练红块模型.cmd，或手动：\n"
              "  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121\n"
              "  pip install ultralytics onnx onnxruntime", file=sys.stderr)
        return None
    print(f"torch {torch.__version__}；CUDA 可用：{torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"GPU：{torch.cuda.get_device_name(0)}")
    return torch


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--epochs", type=int, default=120)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="0", help="0 = 第一块 GPU，cpu = 用 CPU")
    parser.add_argument("--workers", type=int, default=0, help="Windows 下建议 0")
    parser.add_argument("--weights", default=str(ROOT / "training" / "yolov8n.pt"),
                        help="起点权重：training/yolov8n.pt（从零）或某个 best.pt（热启动）")
    parser.add_argument("--out", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--name", default=None,
                        help="本次运行目录名，默认按时间戳，避免覆盖上一次")
    args = parser.parse_args()
    if args.name is None:
        import time
        args.name = "red_block_" + time.strftime("%Y%m%d_%H%M")

    torch = check_torch()
    if torch is None:
        return 2
    if args.device != "cpu" and not torch.cuda.is_available():
        print("CUDA 不可用，改用 CPU（会慢很多）。想快就装 CUDA 版 PyTorch。")
        args.device = "cpu"
    if not args.data.exists():
        print(f"找不到数据集 {args.data}；先跑 windows\\export_yolo.py", file=sys.stderr)
        return 2

    from ultralytics import YOLO

    model = YOLO(args.weights)
    model.train(
        data=str(args.data),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        patience=30,
        device=args.device,
        workers=args.workers,
        project=str(HERE / "runs"),
        name=args.name,
        exist_ok=True,
        scale=0.6,
        translate=0.15,
        degrees=10.0,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        mosaic=1.0,
        close_mosaic=20,
    )

    metrics = model.val(data=str(args.data), imgsz=args.imgsz, split="val", device=args.device)
    print("\n=== 验证集 ===")
    print(f"mAP50    {metrics.box.map50:.3f}")
    print(f"mAP50-95 {metrics.box.map:.3f}")
    print(f"precision {metrics.box.mp:.3f}")
    print(f"recall    {metrics.box.mr:.3f}")

    best = Path(model.trainer.save_dir) / "weights" / "best.pt"
    if not best.exists():
        print(f"没有找到 {best}", file=sys.stderr)
        return 1
    exported = YOLO(str(best)).export(format="onnx", imgsz=(960, args.imgsz), opset=12,
                                      simplify=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(exported, args.out)
    print(f"\nONNX 已放到 {args.out}")
    print("下一步：双击 windows\\比较红块检测.cmd，对比 HSV 与模型的精度和耗时。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
