"""用训练好的 ONNX 红块模型做检测，输出和 HSV 检测器一样的结构。

`run_task.observe` 里红块的来源可以是 HSV（默认）或这个模块；两者都返回
`{"center_px", "contour_px", "bbox_px"}`，后面的归属判断、坐标换算、策略都不用改。

ONNX 是推理格式：训练在 Colab 做，这里只加载 `.onnx`，依赖 `onnxruntime`。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from detect_onnx import (decode, ensure_runtime_paths, letterbox, nms,
                         restore, to_input, xywh_to_xyxy)


class OnnxBlockDetector:
    def __init__(self, model_path, imgsz=(1280, 960), conf: float = 0.25,
                 iou: float = 0.5, device: str = "auto"):
        ensure_runtime_paths()
        try:
            import onnxruntime  # noqa: F401
        except ImportError as error:                            # pragma: no cover
            raise RuntimeError("需要 onnxruntime：装 .deps 或 pip install onnxruntime") from error
        import onnxruntime as ort
        path = Path(model_path)
        if not path.exists():
            raise RuntimeError(f"找不到模型 {path}；先把 best.onnx 放到 windows/models/")
        providers = (["CUDAExecutionProvider", "CPUExecutionProvider"]
                     if device in ("auto", "gpu") else ["CPUExecutionProvider"])
        self.session = ort.InferenceSession(str(path), providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.imgsz = (int(imgsz[0]), int(imgsz[1]))
        self.conf, self.iou = float(conf), float(iou)
        active = self.session.get_providers()
        self.provider = active[0] if active else "unknown"
        if device == "gpu" and "CUDAExecutionProvider" not in active:
            print("警告：CUDA 不可用，推理退回 CPU", flush=True)
        # The first call pays for CUDA context creation; keep it out of the loop.
        dummy = np.zeros((1, 3, self.imgsz[1], self.imgsz[0]), np.float32)
        for _ in range(2):
            self.session.run(None, {self.input_name: dummy})

    def detect(self, image):
        if image is None or image.size == 0:
            return []
        padded, ratio, pad = letterbox(image, self.imgsz)
        output = self.session.run(None, {self.input_name: to_input(padded)})[0]
        boxes, scores = decode(output, self.conf)
        if len(boxes) == 0:
            return []
        boxes = xywh_to_xyxy(restore(boxes, ratio, pad))
        picked = nms(boxes, scores, self.iou)
        height, width = image.shape[:2]
        return boxes_to_detections(boxes, scores, picked, width, height)


def boxes_to_detections(boxes, scores, picked, width: int, height: int):
    """Turn xyxy boxes into the dicts the rest of the pipeline already understands."""
    results = []
    for index in picked:
        x1, y1, x2, y2 = (float(value) for value in boxes[index])
        x1, x2 = max(0.0, x1), min(float(width), x2)
        y1, y2 = max(0.0, y1), min(float(height), y2)
        if x2 - x1 < 3 or y2 - y1 < 3:
            continue
        centre = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        contour = np.float32([[[x1, y1]], [[x2, y1]], [[x2, y2]], [[x1, y2]]])
        results.append({
            "center_px": centre,
            "contour_px": contour,
            "bbox_px": (int(x1), int(y1), int(x2 - x1), int(y2 - y1)),
            "score": float(scores[index]),
        })
    return results
