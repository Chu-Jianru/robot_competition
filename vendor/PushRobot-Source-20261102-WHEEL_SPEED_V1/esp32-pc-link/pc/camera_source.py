"""Reusable single-camera frame source.

This module is the clear interface for the teammate's vision code:
    from camera_source import CameraSource
    cam = CameraSource(0)          # or an RTSP/HTTP URL
    if cam.open():
        frame = cam.read()         # BGR numpy.ndarray or None
        ...
        cam.close()

It has no connection to the serial/motion client. Vision results must be
passed to the control layer (which creates DRIVE commands); never call the
ESP32 from a frame-processing loop without the session/watchdog design.
"""

from __future__ import annotations

import cv2


def parse_source(value: str) -> int | str:
    text = str(value).strip()
    if text.isdigit():
        return int(text)
    return text


class CameraSource:
    """封装 OpenCV 相机/视频流；调用者显式打开和关闭，不接触电机控制。"""

    def __init__(
        self,
        source: int | str,
        *,
        width: int = 1280,
        height: int = 720,
        backend: str = "auto",
    ) -> None:
        self.source = parse_source(source) if isinstance(source, str) else source
        self.width = width
        self.height = height
        self.backend = backend
        self._capture = None
        self._reported_fps = None

    def open(self) -> bool:
        if self._capture is not None:
            return True
        capture = None
        if isinstance(self.source, int):
            if self.backend in ("auto", "dshow") and hasattr(cv2, "CAP_DSHOW"):
                capture = cv2.VideoCapture(self.source, cv2.CAP_DSHOW)
                if not capture.isOpened() and self.backend == "auto":
                    capture.release()
                    capture = None
            if capture is None:
                if self.backend == "msmf" and hasattr(cv2, "CAP_MSMF"):
                    capture = cv2.VideoCapture(self.source, cv2.CAP_MSMF)
                else:
                    capture = cv2.VideoCapture(self.source)
        else:
            capture = cv2.VideoCapture(self.source)

        if not capture.isOpened():
            capture.release()
            return False
        if self.width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._capture = capture
        self._reported_fps = capture.get(cv2.CAP_PROP_FPS)
        return True

    def read(self):
        """返回一帧 BGR 图像（不是 RGB）；失败时返回 None，由上层决定重试。"""
        if self._capture is None:
            return None
        ok, frame = self._capture.read()
        if not ok or frame is None:
            return None
        return frame

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
            self._reported_fps = None

    @property
    def is_open(self) -> bool:
        return self._capture is not None and self._capture.isOpened()

    @property
    def reported_fps(self):
        return self._reported_fps

    def __enter__(self) -> "CameraSource":
        self.open()
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
