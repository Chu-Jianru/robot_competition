"""Read-only camera tab for the debug panel.

The camera preview has no access to LinkClient and never sends serial or
motion commands. OpenCV/Pillow are imported lazily so the rest of the panel
still works if the camera dependencies are unavailable.
"""

from __future__ import annotations

import json
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

PREVIEW_W = 480
PREVIEW_H = 270
FONT = "Microsoft YaHei UI"


class CameraPanel(tk.Frame):
    def __init__(self, parent, config_path, on_status=None) -> None:
        super().__init__(parent, bg="white")
        self.config_path = Path(config_path)
        self.on_status = on_status
        self._config = self._load_config()
        self._width = self._config_int("width", 1920)
        self._height = self._config_int("height", 1080)

        self.source_var = tk.StringVar(
            value=str(self._config.get("source", "0"))
        )
        self.backend_var = tk.StringVar(
            value=str(self._config.get("backend", "msmf"))
        )
        self.status_var = tk.StringVar(value="摄像头未打开")

        self._capture = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._latest_frame = None
        self._frame_count = 0
        self._last_displayed_count = -1
        self._error: str | None = None
        self._after_id = None
        self._source_text = ""
        self._fps_time = time.monotonic()
        self._fps_frames = 0
        self._fps = 0.0
        self._cv2 = None
        self._Image = None
        self._ImageTk = None
        self._photo = None

        self._build_ui()

    # ------------------------------------------------------------------ config

    def _load_config(self) -> dict:
        try:
            data = json.loads(self.config_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}
        return data if isinstance(data, dict) else {}

    def _config_int(self, key: str, fallback: int) -> int:
        try:
            return int(self._config.get(key, fallback))
        except (TypeError, ValueError):
            return fallback

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        toolbar = tk.Frame(self, bg="white")
        toolbar.pack(fill="x", padx=10, pady=(8, 4))
        ttk.Label(toolbar, text="源/URL").pack(side="left")
        self.source_entry = ttk.Entry(toolbar, textvariable=self.source_var, width=12)
        self.source_entry.pack(side="left", padx=(4, 10))
        ttk.Label(toolbar, text="后端").pack(side="left")
        self.backend_combo = ttk.Combobox(
            toolbar,
            textvariable=self.backend_var,
            values=("msmf", "dshow", "auto"),
            width=8,
            state="readonly",
        )
        self.backend_combo.pack(side="left", padx=(4, 10))
        self.open_button = ttk.Button(
            toolbar, text="打开摄像头", style="Accent.TButton", command=self.start
        )
        self.open_button.pack(side="left", padx=4)
        self.close_button = ttk.Button(
            toolbar, text="关闭", command=self.stop
        )
        self.close_button.pack(side="left", padx=4)
        self.close_button.state(["disabled"])

        holder = tk.Frame(self, bg="#0E1A26", width=PREVIEW_W, height=PREVIEW_H)
        holder.pack(padx=12, pady=(6, 4))
        holder.pack_propagate(False)
        self.preview = tk.Label(
            holder,
            bg="#0E1A26",
            fg="#A9B8C7",
            text="摄像头未打开\n点击“打开摄像头”",
            font=(FONT, 11),
        )
        self.preview.pack(fill="both", expand=True)

        ttk.Label(
            self, textvariable=self.status_var, style="Muted.TLabel"
        ).pack(anchor="w", padx=14, pady=(2, 2))
        ttk.Label(
            self,
            text="仅预览，不发送串口或运动指令。",
            style="Muted.TLabel",
        ).pack(anchor="w", padx=14, pady=(0, 2))
        ttk.Label(
            self,
            text=(
                "后端：msmf = Media Foundation；dshow = DirectShow；"
                "auto = 先试 DirectShow。本机 Camo 用 msmf / 编号 0。"
            ),
            style="Muted.TLabel",
            wraplength=460,
        ).pack(anchor="w", padx=14, pady=(0, 8))

    # ------------------------------------------------------------------ feed

    def start(self) -> None:
        if self._running:
            return
        try:
            import cv2
            from PIL import Image, ImageTk

            from camera_source import CameraSource, parse_source
        except Exception as exc:  # missing dependency
            self.status_var.set(f"缺少依赖：{exc}；请运行 setup.cmd")
            return

        source_text = self.source_var.get().strip()
        if not source_text:
            self.status_var.set("请输入摄像头编号或流地址")
            return
        backend = self.backend_var.get().strip() or "msmf"
        source = parse_source(source_text)
        capture = CameraSource(
            source,
            width=self._width,
            height=self._height,
            backend=backend,
        )
        if not capture.open():
            self.status_var.set(f"无法打开 {source_text!r}（{backend}）")
            return

        self._cv2 = cv2
        self._Image = Image
        self._ImageTk = ImageTk
        self._capture = capture
        self._source_text = source_text
        self._latest_frame = None
        self._frame_count = 0
        self._last_displayed_count = -1
        self._error = None
        self._fps_time = time.monotonic()
        self._fps_frames = 0
        self._fps = 0.0
        self._running = True
        self._thread = threading.Thread(
            target=self._reader_loop, name="camera-reader", daemon=True
        )
        self._thread.start()

        self.open_button.state(["disabled"])
        self.close_button.state(["!disabled"])
        self.source_entry.state(["disabled"])
        self.backend_combo.state(["disabled"])
        self.status_var.set(f"已打开 {source_text}（{backend}），等待画面…")
        if self.on_status is not None:
            self.on_status("摄像头已打开")
        self._schedule()

    def stop(self, reset_status: bool = True) -> None:
        was_running = self._running
        self._running = False
        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.6)
        capture = self._capture
        self._capture = None
        if capture is not None:
            try:
                capture.close()
            except Exception:
                pass
        self._latest_frame = None
        self._last_displayed_count = -1
        try:
            self.preview.configure(image="", text="摄像头未打开\n点击“打开摄像头”")
            self._photo = None
            self.open_button.state(["!disabled"])
            self.close_button.state(["disabled"])
            self.source_entry.state(["!disabled"])
            self.backend_combo.state(["!disabled"])
        except tk.TclError:
            pass
        if reset_status:
            self.status_var.set("摄像头已关闭")
        if was_running and self.on_status is not None:
            self.on_status("摄像头已关闭")

    def close(self) -> None:
        self.stop(reset_status=False)

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._frame_count

    def _reader_loop(self) -> None:
        failures = 0
        while self._running:
            capture = self._capture
            if capture is None:
                break
            try:
                frame = capture.read()
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                break
            if frame is None:
                failures += 1
                if failures >= 30:
                    with self._lock:
                        self._error = "连续读取失败"
                    break
                time.sleep(0.05)
                continue
            failures = 0
            with self._lock:
                self._latest_frame = frame.copy()
                self._frame_count += 1
            time.sleep(0.005)

    def _schedule(self) -> None:
        if self._running:
            self._after_id = self.after(50, self._poll)

    def _poll(self) -> None:
        self._after_id = None
        if not self._running:
            return
        with self._lock:
            frame = self._latest_frame
            count = self._frame_count
            error = self._error
        if error is not None:
            self.status_var.set(f"读取错误：{error}")
            self.stop(reset_status=False)
            return
        if frame is not None and count != self._last_displayed_count:
            cv2 = self._cv2
            Image = self._Image
            ImageTk = self._ImageTk
            if cv2 is None or Image is None or ImageTk is None:
                self.stop(reset_status=False)
                return
            try:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(rgb).resize(
                    (PREVIEW_W, PREVIEW_H), Image.Resampling.BILINEAR
                )
                photo = ImageTk.PhotoImage(image)
                self.preview.configure(image=photo, text="")
                self._photo = photo
            except Exception as exc:
                self.status_var.set(f"显示失败：{exc}")
                self.stop(reset_status=False)
                return
            self._last_displayed_count = count
            height, width = frame.shape[:2]
            now = time.monotonic()
            elapsed = now - self._fps_time
            if elapsed >= 0.8:
                self._fps = (count - self._fps_frames) / elapsed
                self._fps_time = now
                self._fps_frames = count
                fps_text = f"约 {self._fps:.1f} FPS"
            else:
                fps_text = "启动中…"
            self.status_var.set(
                f"源 {self._source_text} | {width}x{height} | {fps_text}"
            )
        self._schedule()
