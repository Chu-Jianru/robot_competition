"""独立相机预览组件；不持有串口客户端，不会根据画面发送运动指令。"""
from __future__ import annotations

import json
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk
from runtime_paths import camera_config_file, records_directory

HERE = Path(__file__).resolve().parent
CONFIG = camera_config_file()


class CameraView(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, bg="white")
        try:
            config = json.loads(CONFIG.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            config = {}
        self.source = tk.StringVar(value=str(config.get("source", 0)))
        self.backend = tk.StringVar(value=config.get("backend", "msmf"))
        self.status = tk.StringVar(value="未打开摄像头；请先让 iPhone 画面接入电脑。")
        self.grid_on = tk.BooleanVar(value=True)
        self.events = queue.Queue()
        self.frames = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.thread = None
        self.closed = False
        self.frame = None
        self.photo = None
        tools = tk.Frame(self, bg="white")
        tools.pack(fill="x", padx=12, pady=8)
        ttk.Label(tools, text="设备编号 / 视频流地址").pack(side="left")
        self.source_entry = ttk.Entry(tools, textvariable=self.source, width=21)
        self.source_entry.pack(side="left", padx=6)
        self.backend_combo = ttk.Combobox(tools, textvariable=self.backend, values=("msmf", "dshow", "auto"), state="readonly", width=7)
        self.backend_combo.pack(side="left")
        self.open_button = ttk.Button(tools, text="打开预览", command=self.start)
        self.open_button.pack(side="left", padx=6)
        self.close_button = ttk.Button(tools, text="关闭", command=self.stop, state="disabled")
        self.close_button.pack(side="left")
        ttk.Checkbutton(tools, text="参考网格", variable=self.grid_on, command=self.display).pack(side="right")
        self.canvas = tk.Canvas(self, bg="#152437", height=280, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=12, pady=5)
        self.canvas.bind("<Configure>", lambda event: self.display())
        bottom = tk.Frame(self, bg="white")
        bottom.pack(fill="x", padx=12, pady=10)
        self.status_label = ttk.Label(bottom, textvariable=self.status, wraplength=760)
        self.status_label.pack(side="left")
        self.snapshot_button = ttk.Button(bottom, text="保存当前帧", command=self.snapshot, state="disabled")
        self.snapshot_button.pack(side="right")
        ttk.Button(bottom, text="保存相机源", command=self.save_config).pack(side="right", padx=6)
        self.after_id = self.after(80, self.poll)

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        source, backend = self.source.get().strip(), self.backend.get()
        if not source:
            self.status.set("请填写设备编号或视频流地址")
            return
        self.stop_event = threading.Event()
        self.frame = None
        while not self.frames.empty():
            self.frames.get_nowait()
        self.snapshot_button.configure(state="disabled")
        self.display()
        self.status.set("正在打开预览…")
        self.open_button.configure(state="disabled")
        self.close_button.configure(state="normal")
        self.source_entry.configure(state="disabled")
        self.backend_combo.configure(state="disabled")
        self.thread = threading.Thread(target=self.reader, args=(source, backend, self.stop_event), daemon=True)
        self.thread.start()

    def reader(self, source, backend, stop):
        # 打开/读取相机放在后台，避免摄像头驱动超时时卡住停止按钮。
        capture = None
        try:
            from camera_source import CameraSource
            capture = CameraSource(source, width=1280, height=720, backend=backend)
            if not capture.open():
                raise RuntimeError("无法打开相机，请检查 iPhone/Camo 或设备编号")
            started, count, failures = time.monotonic(), 0, 0
            while not stop.is_set():
                frame = capture.read()
                if frame is None:
                    failures += 1
                    if failures >= 20:
                        raise RuntimeError("连续读取失败，请关闭后重试")
                    stop.wait(0.05)
                    continue
                failures = 0
                count += 1
                item = (frame, count / max(0.01, time.monotonic() - started))
                try:
                    self.frames.get_nowait()
                except queue.Empty:
                    pass
                self.frames.put_nowait(item)
                stop.wait(0.025)
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            if capture:
                try:
                    capture.close()
                except Exception as exc:
                    self.events.put(("error", "关闭预览失败：" + str(exc)))
            self.events.put(("closed", "预览已关闭"))

    def save_config(self):
        try:
            try:
                config = json.loads(CONFIG.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                config = {}
            if not isinstance(config, dict):
                config = {}
            source = self.source.get().strip()
            if not source:
                raise ValueError("请先填写设备编号或地址")
            config.update(source=int(source) if source.isdigit() else source, backend=self.backend.get())
            CONFIG.parent.mkdir(parents=True, exist_ok=True)
            temporary = CONFIG.with_suffix(".tmp")
            temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
            temporary.replace(CONFIG)
            self.status.set("相机源配置已保存，与 DRY_RUN 共用。")
        except (OSError, ValueError) as exc:
            self.status.set("配置未保存：" + str(exc))

    def stop(self):
        self.stop_event.set()
        self.close_button.configure(state="disabled")
        self.status.set("正在关闭预览…")

    def display(self):
        self.canvas.delete("all")
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        if self.frame is None:
            self.canvas.create_text(width/2, height/2, text="iPhone 俯视画面\n打开预览后显示实际视频", justify="center", fill="#A9BCCF", font=("Microsoft YaHei UI", 13))
            return
        from PIL import Image, ImageTk
        image = Image.fromarray(self.frame[:, :, ::-1])
        image.thumbnail((max(1, width), max(1, height)))
        self.photo = ImageTk.PhotoImage(image)
        self.canvas.create_image(width/2, height/2, image=self.photo)
        if self.grid_on.get():
            left, top = (width-image.width)/2, (height-image.height)/2
            for part in (1, 2, 3):
                x, y = left + image.width*part/4, top + image.height*part/4
                self.canvas.create_line(x, top, x, top+image.height, fill="#78D7BC", dash=(4, 5))
                self.canvas.create_line(left, y, left+image.width, y, fill="#78D7BC", dash=(4, 5))

    def poll(self):
        if self.closed:
            return
        while not self.events.empty():
            kind, text = self.events.get_nowait()
            if kind == "error":
                self.status.set(text)
            else:
                self.open_button.configure(state="normal")
                self.close_button.configure(state="disabled")
                self.source_entry.configure(state="normal")
                self.backend_combo.configure(state="readonly")
                self.snapshot_button.configure(state="disabled")
                self.frame = None
                self.display()
                if self.stop_event.is_set():
                    self.status.set(text)
        try:
            frame, fps = self.frames.get_nowait()
            if not self.stop_event.is_set() and self.thread and self.thread.is_alive():
                self.frame = frame
                self.status.set(f"{frame.shape[1]} × {frame.shape[0]}  ·  约 {fps:.1f} FPS  ·  仅预览")
                self.snapshot_button.configure(state="normal")
                self.display()
        except queue.Empty:
            pass
        except Exception as exc:
            self.status.set("显示失败：" + str(exc))
            self.stop_event.set()
        self.after_id = self.after(80, self.poll)

    def snapshot(self):
        if self.frame is None:
            return
        try:
            import cv2
            from bench_model import LOCAL_TIME
            from datetime import datetime
            directory = records_directory()
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f"{datetime.now(LOCAL_TIME):%Y%m%d-%H%M%S-%f}-camera.jpg"
            success, data = cv2.imencode(".jpg", self.frame)
            if not success:
                raise RuntimeError("图像编码失败")
            path.write_bytes(data.tobytes())
            self.status.set(f"已保存原始画面：{path.name}")
        except Exception as exc:
            self.status.set("保存失败：" + str(exc))

    def close(self):
        self.closed = True
        self.stop_event.set()
        self.after_cancel(self.after_id)
