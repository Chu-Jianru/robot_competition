"""菜单 13 遥控时打开的检测视频窗口：复用任务视觉，只显示画面，不控车。

画面和菜单 10/12 相同：场地蓝黄分界线、车顶 tag 边框、红块轮廓与归属
（绿=己方、橙=对方）、策略跟踪到的蓝框障碍。检测在后台线程里跑，窗口只取
最新一帧缩放后画出来；按键始终发给遥控窗口，视频窗口不接收键盘焦点。
"""
from __future__ import annotations

import time

WINDOW_TITLE = "遥控检测 | 场地分界 · tag 7 · 红块归属 · 障碍"
DISPLAY_PX = 600
REFRESH_MS = 100
WAITING = "正在等待手机画面…"


def compose_view(marked, scene, size=DISPLAY_PX):
    """把任务视觉的校正图缩到窗口大小，并补画障碍框（与菜单 10/12 一致）。"""
    import cv2

    display = cv2.resize(marked, (size, size), interpolation=cv2.INTER_AREA)
    if scene is None or not scene.get("valid"):
        return display
    scale = size / 1.2
    direction = -1 if scene["home_side"] == "yellow" else 1
    bias = scene["seam_bias"]
    for x, y, half_x, half_y in scene.get("obstacles") or ():
        left = round((.6 + direction * x - half_x) * scale)
        top = round((.6 - direction * y - bias - half_y) * scale)
        right = round((.6 + direction * x + half_x) * scale)
        bottom = round((.6 - direction * y - bias + half_y) * scale)
        cv2.rectangle(display, (left, top), (right, bottom), (255, 100, 0), 2)
    return display


def view_caption(scene, label, now, stale_s):
    """窗口顶部一行：己方半区、红块数、tag 边长、障碍数、帧龄。"""
    if scene is None:
        return label or "正在等待手机画面…"
    age_ms = 1000 * (now - scene["stamp"])
    if not scene.get("valid"):
        return f"{label}  ·  帧龄 {age_ms:.0f} ms"
    side = "蓝区" if scene["home_side"] == "blue" else "黄区"
    own = sum(1 for block in scene["blocks"] if block[4])
    text = (f"己方{side}  ·  红块 {len(scene['blocks'])} 个（己方 {own}）"
            f"  ·  障碍 {len(scene.get('obstacles') or ())} 个  ·  帧龄 {age_ms:.0f} ms")
    if scene.get("tag_side_mm"):
        text = f"tag {scene['tag_side_mm']:.0f} mm  ·  " + text
    return text + ("  ·  画面过期" if age_ms > 1000 * stale_s else "")


class RemoteView:
    """相机和任务视觉线程的宿主；窗口只是它们的只读显示。"""

    def __init__(self, vision, *, stale_s):
        self.vision = vision
        self.stale_s = stale_s
        self.tk = None
        self.root = None
        self.top = None
        self.canvas = None
        self.frame_item = None
        self.status_item = None
        self.caption = None
        self.photo = None
        self.reported = None
        self.opened_at = time.monotonic()
        self.closed = False

    @classmethod
    def open(cls, *, scrcpy_dir=None, rotate="auto", calibration=None, detector="auto",
             max_size=None, detect_obstacles=None):
        """按任务入口的默认取流/检测参数启动相机和视觉线程。

        场地标定、手机取流或 ONNX 依赖不可用时抛出异常，由调用方决定是否
        仍然只做遥控。`detect_obstacles` 为 None 时按 `detection_flags.json`
        （菜单 14 的开关，当前关闭）。
        """
        import run_task
        from pathlib import Path

        from detection_flags import obstacles_enabled

        options = run_task.build_parser().parse_args([])
        if scrcpy_dir is not None:
            options.scrcpy_dir = Path(scrcpy_dir)
        if calibration is not None:
            options.calibration = Path(calibration)
        if max_size is not None:
            options.max_size = max_size
        options.detector = detector
        options.rotate = None if rotate in (None, "auto") else int(rotate)
        obstacles = obstacles_enabled() if detect_obstacles is None else detect_obstacles
        calib = run_task.load_calibration(options.calibration)
        facing = "auto" if options.rotate is None else f"{options.rotate}°"
        size = tuple(getattr(calib, "source_size", None) or ())
        print(f"检测窗口：相机朝向 {facing}，场地标定 {'×'.join(map(str, size)) or '未知'}"
              f"，障碍检测{'开' if obstacles else '关'}（与菜单 10/11/12 相同）", flush=True)
        vision = run_task.task_vision(options, calib, detect_obstacles=obstacles)
        try:
            vision.camera.start()
            vision.thread.start()
        except Exception:
            vision.stop.set()
            vision.camera.close()
            raise
        return cls(vision, stale_s=run_task.MAX_AGE)

    def attach(self, root):
        """在遥控窗口旁边建只读视频窗口，并把键盘焦点留在遥控窗口。"""
        import tkinter as tk

        self.tk, self.root = tk, root
        self.top = tk.Toplevel(root)
        self.top.title(WINDOW_TITLE)
        frame = tk.Frame(self.top, padx=8, pady=8)
        frame.pack(fill="both", expand=True)
        self.caption = tk.StringVar(master=self.top, value="正在等待手机画面…")
        tk.Label(frame, textvariable=self.caption, anchor="w",
                 justify="left").pack(fill="x", pady=(0, 6))
        # 固定画布：没收到画面时窗口也不会变成一大片黑，原因直接写在中间。
        self.canvas = tk.Canvas(frame, width=DISPLAY_PX, height=DISPLAY_PX,
                                bg="#1f1f1f", highlightthickness=0)
        self.canvas.pack()
        self.frame_item = self.canvas.create_image(0, 0, anchor="nw")
        self.status_item = self.canvas.create_text(
            DISPLAY_PX // 2, DISPLAY_PX // 2, text=WAITING, fill="#e6e6e6",
            width=DISPLAY_PX - 80, justify="center", font=("Microsoft YaHei UI", 10))
        # 视频窗口不接受键盘焦点：点它等于点回遥控窗口。
        for widget in (self.top, self.canvas):
            widget.bind("<Button-1>", lambda _event: root.focus_force())
        self.top.bind("<Map>", lambda _event: root.focus_force())
        root.focus_force()
        self.refresh()

    def refresh(self):
        """刷新窗口；显示出错只停检测，不能影响遥控。"""
        if self.closed:
            return
        try:
            self._draw()
        except Exception as error:
            self._report(f"窗口显示失败：{error}")
            try:
                if self.caption is not None:
                    self.caption.set(f"检测窗口显示失败：{error}（遥控仍可用）")
                if self.canvas is not None:
                    self._show_status(f"检测窗口显示失败\n{error}\n\n遥控仍可用")
            except Exception:  # noqa: BLE001 - 窗口本身出问题时只求别再打断遥控
                pass
            self.stop_detection()
            return
        if not self.closed:
            self.root.after(REFRESH_MS, self.refresh)

    def _draw(self):
        if self.top is None or not self.top.winfo_exists():
            self.close()
            return
        import cv2

        now = time.monotonic()
        problem = self.vision.error or self.vision.camera.error
        if problem:
            self.caption.set(f"检测已停止：{problem}（遥控仍可用）")
            self._show_status(f"检测已停止\n{problem}\n\n遥控仍可用")
            self._report(problem)
            self.stop_detection()
            return
        snapshot = self.vision.latest
        if snapshot is None:
            waited = now - self.opened_at
            self.caption.set(f"正在等待手机画面（{waited:.0f} 秒）…")
            self._show_status(f"正在等待手机画面（{waited:.0f} 秒）…\n"
                              "检查手机取景、USB 线和其它取图窗口")
            return
        scene, marked, label = snapshot
        photo = self.tk.PhotoImage(
            master=self.top,
            data=cv2.imencode(".ppm", compose_view(marked, scene))[1].tobytes())
        self.canvas.itemconfigure(self.frame_item, image=photo)
        self.photo = photo
        self._show_status("")
        self.caption.set(view_caption(scene, label, now, self.stale_s))

    def _show_status(self, text):
        """没画面时把原因写在画面中间；有画面时清空这行字。"""
        self.canvas.itemconfigure(self.status_item, text=text)

    def _report(self, message):
        """停止原因同时打到控制台，窗口排版异常时也能看到。"""
        if message != self.reported:
            self.reported = message
            print(f"检测已停止，遥控仍可用：{message}", flush=True)

    def stop_detection(self):
        """停相机和视觉线程；窗口保留，遥控不受影响。"""
        if self.closed:
            return
        self.closed = True
        self.vision.stop.set()
        self.vision.camera.close()
        if self.vision.thread.is_alive():
            self.vision.thread.join(timeout=1.)

    def close(self):
        self.stop_detection()
        top, self.top = self.top, None
        if top is not None:
            try:
                top.destroy()
            except self.tk.TclError:  # 用户已经点掉了窗口。
                pass
