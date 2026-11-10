"""统一调试工作台：保留整车模拟，并承载四路电机台架。"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import tkinter as tk
from tkinter import ttk

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for path in (HERE, ROOT / "motor-bringup", ROOT / "esp32-pc-link" / "pc"):
    # 源码按原工程目录共享串口和界面模块，解压后无需修改绝对路径。
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from bench_app import BenchApp
from chassis_app import ChassisApp, DEFAULT_POSITION_HINT
from panel import PanelApp
from ui_theme import BG, INK, MUTED, TEAL, RoundedButton, apply_theme, rounded


class ViewHost(tk.Frame):
    """将旧面板嵌入同一窗口，并在切换时清除它注册的定时器和按键。"""
    def __init__(self, root):
        super().__init__(root, bg=BG)
        self.embedded = True
        self.window = root
        self.timer_ids = set()
        self.bindings = []
        self.disposed = False

    def title(self, *args):
        pass

    def geometry(self, *args):
        pass

    def minsize(self, *args):
        pass

    def protocol(self, name, callback):
        self.close_callback = callback

    def bind(self, sequence=None, func=None, add=None):
        identifier = self.window.bind(sequence, func, add="+")
        self.bindings.append((sequence, identifier))
        return identifier

    def after(self, ms, func=None, *args):
        if func is None:
            return super().after(ms)
        identifier = None

        def run():
            self.timer_ids.discard(identifier)
            if not self.disposed:
                func(*args)

        identifier = self.window.after(ms, run)
        self.timer_ids.add(identifier)
        return identifier

    def after_cancel(self, identifier):
        self.timer_ids.discard(identifier)
        self.window.after_cancel(identifier)

    def destroy(self):
        if self.disposed:
            return
        self.disposed = True
        for identifier in list(self.timer_ids):
            self.window.after_cancel(identifier)
        self.timer_ids.clear()
        for sequence, identifier in self.bindings:
            self.window.unbind(sequence, identifier)
        super().destroy()


class Workspace:
    def __init__(self, root, mode="bench", settings_path=None):
        self.root, self.app, self.host = root, None, None
        self.settings_path = settings_path
        self.mode = None
        self.bench_data = None
        self.closed = False
        self.transitioning = False
        root.title("推物块机器人 · 调试工作台")
        width, height = min(1220, root.winfo_screenwidth()-70), min(860, root.winfo_screenheight()-90)
        root.geometry(f"{width}x{height}")
        root.minsize(min(980, width), min(700, height))
        root.option_add("*Font", ("Microsoft YaHei UI", 10))
        root.configure(bg=BG)
        apply_theme(root)
        bar = tk.Frame(root, bg="white", padx=22, pady=13)
        bar.pack(fill="x")
        icon = tk.Canvas(bar, width=38, height=38, bg="white", highlightthickness=0)
        icon.pack(side="left", padx=(0, 12))
        rounded(icon, (1, 1, 37, 37), 11, fill=TEAL, outline="")
        rounded(icon, (9, 12, 29, 28), 5, fill="white", outline="")
        icon.create_line(19, 8, 19, 12, fill="white", width=2)
        for x in (15, 23):
            icon.create_oval(x-1.5, 18, x+1.5, 21, fill=TEAL, outline="")
        brand = tk.Frame(bar, bg="white")
        brand.pack(side="left")
        tk.Label(brand, text="机器人调试工作台", bg="white", fg=INK, font=("Microsoft YaHei UI", 14, "bold")).pack(anchor="w")
        tk.Label(brand, text="连接设备  /  查看反馈  /  记录测试", bg="white", fg=MUTED, font=("Microsoft YaHei UI", 8)).pack(anchor="w", pady=(3, 0))
        nav = tk.Frame(bar, bg="#F0F3F8", padx=3, pady=3)
        nav.pack(side="left", padx=24)
        self.buttons = {}
        for key, title in (("dry", "整车模拟 DRY_RUN"), ("bench", "电机台架"), ("chassis", "麦克纳姆底盘")):
            button = RoundedButton(nav, text=title, command=lambda value=key: self.switch(value), tone="secondary", width=144, height=38)
            button.pack(side="left", padx=2)
            self.buttons[key] = button
        self.badge = tk.Label(bar, bg="#E7F4F0", fg=TEAL, font=("Microsoft YaHei UI", 9), padx=12, pady=7)
        self.badge.pack(side="right")
        self.mode_hint = tk.StringVar()
        tk.Label(root, textvariable=self.mode_hint, bg=BG, fg=MUTED, anchor="w", padx=22, pady=5, font=("Microsoft YaHei UI", 9)).pack(fill="x")
        self.switch(mode)
        root.protocol("WM_DELETE_WINDOW", self.close)

    def switch(self, mode):
        # 先停止并关闭原客户端，保留观测记录；新视图不会自动连接或运行。
        if self.closed or self.transitioning or mode == self.mode:
            return
        self.transitioning = True
        for button in self.buttons.values():
            button.configure(state="disabled")
        threads = []
        if self.app:
            client = self.app.client
            if client:
                threads = [getattr(client, name, None) for name in ("_thread", "_reader", "_writer")]
            if self.mode == "dry":
                self.app.on_close()
            else:
                if self.mode == "bench": self.bench_data = self.app.data
                self.app.shutdown()
                self.host.destroy()
            self.app = self.host = None
        self.badge.configure(text="正在停止并释放串口…")
        self._show_when_closed(mode, threads)

    def _show_when_closed(self, mode, threads):
        if self.closed:
            return
        if any(thread and thread.is_alive() for thread in threads):
            # 等待串口线程真正退出后再建立新视图，避免两个客户端抢同一 COM。
            self.root.after(40, lambda: self._show_when_closed(mode, threads))
            return
        self.host = ViewHost(self.root)
        self.host.pack(fill="both", expand=True)
        self.mode = mode
        if mode == "dry":
            self.app = PanelApp(self.host)
        elif mode == "chassis":
            self.app = ChassisApp(self.host)
        else:
            self.app = BenchApp(self.host, settings_path=self.settings_path, data=self.bench_data)
        apply_theme(self.root)
        self.badge.configure(text="●  模拟模式" if mode == "dry" else ("●  真实输出 · 底盘" if mode == "chassis" else "●  真实输出 · 四路台架"), bg="#E7F4F0" if mode == "dry" else "#FFF3E5", fg=TEAL if mode == "dry" else "#AC7528")
        self.mode_hint.set("统一固件 · 连接时自动进入整车模拟，四路输出关闭。" if mode == "dry" else (f"X 型混控 · {DEFAULT_POSITION_HINT}；实测方向已填入，应用布局后统一前进。" if mode == "chassis" else "选择 A/B/C/D 逐路测试，编码器与标定按通道独立记录。"))
        for key, button in self.buttons.items():
            button.configure(bg="white" if key == mode else "#F0F3F8", fg=TEAL if key == mode else MUTED, state="normal")
        self.transitioning = False

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.app:
            if self.mode == "dry":
                self.app.on_close()
            else:
                self.app.shutdown()
                self.host.destroy()
        self.root.after(400, self.root.destroy)


def main():
    parser = argparse.ArgumentParser(description="推物块机器人统一调试工作台")
    parser.add_argument("--mode", choices=("dry", "bench", "chassis"), default="bench")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        import unittest
        suite = unittest.defaultTestLoader.discover(str(HERE / "tests"), pattern="test_workspace.py")
        return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1
    root = tk.Tk()
    Workspace(root, args.mode)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
