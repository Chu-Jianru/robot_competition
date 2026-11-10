"""四路台架界面：负责交互、显示与现场确认，不直接操作串口。"""
from __future__ import annotations

from datetime import datetime
from decimal import Decimal, InvalidOperation
from pathlib import Path
import time
import re
import tkinter as tk
from tkinter import scrolledtext, ttk

from serial.tools import list_ports
from motor_link import MotorLink
from bench_model import BenchData, LOCAL_TIME, duration_ms, load_settings, save_settings
from ui_theme import BG, INK, MUTED, TEAL, LINE, RoundedButton, RoundedCard, rounded
from runtime_paths import settings_file, records_directory

HERE = Path(__file__).resolve().parent


class BenchApp:
    def __init__(self, root, settings_path=None, data=None):
        self.root = root
        self.settings_path = Path(settings_path or settings_file())
        self.settings = load_settings(self.settings_path)
        self.client = None
        self.camera = None
        self.closing = False
        self.awaiting_start = False
        self.last_jog = self.busy_until = self.manual_until = 0.0
        self.data = data if data is not None else BenchData(time.monotonic())
        for name, value in self.settings["ppr_by_channel"].items():
            self.data.channels[name].ppr = float(value)
        self.channel = tk.StringVar(value=self.data.channel)
        self.encoder_title = tk.StringVar(value=f"{self.data.channel} 路编码器")
        self.command_history = self.settings["history"]
        self.history_index = len(self.command_history)
        self.history_draft = ""
        self._drawn_revision = -1
        self._run_revision = -1
        self._log_signature = None
        self.port = tk.StringVar()
        self.state = tk.StringVar(value="未连接")
        self.ticks = tk.StringVar(value="—")
        self.rate = tk.StringVar(value="—")
        self.age = tk.StringVar(value="—")
        self.encoder_detail = tk.StringVar(value="等待编码器数据")
        self.duty = tk.StringVar(value=self.settings["duty"])
        self.duration = tk.StringVar(value=self.settings["duration"])
        self.unit = tk.StringVar(value=self.settings["unit"])
        self.previous_unit = self.unit.get()
        self.direction = tk.StringVar(value="正向")
        self.confirm = tk.BooleanVar(value=False)
        self.command = tk.StringVar()
        self.message = tk.StringVar(value="连接后先确认固件；窗口不会自动运行电机。")
        self.countdown = tk.StringVar(value="等待开始")
        self.ppr = tk.StringVar(value=str(self.data.ppr or ""))
        self.rpm = tk.StringVar(value="未标定")
        self.verbose = tk.BooleanVar(value=False)
        self.autoscroll = tk.BooleanVar(value=True)
        self.search = tk.StringVar()
        self.chart_kind = tk.StringVar(value="计数变化率")
        self._build()
        if self.ppr.get():
            self.apply_calibration(quiet=True)
        self.refresh()
        self.update_buttons()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda event: self.stop())
        root.after(80, self.poll)

    @staticmethod
    def card(parent, **kwargs):
        return RoundedCard(parent, **kwargs)

    @staticmethod
    def label(parent, text=None, variable=None, size=10, color=INK, bold=False, **kwargs):
        return tk.Label(parent, text=text, textvariable=variable, bg=parent.cget("bg"), fg=color,
                        font=("Microsoft YaHei UI", size, "bold" if bold else "normal"), **kwargs)

    def _build(self):
        root = self.root
        root.configure(bg=BG)
        style = ttk.Style(root)
        style.configure("Bench.TButton", padding=(10, 7))
        style.configure("BenchAccent.TButton", background=TEAL, foreground="white", padding=(14, 9))
        style.map("BenchAccent.TButton", background=[("disabled", "#DCE6E8"), ("active", "#066B65")], foreground=[("disabled", "#8B9EA5")])
        body = tk.Frame(root, bg=BG)
        body.pack(fill="both", expand=True, padx=16, pady=(10, 8))
        bar = self.card(body)
        bar.pack(fill="x")
        self.label(bar, "设备", bold=True).pack(side="left", padx=(12, 8))
        self.combo = ttk.Combobox(bar, textvariable=self.port, state="readonly", width=34)
        self.combo.pack(side="left", fill="x", expand=True, pady=10)
        self.refresh_button = ttk.Button(bar, text="刷新", style="Bench.TButton", command=self.refresh)
        self.refresh_button.pack(side="left", padx=8)
        self.connect_button = ttk.Button(bar, text="连接", style="BenchAccent.TButton", command=self.connect)
        self.connect_button.pack(side="left", padx=(0, 12))
        self.stop_button = RoundedButton(bar, text="停止输出  /  Esc", command=self.stop, tone="danger", width=170, height=40)
        self.stop_button.pack(side="right", padx=(0, 10))
        metrics = tk.Frame(body, bg=BG)
        metrics.pack(fill="x", pady=10)
        for col, (name, value, hint) in enumerate((("设备状态", self.state, "四路电机 · 真实输出"), ("编码器", self.ticks, "当前通道 · 累计计数 · 四边沿"), ("计数变化率", self.rate, "counts/s · 接收间隔估算"), ("最近有效应答", self.age, "断连后清空实时读数"))):
            metrics.columnconfigure(col, weight=1, uniform="metric")
            card = self.card(metrics)
            card.grid(row=0, column=col, sticky="ew", padx=(0 if col == 0 else 8, 0))
            self.label(card, text=name if col != 1 else None, variable=self.encoder_title if col == 1 else None, color=MUTED, size=9).pack(anchor="w", padx=12, pady=(8, 3))
            value_label = self.label(card, variable=value, size=19, bold=True, color=INK if col else MUTED)
            value_label.pack(anchor="w", padx=16)
            if col == 0:
                self.state_label = value_label
            self.label(card, hint, color=MUTED, size=8).pack(anchor="w", padx=12, pady=(2, 8))
        self.notebook = ttk.Notebook(body)
        self.notebook.pack(fill="both", expand=True)
        self.motor_tab = tk.Frame(self.notebook, bg=BG)
        self.records_tab = tk.Frame(self.notebook, bg="white")
        self.terminal_tab = tk.Frame(self.notebook, bg="white")
        self.camera_tab = tk.Frame(self.notebook, bg="white")
        self.hardware_tab = tk.Frame(self.notebook, bg="white")
        for tab, title in ((self.motor_tab, "电机台架"), (self.records_tab, "测试记录"), (self.terminal_tab, "串口终端"), (self.camera_tab, "视觉预览"), (self.hardware_tab, "接线与进度")):
            self.notebook.add(tab, text=title)
        self._motor_ui()
        self._records_ui()
        self._terminal_ui()
        self._hardware_ui()
        self.label(self.camera_tab, "复用原项目的相机源配置，手动打开预览；画面不会发送运动命令。", color=MUTED).pack(anchor="w", padx=16, pady=12)
        self.camera_holder = tk.Frame(self.camera_tab, bg="white")
        self.camera_holder.pack(fill="both", expand=True)
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab)
        self.label(body, variable=self.message, color=MUTED, anchor="w", wraplength=1100).pack(fill="x", pady=(7, 0))

    def _motor_ui(self):
        tab = self.motor_tab
        tab.columnconfigure(0, minsize=315)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(0, weight=1)
        controls = self.card(tab)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=8)
        self.label(controls, "单电机测试", size=14, bold=True).pack(anchor="w", padx=20, pady=(20, 5))
        channel_row = tk.Frame(controls, bg="white")
        channel_row.pack(fill="x", padx=16, pady=(0, 6))
        self.label(channel_row, "测试通道").pack(side="left")
        self.channel_combo = ttk.Combobox(channel_row, textvariable=self.channel, values=tuple("ABCD"), state="readonly", width=6)
        self.channel_combo.pack(side="left", padx=8)
        self.channel_combo.bind("<<ComboboxSelected>>", self.change_channel)
        self.label(channel_row, "切换先停止", color=MUTED, size=8).pack(side="left")
        form = tk.Frame(controls, bg="white")
        form.pack(fill="x", padx=16)
        for row, (name, var) in enumerate((("输出 (%)", self.duty), ("持续时间", self.duration))):
            self.label(form, name).grid(row=row, column=0, sticky="w", pady=8)
            ttk.Entry(form, textvariable=var, width=10).grid(row=row, column=1, padx=8, sticky="ew")
        form.columnconfigure(1, weight=1)
        self.label(form, "1～100").grid(row=0, column=2)
        unit_combo = ttk.Combobox(form, textvariable=self.unit, values=("秒", "毫秒"), state="readonly", width=5)
        unit_combo.grid(row=1, column=2)
        unit_combo.bind("<<ComboboxSelected>>", self.convert_unit)
        directions = tk.Frame(controls, bg="white")
        directions.pack(fill="x", padx=16, pady=10)
        for label in ("正向", "反向"):
            ttk.Radiobutton(directions, text=label, value=label, variable=self.direction).pack(side="left", padx=(0, 18))
        ttk.Checkbutton(controls, text="电机已固定，输出轴悬空\n本次仅运行所选通道", variable=self.confirm).pack(anchor="w", padx=12, pady=(0, 10))
        self.jog_button = RoundedButton(controls, text="开始本次测试", command=self.jog_parameters, height=42)
        self.jog_button.pack(fill="x", padx=16, pady=(0, 10))
        self.progress = ttk.Progressbar(controls, maximum=100, length=100)
        self.progress.pack(fill="x", padx=16)
        self.label(controls, variable=self.countdown, color=MUTED, size=9).pack(anchor="w", padx=16, pady=(6, 10))
        actions = tk.Frame(controls, bg="white")
        actions.pack(fill="x", padx=16)
        self.query_button = ttk.Button(actions, text="读取状态", command=lambda: self.query("STATUS"))
        self.query_button.pack(side="left")
        self.zero_button = ttk.Button(actions, text="编码器清零", command=lambda: self.query("ENCZERO"))
        self.zero_button.pack(side="left", padx=8)
        ttk.Button(controls, text="保存当前参数", command=self.persist).pack(anchor="w", padx=16, pady=12)
        self.label(controls, "现场观察：约 50% 开始起转\n这不是带载或速度标定结果。", color=MUTED, size=9, justify="left").pack(anchor="w", padx=16, pady=(0, 10))
        right = self.card(tab)
        right.grid(row=0, column=1, sticky="nsew", pady=8)
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)
        head = tk.Frame(right, bg="white")
        head.grid(row=0, column=0, sticky="ew", padx=14, pady=(12, 2))
        self.label(head, "编码器实时曲线", size=13, bold=True).pack(side="left")
        chart_select = ttk.Combobox(head, textvariable=self.chart_kind, values=("计数变化率", "累计计数"), state="readonly", width=12)
        chart_select.pack(side="right")
        chart_select.bind("<<ComboboxSelected>>", lambda event: self.draw_chart())
        self.label(right, variable=self.encoder_detail, color=MUTED, size=9).grid(row=1, column=0, sticky="w", padx=14)
        self.canvas = tk.Canvas(right, bg="white", highlightthickness=0, height=160)
        self.canvas.grid(row=2, column=0, sticky="nsew", padx=10, pady=8)
        self.canvas.bind("<Configure>", lambda event: self.draw_chart())
        calibration = tk.Frame(right, bg="white")
        calibration.grid(row=3, column=0, sticky="ew", padx=14, pady=(2, 6))
        self.label(calibration, "输出轴每圈计数").pack(side="left")
        ttk.Entry(calibration, textvariable=self.ppr, width=9).pack(side="left", padx=6)
        ttk.Button(calibration, text="应用", command=self.apply_calibration).pack(side="left")
        self.label(calibration, variable=self.rpm, color=TEAL, bold=True).pack(side="right")
        self.label(right, "每圈计数需实测，包含四倍频。未标定时只显示 counts/s，不猜测 RPM。", color=MUTED, size=9, wraplength=560, justify="left").grid(row=4, column=0, sticky="w", padx=14, pady=(0, 10))

    def _records_ui(self):
        toolbar = tk.Frame(self.records_tab, bg="white")
        toolbar.pack(fill="x", padx=12, pady=10)
        self.label(toolbar, "只有设备确认运行后才生成记录；停止后等待末次编码器读数。", color=MUTED, size=9).pack(side="left")
        ttk.Button(toolbar, text="导出测试 CSV", command=lambda: self.export("runs")).pack(side="right")
        ttk.Button(toolbar, text="导出采样 CSV", command=lambda: self.export("samples")).pack(side="right", padx=6)
        columns = ("time", "channel", "duty", "planned", "observed", "delta", "invalid", "result")
        self.run_table = ttk.Treeview(self.records_tab, columns=columns, show="headings", height=9)
        self.run_table.tag_configure("alternate", background="#F6F9FC")
        for name, title, width in zip(columns, ("时间", "通道", "输出 %", "设定秒", "观测秒", "计数差", "无效增量", "结束原因"), (140, 55, 70, 80, 80, 90, 85, 250)):
            self.run_table.heading(name, text=title)
            self.run_table.column(name, width=width, minwidth=55, stretch=name in ("time", "result"))
        self.run_table.pack(fill="both", expand=True, padx=12, pady=(0, 8))
        self.label(self.records_tab, "CSV 为设备实际回传数据；采样最多保留最近 20,000 条，测试最多 500 条。计数差使用观测到的起止读数。", color=MUTED, size=9).pack(anchor="w", padx=12, pady=(0, 12))

    def _terminal_ui(self):
        tab = self.terminal_tab
        tools = tk.Frame(tab, bg="white")
        tools.pack(fill="x", padx=10, pady=8)
        ttk.Checkbutton(tools, text="显示周期收发", variable=self.verbose, command=self.render_log).pack(side="left")
        ttk.Checkbutton(tools, text="自动滚动", variable=self.autoscroll).pack(side="left", padx=8)
        self.label(tools, "筛选").pack(side="left")
        entry = ttk.Entry(tools, textvariable=self.search, width=14)
        entry.pack(side="left", padx=6)
        entry.bind("<KeyRelease>", lambda event: self.render_log())
        for name, callback in (("导出", lambda: self.export("log")), ("复制", self.copy_log), ("清空", self.clear_log)):
            ttk.Button(tools, text=name, command=callback).pack(side="right", padx=3)
        self.log_text = scrolledtext.ScrolledText(tab, height=9, state="disabled", bg="#172337", fg="#DCE6F2", font=("Consolas", 10), relief="flat", padx=16, pady=14, wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=10)
        for tag, color in (("tx", "#9CC3FF"), ("tx_auto", "#8397AF"), ("rx", "#86DFC5"), ("error", "#FF9EA9"), ("info", "#DCE6F2")):
            self.log_text.tag_configure(tag, foreground=color)
        row = tk.Frame(tab, bg="white")
        row.pack(fill="x", padx=10, pady=10)
        self.command_entry = ttk.Combobox(row, textvariable=self.command, values=tuple(reversed(self.command_history)) or ("STATUS", "ENC", "ENCZERO", "JOG 50 5000", "STOP", "HELP"))
        self.command_entry.pack(side="left", fill="x", expand=True)
        self.command_entry.bind("<Return>", self.send_command)
        self.command_entry.bind("<Up>", lambda event: self.history_move(-1))
        self.command_entry.bind("<Down>", lambda event: self.history_move(1))
        self.send_button = ttk.Button(row, text="发送 / Enter", command=self.send_command)
        self.send_button.pack(side="left", padx=(8, 0))
        self.label(tab, "回车发送单行命令，↑↓ 调出历史；完整收发仍保留在导出日志中。", color=MUTED, size=9).pack(anchor="w", padx=12, pady=(0, 8))

    def _hardware_ui(self):
        self.label(self.hardware_tab, "当前接线与能力", size=14, bold=True).pack(anchor="w", padx=18, pady=14)
        text = ("四路接线 FOUR_V1（IN2 / IN1 / 编码器 A / 编码器 B）\n"
                "A：GPIO19 / GPIO21 / GPIO22 / GPIO23\n"
                "B：GPIO4 / GPIO16 / GPIO17 / GPIO18\n"
                "C：GPIO14 / GPIO27 / GPIO26 / GPIO25\n"
                "D：GPIO33 / GPIO32 / GPIO35 / GPIO34\n"
                "STBY → GPIO13；ESP32 与 D24A 共地\n"
                "PWMA/B/C/D → D24A 自身 5V；编码器使用 3.3V\n\n"
                "连接需确认 FOUR_V1 固件；每次仅运行所选通道，停止关闭四路输出。\n\n"
                "速度闭环  待完成编码器验证与输出轴每圈计数标定\n"
                "整车运动  使用上方 DRY_RUN 模式验证指令，底盘装好后再调实车\n"
                "视觉识别  预览先独立验证；尚未接入自动运动控制\n\n"
                "当前仍在试验 ESP32 3.3V 直连识别；能转动不代表完成电平兼容性验证。")
        self.label(self.hardware_tab, text, justify="left", anchor="nw", wraplength=930).pack(fill="both", expand=True, padx=18, pady=(0, 16))

    def refresh(self):
        if self.client:
            return
        ports = sorted(list_ports.comports(), key=lambda p: (p.vid != 0x10C4, p.device))
        self.ports = {f"{p.device} — {p.description}": p.device for p in ports}
        self.combo["values"] = list(self.ports)
        if self.port.get() not in self.ports:
            self.port.set(next((label for label, device in self.ports.items() if device == self.settings["port"]), next(iter(self.ports), "")))

    def connect(self):
        # FOUR_V1 标识对应最终接线；旧固件即使会回 PONG，也不能开放运行按钮。
        if self.client:
            self.client.close()
            self.message.set("正在停止并断开…")
            return
        port = self.ports.get(self.port.get())
        if not port:
            self.message.set("未发现串口，请检查 USB 连接。")
            return
        self.confirm.set(False)
        self.data.clear_baseline()
        self.state.set("确认固件中")
        self.client = MotorLink(require_profile="FOUR_V1")
        self.client.connect(port)
        self.connect_button.configure(text="断开")
        self.combo.configure(state="disabled")
        self.refresh_button.configure(state="disabled")

    def submit(self, text):
        # 按钮与输入框共用检查和发送通道，禁止多行命令绕过确认或通道选择。
        if not isinstance(text, str) or any(ord(char) < 32 or ord(char) > 126 for char in text):
            self.message.set("每次只输入一行英文命令，不支持换行或控制字符。")
            return False
        if not self.client or not self.client.ready:
            self.message.set("请先连接，并等待固件身份与停止状态确认。")
            return False
        parts = text.split()
        is_jog = bool(parts and parts[0].upper() == "JOG")
        if is_jog:
            if len(parts) == 3:
                parts.insert(1, self.channel.get())
            elif len(parts) == 4 and parts[1].upper() != self.channel.get():
                self.message.set("请先选择对应测试通道，再确认并发送运行指令。")
                return False
            text = " ".join(parts)
        elif parts and parts[0].upper() in ("ENC", "ENCZERO") and len(parts) == 1:
            text = f"{parts[0].upper()} {self.channel.get()}"
            parts = text.split()
        if is_jog and not self.confirm.get():
            self.message.set("请先勾选电机固定与输出轴悬空确认。")
            return False
        if parts and parts[0].upper() == "ENCZERO" and self.data.motor_state == "运行中":
            self.message.set("请先停止电机，再清零编码器。")
            return False
        try:
            sent = self.client.send_line(text)
        except ValueError as exc:
            self.message.set(str(exc))
            self.log(str(exc), "error")
            return False
        if not sent:
            self.message.set("未发送：正在运行或冷却，请先停止或等待。")
            return False
        self.manual_until = time.monotonic() + 0.65
        if is_jog:
            self.busy_until = time.monotonic() + int(parts[-1]) / 1000 + 1.3
            self.awaiting_start = True
            self.last_jog = time.monotonic()
        elif text.strip().upper() == "STOP":
            self.awaiting_start = False
            self.busy_until = time.monotonic() + 1.1
        elif parts and parts[0].upper() == "ENCZERO":
            for name in ("ABCD" if parts[-1].upper() == "ALL" else parts[-1].upper()):
                self.data.clear_baseline(zero=True, channel=name)
        self.message.set("已提交：" + text.strip())
        self.update_buttons()
        return True

    def jog_parameters(self):
        try:
            duty = int(self.duty.get())
            milliseconds = duration_ms(self.duration.get(), self.unit.get())
            if not 1 <= duty <= 100:
                raise ValueError("输出请填写 1～100；方向单独选择。")
        except ValueError as exc:
            self.message.set(str(exc))
            return
        self.submit(f"JOG {-duty if self.direction.get() == '反向' else duty} {milliseconds}")

    def change_channel(self, event=None):
        # 换通道是换观测与控制对象：先清掉待发动作，再让用户重新现场确认。
        name = self.channel.get()
        if name not in "ABCD" or len(name) != 1:
            self.channel.set(self.data.channel)
            return
        if name == self.data.channel:
            return
        self.stop()
        self.confirm.set(False)
        self.data.select_channel(name)
        self.encoder_title.set(f"{name} 路编码器")
        self.ppr.set(str(self.data.ppr or ""))
        self.ticks.set("—")
        self.rate.set("—")
        self.rpm.set("等待新采样" if self.data.ppr else "未标定")
        self.encoder_detail.set("等待所选通道数据")
        self.message.set(f"已选择 {name} 路并请求全部停止，请重新确认电机后再运行。")
        self.draw_chart()
        self.update_buttons()

    def convert_unit(self, event=None):
        try:
            milliseconds = duration_ms(self.duration.get(), self.previous_unit)
            value = Decimal(milliseconds) / 1000 if self.unit.get() == "秒" else Decimal(milliseconds)
            self.duration.set(format(value.normalize(), "f"))
        except ValueError:
            self.message.set("时间未能换算，请重新填写。")
        self.previous_unit = self.unit.get()

    def query(self, command):
        self.submit(command)

    def stop(self):
        self.awaiting_start = False
        self.busy_until = time.monotonic() + 1.1
        if self.client and self.client.send("STOP"):
            self.message.set("已请求停止，等待设备确认。")

    def send_command(self, event=None):
        text = self.command.get()
        if self.submit(text):
            self.command_history.append(text.strip())
            self.command_history = self.command_history[-50:]
            self.history_index = len(self.command_history)
            self.history_draft = ""
            self.command_entry["values"] = tuple(reversed(dict.fromkeys(self.command_history)))
            self.command.set("")
        return "break"

    def history_move(self, step):
        if self.command_history:
            if self.history_index == len(self.command_history):
                self.history_draft = self.command.get()
            self.history_index = max(0, min(len(self.command_history), self.history_index + step))
            self.command.set(self.command_history[self.history_index] if self.history_index < len(self.command_history) else self.history_draft)
        return "break"

    def apply_calibration(self, quiet=False):
        # 每圈计数是当前电机输出轴的实测值，已包含编码器四倍频，不再乘 4。
        text = self.ppr.get().strip()
        try:
            number = Decimal(text) if text else None
            if number is not None and (not number.is_finite() or not 0 < number <= 1e12):
                raise InvalidOperation
        except InvalidOperation:
            self.message.set("每圈计数请填写实测正数；留空则取消 RPM 换算。")
            return
        self.data.ppr = float(number) if number else None
        self.data.clear_baseline()
        self.rpm.set("等待新采样" if number else "未标定")
        if not quiet:
            self.message.set("换算参数已应用，之后的新采样才使用该标定值。")

    def persist(self, quiet=False):
        try:
            duty = int(self.duty.get())
            if not 1 <= duty <= 100:
                raise ValueError("输出范围为 1～100")
            duration_ms(self.duration.get(), self.unit.get())
            settings = dict(duty=str(duty), duration=self.duration.get(), unit=self.unit.get(),
                            ppr=str(self.data.channels["A"].ppr or ""),
                            ppr_by_channel={name: str(state.ppr or "") for name, state in self.data.channels.items()},
                            port=self.ports.get(self.port.get(), "COM4"), history=self.command_history)
            save_settings(self.settings_path, settings)
            if not quiet:
                self.message.set("参数已保存；重新打开仍需手动连接和确认，不会恢复动作。")
        except (OSError, ValueError) as exc:
            if not quiet:
                self.message.set("参数未保存：" + str(exc))

    def log(self, text, tag="info"):
        self.data.log(tag, text)
        self.render_log()

    def render_log(self):
        query = self.search.get().casefold()
        signature = (len(self.data.logs), self.data.logs[-1] if self.data.logs else None, query, self.verbose.get(), self.autoscroll.get())
        if signature == self._log_signature:
            return
        self._log_signature = signature
        position = self.log_text.yview()[0]
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        for stamp, kind, text in self.data.logs:
            if kind == "auto" and not self.verbose.get():
                continue
            if query and query not in text.casefold():
                continue
            self.log_text.insert("end", f"{stamp[11:23]} {text}\n", "rx" if kind == "auto" else kind)
        if self.autoscroll.get():
            self.log_text.see("end")
        else:
            self.log_text.yview_moveto(position)
        self.log_text.configure(state="disabled")

    def clear_log(self):
        self.data.logs.clear()
        self.render_log()

    def copy_log(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.log_text.get("1.0", "end-1c"))
        self.message.set("当前筛选后的日志已复制。导出可保存完整日志。")

    def export(self, kind):
        try:
            directory = records_directory()
            directory.mkdir(parents=True, exist_ok=True)
            suffix = "txt" if kind == "log" else "csv"
            path = directory / f"{datetime.now(LOCAL_TIME):%Y%m%d-%H%M%S-%f}-{kind}.{suffix}"
            if kind == "log":
                path.write_text("\n".join(f"{stamp} [{tag}] {text}" for stamp, tag, text in self.data.logs), encoding="utf-8")
            else:
                self.data.export_csv(path, runs=kind == "runs")
            self.message.set(f"已保存：{path}" + (f"（较早采样已超出缓存 {self.data.dropped} 条）" if self.data.dropped else ""))
        except OSError as exc:
            self.message.set("导出失败：" + str(exc))

    def on_tab(self, event=None):
        if self.notebook.select() == str(self.camera_tab) and self.camera is None:
            from camera_view import CameraView
            self.camera = CameraView(self.camera_holder)
            self.camera.pack(fill="both", expand=True)

    def draw_chart(self):
        canvas = self.canvas
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 80 or height < 70:
            return
        left, top, right, bottom = 58, 18, width - 15, height - 28
        now = time.monotonic() - self.data.started
        samples = [s for s in self.data.samples if s.channel == self.channel.get() and now - s.elapsed_s <= 60]
        canvas.create_text(left, 7, text="计数" if self.chart_kind.get() == "累计计数" else "counts/s", anchor="w", fill=MUTED, font=("Segoe UI", 8))
        key = "ticks" if self.chart_kind.get() == "累计计数" else "counts_s"
        values = [getattr(s, key) for s in samples if getattr(s, key) is not None]
        if not values:
            for fraction in (0.2, 0.4, 0.6, 0.8):
                y = top + (bottom-top)*fraction
                canvas.create_line(left, y, right, y, fill="#F0F3F7")
            cx, cy = (left+right)/2, (top+bottom)/2 - 30
            rounded(canvas, (cx-23, cy-23, cx+23, cy+23), 12, fill="#E9F6F2", outline="")
            for offset, size in ((-10, 10), (0, 21), (10, 15)):
                canvas.create_line(cx+offset, cy+10, cx+offset, cy+10-size, fill="#69B9A9", width=4)
            canvas.create_text(cx, cy+47, text="等待编码器数据", fill=INK, font=("Microsoft YaHei UI", 12, "bold"))
            canvas.create_text(cx, cy+75, text="连接设备后，实时曲线会显示在这里", fill=MUTED, font=("Microsoft YaHei UI", 9))
            return
        low, high = (min(0, min(values)), max(0, max(values))) if values else (-1, 1)
        if low == high:
            high = low + 1
        for index in range(5):
            y = top + (bottom - top) * index / 4
            canvas.create_line(left, y, right, y, fill="#E7EDF2")
            value = high - (high-low)*index/4
            canvas.create_text(left - 7, y, text=f"{0 if abs(value) < 1e-10 else value:.3g}", anchor="e", fill=MUTED, font=("Segoe UI", 8))
        canvas.create_text(left, height - 12, text="过去 60 秒", anchor="w", fill=MUTED, font=("Microsoft YaHei UI", 8))
        canvas.create_text(right, height - 12, text="现在", anchor="e", fill=MUTED, font=("Microsoft YaHei UI", 8))
        previous = None
        for sample in samples:
            value = getattr(sample, key)
            if value is None:
                previous = None
                continue
            x = left + (sample.elapsed_s - (now - 60)) / 60 * (right - left)
            y = bottom - (value - low) / (high - low) * (bottom - top)
            if previous and previous[2] == sample.segment and sample.elapsed_s - previous[3] < 3:
                canvas.create_line(previous[0], previous[1], x, y, fill=TEAL, width=2)
            canvas.create_oval(x-2, y-2, x+2, y+2, fill=TEAL, outline="")
            previous = (x, y, sample.segment, sample.elapsed_s)

    def update_buttons(self):
        ready = bool(self.client and self.client.ready)
        self.state_label.configure(fg=TEAL if ready else MUTED)
        for button in (self.stop_button, self.query_button, self.send_button):
            button.configure(state="normal" if ready else "disabled")
        idle = time.monotonic() >= self.busy_until and self.data.motor_state != "运行中"
        self.jog_button.configure(state="normal" if ready and idle and self.confirm.get() else "disabled")
        self.zero_button.configure(state="normal" if ready and idle else "disabled")

    def handle_event(self, kind, payload, now):
        # 后台线程只传事件，Tk 控件统一在主线程更新，避免随机界面异常。
        if kind == "ready":
            self.state.set("已停止 · 就绪")
            self.data.motor_state = "已停止"
            self.message.set("已确认四路固件，选择通道并确认现场条件后开始测试。")
            self.log("固件身份与停止状态确认完成。")
        elif kind == "error":
            self.message.set(str(payload) + "；若串口占用，请退出其他监视器。")
            self.log(str(payload), "error")
        elif kind == "reset":
            self.confirm.set(False)
            self.awaiting_start = False
            self.data.disconnect(now, "设备重启，结束状态未确认")
            self.state.set("设备重启 · 同步中")
            self.log(str(payload), "error")
        elif kind == "closed":
            self.client = None
            self.confirm.set(False)
            self.awaiting_start = False
            self.data.disconnect(now)
            self.state.set("未连接")
            self.connect_button.configure(text="连接")
            self.combo.configure(state="readonly")
            self.refresh_button.configure(state="normal")
            self.ticks.set("—")
            self.rate.set("—")
            self.rpm.set("未标定" if not self.data.ppr else "等待连接")
            self.encoder_detail.set("等待编码器数据")
        elif kind == "rx":
            self.data.receive(payload, now)
            if re.match(r"JOG [A-D] ", payload):
                self.awaiting_start = False
                self.state.set("运行中")
            elif payload.startswith("STOPPED"):
                self.state.set("已停止 · 就绪")
                if payload in ("STOPPED command", "STOPPED timer", "STOPPED link_timeout"):
                    self.awaiting_start = False
                    self.busy_until = now + 1.1
                    self.message.set("通信超时，设备已关闭输出。" if payload.endswith("link_timeout") else "设备已确认停止。")
            elif payload.startswith("ERROR"):
                if self.awaiting_start:
                    self.busy_until = now + 1.1
                self.awaiting_start = False
                self.message.set(payload)
            noise = payload.startswith(("ENC ", "PONG ", "RUNNING ")) or payload in ("STOPPED A", "STOPPED ALL")
            self.data.log("error" if payload.startswith("ERROR") else "auto" if noise and now > self.manual_until else "rx", "← " + payload)
        elif kind in ("tx", "tx_auto"):
            self.data.log("auto" if kind == "tx_auto" else "tx", "→ " + payload)

    def poll(self):
        if self.closing:
            return
        now = time.monotonic()
        if self.client:
            for kind, payload in self.client.poll():
                self.handle_event(kind, payload, now)
        self.data.tick(now)
        if self.awaiting_start and now - self.last_jog > 1:
            self.stop()
            self.message.set("未收到运行确认，已请求停止。")
        latest = self.data.latest
        stale = self.data.last_sample_at is None or now - self.data.last_sample_at > 2.5
        if latest and not stale:
            self.ticks.set(str(latest.ticks))
            self.rate.set(f"{latest.counts_s:.1f}" if latest.counts_s is not None else "—")
            self.encoder_detail.set(f"A={latest.a}   B={latest.b}   无效跳变={latest.invalid}   采样 {len(self.data.samples)} 条")
            self.rpm.set(f"{latest.rpm:.1f} RPM（估算）" if latest.rpm is not None else "等待换算" if self.data.ppr else "未标定")
        elif stale:
            self.rate.set("—")
            if latest:
                self.encoder_detail.set("采样已过期，等待新数据")
                self.ticks.set("—")
                self.rpm.set("采样已过期")
        self.age.set(f"{now-self.data.last_reply_at:.1f} 秒前" if self.data.last_reply_at is not None else "—")
        active = self.data.active
        if active and active.get("stop_at") is None:
            elapsed = now - active["start"]
            total = active["duration"] / 1000
            self.countdown.set(f"已运行 {elapsed:.1f} 秒 · 预计剩余 {max(0, total-elapsed):.1f} 秒")
            self.progress["value"] = min(100, elapsed / total * 100)
        else:
            self.countdown.set("等待设备开始确认…" if self.awaiting_start else "已停止" if self.data.motor_state == "已停止" else "等待开始")
            self.progress["value"] = 0
        if self.data.revision != self._drawn_revision:
            self.draw_chart()
            self._drawn_revision = self.data.revision
        if len(self.data.runs) != self._run_revision or (self.data.runs and self.run_table.get_children() and self.run_table.item(self.run_table.get_children()[-1], "values")[0] != self.data.runs[-1].timestamp[11:19]):
            self.run_table.delete(*self.run_table.get_children())
            for index, run in enumerate(self.data.runs):
                self.run_table.insert("", "end", tags=("alternate",) if index % 2 else (), values=(run.timestamp[11:19], run.channel, run.duty, run.requested_ms/1000, run.observed_s, run.ticks_delta if run.ticks_delta is not None else "未确认", run.invalid_delta if run.invalid_delta is not None else "—", run.result))
            self._run_revision = len(self.data.runs)
        self.render_log()
        self.update_buttons()
        self.root.after(100, self.poll)

    def shutdown(self):
        self.closing = True
        if self.client:
            self.client.close()
        self.data.disconnect(time.monotonic(), "离开台架模式，结束状态未确认")
        if self.camera:
            self.camera.close()
        self.persist(quiet=True)

    def close(self):
        self.shutdown()
        self.root.after(350, self.root.destroy)
