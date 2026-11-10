"""真实麦克纳姆底盘界面：定时开环/轮速闭环控制。

阅读顺序：_build 建界面 -> run_motion/start_calibration 发请求 -> pump 收事件。
update_controls 统一刷新可用状态，串口线程不直接操作控件。
固件决定实际运行/故障状态；按钮入队成功不等于电机已启动。
调试路径、协议字段与故障排查见 ../motor-bringup/DEBUGGING.md。
"""
import re
import time
import tkinter as tk
from tkinter import messagebox, ttk

from serial.tools import list_ports
from chassis_link import ChassisLink
from bench_model import duration_ms
from ui_theme import BG, INK, MUTED, TEAL, RoundedButton

# 2026-10-27 最新实测修正了 B/C；四路电气反向均固化在 V2 驱动。
# 上层和界面四轮仍全部 +1，不能重复补偿。
DEFAULT_CHANNELS = ("C", "B", "A", "D")  # 左前、右前、左后、右后。
DEFAULT_SIGNS = (1, 1, 1, 1)
DEFAULT_POSITION_HINT = "左前 C / 右前 B / 左后 A / 右后 D"
WHEEL_LABELS = {"A": "A 左后 RL", "B": "B 右前 FR", "C": "C 左前 FL", "D": "D 右后 RR"}
CAL_TIMEOUT_SECONDS = 20.0
UI_POLL_INTERVAL_MS = 50  # 界面取事件周期，独立于固件 10 ms 控制周期。
RUN_ACK_TIMEOUT_SECONDS = 3.0  # 入队后未收到运行确认，请求 STOP。


class ChassisApp:
    def __init__(self, root):
        self.root = root
        self.client = None
        self.closed = False
        self.ports = {}
        self.port = tk.StringVar()
        self.state = tk.StringVar(value="未连接 · 默认无输出")
        self.confirm = tk.BooleanVar(value=False)
        self.x_layout = tk.BooleanVar(value=False)
        self.layout_status = tk.StringVar(value="已填入实测布局；连接后点击应用，前进方向自动统一")
        self.channels = [tk.StringVar(value=c) for c in DEFAULT_CHANNELS]
        self.signs = [tk.StringVar(value=f"{s:+d}") for s in DEFAULT_SIGNS]
        self.power = tk.StringVar(value="50")
        self.duration = tk.StringVar(value="0.5")
        self.axes = [tk.StringVar(value="0") for _ in range(3)]
        self.wheels = [tk.StringVar(value="0") for _ in range(4)]
        self.closed_loop = tk.BooleanVar(value=False)
        self.closed_status = tk.StringVar(value="当前固件不支持轮速闭环")
        self.kp = tk.StringVar(value="0.000")
        self.ki = tk.StringVar(value="0.000")
        self.limit = tk.StringVar(value="80")
        self.cal_progress = tk.StringVar(value="未开始标定")
        self.motion_hint = tk.StringVar()
        self.calibrating = False
        self.fault_latched = False
        self.cal_started_at = 0.0  # 最近一次标定进度时间；超时按“无进度”计算。
        self.telem_on = False
        self.telem_allowed = True
        self.dialog_open = False
        self.motion_buttons = []
        self.mixed_buttons = []
        self.encoder_values = {}
        self.last_reply = None
        self.reply_age = tk.StringVar(value="最近应答：—")
        self.cooldown_until = 0.0
        self.device_running = False
        self.pending_run = False  # 等待 CHASSIS_RUN/SPEED_RUN，与 device_running 分开。
        self.pending_at = 0.0
        self.log_lines = []
        self._logged_fault_line = None
        self._build()
        self.refresh_ports()
        self.root.bind("<Escape>", lambda event: self.stop())
        self.root.bind("<FocusOut>", self.on_focus_out)
        self.root.after(UI_POLL_INTERVAL_MS, self.pump)

    def _button(self, parent, text, command, *, motion=False, mixed=False, **options):
        button = RoundedButton(parent, text=text, command=command, height=36, **options)
        if motion:
            self.motion_buttons.append(button)
            button.configure(state="disabled")
        if mixed:
            self.mixed_buttons.append(button)
        return button

    def _build(self):
        body = tk.Frame(self.root, bg=BG, padx=16, pady=8)
        body.pack(fill="both", expand=True)
        connect = ttk.Frame(body, padding=10)
        connect.pack(fill="x")
        ttk.Label(connect, text="USB / 蓝牙串口").pack(side="left", padx=(0, 10))
        self.port_box = ttk.Combobox(connect, textvariable=self.port, state="readonly", width=54)
        self.port_box.pack(side="left", fill="x", expand=True)
        self._button(connect, "刷新", self.refresh_ports, width=72, tone="secondary").pack(side="left", padx=6)
        self.connect_button = self._button(connect, "连接", self.toggle_connection, width=90)
        self.connect_button.pack(side="left", padx=6)
        self._button(connect, "停止全部 / Esc", self.stop, width=160, tone="danger").pack(side="left")
        status = ttk.Frame(body, padding=10)
        status.pack(fill="x", pady=(8, 8))
        ttk.Label(status, textvariable=self.state, foreground=TEAL, font=("Microsoft YaHei UI", 12, "bold")).pack(side="left")
        ttk.Label(status, textvariable=self.reply_age, foreground=MUTED).pack(side="right")

        notebook = ttk.Notebook(body)
        notebook.pack(fill="both", expand=True)
        control, calib, direct, feedback, instructions = [ttk.Frame(notebook, padding=14) for _ in range(5)]
        for frame, name in ((control, "麦克纳姆控制"), (calib, "闭环调参"), (direct, "四路独立输出"),
                            (feedback, "编码器与日志"), (instructions, "使用与接线")):
            notebook.add(frame, text=name)
        notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)
        self.notebook, self.mecanum_tab, self.cal_tab = notebook, control, calib
        left, right = ttk.Frame(control), ttk.Frame(control)
        left.pack(side="left", fill="y", padx=(0, 26))
        right.pack(side="left", fill="both", expand=True)
        ttk.Label(left, text="实测布局 · 统一前进方向", font=("Microsoft YaHei UI", 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))
        for i, name in enumerate(("左前 FL", "右前 FR", "左后 RL", "右后 RR")):
            ttk.Label(left, text=name).grid(row=i+1, column=0, padx=(0, 16), pady=7)
            box = ttk.Combobox(left, textvariable=self.channels[i], values=list("ABCD"), state="readonly", width=5)
            box.grid(row=i+1, column=1, padx=4)
            sign = ttk.Combobox(left, textvariable=self.signs[i], values=("+1",), state="disabled", width=5)
            sign.grid(row=i+1, column=2, padx=4)
            for widget in (box, sign): widget.bind("<<ComboboxSelected>>", self.invalidate_layout)
        ttk.Label(left, text="四路 +1 全部表示向前，-1 表示向后。\n反向修正已写入驱动，不再由面板补偿。", foreground=MUTED).grid(row=5, column=0, columnspan=3, sticky="w", pady=10)
        ttk.Label(left, text="车头 ↑     标准 X 型滚轮\n左前  \\        /  右前\n左后  /        \\  右后", font=("Consolas", 12)).grid(row=6, column=0, columnspan=3, pady=8)
        ttk.Checkbutton(left, text="已核对实物为上述 X 型", variable=self.x_layout, command=self.invalidate_layout).grid(row=7, column=0, columnspan=3, sticky="w", pady=8)
        self.apply_button = self._button(left, "应用轮位和方向", self.apply_layout, width=260)
        self.apply_button.grid(row=8, column=0, columnspan=3, pady=6)
        ttk.Label(left, textvariable=self.layout_status, wraplength=300, foreground=MUTED).grid(row=9, column=0, columnspan=3, sticky="w", pady=8)

        ttk.Label(right, text="定时运动 · 输出百分比", font=("Microsoft YaHei UI", 12, "bold")).pack(anchor="w")
        loop_row = ttk.Frame(right)
        loop_row.pack(fill="x", pady=(6, 0))
        self.closed_loop_box = ttk.Checkbutton(loop_row, text="轮速闭环", variable=self.closed_loop, command=self.on_closed_loop_toggle)
        self.closed_loop_box.pack(side="left")
        self.closed_status_label = ttk.Label(loop_row, textvariable=self.closed_status, foreground=MUTED)
        self.closed_status_label.pack(side="left", padx=(10, 0))
        settings = ttk.Frame(right)
        settings.pack(fill="x", pady=8)
        ttk.Label(settings, text="方向键力度 %").grid(row=0, column=0)
        ttk.Entry(settings, textvariable=self.power, width=8).grid(row=0, column=1, padx=8)
        ttk.Label(settings, text="持续时间 秒").grid(row=0, column=2)
        ttk.Entry(settings, textvariable=self.duration, width=8).grid(row=0, column=3, padx=8)
        grid = ttk.Frame(right)
        grid.pack(anchor="w", pady=4)
        directions = [
            ("左前 ↖", 1, 1, 0), ("前进 ↑", 1, 0, 0), ("右前 ↗", 1, -1, 0),
            ("左移 ←", 0, 1, 0), ("停止", 0, 0, 0), ("右移 →", 0, -1, 0),
            ("左后 ↙", -1, 1, 0), ("后退 ↓", -1, 0, 0), ("右后 ↘", -1, -1, 0),
            ("逆时针 ↶", 0, 0, 1), ("顺时针 ↷", 0, 0, -1),
        ]
        for i, (label, x, y, z) in enumerate(directions):
            button = self._button(grid, label, lambda v=(x, y, z): self.direction(v), motion=bool(x or y or z), mixed=bool(x or y or z), width=112, tone="secondary")
            button.grid(row=i//3, column=i%3, padx=3, pady=4)
        custom = ttk.Frame(right)
        custom.pack(anchor="w", pady=10)
        for i, name in enumerate(("前进 x", "左移 y", "逆时针 z")):
            ttk.Label(custom, text=name).grid(row=0, column=i)
            ttk.Entry(custom, textvariable=self.axes[i], width=8).grid(row=1, column=i, padx=6)
        self._button(right, "按 x/y/z 合成运行", self.run_axes, motion=True, mixed=True, width=350).pack(anchor="w", pady=5)
        ttk.Checkbutton(right, text="场地已清空，车轮/电机已固定，可开始本次测试", variable=self.confirm, command=self.update_controls).pack(anchor="w", pady=10)
        ttk.Label(right, text="每次一个定时动作；不排队续跑。\n运行结束 / 手动停止后约 1 秒可开始下一次。", foreground=MUTED).pack(anchor="w")
        ttk.Label(right, textvariable=self.motion_hint, foreground=MUTED, wraplength=520).pack(anchor="w")

        ttk.Label(direct, text="四路独立输出 · 正数统一向前", font=("Microsoft YaHei UI", 12, "bold")).pack(anchor="w")
        ttk.Label(direct, text="A/B/C/D 都以正数表示朝车头，负数表示朝车尾，0 不输出。四路都填 +50 即为共同前进。范围 -100～100；时间共用上一页。", wraplength=850).pack(anchor="w", pady=10)
        fields = ttk.Frame(direct)
        fields.pack(anchor="w", pady=14)
        for i, channel in enumerate("ABCD"):
            ttk.Label(fields, text=f"{channel} 路 %").grid(row=0, column=i, padx=18)
            ttk.Entry(fields, textvariable=self.wheels[i], width=10).grid(row=1, column=i, padx=18, pady=8)
        ttk.Checkbutton(direct, text="场地已清空，车轮/电机已固定，可开始本次测试", variable=self.confirm, command=self.update_controls).pack(anchor="w", pady=12)
        self._button(direct, "按 A/B/C/D 同步运行", self.run_wheels, motion=True, width=350).pack(anchor="w", pady=8)
        ttk.Label(direct, text="逐路核对：只填一路非零、其余为 0。正数时该轮顶部应朝车头运动。\n若换线后不符，请先停止并重新标定驱动；不要在面板再次取反。", foreground=MUTED).pack(anchor="w", pady=18)

        ttk.Label(calib, text="轮速闭环标定 · PID 调参 · 四轮遥测", font=("Microsoft YaHei UI", 12, "bold")).pack(anchor="w")
        cal_row = ttk.Frame(calib)
        cal_row.pack(fill="x", pady=(10, 4))
        self.cal_button = self._button(cal_row, "开始标定", self.start_calibration, width=150)
        self.cal_button.pack(side="left")
        self.cal_reset_button = self._button(cal_row, "清除标定", self.reset_calibration, width=130, tone="secondary")
        self.cal_reset_button.pack(side="left", padx=10)
        ttk.Label(cal_row, textvariable=self.cal_progress, foreground=MUTED).pack(side="left", padx=12)
        ttk.Label(calib, text="标定前请确认四轮悬空、周围无障碍、电池电压正常；标定期间运动按钮全部禁用。", foreground=MUTED).pack(anchor="w")

        pid_row = ttk.Frame(calib)
        pid_row.pack(fill="x", pady=(12, 0))
        for i, (label, variable) in enumerate((("Kp", self.kp), ("Ki", self.ki), ("上限 %", self.limit))):
            ttk.Label(pid_row, text=label).grid(row=0, column=i, sticky="w", padx=(0, 18))
            ttk.Entry(pid_row, textvariable=variable, width=10).grid(row=1, column=i, sticky="w", padx=(0, 18), pady=4)
        pid_buttons = ttk.Frame(calib)
        pid_buttons.pack(fill="x", pady=4)
        self.pid_button = self._button(pid_buttons, "应用 PID", self.apply_pid, width=130)
        self.pid_button.pack(side="left")
        self.save_button = self._button(pid_buttons, "保存参数", self.save_params, width=130, tone="secondary")
        self.save_button.pack(side="left", padx=10)
        ttk.Label(pid_buttons, text="Kp/Ki 支持小数（如 2.500）；上限为 10～100 的整数百分比。", foreground=MUTED).pack(side="left", padx=10)

        ttk.Label(calib, text="四轮实时目标 / 实际 / 占空比", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w", pady=(14, 6))
        self.telemetry_table = ttk.Treeview(calib, columns=("wheel", "target", "actual", "duty"), show="headings", height=4)
        for column, label, width in (("wheel", "轮位", 200), ("target", "目标 counts/s", 170),
                                     ("actual", "实际 counts/s", 170), ("duty", "占空比 %", 140)):
            self.telemetry_table.heading(column, text=label)
            self.telemetry_table.column(column, width=width, anchor="center")
        self.telemetry_table.pack(fill="x")
        for channel in "ABCD":
            self.telemetry_table.insert("", "end", iid=channel, values=(WHEEL_LABELS[channel], "—", "—", "—"))
        ttk.Label(calib, text="遥测只在勾选“轮速闭环”或本页激活时开启；TLM 只刷新本表，不写入日志。", foreground=MUTED).pack(anchor="w", pady=8)

        self.encoder_table = ttk.Treeview(feedback, columns=("channel", "ticks", "invalid", "level"), show="headings", height=4)
        for column, label in (("channel", "通道"), ("ticks", "累计四倍频计数"), ("invalid", "无效跳变"), ("level", "A/B 电平")):
            self.encoder_table.heading(column, text=label)
            self.encoder_table.column(column, width=180, anchor="center")
        self.encoder_table.pack(fill="x")
        for channel in "ABCD": self.encoder_table.insert("", "end", iid=channel, values=(channel, "—", "—", "—"))
        ttk.Label(feedback, text="持续回传四路编码器；闭环目标/实际/占空比在“闭环调参”页实时显示。", foreground=MUTED).pack(anchor="w", pady=8)
        self.log_widget = tk.Text(feedback, height=12, state="disabled", bg="#162637", fg="#DDE9EF", wrap="word", font=("Consolas", 10))
        self.log_widget.pack(fill="both", expand=True)
        ttk.Label(instructions, text=(
            "蓝牙：在 Windows 中配对 PushRobot-ESP32；若要求 PIN，输入 271038。\n"
            "配对后在本页设备列表选择蓝牙的传出 COM 端口，点击连接；USB 与蓝牙使用同一面板。\n"
            "两种连接仅一端持有控制权；切换前先断开原连接，等待约 2 秒，再连接另一端。\n"
            "蓝牙断线/心跳超时会停车并撤销布局，重连后重新核对 X 型、应用轮位和现场确认。\n\n"
            "本版：FOUR_V1 接线，IN1/IN2 调 PWM，PWMA/B/C/D 固定接驱动板自身 5V。\n\n"
            "A 左后：IN2 G19 / IN1 G21 / E1A G22 / E1B G23\n"
            "B 右前：IN2 G4 / IN1 G16 / E2A G17 / E2B G18\n"
            "C 左前：IN2 G14 / IN1 G27 / E3A G26 / E3B G25\n"
            "D 右后：IN2 G33 / IN1 G32 / E4A G35 / E4B G34\nSTBY G13；ESP32 与驱动板共地。\n\n"
            "当前布局 C/B/A/D，四个方向统一 +1；通道电气方向修正已写入固件驱动。\n"
            "台架、四路独立输出、麦克纳姆混控全部使用正数向前；换线或换车后需重新标定驱动。\n"
            "只支持上述 X 型、45° 滚轮；不确定排列时不勾选 X 型，不运行合成动作。\n\n"
            "每个动作由 ESP32 自行计时；1.5 秒无有效 PING/STATUS 则关闭全部输出。\n"
            "Esc、断开、关窗、切换模式和窗口失焦均请求 STOP；实体急停仍应切断驱动电源。\n"
            "本版提供开环百分比混控；轮速闭环需先在“闭环调参”页完成四轮悬空标定。\n"
            "闭环目标是标定满量程百分比，不承诺毫米/秒速度、位置精度或四轮同步转速。"
        ), justify="left", wraplength=1000).pack(anchor="nw")
        self.update_controls()

    def log(self, text):
        self.log_lines.append(text)
        self.log_lines = self.log_lines[-800:]
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", time.strftime("%H:%M:%S ") + text + "\n")
        if int(self.log_widget.index("end-1c").split(".")[0]) > 800:
            self.log_widget.delete("1.0", "2.0")
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

    def refresh_ports(self):
        self.ports = {f"{p.device} — {p.description}": p.device for p in list_ports.comports()}
        self.port_box["values"] = list(self.ports)
        if self.port.get() not in self.ports:
            self.port.set(next((name for name in self.ports if "CP210" in name), next(iter(self.ports), "")))

    def toggle_connection(self):
        if self.client:
            self.disconnect()
            return
        port = self.ports.get(self.port.get())
        if not port:
            self.state.set("先选择 ESP32 的 USB 或蓝牙串口")
            return
        self.confirm.set(False)
        self.client = ChassisLink()
        self.client.connect(port)
        self.connect_button.configure(text="断开")
        self.state.set("正在确认 FOUR_V1 / MECANUM_V2，等待停止应答…")

    def disconnect(self):
        if self.client:
            self.client.send("STOP")
            self.send_telemetry_off()
            self.client.close()
            self.client = None
        self.telem_on = False
        self.telem_allowed = True
        self.closed_loop.set(False)
        self.fault_latched = False
        self.calibrating = False
        self.confirm.set(False)
        self.pending_run = self.device_running = False
        self._logged_fault_line = None
        self.clear_telemetry()
        self.cal_progress.set("未开始标定")
        self.closed_status.set("未连接 · 闭环不可用")
        self.connect_button.configure(text="连接")
        self.state.set("已断开 · 控制撤销")
        self.layout_status.set("未连接；连接后需重新应用布局")
        self.update_controls()

    def clear_telemetry(self):
        self.last_reply = None
        self.encoder_values.clear()
        for channel in "ABCD":
            self.encoder_table.item(channel, values=(channel, "—", "—", "—"))
            self.telemetry_table.item(channel, values=(WHEEL_LABELS[channel], "—", "—", "—"))

    def invalidate_layout(self, event=None):
        if self.client:
            self.client.send("STOP")
            self.client.layout_confirmed = False
            self.client.pending_layout = None
        self.confirm.set(False)
        self.layout_status.set("设置已变更，需重新应用并等待设备确认")
        self.update_controls()

    def apply_layout(self):
        if not self.client or not self.client.ready:
            self.state.set("请先连接设备并等待就绪")
            return
        if not self.x_layout.get():
            self.state.set("请先核对实物滚轮为图示 X 型")
            return
        try:
            if self.client.configure_layout("".join(v.get() for v in self.channels), [int(s.get()) for s in self.signs]):
                self.layout_status.set("布局已发送，等待设备回读确认…")
            else:
                self.state.set("请先停止并等待冷却后应用布局")
        except ValueError as exc:
            self.state.set(str(exc))
        self.update_controls()

    def direction(self, vector):
        if not any(vector):
            self.stop()
            return
        try:
            power = int(self.power.get())
            if not 1 <= power <= 100: raise ValueError("力度填写 1～100 的整数")
            if self.closed_loop.get() and power < 5:
                self.state.set("闭环最低力度 5%，请调高力度后再运行")
                return
            self.run_motion([power*v for v in vector], mixed=True)
        except ValueError as exc:
            self.state.set(str(exc))

    def run_axes(self):
        try: self.run_motion([int(v.get()) for v in self.axes], mixed=True)
        except ValueError as exc: self.state.set(str(exc))

    def run_wheels(self):
        try:
            values = [int(v.get()) for v in self.wheels]
            if self.closed_loop.get() and any(0 < abs(v) < 5 for v in values):
                self.state.set("闭环最低力度 5%，每路非零输出需 ≥5%")
                return
            self.run_motion(values, mixed=False)
        except ValueError as exc: self.state.set(str(exc))

    def run_motion(self, values, *, mixed):
        """核对现场状态后入队；是否真的启动由 pump 中的设备应答确认。"""
        if not self.client or not self.client.ready or not self.confirm.get():
            self.state.set("设备未就绪，或尚未确认现场条件")
            return
        if self.fault_latched:
            self.state.set("故障闭锁中，请先按“停止全部 / Esc”清除")
            return
        if self.device_running or self.pending_run or self.calibrating or time.monotonic() < self.cooldown_until:
            self.state.set("等待当前动作停止及冷却，不排队续跑")
            return
        closed = self.closed_loop.get()
        if closed and not (self.client.closed_loop_supported and self.client.cal_valid):
            self.state.set("轮速闭环未就绪，请先标定")
            self.update_controls()
            return
        try:
            milliseconds = duration_ms(self.duration.get(), "秒")
            send = self.client.speed if closed else self.client.motion
            if send(values, milliseconds, mixed=mixed):
                self.telem_allowed = True
                if any(values):
                    self.pending_run = True
                    self.pending_at = time.monotonic()
                    self.state.set("指令已发送，等待设备运行确认…")
                self.update_controls()
            else:
                self.state.set("设备忙或冷却中，请稍后再试")
        except ValueError as exc:
            self.state.set(str(exc))

    def closed_loop_available(self):
        return bool(self.client and self.client.ready and self.client.closed_loop_supported
                    and self.client.cal_valid and not self.calibrating and not self.fault_latched)

    def on_closed_loop_toggle(self):
        if self.closed_loop.get():
            self.telem_allowed = True
            if not (self.client and self.client.closed_loop_supported and self.client.cal_valid):
                self.closed_loop.set(False)
                if self.client and self.client.closed_loop_supported:
                    self.state.set("轮速闭环尚未标定，请先在“闭环调参”页标定")
                else:
                    self.state.set("当前固件不支持轮速闭环，请更新固件")
        self.update_controls()

    def on_tab_changed(self, event=None):
        # 进入/离开闭环调参页都会重新评估遥测开关。
        self.telem_allowed = True
        self.update_controls()

    def _ask(self, title, message):
        # 模态框打开期间窗口会短暂失焦；标记后跳过焦点丢失触发的 STOP。
        self.dialog_open = True
        try:
            return bool(messagebox.askyesno(title, message))
        finally:
            self.dialog_open = False

    def request_cal_refresh(self):
        # 旧固件没有 CHASSIS_CAPS2；CAL? 只会得到 ERROR，只在支持闭环时刷新。
        if self.client and self.client.ready and self.client.closed_loop_supported:
            return self.client.request_cal()
        return False

    def send_telemetry_off(self):
        if self.client and self.client.ready and self.client.closed_loop_supported:
            self.client.telem(False)
        self.telem_on = False

    def start_calibration(self):
        if not self.client or not self.client.ready:
            self.state.set("请先连接设备并等待就绪")
            return
        if not self.client.closed_loop_supported:
            self.state.set("当前固件不支持轮速闭环，请更新固件")
            return
        if self.fault_latched:
            self.state.set("故障闭锁中，请先按“停止全部 / Esc”清除")
            return
        if not (self.client.layout_confirmed and self.x_layout.get()):
            self.state.set("请先核对 X 型滚轮并应用布局，等待设备确认")
            return
        if not self.confirm.get():
            self.state.set("请先勾选“场地已清空…”现场确认")
            return
        if self.device_running or self.pending_run or self.calibrating:
            self.state.set("请先停止当前动作再开始标定")
            return
        if not self._ask("开始标定",
                         "请确认四轮悬空、周围无障碍、电池电压正常。\n标定期间车轮会转动，是否开始？"):
            self.state.set("已取消标定")
            return
        try:
            started = self.client.calibrate()
        except ValueError as exc:
            self.state.set(str(exc))
            return
        if started:
            self.calibrating = True
            self.cal_started_at = time.monotonic()
            self.cal_progress.set("CAL 已发送，等待设备开始…")
            self.state.set("标定中：四轮悬空，请勿触碰")
            self.telem_allowed = True
            self.log("开始轮速闭环标定")
        else:
            self.state.set("设备忙或冷却中，请稍后再试")
        self.update_controls()

    def reset_calibration(self):
        if not self.client or not self.client.ready:
            self.state.set("请先连接设备并等待就绪")
            return
        if not self.client.closed_loop_supported:
            self.state.set("当前固件不支持轮速闭环，请更新固件")
            return
        if self.fault_latched:
            self.state.set("故障闭锁中，请先按“停止全部 / Esc”清除")
            return
        if self.device_running or self.pending_run or self.calibrating:
            self.state.set("请先停止当前动作再清除标定")
            return
        if not self._ask("清除标定", "清除后必须重新标定才能使用轮速闭环，是否继续？"):
            self.state.set("已取消清除标定")
            return
        try:
            if self.client.cal_reset():
                self.state.set("已发送清除标定；等待设备确认")
                self.cal_progress.set("CAL RESET 已发送")
            else:
                self.state.set("设备忙，请稍后再试")
        except ValueError as exc:
            self.state.set(str(exc))

    def apply_pid(self):
        # 界面允许小数，线上协议用整数千分值；PID_OK 回读后再同步显示。
        if not self.client or not self.client.ready:
            self.state.set("请先连接设备并等待就绪")
            return
        if not self.client.closed_loop_supported:
            self.state.set("当前固件不支持轮速闭环，请更新固件")
            return
        try:
            kp = round(float(self.kp.get()) * 1000)
            ki = round(float(self.ki.get()) * 1000)
            limit = int(self.limit.get().strip())
        except (ValueError, OverflowError):
            self.state.set("Kp/Ki 填十进制小数，上限填 10～100 的整数百分比")
            return
        try:
            if self.client.set_pid(kp, ki, limit):
                self.state.set(f"PID 已发送：Kp={kp/1000:.3f} Ki={ki/1000:.3f} 上限={limit}%")
            else:
                self.state.set("设备忙或冷却中，请稍后再试")
        except ValueError as exc:
            self.state.set(str(exc))

    def save_params(self):
        if not self.client or not self.client.ready:
            self.state.set("请先连接设备并等待就绪")
            return
        if self.calibrating or self.device_running or self.pending_run:
            self.state.set("标定或运行中不能保存参数，请先停止")
            return
        try:
            if self.client.save():
                self.state.set("参数保存指令已发送")
            else:
                self.state.set("设备忙，请稍后再试")
        except ValueError as exc:
            self.state.set(str(exc))

    def telemetry_active(self):
        if not self.client or not self.client.closed_loop_supported:
            return False
        try:
            tab_active = self.notebook.select() == str(self.cal_tab)
        except tk.TclError:
            tab_active = False
        return bool(self.closed_loop.get() or tab_active)

    def sync_telemetry(self):
        # 停止/断开/失焦后 telem_allowed=False，必须先有新的用户动作才恢复遥测。
        want = bool(self.telem_allowed and self.client and self.client.ready and self.telemetry_active())
        if not (self.client and self.client.ready):
            self.telem_on = False
            return
        if want == self.telem_on:
            return
        try:
            if self.client.telem(want):
                self.telem_on = want
        except ValueError:
            self.telem_on = False

    def refresh_closed_status(self):
        if not self.client or not self.client.ready:
            self.closed_status.set("未连接 · 闭环不可用")
        elif not self.client.closed_loop_supported:
            self.closed_status.set("当前固件不支持轮速闭环")
        elif self.calibrating:
            self.closed_status.set("标定中…")
        elif self.client.cal_valid:
            self.closed_status.set(
                f"闭环已标定 · VMAX={self.client.cal_vmax:.1f} · LIMIT={self.client.cal_limit}%")
        else:
            self.closed_status.set("未标定，请先标定")

    def update_telemetry(self, values):
        """按 A/B/C/D 更新目标、实测、输出；显示斜坡目标，不推断车辆位移。"""
        if len(values) != 13:
            return
        _, rA, rB, rC, rD, sA, sB, sC, sD, uA, uB, uC, uD = values
        for channel, target, actual, duty in zip(
                "ABCD", (rA, rB, rC, rD), (sA, sB, sC, sD), (uA, uB, uC, uD)):
            self.telemetry_table.item(
                channel, values=(WHEEL_LABELS[channel], target, actual, f"{duty}%"))

    def stop(self):
        if self.client:
            # STOP 永远发送且优先入队；遥测与 CAL? 仅闭环固件支持时发送。
            self.client.send("STOP")
            self.send_telemetry_off()
            self.request_cal_refresh()
        self.telem_on = False
        self.telem_allowed = False
        self.confirm.set(False)
        self.pending_run = False
        self.calibrating = False
        self.fault_latched = False
        self.cooldown_until = time.monotonic() + 1.1
        self.state.set("已请求全部停止，等待设备确认")
        self.update_controls()

    def on_focus_out(self, event=None):
        # 在窗口控件间移动焦点不停止；真正切到其他窗口时撤销控制。
        def check():
            if self.closed or self.dialog_open:
                return
            if self.root.window.focus_displayof() is None:
                if (self.confirm.get() or self.device_running or self.pending_run or self.telem_on
                        or self.calibrating or self.fault_latched):
                    self.stop()
        self.root.after(60, check)

    def update_controls(self):
        """集中计算按钮门控；界面禁用不能代替固件对命令的最终检查。"""
        ready = bool(self.client and self.client.ready)
        active = (ready and self.confirm.get() and not self.device_running and not self.pending_run
                  and not self.calibrating and not self.fault_latched
                  and time.monotonic() >= self.cooldown_until)
        for button in self.motion_buttons:
            mixed_ready = button not in self.mixed_buttons or bool(self.client and self.client.layout_confirmed and self.x_layout.get())
            button.configure(state="normal" if active and mixed_ready else "disabled")
        closed_valid = bool(self.client and self.client.closed_loop_supported and self.client.cal_valid)
        if self.closed_loop.get() and not closed_valid:
            self.closed_loop.set(False)
        self.closed_loop_box.configure(
            state="normal" if self.closed_loop_available() and not self.device_running and not self.pending_run
            else "disabled")
        if hasattr(self, "cal_button"):
            cal_ready = (active and self.client.layout_confirmed and self.x_layout.get()
                         and self.client.closed_loop_supported)
            self.cal_button.configure(state="normal" if cal_ready else "disabled")
            self.cal_reset_button.configure(
                state="normal" if ready and self.client.closed_loop_supported and not self.device_running
                and not self.pending_run and not self.calibrating and not self.fault_latched else "disabled")
            self.pid_button.configure(
                state="normal" if ready and self.client.closed_loop_supported and not self.device_running
                and not self.pending_run and not self.calibrating else "disabled")
            # 保存会写 NVS；标定/运行期间固件也会拒绝，按钮提前禁用。
            self.save_button.configure(
                state="normal" if ready and self.client.closed_loop_supported
                and not self.calibrating and not self.device_running
                and not self.pending_run else "disabled")
        if hasattr(self, "apply_button"):
            self.apply_button.configure(state="normal" if active else "disabled")
        self.refresh_closed_status()
        if self.closed_loop.get():
            self.motion_hint.set("闭环目标是标定满量程的百分比，不是 mm/s；每路非零目标最低 5%。")
        else:
            self.motion_hint.set("百分比不是实际车速；勾选“轮速闭环”需先完成悬空标定。")
        self.sync_telemetry()

    def pump(self):
        """Tk 主线程消费串口事件，更新运行/标定/故障状态，然后重新安排轮询。

        rx 保留原始回报；telemetry 等结构事件用于表格。STOPPED ALL 只是静止
        状态，不能据此解除故障；闩锁的清除需以明确停止/复位等既有处理为准。
        """
        if self.closed: return
        if self.client:
            for kind, value in self.client.poll():
                if kind == "rx":
                    self.last_reply = time.monotonic()
                    # TLM 只进闭环表格，避免 20 Hz 遥测刷屏日志。
                    # FAULT 行每 0.7 s 随 STATUS 重复：只在内容变化时记录。
                    if value.startswith("FAULT "):
                        if value != self._logged_fault_line:
                            self._logged_fault_line = value
                            self.log("← " + value)
                    elif not value.startswith(("ENC ", "STOPPED ALL", "PONG ", "TLM ")):
                        self.log("← " + value)
                    if value.startswith(("RUNNING CHASSIS", "CHASSIS_RUN ", "SPEED_RUN ")):
                        self.device_running, self.pending_run = True, False
                        self.state.set("四轮运行中 · 定时自动停止")
                    elif value.startswith("STOPPED"):
                        was_active = self.device_running
                        self.device_running = False
                        if value == "STOPPED command":
                            # 只有设备确认收到操作员的 STOP 命令才解除故障闭锁；
                            # 空闲 STATUS 轮询回的 STOPPED ALL 不能解除。
                            self.fault_latched = False
                            self.calibrating = False
                        if value != "STOPPED ALL":
                            self.pending_run = False
                            self.cooldown_until = time.monotonic() + 1.1
                        elif was_active:
                            self.cooldown_until = time.monotonic() + 1.1
                        # 空闲轮询的 STOPPED ALL 不能覆盖用户可见的提示
                        # （5% 拒绝、标定失败、故障闭锁）；只有确实结束了一次
                        # 动作、或收到显式 STOP 确认时才回到“就绪”。
                        if not self.fault_latched and (was_active or value != "STOPPED ALL"):
                            self.state.set("已停止 · 就绪")
                    elif value.startswith("ERROR "):
                        # 与标定无关的 ERROR（如运动命令被拒）不能结束标定；
                        # 标定只由 CAL DONE/CAL FAIL/CAL RESET/STOP 收尾。
                        self.pending_run = False
                        if not self.calibrating:
                            self.state.set(value)
                        self.request_cal_refresh()
                    elif value == "CAL RESET":
                        self.calibrating = False
                        self.cal_progress.set("CAL RESET")
                    elif value.startswith("CAL START"):
                        self.calibrating = True
                        self.cal_started_at = time.monotonic()
                    match = re.fullmatch(r"ENC ([A-D]) ticks=(-?\d+) invalid=(\d+) A=([01]) B=([01])", value)
                    if match:
                        self.encoder_values[match[1]] = int(match[2])
                        self.encoder_table.item(match[1], values=(match[1], match[2], match[3], f"{match[4]} / {match[5]}"))
                elif kind == "tx": self.log("→ " + value)
                elif kind == "ready": self.state.set("已停止 · 就绪；请核对并应用轮位")
                elif kind == "layout": self.layout_status.set("设备已确认轮位与方向" if value else "设备布局与本次设置不一致，需重新应用")
                elif kind == "reset":
                    self.clear_telemetry()
                    self.confirm.set(False)
                    self.closed_loop.set(False)
                    self.device_running = self.pending_run = False
                    self.calibrating = self.fault_latched = False
                    self._logged_fault_line = None
                    self.cal_progress.set("未开始标定")
                    self.state.set(value)
                    self.layout_status.set("开发板重启；需重新应用布局")
                elif kind == "error":
                    self.confirm.set(False)
                    self.state.set(value)
                    self.log(value)
                elif kind == "closed":
                    self.clear_telemetry()
                    self.confirm.set(False)
                    self.closed_loop.set(False)
                    self.device_running = self.pending_run = False
                    self.calibrating = self.fault_latched = False
                    self.telem_on = False
                    self._logged_fault_line = None
                    self.closed_status.set("未连接 · 闭环不可用")
                    self.connect_button.configure(text="断开并重连")
                elif kind == "capabilities2":
                    self.log("设备支持轮速闭环；已自动读取标定与 PID 参数")
                elif kind == "cal_status":
                    self.refresh_closed_status()
                elif kind == "cal_reset":
                    self.calibrating = False
                    self.cal_progress.set("CAL RESET")
                elif kind == "cal_progress":
                    self.calibrating = True
                    self.cal_started_at = time.monotonic()
                    self.cal_progress.set("CAL PROGRESS " + str(value))
                    self.log("CAL PROGRESS " + str(value))
                elif kind == "cal_done":
                    self.calibrating = False
                    self.cal_progress.set(
                        f"CAL DONE VALID={1 if value.get('valid') else 0} VMAX={value.get('vmax', 0.0):.1f}")
                    self.state.set("标定完成，正在刷新每轮参数…")
                    self.log("标定完成；正在刷新每轮 CALW 与 Vmax")
                    self.request_cal_refresh()
                elif kind == "cal_fail":
                    self.calibrating = False
                    self.cal_progress.set("CAL FAIL " + str(value))
                    self.state.set("标定失败：" + str(value) + "；请排查后重试")
                    self.log("标定失败：" + str(value))
                elif kind == "pid":
                    self.kp.set(f"{value['kp']:.3f}")
                    self.ki.set(f"{value['ki']:.3f}")
                    self.limit.set(str(value["limit"]))
                    self.request_cal_refresh()
                elif kind == "telemetry":
                    self.update_telemetry(value)
                elif kind == "fault":
                    self.fault_latched = True
                    self.state.set("故障闭锁：" + str(value) + "；请按“停止全部 / Esc”清除")
        if self.calibrating and time.monotonic() - self.cal_started_at > CAL_TIMEOUT_SECONDS:
            self.log("标定 20 秒无进度，已请求停止")
            self.stop()
            self.state.set("标定超时，已请求停止；请检查日志")
        if self.pending_run and time.monotonic()-self.pending_at > RUN_ACK_TIMEOUT_SECONDS:
            self.stop()
            self.state.set("未收到运行确认，已请求停止；请检查日志")
        self.reply_age.set(f"最近应答：{time.monotonic()-self.last_reply:.1f} 秒前" if self.last_reply is not None else "最近应答：—")
        self.update_controls()
        self.root.after(UI_POLL_INTERVAL_MS, self.pump)

    def shutdown(self):
        if self.closed: return
        self.closed = True
        if self.client:
            self.client.send("STOP")
            self.send_telemetry_off()
            self.client.close()
        self.telem_on = False
