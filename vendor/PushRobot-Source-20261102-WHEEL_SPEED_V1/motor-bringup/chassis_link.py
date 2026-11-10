"""底盘协议适配：把面板操作入队，把设备应答转换成界面事件。

串口读写、握手、0.7 s STATUS 心跳由 MotorLink 后台线程处理；
handle_response 也在该线程运行，只发事件，不能访问 Tk 控件。
面板在 ChassisApp.pump 中消费事件；排查时按 tx -> rx -> 解析事件 -> UI 跟踪。
所有四轮协议字段按 A/B/C/D 排列，mixed=True 的输入才是 x/y/z。
"""
import re
import time

from motor_link import MotorLink, MAX_DURATION_MS

LAYOUT_ACK = re.compile(r"LAYOUT CONFIGURED=1 ORDER=([ABCD]{4}) SIGNS=(1),(1),(1),(1) TYPE=X")
CAL_STATUS = re.compile(r"CAL VALID=([01]) VMAX=(-?\d+(?:\.\d+)?) LIMIT=(\d+)")
CAL_WHEEL = re.compile(
    r"CALW ([A-D]) SIGN=(-?\d+) V100=(-?\d+(?:\.\d+)?)"
    r" DEADBAND=(-?\d+(?:\.\d+)?) SLOPE=(-?\d+(?:\.\d+)?)")
CAL_DONE = re.compile(r"CAL DONE VALID=([01]) VMAX=(-?\d+(?:\.\d+)?)")
PID_OK = re.compile(
    r"PID_OK KP=(-?\d+(?:\.\d+)?) KI=(-?\d+(?:\.\d+)?) LIMIT=(\d+)")
# TLM: ESP32 毫秒 + 目标 A..D + 实测 A..D + 输出 A..D。
# 目标/实测为 counts/s，输出为占空比 %；目标是斜坡后的值，字段均为整数。
TELEMETRY_FIELDS = 13


class ChassisLink(MotorLink):
    """能力由 CAPS2 确认；支持闭环与标定有效是两个独立条件。"""

    def __init__(self, **kwargs):
        super().__init__(mode="CHASSIS", require_profile="FOUR_V1", **kwargs)
        self.layout_confirmed = False
        self.pending_layout = None
        self._reset_closed_loop()

    def _reset_closed_loop(self):
        """重连/重启/模式切换后清掉旧能力与参数，等待设备重新回读。"""
        self.closed_loop_supported = False
        self.cal_valid = False
        self.cal_vmax = 0.0
        self.cal_limit = 0
        self.cal_wheels = {}
        self.pid_values = None
        self._last_fault = None

    def connect(self, port):
        self.layout_confirmed = False
        self.pending_layout = None
        # Reset before starting the worker: otherwise its first CAPS reply could
        # race with this assignment and lose the new capabilities.
        self._reset_closed_loop()
        super().connect(port)

    def configure_layout(self, order, signs):
        if not isinstance(order, str) or sorted(order) != list("ABCD") or len(signs) != 4 or any(type(s) is not int or s != 1 for s in signs):
            raise ValueError("轮位必须为四个不同通道；MECANUM_V2 已统一前进方向，四个方向均保持 +1")
        with self._lock:
            if not self._ready or time.monotonic() < self._next_jog:
                return False
            self.layout_confirmed = False
            self.pending_layout = (order, tuple(signs))
            self._commands.append("LAYOUT " + order + " " + " ".join(map(str, signs)))
            return True

    def _cal_status(self):
        return {"valid": self.cal_valid, "vmax": self.cal_vmax, "limit": self.cal_limit}

    def handle_response(self, line):
        """解析设备原始行；未知行仍由基类作为 rx 事件保留，便于排查协议差异。"""
        if line.startswith(("UNIFIED_BOOT ", "MODE_OK ")):
            self.layout_confirmed = False
            self.pending_layout = None
            self._reset_closed_loop()
        match = LAYOUT_ACK.fullmatch(line)
        if match:
            received = (match[1], tuple(int(match[i]) for i in range(2, 6)))
            with self._lock:
                self.layout_confirmed = received == self.pending_layout
            self.events.put(("layout", self.layout_confirmed))
            return
        if line.startswith("CHASSIS_CAPS2 "):
            self.closed_loop_supported = "CLOSED=1" in line.split()
            if self.closed_loop_supported:
                self.events.put(("capabilities2", True))
                # The worker only dequeues once ready; queueing here is safe and
                # keeps the first status read right after the handshake.
                with self._lock:
                    for command in ("CAL?", "PID?"):
                        if command not in self._commands:
                            self._commands.append(command)
            return
        if line.startswith("CALW "):
            match = CAL_WHEEL.fullmatch(line)
            if match:
                channel, sign, v100, deadband, slope = match.groups()
                # Firmware names the line-fit parameters DEADBAND/SLOPE; expose
                # them under the client-side a/b names from the calibration model.
                self.cal_wheels[channel] = {
                    "sign": int(sign),
                    "v100": float(v100),
                    "a": float(deadband),
                    "b": float(slope),
                }
            return
        if line == "CAL START":
            # The firmware invalidates the stored calibration before it prints
            # CAL START; closed-loop motion must stay disabled for the whole run.
            self.cal_valid = False
            self.events.put(("cal_status", self._cal_status()))
            self.events.put(("cal_progress", "start"))
            return
        if line.startswith("CAL PROGRESS "):
            self.events.put(("cal_progress", line[len("CAL PROGRESS "):]))
            return
        if line.startswith("CAL DONE "):
            match = CAL_DONE.fullmatch(line)
            if match:
                self.cal_valid = match[1] == "1"
                self.cal_vmax = float(match[2])
                self.events.put(("cal_done", {"valid": self.cal_valid, "vmax": self.cal_vmax}))
            return
        if line.startswith("CAL FAIL "):
            self.cal_valid = False
            self.events.put(("cal_fail", line[len("CAL FAIL "):]))
            return
        if line == "CAL RESET":
            self.cal_valid = False
            self.events.put(("cal_reset", None))
            self.events.put(("cal_status", self._cal_status()))
            return
        if line.startswith("CAL "):
            match = CAL_STATUS.fullmatch(line)
            if match:
                self.cal_valid = match[1] == "1"
                self.cal_vmax = float(match[2])
                self.cal_limit = int(match[3])
                self.events.put(("cal_status", self._cal_status()))
            return
        if line.startswith("PID_OK "):
            match = PID_OK.fullmatch(line)
            if match:
                self.pid_values = {
                    "kp": float(match[1]),
                    "ki": float(match[2]),
                    "limit": int(match[3]),
                }
                self.events.put(("pid", dict(self.pid_values)))
            return
        if line.startswith("TLM "):
            parts = line.split()
            if len(parts) != TELEMETRY_FIELDS + 1:
                return
            try:
                values = tuple(int(part) for part in parts[1:])
            except ValueError:
                return
            self.events.put(("telemetry", values))
            return
        if line.startswith("FAULT "):
            reason = line[len("FAULT "):].strip()
            if reason.startswith("last="):
                reason = reason[len("last="):]
            if reason == "none":
                self._last_fault = None
            elif reason and reason != self._last_fault:
                # STATUS repeats "FAULT last=" every 0.7 s; only surface a new fault.
                self._last_fault = reason
                self.events.put(("fault", reason))

    def motion(self, values, milliseconds, *, mixed=True):
        """开环：values 是 x/y/z 分量或 A/B/C/D 输出占空比，范围 ±100。"""
        values = tuple(values)
        if len(values) != (3 if mixed else 4) or any(type(v) is not int or not -100 <= v <= 100 for v in values):
            raise ValueError("每个输出分量必须为 -100～100 的整数")
        if type(milliseconds) is not int or not 1 <= milliseconds <= MAX_DURATION_MS:
            raise ValueError("持续时间必须为 1～2147483647 毫秒")
        if not any(values):
            return self.send("STOP")
        with self._lock:
            now = time.monotonic()
            if not self._ready or now < self._next_jog:
                return False
            if mixed and not self.layout_confirmed:
                raise ValueError("先确认 X 型滚轮与四轮位置，并应用布局，等待设备确认")
            if any(cmd.startswith(("WHEELS ", "MECANUM ")) for cmd in self._commands):
                return False
            self._commands.append(("MECANUM " if mixed else "WHEELS ") + " ".join(map(str, (*values, milliseconds))))
            self._next_jog = now + milliseconds / 1000 + 1.3
            return True

    def speed(self, values, milliseconds, *, mixed=True):
        """闭环：values 是 Vmax 的百分比，不是占空比；混控用 SPEED，直控用 SPEEDW。

        返回 True 只表示已入队，实际启动必须等固件 SPEED_RUN 应答。
        混控后的每轮 5% 下限由固件最终校验；不会把不支持的小目标自动抬高。
        """
        values = tuple(values)
        if len(values) != (3 if mixed else 4) or any(type(v) is not int or not -100 <= v <= 100 for v in values):
            raise ValueError("每个输出分量必须为 -100～100 的整数")
        if type(milliseconds) is not int or not 1 <= milliseconds <= MAX_DURATION_MS:
            raise ValueError("持续时间必须为 1～2147483647 毫秒")
        # All-zero is a stop request, exactly like the firmware's handling; it
        # stays available even when the calibration is missing.
        if not any(values):
            return self.send("STOP")
        if not self.closed_loop_supported:
            raise ValueError("当前固件不支持轮速闭环，请更新固件")
        if not self.cal_valid:
            raise ValueError("轮速闭环尚未标定，请先执行标定")
        with self._lock:
            now = time.monotonic()
            if not self._ready or now < self._next_jog:
                return False
            if mixed and not self.layout_confirmed:
                raise ValueError("先确认 X 型滚轮与四轮位置，并应用布局，等待设备确认")
            if any(cmd.startswith(("SPEED ", "SPEEDW ")) for cmd in self._commands):
                return False
            self._commands.append(("SPEED " if mixed else "SPEEDW ") + " ".join(map(str, (*values, milliseconds))))
            self._next_jog = now + milliseconds / 1000 + 1.3
            return True

    def jog(self, *args, **kwargs):
        raise ValueError("逐路 JOG 请切换到电机台架；底盘使用四路输出或麦克纳姆控制")

    def calibrate(self):
        """请求会真实转轮的 CAL；现场架空确认由界面负责，固件检查状态与布局。"""
        if not self.closed_loop_supported:
            raise ValueError("当前固件不支持轮速闭环，请更新固件")
        with self._lock:
            now = time.monotonic()
            if not self._ready or now < self._next_jog:
                return False
            self._commands.append("CAL")
            self._next_jog = now + 1.1
            return True

    def cal_reset(self):
        return self._enqueue("CAL RESET")

    def request_cal(self):
        return self.send_line("CAL?")

    def set_pid(self, kp_milli, ki_milli, limit):
        """协议使用整数千分值：2500 表示 2.500；limit 是目标速度量程百分比。"""
        if type(kp_milli) is not int or not 0 <= kp_milli <= 10000:
            raise ValueError("Kp 必须为 0～10000 的整数（千分之一）")
        if type(ki_milli) is not int or not 0 <= ki_milli <= 10000:
            raise ValueError("Ki 必须为 0～10000 的整数（千分之一）")
        if type(limit) is not int or not 10 <= limit <= 100:
            raise ValueError("速度上限必须为 10～100 的整数")
        return self._enqueue(f"PID {kp_milli} {ki_milli} {limit}")

    def request_pid(self):
        return self.send_line("PID?")

    def save(self):
        return self.send_line("SAVE")

    def telem(self, on):
        if type(on) is not bool:
            raise ValueError("遥测开关必须为 True 或 False")
        return self._enqueue("TELEM 1" if on else "TELEM 0")

    def close(self):
        # Best effort shutdown: telemetry off first, then STOP, before the
        # worker is allowed to exit and its finally-block re-sends STOP.
        # Old firmware has no TELEM command; sending it would only add noise.
        with self._lock:
            ready = self._ready
            self._commands.clear()
            if ready:
                commands = ["STOP"]
                if self.closed_loop_supported:
                    commands.insert(0, "TELEM 0")
                self._commands.extend(commands)
        if ready and self._thread is not None and self._thread.is_alive():
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                with self._lock:
                    if not self._commands:
                        break
                time.sleep(0.005)
        super().close()

    def send_line(self, text):
        # 限制为查询、SAVE 和 STOP；底盘原始协议实验见 DEBUGGING.md 与验收清单。
        # 运动与标定必须走专用方法，不能借任意文本绕过界面校验。
        if text not in ("STOP", "STATUS", "PING", "ENC", "CAPS", "IO", "LAYOUT?", "POLARITY",
                        "PID?", "CAL?", "SAVE"):
            raise ValueError("底盘界面仅允许查询；运动请使用专用按钮")
        return self._enqueue(text)
