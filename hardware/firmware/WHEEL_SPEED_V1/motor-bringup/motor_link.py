"""台架串口后台线程：确认固件与接线版本，串行发送查询/运行/停止指令。"""
from __future__ import annotations

from collections import deque
import queue
import re
import threading
import time

import serial

MAX_DURATION_MS = 2_147_483_647
CHASSIS_CAPABILITY = "CHASSIS_CAPS API=MECANUM_V2 MIX=X UNITS=PERCENT FORWARD=UNIFIED"


class MotorLink:
    """主线程只入队/取事件，后台线程独占串口，STOP 会优先清空待发队列。

    ready 表示身份、模式、引脚与停止状态已确认，不代表布局或标定有效。
    _next_jog 是客户端限流时间；动作定时、故障与失联停车仍以 ESP32 为准。
    """
    def __init__(self, serial_factory=serial.Serial, *, require_profile=None, mode="MOTOR_A_TEST"):
        if mode not in ("MOTOR_A_TEST", "CHASSIS"):
            raise ValueError("不支持的真实控制模式")
        self.mode = mode
        self.factory = serial_factory
        self.require_profile = require_profile
        self.supported_channels = ("A",)
        self.profile = None
        self.events = queue.Queue()
        self._lock = threading.Lock()
        self._commands = deque()
        self._exit = threading.Event()
        self._thread = None
        self._ready = False
        self._next_jog = 0.0

    @property
    def ready(self):
        with self._lock:
            return self._ready

    def connect(self, port):
        if self._thread and self._thread.is_alive():
            raise RuntimeError("请先断开当前连接")
        self._exit.clear()
        self.supported_channels = ("A",)
        self.profile = None
        with self._lock:
            self._ready = False
            self._commands.clear()
            self._next_jog = 0.0
        self._thread = threading.Thread(target=self._run, args=(port,), daemon=True)
        self._thread.start()

    def send(self, command):
        if command not in ("STOP", "STATUS", "ENC", "ENCZERO", "PING", "CAPS"):
            raise ValueError("不支持的查询命令")
        return self._enqueue(command)

    def send_line(self, text):
        """Send one edited serial line through the same connection as buttons."""
        if not isinstance(text, str) or not text or len(text) > 63:
            raise ValueError("每次输入一条命令，长度不超过 63 个字符")
        if any(ord(char) < 32 or ord(char) > 126 for char in text):
            raise ValueError("请输入英文命令；不要粘贴多行或控制字符")
        parts = text.strip().split()
        if not parts:
            raise ValueError("请输入命令")
        parts[0] = parts[0].upper()
        command = " ".join(parts)
        if parts[0] == "MODE":
            raise ValueError("请使用顶部模式切换；串口输入框不直接切换固件模式")
        if parts[0] == "JOG":
            match = re.fullmatch(r"JOG (?:([A-Da-d]) )?([+-]?[0-9]{1,3}) ([0-9]{1,10})", command)
            if not match:
                raise ValueError("格式：JOG B 50 1000（B 路，50%，1000 毫秒）")
            return self.jog(int(match[2]), int(match[3]), channel=(match[1] or "A").upper())
        if parts[0] in ("ENC", "ENCZERO") and len(parts) == 2:
            channel = parts[1].upper()
            if channel not in self.supported_channels and not (parts[0] == "ENCZERO" and channel == "ALL" and len(self.supported_channels) == 4):
                raise ValueError("当前固件不支持该通道，请先更新四路固件")
            return self._enqueue(f"{parts[0]} {channel}")
        if parts[0] in ("STOP", "STATUS", "ENC", "ENCZERO", "PING") and len(parts) != 1:
            raise ValueError("这条命令不需要参数")
        return self._enqueue(command)

    def _enqueue(self, command):
        # STOP 优先并丢弃未发送的运行指令，防止点击停止后又排队启动。
        with self._lock:
            if not self._ready:
                return False
            if command == "STOP":
                self._commands.clear()
                self._commands.appendleft(command)
                self._next_jog = time.monotonic() + 1.1
            elif command not in self._commands:
                self._commands.append(command)
            return True

    def jog(self, duty, milliseconds=300, channel="A"):
        # 客户端也做边界和冷却检查；最终计时及断连停止仍由 ESP32 自己执行。
        if (type(duty) is not int or not 1 <= abs(duty) <= 100 or
                type(milliseconds) is not int or not 1 <= milliseconds <= MAX_DURATION_MS):
            raise ValueError("输出范围为 ±1～100%；时间填正整数毫秒（最大 2147483647）")
        if channel not in self.supported_channels:
            raise ValueError("当前固件不支持该电机通道，请先更新四路固件")
        with self._lock:
            now = time.monotonic()
            if not self._ready or now < self._next_jog:
                return False
            # Never queue up motion. Each button click is one bounded pulse.
            if any(cmd.startswith("JOG ") for cmd in self._commands):
                return False
            prefix = f"JOG {channel}" if len(self.supported_channels) == 4 else "JOG"
            self._commands.append(f"{prefix} {duty} {milliseconds}")
            self._next_jog = now + milliseconds / 1000 + 1.3
            return True

    def close(self):
        with self._lock:
            self._ready = False
            self._commands.clear()
        self._exit.set()

    def poll(self):
        items = []
        for _ in range(100):
            try:
                items.append(self.events.get_nowait())
            except queue.Empty:
                break
        return items

    def handle_response(self, line):
        """子类可解析附加状态；调用位置仍在串口线程，不能直接更新 Tk 控件。"""
        pass

    def _run(self, port):
        # WHO/MODE/PONG 只确认协议；CAPS 的 FOUR_V1 才确认真实 GPIO 接线版本。
        # 排查“连上但未就绪”：依次看 WHO -> MODE_OK -> PONG -> CAPS -> STOPPED。
        # 自动 STATUS 同时保活固件；TLM 只提供观测，不能代替控制端心跳。
        device = None
        identified = False
        selecting_mode = False
        unified = False
        selected_mode = False
        profile_confirmed = not self.require_profile
        feature_confirmed = self.mode != "CHASSIS"
        try:
            device = self.factory(port=port, baudrate=115200, timeout=0.05, write_timeout=0.2)
            opened = time.monotonic()
            last_probe = last_query = 0.0
            last_reply = opened
            pending = bytearray()

            def write(command, automatic=False):
                device.write((command + "\n").encode("ascii"))
                self.events.put(("tx_auto" if automatic else "tx", command))

            while not self._exit.is_set():
                now = time.monotonic()
                if not identified:
                    if now - opened > 6:
                        raise RuntimeError("未确认当前接线对应的四路固件，请烧录 motor-bringup 最新版。")
                    if now - last_probe >= 0.4:
                        write("WHO", automatic=True)
                        write("PING", automatic=True)
                        last_probe = now
                elif now - last_reply > 2.5:
                    raise RuntimeError("设备应答超时，已撤销窗口控制；请检查连接后重连。")
                if identified and not self.ready and now - opened > 6:
                    if self.mode == "CHASSIS" and not feature_confirmed:
                        raise RuntimeError("底盘固件需更新为 MECANUM_V2（驱动统一前进方向），请更新面板并重新烧录。")
                    raise RuntimeError("固件引脚版本不匹配，需烧录 FOUR_V1 四路固件后再测试。")

                with self._lock:
                    command = self._commands.popleft() if self._commands and self._ready else None
                    # Serialize this short write with STOP/close so an unsent
                    # pulse cannot slip out after the queue has been cleared.
                    if command and not self._exit.is_set():
                        write(command)
                if identified and now - last_query > 0.7:
                    write("STATUS", automatic=True)
                    last_query = now

                incoming = device.read(min(device.in_waiting or 1, 2048))
                for byte in incoming:
                    if byte != 10:
                        if len(pending) < 4096:
                            pending.append(byte)
                        else:
                            pending.clear()
                        continue
                    line = pending.decode("utf-8", errors="replace").strip()
                    pending.clear()
                    if not line:
                        continue
                    # Preserve all device output, including boot text, unknown
                    # command errors and custom replies, for the terminal.
                    self.events.put(("rx", line))
                    self.handle_response(line)
                    if line.startswith("ERROR control_owned_by_other_port"):
                        raise RuntimeError("机器人正在由另一个 USB/蓝牙连接控制，请先断开另一面板，再重新连接。")
                    if line.startswith("UNIFIED_BOOT "):
                        identified = selecting_mode = selected_mode = False
                        profile_confirmed = not self.require_profile
                        feature_confirmed = self.mode != "CHASSIS"
                        self.supported_channels, self.profile = ("A",), None
                        opened = now
                        with self._lock:
                            self._ready = False
                            self._commands.clear()
                        self.events.put(("reset", "统一固件已启动；默认模拟模式，正在重新确认台架模式"))
                    elif line == "CAPS MOTORS=ABCD PINS=FOUR_V1":
                        self.supported_channels, self.profile = tuple("ABCD"), "FOUR_V1"
                        profile_confirmed = self.require_profile in (None, "FOUR_V1")
                        self.events.put(("capabilities", tuple("ABCD")))
                    elif line == CHASSIS_CAPABILITY:
                        feature_confirmed = True
                    elif line.startswith("UNIFIED_ROBOT 1 mode="):
                        unified = True
                        if not selected_mode and not selecting_mode:
                            selecting_mode = True
                            write(f"MODE {self.mode}")
                    elif line == f"MODE_OK {self.mode} STOPPED":
                        selected_mode = True
                        selecting_mode = False
                        opened = now
                        write("PING")
                    elif line == f"PONG {self.mode}" and not selecting_mode and (not unified or selected_mode):
                        if not identified:
                            identified = True
                            last_reply = now
                            write("STOP")
                            write("CAPS", automatic=True)
                            write("STATUS")
                    elif identified and line.startswith(self.mode + ":"):
                        # An online reset cancels all pending motion; re-identify.
                        identified = False
                        profile_confirmed = not self.require_profile
                        feature_confirmed = self.mode != "CHASSIS"
                        self.supported_channels, self.profile = ("A",), None
                        opened = now
                        with self._lock:
                            self._commands.clear()
                            self._ready = False
                        self.events.put(("reset", "开发板已重启，正在重新确认停止状态"))
                    elif identified and line in ("STOPPED A", "STOPPED ALL") and not selecting_mode and profile_confirmed and feature_confirmed:
                        # 仅在固件身份、模式、引脚版本、停止状态全部确认后开放操作。
                        with self._lock:
                            first_ready = not self._ready
                            self._ready = True
                        if first_ready:
                            self.events.put(("ready", port))
                    elif identified and line in ("STOPPED command", "STOPPED timer", "STOPPED link_timeout"):
                        with self._lock:
                            if self._ready:
                                self._next_jog = time.monotonic() + 1.1
                    if identified and line.startswith((f"PONG {self.mode}", "STOPPED", "RUNNING", "ENC ", "JOG A ", "CHASSIS_RUN ", "LAYOUT ", "ERROR ")):
                        last_reply = time.monotonic()
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            with self._lock:
                self._ready = False
                self._commands.clear()
            if device is not None:
                try:
                    if identified:
                        device.write(b"STOP\n")
                except Exception:
                    pass
                try:
                    device.close()
                except Exception:
                    pass
            self.events.put(("closed", None))
