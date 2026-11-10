"""ESP32 底盘串口链路：按 MECANUM_V2 协议握手、下发有界脉冲、维持心跳。

只负责传输与安全，不做视觉或闭环；控制律在 nav_servo.py。
协议来源：hardware/workbench/source/motor-bringup/src/main.cpp
- 上电默认 DRY_RUN，必须显式 MODE CHASSIS；
- 每次连接都要重新 LAYOUT（固件不写 Flash）；
- MECANUM <forward> <left> <ccw> <ms> 是定时开环动作，运行中拒绝新指令；
- 停止后约 1 s 才接受下一个动作；1.5 s 无 PING/STATUS 固件自行停车。

pyserial 只在真正打开串口时导入，离线测试可以注入假串口。
"""
from __future__ import annotations

import threading
import time
from collections import deque

DEFAULT_LAYOUT = "ADBC 1 1 1 1"
CHASSIS_CAPS = "CHASSIS_CAPS API=MECANUM_V2 MIX=X UNITS=PERCENT FORWARD=UNIFIED"
MAX_PULSE_MS = 60_000


class ChassisError(RuntimeError):
    """链路无法确认身份、失去应答或握手超时。"""


class ChassisLink:
    """单串口、单动作的底盘链路；pulse() 只入队，实际发送在后台线程。"""

    def __init__(self, port, *, layout=DEFAULT_LAYOUT, baudrate=115200,
                 serial_factory=None, heartbeat_s=0.7, cooldown_s=1.05,
                 connect_timeout_s=8.0, reply_timeout_s=2.5, clock=time.monotonic,
                 strict=False):
        self.port = port
        self.layout = layout
        self.baudrate = baudrate
        self.heartbeat_s = heartbeat_s
        self.cooldown_s = cooldown_s
        self.connect_timeout_s = connect_timeout_s
        self.reply_timeout_s = reply_timeout_s
        self.clock = clock
        self.strict = strict
        self._factory = serial_factory
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._device = None
        # 握手阶段：CONNECT -> MODE -> CAPS -> LAYOUT -> STATUS -> READY
        self._phase = "CONNECT"
        self._phase_sent_at = 0.0
        self._ready = False
        self._running = False
        self._pending = None          # 待发命令文本
        self._send_by = None
        self._pulse_ms = 0
        self._pulse_sent_at = 0.0
        self._next_ok = 0.0           # 下一次允许 MECANUM 的时刻
        self._last_reply = 0.0
        self._last_heartbeat = 0.0
        self._error = None
        self._last_stop = None
        self._enc = {}
        self.events = deque(maxlen=200)

    # ---------- 对外接口 ----------

    @property
    def ready(self):
        with self._lock:
            return self._ready

    @property
    def busy(self):
        """上一次脉冲还没到点，或还在冷却。"""
        with self._lock:
            return self._running or self.clock() < self._next_ok or self._pending is not None

    @property
    def error(self):
        with self._lock:
            return self._error

    @property
    def encoders(self):
        with self._lock:
            return dict(self._enc)

    def start(self):
        if self._thread and self._thread.is_alive():
            raise ChassisError("链路已经启动")
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="chassis-link", daemon=True)
        self._thread.start()

    def wait_ready(self, timeout_s=8.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.ready:
                return True
            if self.error:
                raise ChassisError(self.error)
            time.sleep(0.02)
        raise ChassisError(f"{timeout_s:.0f} s 内未完成 MODE/LAYOUT 握手")

    def pulse(self, forward, left, ccw, ms, *, send_by=None):
        """排入一个 MECANUM 脉冲；不满足安全条件时返回 False，不排队。"""
        for name, value in (("forward", forward), ("left", left), ("ccw", ccw)):
            if type(value) is not int or not -100 <= value <= 100:
                raise ValueError(f"{name} 必须是 -100..100 的整数")
        if type(ms) is not int or not 1 <= ms <= MAX_PULSE_MS:
            raise ValueError(f"ms 必须是 1..{MAX_PULSE_MS} 的整数")
        with self._lock:
            now = self.clock()
            if not self._ready or self._pending is not None or self._running:
                return False
            if now < self._next_ok or (send_by is not None and now >= send_by):
                return False
            self._pending = f"MECANUM {forward} {left} {ccw} {ms}"
            self._send_by = send_by
            self._pulse_ms = ms
            self._pulse_sent_at = 0.0
            # 先按“已开始”占位，避免同一帧重复入队；收到 STOPPED 后解除。
            self._running = True
            self._next_ok = now + ms / 1000.0 + self.cooldown_s
            return True

    def stop(self):
        """取消待发动作并立即请求停车；停车后仍要等冷却。"""
        with self._lock:
            self._pending = "STOP"
            self._send_by = None
            self._pulse_ms = 0
            self._running = False
            self._next_ok = self.clock() + self.cooldown_s

    def close(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3)
        with self._lock:
            self._ready = False
            self._pending = None
            device = self._device
            self._device = None
        if device is not None:
            try:
                device.write(b"STOP\n")
            except Exception:            # noqa: BLE001 - 关链路时不再抛出
                pass
            try:
                device.close()
            except Exception:            # noqa: BLE001
                pass

    # ---------- 后台线程 ----------

    def _open(self):
        timeout = 0.015 if self.strict else 0.05
        if self._factory is not None:
            return self._factory(port=self.port, baudrate=self.baudrate,
                                 timeout=timeout, write_timeout=0.2)
        import serial                     # 仅真实串口需要 pyserial

        return serial.Serial(port=self.port, baudrate=self.baudrate,
                             timeout=timeout, write_timeout=0.2)

    def _run(self):
        try:
            device = self._open()
            with self._lock:
                self._device = device
                self._last_reply = self.clock()
            self._write("WHO")
            with self._lock:
                self._phase = "CONNECT"
                self._phase_sent_at = self.clock()
            while not self._stop_event.is_set():
                now = self.clock()
                try:
                    line = device.readline()
                except Exception as exc:  # noqa: BLE001 - 串口被拔/驱动异常
                    raise ChassisError(f"串口读取失败：{exc}") from exc
                if line:
                    self._handle_line(line.decode("utf-8", "replace").strip(), now)
                self._service(now)
                if line:
                    continue
                time.sleep(0.005)
        except Exception as exc:          # noqa: BLE001 - 交给 UI 统一处理
            with self._lock:
                self._error = str(exc)
                self._ready = False
                self._pending = None
        finally:
            with self._lock:
                self._ready = False
                self._running = False
                device = self._device
                self._device = None
            if device is not None:
                try:
                    device.write(b"STOP\n")
                except Exception:         # noqa: BLE001
                    pass
                try:
                    device.close()
                except Exception:         # noqa: BLE001
                    pass

    def _write(self, command):
        with self._lock:
            device = self._device
        if device is None:
            raise ChassisError("串口未打开")
        self._write_line(device, command)

    @staticmethod
    def _write_line(device, command):
        data = (command + "\n").encode("ascii")
        if device.write(data) != len(data):
            raise ChassisError("串口写入不完整")

    def _service(self, now):
        with self._lock:
            phase, running, pulse_ms = self._phase, self._running, self._pulse_ms
            last_reply, last_beat = self._last_reply, self._last_heartbeat
            sent_at = self._phase_sent_at
            pulse_sent_at = self._pulse_sent_at

        if self._error:
            return
        if phase != "READY":
            if now - sent_at > self.connect_timeout_s:
                raise ChassisError(f"握手卡在 {phase}")
            return
        if now - last_reply > self.reply_timeout_s:
            raise ChassisError("设备超过 2.5 s 无应答，已放弃控制")
        # 取出、校验时效和写出必须与 stop() 串行，防止覆盖新入队的 STOP。
        with self._lock:
            pending = self._pending
            if pending is not None:
                now = self.clock()
                self._pending = None
                if self._send_by is not None and now >= self._send_by:
                    self._running = False
                    self._next_ok = now
                    return
                if pending.startswith("MECANUM"):
                    self._pulse_sent_at = now
                    self._next_ok = now + self._pulse_ms / 1000.0 + self.cooldown_s
                self._write_line(self._device, pending)
                self._last_heartbeat = now
                return
        # 脉冲到点后固件自己停；这里兜底解除占位，避免应答丢失时永久 busy。
        if running and pulse_sent_at and now - pulse_sent_at > pulse_ms / 1000.0 + 0.8:
            if self.strict:
                raise ChassisError("未收到定时停车回执")
            with self._lock:
                self._running = False
        if now - last_beat >= self.heartbeat_s:
            self._write("STATUS")
            with self._lock:
                self._last_heartbeat = now

    def _advance(self, phase, command=None):
        with self._lock:
            self._phase = phase
            self._phase_sent_at = self.clock()
        if command:
            self._write(command)

    def _handle_line(self, line, now):
        if not line:
            return
        self.events.append(line)
        with self._lock:
            self._last_reply = now
        phase = self._phase
        if line.startswith("UNIFIED_ROBOT"):
            if phase == "CONNECT":
                self._advance("MODE", "MODE CHASSIS")
        elif line == "MODE_OK CHASSIS STOPPED":
            if phase == "MODE":
                self._advance("CAPS", "CAPS")
        elif line == CHASSIS_CAPS:
            if phase == "CAPS":
                self._advance("LAYOUT", f"LAYOUT {self.layout}")
        elif line.startswith("LAYOUT CONFIGURED=1"):
            if phase == "LAYOUT":
                order, *signs = self.layout.split()
                if not {f"ORDER={order}", f"SIGNS={','.join(signs)}"} <= set(line.split()):
                    raise ChassisError(f"设备轮位回读不符：{line}")
                self._advance("STATUS", "STATUS")
        elif line == "STOPPED ALL":
            if not self.strict:
                with self._lock:
                    self._running = False
            if phase == "STATUS":
                with self._lock:
                    if self.strict:
                        self._next_ok = now + self.cooldown_s
                    self._ready = True
                self._advance("READY")
                self._write("PING")
        elif line.startswith("CHASSIS_RUN"):
            with self._lock:
                self._running = True
        elif line.startswith("STOPPED"):
            if self.strict and line not in ("STOPPED timer", "STOPPED command"):
                raise ChassisError(line)
            with self._lock:
                self._running = False
                self._last_stop = line
                self._next_ok = max(self._next_ok, now + self.cooldown_s)
        elif line.startswith(("ERROR", "FAULT")):
            if self.strict:
                raise ChassisError(line)
            with self._lock:
                self._running = False
        elif line.startswith("ENC "):
            self._parse_encoder(line)


    def _parse_encoder(self, line):
        # ENC A ticks=123 invalid=0 A=1 B=0
        parts = line.split()
        if len(parts) < 3 or len(parts[1]) != 1:
            return
        channel = parts[1]
        try:
            ticks = int(parts[2].split("=", 1)[1])
        except (IndexError, ValueError):
            return
        with self._lock:
            self._enc[channel] = ticks


def list_ports():
    """返回可用串口名；没有 pyserial 时返回空列表。"""
    try:
        from serial.tools import list_ports as _list
    except ImportError:
        return []
    return [p.device for p in _list.comports()]
