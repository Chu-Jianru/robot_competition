"""ESP32 底盘串口链路：按 MECANUM_V2 协议握手、下发有界脉冲、维持心跳。

只负责传输与安全，不做视觉或闭环；控制律在 nav_servo.py。
协议来源：hardware/firmware/WHEEL_SPEED_V1/motor-bringup/src/main.cpp
- 上电默认 DRY_RUN，必须显式 MODE CHASSIS；
- 每次连接都要重新 LAYOUT（固件不写 Flash）；
- MECANUM <forward> <left> <ccw> <ms> 是定时开环动作，运行中拒绝新指令；
- 支持 CHASSIS_HOLD 的固件可用 MECANUM_HOLD 在同一方向上连续续期；
- 支持 CHASSIS_HOLD_RELEASE 的固件对正常松键只保留短暂停机间隔；其它动作约 1 s。
- 支持 CHASSIS_MANUAL 的固件直接更新键盘目标，零目标停车，MECANUM_KEEP 只续期。
- CHASSIS_STREAM 支持自主任务混控，短租期由带时限的新目标续期。
- 1.5 s 无 PING/STATUS 固件自行停车。

pyserial 只在真正打开串口时导入，离线测试可以注入假串口。
"""
from __future__ import annotations

import threading
import time
from collections import deque

DEFAULT_LAYOUT = "ADBC 1 1 1 1"
CHASSIS_CAPS = "CHASSIS_CAPS API=MECANUM_V2 MIX=X UNITS=PERCENT FORWARD=UNIFIED"
HOLD_CAPS = "CHASSIS_HOLD LEASE_MS=250"
HOLD_RELEASE_CAPS = "CHASSIS_HOLD_RELEASE GUARD_MS=60"
HOLD_RELEASE_GUARD_S = 0.060
STOP_COMMANDS = ("STOP", "STOP HOLD")
MANUAL_CAPS = "CHASSIS_MANUAL LEASE_MS=250"
MANUAL_PREFIX = "MECANUM_MANUAL "
MANUAL_KEEP = "MECANUM_KEEP"
STREAM_CAPS = "CHASSIS_STREAM LEASE_MS=150"
STREAM_PREFIX = "MECANUM_STREAM "
SPEED_STREAM_CAPS = "CHASSIS_SPEED_STREAM LEASE_MS=150"
SPEED_STREAM_PREFIX = "SPEED_STREAM "
STREAM_MAX_MS = 150
STREAM_MIN_MS = 60
STREAM_SEND_MARGIN_S = .04
MAX_PULSE_MS = 60_000


class ChassisError(RuntimeError):
    """链路无法确认身份、失去应答或握手超时。"""


class ChassisLink:
    """单串口、单动作的底盘链路；pulse() 只入队，实际发送在后台线程。"""

    def __init__(self, port, *, layout=DEFAULT_LAYOUT, baudrate=115200,
                 serial_factory=None, heartbeat_s=0.7, cooldown_s=1.05,
                 connect_timeout_s=8.0, reply_timeout_s=2.5, clock=time.monotonic,
                 strict=False, recover_cooldown=False, require_stream=False,
                 require_speed_stream=False, stream_pipeline=False):
        self.port = port
        self.layout = layout
        self.baudrate = baudrate
        self.heartbeat_s = heartbeat_s
        self.cooldown_s = cooldown_s
        self.connect_timeout_s = connect_timeout_s
        self.reply_timeout_s = reply_timeout_s
        self.clock = clock
        self.strict = strict
        self._recover_cooldown = recover_cooldown
        self._require_stream = require_stream or require_speed_stream
        self._require_speed_stream = require_speed_stream
        self._stream_pipeline = stream_pipeline
        self._stream_prefix = SPEED_STREAM_PREFIX if require_speed_stream else STREAM_PREFIX
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
        self._holding = False
        self._hold_awaiting_reply = False
        self._supports_hold = False
        self._supports_hold_release = False
        self._supports_manual = False
        self._manual_active = False
        self._supports_stream = False
        self._supports_speed_stream = False
        self._speed_info = None
        self._encoder_sample = None
        self._stream_reply_by = None
        self._stream_written = None
        self._stream_confirmed = None
        self._pending = None          # 待发命令文本
        self._send_by = None
        self._pulse_ms = 0
        self._pulse_sent_at = 0.0
        self._stop_sent_at = None
        self._stop_command = None
        self._next_ok = 0.0           # 下一次允许 MECANUM 的时刻
        self._hold_restart_at = None  # 仅适用于正常松键后的 MECANUM_HOLD
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
    def supports_hold(self):
        with self._lock:
            return self._supports_hold

    @property
    def supports_manual(self):
        with self._lock:
            return self._supports_manual

    @property
    def manual_active(self):
        with self._lock:
            return self._manual_active

    @property
    def stream_outputs(self):
        """Targets that may still be applied while an update is in transit."""
        with self._lock:
            return tuple(dict.fromkeys(command for command in
                                       (self._stream_confirmed, self._stream_written) if command))

    @property
    def speed_info(self):
        with self._lock:
            return self._speed_info

    def encoder_snapshot(self, timeout_s=1.5):
        """Read all four counters in one firmware snapshot after STOP is acknowledged."""
        deadline = time.monotonic() + timeout_s
        requested = False
        while time.monotonic() < deadline:
            with self._lock:
                if self._error or not self._ready:
                    raise ChassisError(self._error or "底盘链路未就绪")
                if requested and self._encoder_sample is not None:
                    return self._encoder_sample
                if (not requested and not self._running and self._pending is None
                        and self._stop_sent_at is None):
                    self._encoder_sample = None
                    self._pending, self._send_by = "ENC_SNAPSHOT", None
                    requested = True
            time.sleep(.01)
        raise ChassisError("停车或四路编码器快照应答超时")

    @property
    def holding(self):
        with self._lock:
            return self._holding

    @property
    def supports_hold_release(self):
        with self._lock:
            return self._supports_hold_release

    @property
    def hold_restarting(self):
        """A normal release is awaiting its reply or short guard."""
        with self._lock:
            if "STOP" in (self._pending, self._stop_command):
                return False
            return (self._pending == "STOP HOLD" or self._stop_command == "STOP HOLD"
                    or (self._hold_restart_at is not None and self.clock() < self._hold_restart_at))

    @property
    def hold_busy(self):
        with self._lock:
            return self._busy_until(self._hold_ready_at())

    @property
    def busy(self):
        """上一次脉冲还没到点，或还在冷却。"""
        with self._lock:
            return self._busy_until(self._next_ok)

    def _busy_until(self, ready_at):
        return (self._running or self.clock() < ready_at
                or self._pending is not None or self._stop_sent_at is not None)

    def _hold_ready_at(self):
        return self._next_ok if self._hold_restart_at is None else self._hold_restart_at

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

    @staticmethod
    def _validate_axes(forward, left, ccw):
        for name, value in (("forward", forward), ("left", left), ("ccw", ccw)):
            if type(value) is not int or not -100 <= value <= 100:
                raise ValueError(f"{name} 必须是 -100..100 的整数")

    def pulse(self, forward, left, ccw, ms, *, send_by=None):
        """排入一个 MECANUM 脉冲；不满足安全条件时返回 False，不排队。"""
        self._validate_axes(forward, left, ccw)
        if type(ms) is not int or not 1 <= ms <= MAX_PULSE_MS:
            raise ValueError(f"ms 必须是 1..{MAX_PULSE_MS} 的整数")
        with self._lock:
            now = self.clock()
            if (not self._ready or self._pending is not None or self._running
                    or self._stop_sent_at is not None):
                return False
            if now < self._next_ok or (send_by is not None and now >= send_by):
                return False
            self._pending = f"MECANUM {forward} {left} {ccw} {ms}"
            self._hold_restart_at = None
            self._send_by = send_by
            self._pulse_ms = ms
            self._pulse_sent_at = 0.0
            # 先按“已开始”占位，避免同一帧重复入队；收到 STOPPED 后解除。
            self._running = True
            self._next_ok = now + ms / 1000.0 + self.cooldown_s
            return True

    def hold(self, forward, left, ccw, *, renew=False, send_by=None):
        """Start or renew a firmware lease. A renewal never starts a stopped motion."""
        self._validate_axes(forward, left, ccw)
        if not any((forward, left, ccw)):
            raise ValueError("全零请调用 stop()")
        with self._lock:
            now = self.clock()
            if (not self._ready or not self._supports_hold or self._pending is not None
                    or self._stop_sent_at is not None or self._hold_awaiting_reply
                    or (send_by is not None and now >= send_by)):
                return False
            if not self._holding and (renew or self._running or now < self._hold_ready_at()):
                return False
            self._pending = f"MECANUM_HOLD {forward} {left} {ccw}"
            self._hold_restart_at = None
            self._send_by = now + .15 if send_by is None else min(now + .15, send_by)
            self._pulse_ms = 0
            self._pulse_sent_at = 0.0
            self._running = self._holding = True
            return True

    def manual(self, forward, left, ccw):
        """Replace the pending keyboard target. Zero cancels motion immediately."""
        self._validate_axes(forward, left, ccw)
        axes = (forward, left, ccw)
        if sum(value != 0 for value in axes) > 1:
            raise ValueError("键盘遥控每次只能使用一个轴")
        with self._lock:
            if not self._ready or not self._supports_manual or self._stop_sent_at is not None:
                return False
            if self._running and not self._manual_active:
                return False
            if self._pending is not None and not self._pending.startswith((MANUAL_PREFIX, MANUAL_KEEP)):
                return False
            now = self.clock()
            self._pending = f"{MANUAL_PREFIX}{forward} {left} {ccw}"
            self._running = self._manual_active = any(axes)
            self._send_by = now + .15 if self._manual_active else None
            return True

    def renew_manual(self):
        with self._lock:
            if self._ready and self._manual_active and self._pending is None:
                self._pending = MANUAL_KEEP
                self._send_by = self.clock() + .15

    def stream(self, forward, left, ccw, *, valid_until):
        """Replace an unsent target and cap its firmware lease at the scene deadline."""
        axes = (forward, left, ccw)
        if any(type(value) is not int for value in axes):
            raise ValueError("连续混控目标必须为整数")
        with self._lock:
            if self._require_speed_stream:
                if self._speed_info is None:
                    return False
                limit = self._speed_info[0]
            else:
                limit = 40
            if not 0 < sum(map(abs, axes)) <= limit:
                raise ValueError(f"连续混控目标必须非零，三轴绝对值之和不得超过 {limit}")
            send_by = valid_until - STREAM_SEND_MARGIN_S
            if (not self._ready or not (self._supports_stream or self._supports_speed_stream)
                    or self._stop_sent_at is not None
                    or (self._running and (self._pulse_ms or self._holding or self._manual_active))
                    or (self._pending is not None and not self._pending.startswith(self._stream_prefix))
                    or send_by - self.clock() < STREAM_MIN_MS / 1000):
                return False
            self._pending = f"{self._stream_prefix}{forward} {left} {ccw}"
            self._send_by = send_by
            return True

    def stop(self, *, release=False):
        """Cancel pending motion and request an immediate stop."""
        with self._lock:
            if "STOP" in (self._pending, self._stop_command):
                return
            if release and "STOP HOLD" in (self._pending, self._stop_command):
                return
            command = "STOP HOLD" if release and self._holding and self._supports_hold_release else "STOP"
            self._queue_stop(self.clock(), command)

    def _queue_stop(self, now, command="STOP"):
        # Caller holds _lock.
        self._pending = command
        self._send_by = None
        self._pulse_ms = 0
        self._pulse_sent_at = 0.0
        self._running = self._holding = self._hold_awaiting_reply = False
        self._manual_active = False
        self._next_ok = now + self.cooldown_s
        self._hold_restart_at = None

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
            reply = bytearray()
            while not self._stop_event.is_set():
                try:
                    line = device.readline()
                except Exception as exc:  # noqa: BLE001 - 串口被拔/驱动异常
                    raise ChassisError(f"串口读取失败：{exc}") from exc
                now = self.clock()
                if line:
                    # A serial read timeout can split a reply before the newline.
                    reply.extend(line)
                    if line.endswith(b"\n"):
                        self._handle_line(reply.decode("utf-8", "replace").strip(), now)
                        reply.clear()
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
                self._holding = False
                self._hold_awaiting_reply = False
                self._manual_active = False
                self._stream_reply_by = None
                self._stream_written = self._stream_confirmed = None
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
            now = self.clock()
            if (self._stream_reply_by is not None and now >= self._stream_reply_by
                    and not self._stream_pipeline):
                raise ChassisError("未在连续驱动租期内收到应答，已放弃控制")
            if pending is not None:
                if pending.startswith(self._stream_prefix):
                    if self._stream_reply_by is not None and not self._stream_pipeline:
                        return  # One unacknowledged target at most. Replace waiting targets in stream().
                    self._pending = None
                    lease_ms = min(STREAM_MAX_MS, int(1000 * (self._send_by - now)))
                    if lease_ms < STREAM_MIN_MS:
                        if self._running:
                            self._queue_stop(now)
                        return
                    self._write_line(self._device, f"{pending} {lease_ms}")
                    self._running = True
                    self._stream_written = tuple(map(int, pending.split()[1:]))
                    self._stream_reply_by = now + lease_ms / 1000
                    self._last_heartbeat = now
                    return
                self._pending = None
                if self._send_by is not None and now >= self._send_by:
                    if self._manual_active:
                        self._running = self._manual_active = False
                        self._pending = f"{MANUAL_PREFIX}0 0 0"
                        self._send_by = None
                    elif self._holding:
                        self._queue_stop(now)
                    else:
                        self._running = False
                        self._next_ok = now
                    return
                if pending.startswith("MECANUM "):
                    self._pulse_sent_at = now
                    self._next_ok = now + self._pulse_ms / 1000.0 + self.cooldown_s
                elif pending.startswith("MECANUM_HOLD "):
                    self._hold_awaiting_reply = True
                elif pending == f"{MANUAL_PREFIX}0 0 0":
                    self._next_ok = now + self.cooldown_s  # Only gates legacy motion.
                elif pending in STOP_COMMANDS and (self.strict or self._supports_hold_release):
                    self._stop_sent_at = now
                    self._stop_command = pending
                self._write_line(self._device, pending)
                if not self._require_speed_stream:
                    self._last_heartbeat = now
                    return
                # STOP and ENC_SNAPSHOT do not renew the firmware's owner
                # lease. Send a due heartbeat after the queued command.
            if (self._stop_sent_at is not None
                    and now - self._stop_sent_at > self.reply_timeout_s):
                raise ChassisError("未收到停车回执")
        # 脉冲到点后固件自己停；这里兜底解除占位，避免应答丢失时永久 busy。
        if running and pulse_sent_at and now - pulse_sent_at > pulse_ms / 1000.0 + 0.8:
            if self.strict:
                raise ChassisError("未收到定时停车回执")
            with self._lock:
                self._running = False
        if now - last_beat >= self.heartbeat_s:
            self._write("PING" if self._require_speed_stream else "STATUS")
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
                if self._require_speed_stream and not self._supports_speed_stream:
                    raise ChassisError("请烧录支持 CHASSIS_SPEED_STREAM 的底盘固件")
                if self._require_stream and not self._require_speed_stream and not self._supports_stream:
                    raise ChassisError("底盘固件不支持连续自主驱动，请更新到支持 CHASSIS_STREAM 的版本")
                self._advance("LAYOUT", f"LAYOUT {self.layout}")
        elif phase == "CAPS" and line == HOLD_CAPS:
            with self._lock:
                self._supports_hold = True
        elif phase == "CAPS" and line == HOLD_RELEASE_CAPS:
            with self._lock:
                self._supports_hold_release = True
        elif phase == "CAPS" and line == MANUAL_CAPS:
            with self._lock:
                self._supports_manual = True
        elif phase == "CAPS" and line == STREAM_CAPS:
            with self._lock:
                self._supports_stream = True
        elif phase == "CAPS" and line == SPEED_STREAM_CAPS:
            with self._lock:
                self._supports_speed_stream = True
        elif line.startswith("LAYOUT CONFIGURED=1"):
            if phase == "LAYOUT":
                order, *signs = self.layout.split()
                if not {f"ORDER={order}", f"SIGNS={','.join(signs)}"} <= set(line.split()):
                    raise ChassisError(f"设备轮位回读不符：{line}")
                if self._require_speed_stream:
                    self._advance("SPEED_INFO", "SPEED_INFO")
                else:
                    self._advance("STATUS", "STATUS")
        elif phase == "SPEED_INFO" and line.startswith("SPEED_INFO "):
            limit, *signs = map(int, line.split()[1:])
            if not 0 < limit <= 1_000_000 or len(signs) != 4 or any(abs(sign) != 1 for sign in signs):
                raise ChassisError("ESP32 缺少有效的轮速标定，请先架空四轮标定并保存")
            with self._lock:
                self._speed_info = limit, tuple(signs)
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
                with self._lock:
                    self._last_heartbeat = now
        elif line.startswith("CHASSIS_RUN") or line == "HOLD_OK":
            with self._lock:
                self._hold_awaiting_reply = False
        elif line == "STREAM_OK":
            with self._lock:
                if (self._stream_reply_by is not None and now >= self._stream_reply_by
                        and not self._stream_pipeline):
                    raise ChassisError("连续驱动应答超时，已放弃控制")
                if self._stream_written is not None:
                    self._stream_confirmed = self._stream_written
                    self._stream_written = None
                self._stream_reply_by = None
        elif line.startswith("STOPPED"):
            release_ack = self._supports_hold_release and line == "STOPPED hold_release"
            if self.strict and not release_ack and line not in ("STOPPED timer", "STOPPED command"):
                raise ChassisError(line)
            with self._lock:
                released = release_ack and self._stop_command == "STOP HOLD"
                if (self._require_stream and line == "STOPPED command" and self._stop_sent_at is None
                        and (self._running or (self._pending or "").startswith(self._stream_prefix))):
                    raise ChassisError("底盘收到外部停车，本轮已结束")
                if released or line == "STOPPED command":
                    self._stop_sent_at = None
                    self._stop_command = None
                    self._stream_reply_by = None
                    self._stream_written = self._stream_confirmed = None
                self._hold_restart_at = (now + HOLD_RELEASE_GUARD_S
                                         if released and self._pending != "STOP" else None)
                if ((self._holding or self._manual_active) and self._pending is not None
                        and self._pending not in STOP_COMMANDS):
                    self._pending = None
                    self._send_by = None
                self._running = False
                self._holding = self._hold_awaiting_reply = False
                self._manual_active = False
                self._last_stop = line
                self._next_ok = max(self._next_ok, now + self.cooldown_s)
        elif line == "FAULT none":
            # A healthy STATUS reply reports this after the encoder lines.
            return
        elif phase == "READY" and self._recover_cooldown and line in (
                "ERROR wait 1 second after stop", "ERROR hold release guard"):
            with self._lock:
                if self._pending not in STOP_COMMANDS:
                    self._pending = None
                    self._send_by = None
                self._running = self._holding = self._hold_awaiting_reply = False
                self._next_ok = max(self._next_ok, now + self.cooldown_s)
                self._hold_restart_at = (now + HOLD_RELEASE_GUARD_S
                                         if line == "ERROR hold release guard" else None)
        elif line.startswith(("ERROR", "FAULT")):
            if self.strict:
                raise ChassisError(line)
            with self._lock:
                self._running = False
        elif line.startswith("ENC "):
            self._parse_encoder(line)
        elif line.startswith("ENC_SNAPSHOT "):
            counts = tuple(map(int, line.split()[1:]))
            if len(counts) != 4:
                raise ChassisError("四路编码器快照格式错误")
            with self._lock:
                self._encoder_sample = counts


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
