"""PC-side serial client for the DRY_RUN link (protocol v1).

Design rules implemented here:
- Periodic DRIVE commands are coalesced: only the newest target is kept.
- STOP has the highest priority and clears pending motion/commands.
- The port is not considered usable until a valid protocol identity is seen.
- On identity the client sends STOP, then STATUS, and requires DISARMED before
  it exposes `can_arm()`.
- A boot_id change is treated as an ESP32 reboot: motion/session are cleared
  and a fresh sync is required. Nothing is ever auto-armed.
"""

from __future__ import annotations

import json
import queue
import threading
import time
from collections import deque

import serial

PROTOCOL_VERSION = 1
MODE_DRY_RUN = "DRY_RUN"
WATCHDOG_MS = 300
MAX_LINEAR_MM_S = 300
MAX_ANGULAR_DEG_S = 180
MAX_SERIAL_LINE = 4096


class ConnectionState:
    DISCONNECTED = "DISCONNECTED"
    WAITING_IDENTITY = "WAITING_IDENTITY"
    SYNCING = "SYNCING"
    READY = "READY"
    ERROR = "ERROR"


class LineFramer:
    """Incrementally split bytes into text lines.

    Handles CRLF, arbitrary fragmentation/coalescing, and discards overlong
    boot-noise lines so the caller only sees complete, sane lines.
    """

    def __init__(self, max_len: int = MAX_SERIAL_LINE) -> None:
        self.max_len = max_len
        self._buffer = bytearray()
        self._discarding = False
        self.dropped_long = 0

    def reset(self) -> None:
        self._buffer.clear()
        self._discarding = False

    def feed(self, data: bytes) -> list[str]:
        lines: list[str] = []
        for byte in data:
            if byte == 0x0A:
                if self._discarding:
                    self._discarding = False
                    self._buffer.clear()
                    continue
                raw = bytes(self._buffer)
                self._buffer.clear()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").replace("\r", "")
                if text:
                    lines.append(text)
                continue
            if self._discarding:
                continue
            if byte == 0x0D:
                continue
            self._buffer.append(byte)
            if len(self._buffer) > self.max_len:
                self._buffer.clear()
                self._discarding = True
                self.dropped_long += 1
        return lines


def _int_or_none(value: object) -> int | None:
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    return None


class LinkClient:
    """Single-use serial link + safety-aware command pipeline.

    The panel creates one client per connection. All public methods are safe
    to call from the Tkinter thread. Serial I/O runs on two daemon threads.
    """

    def __init__(
        self,
        serial_factory=None,
        *,
        identity_grace_s: float = 1.5,
        probe_interval_s: float = 0.3,
        identity_timeout_s: float = 5.0,
        sync_retry_s: float = 0.6,
        sync_timeout_s: float = 3.0,
        select_unified_mode: bool = False,
    ) -> None:
        self._serial_factory = serial_factory or serial.Serial
        self.identity_grace_s = identity_grace_s
        self.probe_interval_s = probe_interval_s
        self.identity_timeout_s = identity_timeout_s
        self.sync_retry_s = sync_retry_s
        self.sync_timeout_s = sync_timeout_s
        self.select_unified_mode = select_unified_mode
        self._unified_phase = "discover" if select_unified_mode else "legacy"

        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._events: queue.Queue = queue.Queue()
        self._framer = LineFramer()

        self._running = False
        self._serial = None
        self._reader = None
        self._writer = None
        self._disconnect_reported = False

        self._connection = ConnectionState.DISCONNECTED
        self._boot_id: int | None = None
        self._arm_token: int | None = None
        self._fw: str | None = None
        self._mode: str | None = None
        self._protocol: int | None = None
        self._esp_state = "UNKNOWN"
        self._session: int | None = None
        self._last_seq = 0
        self._next_seq = 1
        self._last_status: dict = {}

        self._opened_at = 0.0
        self._last_probe = 0.0
        self._identity_timeout_reported = False
        self._sync_started = 0.0
        self._last_sync_probe = 0.0
        self._sync_timeout_reported = False

        self._stop_requested = False
        self._stop_command = "STOP"
        self._controls: deque = deque()
        self._latest_motion: tuple[int, int, int] | None = None
        self._motion_dirty = False
        self._status_pending = False

        # Local control generation. A late ARM/DRIVE ack from a previous
        # ARM/STOP cycle must not revive motion after STOP or reconnect.
        self._control_epoch = 0
        self._pending_arm_epoch: int | None = None
        self._arm_pending = False
        self._require_disarmed = False
        self._stop_requested_at = 0.0
        self._last_disarm_at = 0.0
        self._arm_confirmed_at = 0.0

    # ------------------------------------------------------------------ public

    def connect(self, port: str, baud: int = 115200) -> None:
        with self._lock:
            if self._running:
                raise RuntimeError("already connected")
        serial_port = self._serial_factory(
            port=port, baudrate=baud, timeout=0.1, write_timeout=0.2
        )
        with self._lock:
            self._serial = serial_port
            self._running = True
            self._disconnect_reported = False
            self._connection = ConnectionState.WAITING_IDENTITY
            self._framer = LineFramer()
            self._events = queue.Queue()
            self._boot_id = None
            self._arm_token = None
            self._fw = None
            self._mode = None
            self._protocol = None
            self._esp_state = "UNKNOWN"
            self._session = None
            self._last_seq = 0
            self._next_seq = 1
            self._last_status = {}
            self._opened_at = time.monotonic()
            self._unified_phase = "discover" if self.select_unified_mode else "legacy"
            self._last_probe = 0.0
            self._identity_timeout_reported = False
            self._sync_started = 0.0
            self._last_sync_probe = 0.0
            self._sync_timeout_reported = False
            self._stop_requested = False
            self._stop_command = "STOP"
            self._controls.clear()
            self._latest_motion = None
            self._motion_dirty = False
            self._status_pending = False
            self._control_epoch += 1
            self._pending_arm_epoch = None
            self._arm_pending = False
            self._require_disarmed = False
            self._stop_requested_at = 0.0
            self._last_disarm_at = 0.0
            self._arm_confirmed_at = 0.0
            self._reader = threading.Thread(
                target=self._reader_loop, name="link-reader", daemon=True
            )
            self._writer = threading.Thread(
                target=self._writer_loop, name="link-writer", daemon=True
            )
            self._reader.start()
            self._writer.start()
        self._emit("state_change", self.snapshot())

    def close(self, send_stop: bool = True, timeout: float = 0.5) -> None:
        with self._lock:
            if self._serial is None and not self._running:
                return
            if send_stop and self._connection in (
                ConnectionState.SYNCING,
                ConnectionState.READY,
            ):
                self._stop_requested = True
                self._stop_command = "STOP"
                self._controls.clear()
                self._latest_motion = None
                self._motion_dirty = False
                self._session = None
                self._condition.notify_all()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if not self._stop_requested:
                    break
            time.sleep(0.01)
        with self._lock:
            self._running = False
            self._condition.notify_all()
            reader = self._reader
            writer = self._writer
        current = threading.current_thread()
        for thread in (writer, reader):
            if thread is not None and thread.is_alive() and thread is not current:
                thread.join(timeout=0.5)
        with self._lock:
            serial_port = self._serial
            self._serial = None
            self._connection = ConnectionState.DISCONNECTED
            self._session = None
            self._esp_state = "UNKNOWN"
            self._status_pending = False
            self._controls.clear()
            self._stop_requested = False
            self._latest_motion = None
            self._motion_dirty = False
            self._control_epoch += 1
            self._pending_arm_epoch = None
            self._arm_pending = False
            self._require_disarmed = False
            self._arm_confirmed_at = 0.0
            self._last_status = {}
            self._last_seq = 0
            self._next_seq = 1
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        self._emit("state_change", self.snapshot())

    def poll(self) -> list[tuple[str, object]]:
        items: list[tuple[str, object]] = []
        while True:
            try:
                items.append(self._events.get_nowait())
            except queue.Empty:
                return items

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "connection": self._connection,
                "connected": self._running,
                "boot_id": self._boot_id,
                "arm_token": self._arm_token,
                "fw": self._fw,
                "mode": self._mode,
                "protocol": self._protocol,
                "esp_state": self._esp_state,
                "session": self._session,
                "last_seq": self._last_seq,
                "last_status": dict(self._last_status),
                "can_arm": self._can_arm_locked(),
                "arm_pending": self._arm_pending,
                "stopping": self._require_disarmed or self._stop_requested,
            }

    def can_arm(self) -> bool:
        with self._lock:
            return self._can_arm_locked()

    def arm(self, session: int) -> bool:
        if not (0 < int(session) <= 0x7FFFFFFF):
            raise ValueError("session must be a positive 31-bit integer")
        with self._lock:
            token = self._arm_token
            if not self._can_arm_locked() or token is None:
                return False
            self._control_epoch += 1
            self._pending_arm_epoch = self._control_epoch
            self._arm_pending = True
            self._require_disarmed = False
            self._controls.append(("ARM", int(session), int(token)))
            self._session = None
            self._next_seq = 1
            self._latest_motion = None
            self._motion_dirty = False
            self._last_status = {}
            self._condition.notify_all()
            return True

    def stop(self) -> None:
        self._request_stop("STOP")

    def disarm(self) -> None:
        self._request_stop("DISARM")

    def set_motion(self, vx: int, vy: int, w: int) -> bool:
        vx = max(-MAX_LINEAR_MM_S, min(MAX_LINEAR_MM_S, int(vx)))
        vy = max(-MAX_LINEAR_MM_S, min(MAX_LINEAR_MM_S, int(vy)))
        w = max(-MAX_ANGULAR_DEG_S, min(MAX_ANGULAR_DEG_S, int(w)))
        with self._lock:
            if self._session is None or self._esp_state != "ARMED":
                return False
            self._latest_motion = (vx, vy, w)
            self._motion_dirty = True
            self._condition.notify_all()
            return True

    def request_status(self) -> None:
        with self._lock:
            if self._running:
                self._status_pending = True
                self._condition.notify_all()

    def send_raw(self, line: str) -> bool:
        text = line.strip()
        if not text:
            return False
        if any(ord(char) < 32 or ord(char) > 126 for char in text):
            self._emit("error", "每次只发送一行英文命令，不支持换行或控制字符。")
            return False
        if text.split()[0].upper() == "MODE":
            self._emit("error", "请使用顶部模式切换，不直接发送 MODE 命令。")
            return False
        upper = text.upper()
        if upper == "STOP":
            self.stop()
            return True
        if upper == "DISARM":
            self.disarm()
            return True
        with self._lock:
            if self._connection != ConnectionState.READY:
                return False
            self._controls.append(("RAW", text))
            self._condition.notify_all()
            return True

    # ------------------------------------------------------------------ internals

    def _request_stop(self, command: str) -> None:
        with self._lock:
            self._control_epoch += 1
            self._pending_arm_epoch = None
            self._arm_pending = False
            self._require_disarmed = True
            self._stop_requested_at = time.monotonic()
            self._last_disarm_at = self._stop_requested_at
            self._stop_requested = True
            self._stop_command = command
            self._controls.clear()
            self._latest_motion = None
            self._motion_dirty = False
            self._session = None
            self._next_seq = 1
            self._last_status = {}
            self._status_pending = False
            self._condition.notify_all()

    def _can_arm_locked(self) -> bool:
        return (
            self._connection == ConnectionState.READY
            and self._esp_state == "DISARMED"
            and self._mode == MODE_DRY_RUN
            and self._protocol == PROTOCOL_VERSION
            and self._arm_token is not None
            and not self._arm_pending
            and not self._require_disarmed
        )

    def _emit(self, kind: str, payload: object) -> None:
        self._events.put((kind, payload))

    def _set_local_status_locked(
        self,
        state: str,
        reason: str,
        session: int = 0,
        last_seq: int = 0,
        last_session: int = 0,
    ) -> None:
        self._last_status = {
            "v": PROTOCOL_VERSION,
            "type": "local",
            "state": state,
            "reason": reason,
            "session": session,
            "last_session": last_session,
            "last_seq": last_seq,
            "vx": 0,
            "vy": 0,
            "w": 0,
            "age_ms": 0 if state == "ARMED" else -1,
            "watchdog_ms": WATCHDOG_MS,
            "dry_run": True,
        }

    def _has_work_locked(self) -> bool:
        return (
            self._stop_requested
            or bool(self._controls)
            or self._motion_dirty
            or self._status_pending
        )

    def _supervise_locked(self, now: float) -> str | None:
        # 先协商 DRY_RUN 再同步停止状态；连接成功不等于自动 ARM。
        if self._connection == ConnectionState.WAITING_IDENTITY:
            elapsed = now - self._opened_at
            if self.select_unified_mode and self._unified_phase in ("discover", "selecting"):
                if elapsed >= self.identity_timeout_s:
                    self._connection = ConnectionState.ERROR
                    self._emit("error", "统一固件模式确认超时；请确认已烧录支持两种模式的新固件。")
                    self._emit("state_change", self.snapshot())
                    return None
                if self._unified_phase == "discover" and elapsed >= max(1.0, self.identity_grace_s + 0.6):
                    # Compatible with the original pure DRY_RUN firmware.
                    self._unified_phase = "legacy"
                    self._last_probe = now
                    return "PING\n"
                if now - self._last_probe >= self.probe_interval_s:
                    self._last_probe = now
                    return "MODE DRY_RUN\n" if self._unified_phase == "selecting" else "WHO\n"
                return None
            if (
                elapsed >= self.identity_grace_s
                and now - self._last_probe >= self.probe_interval_s
            ):
                self._last_probe = now
                return "PING\n"
            if (
                elapsed >= self.identity_timeout_s
                and not self._identity_timeout_reported
            ):
                self._identity_timeout_reported = True
                self._emit(
                    "error",
                    "等待 ESP32 启动/hello/PONG 超时；请检查端口、"
                    "波特率和 VS Code Monitor 是否占用。",
                )
            return None
        if self._connection == ConnectionState.SYNCING:
            if now - self._last_sync_probe >= self.sync_retry_s:
                self._last_sync_probe = now
                return "STATUS\n"
            if (
                now - self._sync_started >= self.sync_timeout_s
                and not self._sync_timeout_reported
            ):
                self._sync_timeout_reported = True
                self._emit("error", "同步超时：设备没有进入 DISARMED。")
            return None
        return None

    def _writer_loop(self) -> None:
        # 停止指令优先于控制队列；运动目标只保留最新一份，不排队追赶旧动作。
        while True:
            line: str | None = None
            with self._lock:
                if not self._running:
                    return
                now = time.monotonic()
                if self._has_work_locked():
                    if self._stop_requested:
                        command = self._stop_command
                        self._stop_requested = False
                        self._controls.clear()
                        self._latest_motion = None
                        self._motion_dirty = False
                        self._status_pending = True
                        line = command + "\n"
                    elif self._controls:
                        item = self._controls.popleft()
                        if item[0] == "ARM":
                            _, session, token = item
                            line = f"ARM {session} {token}\n"
                        elif item[0] == "RAW":
                            line = item[1] + "\n"
                    elif self._motion_dirty:
                        if self._session is None or self._latest_motion is None:
                            self._motion_dirty = False
                            continue
                        seq = self._next_seq
                        self._next_seq += 1
                        vx, vy, w = self._latest_motion
                        line = f"DRIVE {self._session} {seq} {vx} {vy} {w}\n"
                        self._motion_dirty = False
                    elif self._status_pending:
                        self._status_pending = False
                        line = "STATUS\n"
                else:
                    probe = self._supervise_locked(now)
                    if probe is None:
                        self._condition.wait(0.05)
                        continue
                    line = probe
            if line is not None:
                self._write(line)

    def _reader_loop(self) -> None:
        while True:
            with self._lock:
                if not self._running:
                    return
                serial_port = self._serial
            if serial_port is None:
                return
            try:
                pending = serial_port.in_waiting
                data = serial_port.read(pending or 1)
            except Exception as exc:  # pyserial raises SerialException
                self._handle_disconnect(f"读串口失败: {exc}")
                return
            if not data:
                continue
            for line in self._framer.feed(data):
                self._handle_line(line)

    def _write(self, line: str) -> None:
        with self._lock:
            serial_port = self._serial
        if serial_port is None:
            return
        self._emit("log", {"dir": "tx", "text": line.rstrip("\r\n")})
        try:
            payload = line.encode("ascii")
        except UnicodeEncodeError:
            self._emit("error", "原始命令包含非 ASCII 字符，未发送。")
            return
        try:
            serial_port.write(payload)
            serial_port.flush()
        except Exception as exc:
            self._handle_disconnect(f"写串口失败: {exc}")

    def _handle_disconnect(self, message: str) -> None:
        with self._lock:
            if self._disconnect_reported:
                return
            self._disconnect_reported = True
            self._running = False
            self._connection = ConnectionState.DISCONNECTED
            self._session = None
            self._esp_state = "UNKNOWN"
            self._latest_motion = None
            self._motion_dirty = False
            self._status_pending = False
            self._controls.clear()
            self._stop_requested = False
            self._control_epoch += 1
            self._pending_arm_epoch = None
            self._arm_pending = False
            self._require_disarmed = False
            self._last_disarm_at = 0.0
            self._arm_confirmed_at = 0.0
            self._last_status = {}
            self._last_seq = 0
            self._next_seq = 1
            self._condition.notify_all()
        self._emit("disconnected", message)
        self._emit("state_change", self.snapshot())

    def _handle_line(self, line: str) -> None:
        self._emit("log", {"dir": "rx", "text": line})
        if self.select_unified_mode:
            with self._lock:
                if line.startswith("UNIFIED_ROBOT 1 mode=") and self._unified_phase == "discover":
                    self._unified_phase = "selecting"
                    self._last_probe = 0.0
                    self._condition.notify_all()
                    return
                if line == "MODE_OK DRY_RUN STOPPED" and self._unified_phase == "selecting":
                    self._unified_phase = "ready"
                    return
                if self._unified_phase in ("discover", "selecting"):
                    return
        if not line.startswith("{"):
            # ESP32 boot ROM output / non-protocol text. Not an error.
            return
        try:
            message = json.loads(line)
        except (ValueError, TypeError):
            self._emit("error", "收到无法解析的 JSON 行（已忽略）。")
            return
        if not isinstance(message, dict):
            return

        with self._lock:
            version = message.get("v")
            if version != PROTOCOL_VERSION:
                self._connection = ConnectionState.ERROR
                self._emit(
                    "error",
                    f"协议版本不匹配：设备 v={version!r}，上位机 v={PROTOCOL_VERSION}",
                )
                self._condition.notify_all()
                self._emit("state_change", self.snapshot())
                return

            self._update_identity_locked(message)
            message_type = message.get("type")
            if message_type == "hello":
                self._begin_sync_if_waiting_locked()
            elif message_type == "pong":
                self._begin_sync_if_waiting_locked()
            elif message_type == "status":
                self._begin_sync_if_waiting_locked()
                self._apply_status_locked(message)
                if (
                    self._connection == ConnectionState.SYNCING
                    and self._esp_state == "DISARMED"
                ):
                    self._connection = ConnectionState.READY
                    self._sync_timeout_reported = False
                    self._emit("ready", self.snapshot())
                    self._emit("state_change", self.snapshot())
                elif (
                    self._connection == ConnectionState.SYNCING
                    and self._esp_state == "ARMED"
                ):
                    self._stop_requested = True
                    self._stop_command = "STOP"
                    self._status_pending = True
                    self._condition.notify_all()
                    self._emit(
                        "log",
                        {"dir": "info", "text": "设备当前 ARMED，已发送 STOP 同步。"},
                    )
            elif message_type == "ack":
                self._handle_ack_locked(message)
            elif message_type == "event":
                self._handle_event_locked(message)
            self._condition.notify_all()

    def _update_identity_locked(self, message: dict) -> None:
        boot = _int_or_none(message.get("boot_id"))
        if boot is not None and boot != 0:
            if self._boot_id is None:
                self._boot_id = boot
            elif boot != self._boot_id:
                self._handle_reboot_locked(boot)

        token = _int_or_none(message.get("arm_token"))
        if token is not None and token != 0:
            self._arm_token = token

        fw = message.get("fw")
        if isinstance(fw, str):
            self._fw = fw
        mode = message.get("mode")
        if isinstance(mode, str):
            if mode != MODE_DRY_RUN:
                self._emit(
                    "error",
                    f"设备模式为 {mode!r}，不是 DRY_RUN；ARM 已禁用。",
                )
            self._mode = mode
        protocol = _int_or_none(message.get("protocol"))
        if protocol is not None:
            self._protocol = protocol
        else:
            version = _int_or_none(message.get("v"))
            if version is not None:
                self._protocol = version

    def _handle_reboot_locked(self, new_boot_id: int) -> None:
        old_boot_id = self._boot_id
        self._boot_id = new_boot_id
        self._session = None
        self._next_seq = 1
        self._last_seq = 0
        self._latest_motion = None
        self._motion_dirty = False
        self._esp_state = "UNKNOWN"
        self._connection = ConnectionState.SYNCING
        self._sync_started = time.monotonic()
        self._last_sync_probe = self._sync_started
        self._sync_timeout_reported = False
        self._stop_requested = True
        self._stop_command = "STOP"
        self._status_pending = True
        self._control_epoch += 1
        self._pending_arm_epoch = None
        self._arm_pending = False
        self._require_disarmed = False
        self._last_disarm_at = 0.0
        self._arm_confirmed_at = 0.0
        self._last_status = {}
        self._emit(
            "reboot",
            {"old_boot_id": old_boot_id, "new_boot_id": new_boot_id},
        )
        self._emit("state_change", self.snapshot())

    def _begin_sync_if_waiting_locked(self) -> None:
        if self._connection != ConnectionState.WAITING_IDENTITY:
            return
        self._connection = ConnectionState.SYNCING
        self._sync_started = time.monotonic()
        self._last_sync_probe = self._sync_started
        self._sync_timeout_reported = False
        self._stop_requested = True
        self._stop_command = "STOP"
        self._status_pending = True
        self._emit(
            "log",
            {"dir": "info", "text": "已确认协议，发送 STOP 后读取 STATUS。"},
        )
        self._emit("state_change", self.snapshot())

    def _apply_status_locked(self, message: dict) -> None:
        state = message.get("state")
        previous_state = self._esp_state
        now = time.monotonic()

        # A STATUS response generated before a fresh ARM can arrive after the
        # ARM ack and still say DISARMED. Do not let it kill the new session.
        if (
            state == "DISARMED"
            and self._session is not None
            and (now - self._arm_confirmed_at) < 0.5
        ):
            self._emit(
                "log",
                {"dir": "info", "text": "忽略 ARM 之后迟到的 DISARMED 状态"},
            )
            return

        # A status generated before a STOP/超时/重连 can arrive late. While we
        # have no session and a disarm just happened, ignore ARMED statuses.
        if state == "ARMED" and self._session is None:
            if self._require_disarmed or (now - self._last_disarm_at) < 1.0:
                self._emit(
                    "log",
                    {"dir": "info", "text": "忽略 STOP/超时之后迟到的 ARMED 状态"},
                )
                return
            self._require_disarmed = False
            if (
                self._connection == ConnectionState.READY
                and not self._arm_pending
            ):
                self._emit(
                    "error",
                    "设备报告 ARMED，但本面板没有活动会话；请先 STOP。",
                )

        if state in ("ARMED", "DISARMED"):
            self._esp_state = state
        if state == "DISARMED":
            self._require_disarmed = False
            self._last_disarm_at = now
        last_seq = _int_or_none(message.get("last_seq"))
        if last_seq is not None:
            self._last_seq = last_seq
        self._last_status = dict(message)

        if state == "DISARMED":
            if self._session is not None or previous_state == "ARMED":
                self._session = None
                self._next_seq = 1
                self._latest_motion = None
                self._motion_dirty = False
                self._emit("disarmed", message)
        elif state == "ARMED":
            session = _int_or_none(message.get("session"))
            if (
                self._session is not None
                and session is not None
                and session != self._session
            ):
                self._session = None
                self._next_seq = 1
                self._latest_motion = None
                self._motion_dirty = False
                self._emit(
                    "error",
                    "设备被另一个 session 控制；请先 STOP，再重新 ARM。",
                )

    def _handle_ack_locked(self, message: dict) -> None:
        command = message.get("cmd")
        ok = bool(message.get("ok"))
        error = message.get("error", "NONE")
        now = time.monotonic()

        if command == "ARM":
            # If STOP/DISARM/reconnect happened after this ARM request, the
            # ack is stale and must not re-enable motion.
            if (
                not self._arm_pending
                or self._pending_arm_epoch != self._control_epoch
            ):
                self._emit(
                    "log",
                    {"dir": "info", "text": "忽略 STOP/重连之后迟到的 ARM 应答"},
                )
                self._status_pending = True
                return
            self._arm_pending = False
            self._pending_arm_epoch = None
            if ok:
                session = _int_or_none(message.get("session"))
                if session is not None and session > 0:
                    self._session = session
                    self._esp_state = "ARMED"
                    self._require_disarmed = False
                    self._arm_confirmed_at = now
                    self._next_seq = 1
                    self._last_seq = 0
                    self._latest_motion = None
                    self._motion_dirty = False
                    self._set_local_status_locked("ARMED", "NONE", session, 0, session)
                    self._status_pending = True
                    self._emit("armed", message)
                else:
                    self._emit("error", "ARM 应答缺少有效 session。")
            else:
                self._session = None
                self._latest_motion = None
                self._motion_dirty = False
                self._emit(
                    "arm_failed",
                    {"error": error, "message": message},
                )
                self._status_pending = True
        elif command in ("STOP", "DISARM"):
            self._arm_pending = False
            self._pending_arm_epoch = None
            if ok:
                self._session = None
                self._esp_state = "DISARMED"
                self._last_disarm_at = now
                self._next_seq = 1
                self._latest_motion = None
                self._motion_dirty = False
                last_seq = _int_or_none(message.get("last_seq")) or 0
                last_session = _int_or_none(message.get("last_session")) or 0
                self._set_local_status_locked(
                    "DISARMED",
                    str(message.get("reason", "STOP")),
                    0,
                    last_seq,
                    last_session,
                )
                self._emit("disarmed", message)
                self._status_pending = True
            else:
                self._emit(
                    "error",
                    f"{command} 失败：{error}；设备可能仍未撤销使能。",
                )
                self._status_pending = True
        elif command == "DRIVE":
            ack_session = _int_or_none(message.get("session"))
            if self._session is None or (
                ack_session is not None and ack_session != self._session
            ):
                self._emit(
                    "log",
                    {"dir": "info", "text": "忽略 STOP/重连之后迟到的 DRIVE 应答"},
                )
                return
            if ok:
                last_seq = _int_or_none(message.get("last_seq"))
                if last_seq is not None:
                    self._last_seq = last_seq
            else:
                self._emit(
                    "drive_error",
                    {"error": error, "message": message},
                )
                if error in ("NOT_ARMED", "SESSION_MISMATCH"):
                    self._session = None
                    self._next_seq = 1
                    self._latest_motion = None
                    self._motion_dirty = False
                    if error == "NOT_ARMED":
                        self._esp_state = "DISARMED"
                        self._require_disarmed = False
                        self._last_disarm_at = now
                    self._status_pending = True

    def _handle_event_locked(self, message: dict) -> None:
        event = message.get("event")
        if event == "WATCHDOG_STOP":
            self._control_epoch += 1
            self._pending_arm_epoch = None
            self._arm_pending = False
            self._require_disarmed = False
            self._last_disarm_at = time.monotonic()
            self._session = None
            self._esp_state = "DISARMED"
            self._next_seq = 1
            self._latest_motion = None
            self._motion_dirty = False
            last_seq = _int_or_none(message.get("last_seq")) or 0
            last_session = _int_or_none(message.get("last_session")) or 0
            self._last_seq = last_seq
            self._set_local_status_locked(
                "DISARMED", "WATCHDOG", 0, last_seq, last_session
            )
            self._emit("watchdog", message)
            self._status_pending = True
