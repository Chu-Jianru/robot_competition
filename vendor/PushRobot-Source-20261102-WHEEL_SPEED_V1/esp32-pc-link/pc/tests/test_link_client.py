"""Offline tests for the PC-side link client using a fake serial device.

Run from `pc/`:
    python -m unittest discover -s tests -v

These tests do not touch real hardware.
"""

from __future__ import annotations

import json
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import serial  # noqa: E402

from link_client import (  # noqa: E402
    ConnectionState,
    LineFramer,
    LinkClient,
    MODE_DRY_RUN,
)


def wait_until(predicate, timeout=2.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class FakeSerial:
    def __init__(self, device, **kwargs):
        self.device = device
        self.kwargs = kwargs
        self._rx = bytearray()
        self._lock = threading.Lock()
        self.written: list[str] = []
        self.closed = False
        self.fail_reads = False
        self.gate: threading.Event | None = None

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._rx)

    def read(self, size: int = 1) -> bytes:
        if self.closed:
            raise serial.SerialException("fake port closed")
        if self.fail_reads:
            raise serial.SerialException("simulated unplug")
        deadline = time.monotonic() + 0.05
        while time.monotonic() < deadline:
            with self._lock:
                if self._rx:
                    data = bytes(self._rx[:size])
                    del self._rx[:size]
                    return data
            time.sleep(0.002)
        return b""

    def write(self, data: bytes) -> int:
        if self.gate is not None:
            self.gate.wait(timeout=3.0)
        if self.closed:
            raise serial.SerialException("fake port closed")
        text = data.decode("ascii", errors="replace")
        self.written.append(text)
        self.device.on_write(text)
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True

    def inject(self, text: str) -> None:
        with self._lock:
            self._rx.extend(text.encode("utf-8"))


class FakeHarness:
    def __init__(self, device: "FakeEsp32") -> None:
        self.device = device
        self.serial: FakeSerial | None = None

    def factory(self, **kwargs) -> FakeSerial:
        self.serial = FakeSerial(self.device, **kwargs)
        self.device.attach(self.serial)
        return self.serial


class FakeEsp32:
    """Small protocol peer used only to exercise the PC client."""

    def __init__(
        self,
        boot_id: int = 0x11111111,
        token: int = 0x22222222,
        state: str = "DISARMED",
        auto_hello: bool = True,
        reject_arm_token: bool = False,
    ) -> None:
        self.boot_id = boot_id
        self.token = token
        self.state = state
        self.auto_hello = auto_hello
        self.reject_arm_token = reject_arm_token
        self.session = 0
        self.last_session = 0
        self.last_seq = 0
        self.vx = 0
        self.vy = 0
        self.w = 0
        self.reason = "BOOT"
        self.serial: FakeSerial | None = None
        self.hold_arm_ack = False
        self.hold_stop_ack = False
        self.hold_drive_ack = False
        self._held_arm_ack: dict | None = None
        self._held_stop_ack: dict | None = None
        self._held_drive_ack: dict | None = None

    def attach(self, serial_port: FakeSerial) -> None:
        self.serial = serial_port
        if self.auto_hello:
            self.send_hello()

    def _send(self, message: dict) -> None:
        assert self.serial is not None
        self.serial.inject(json.dumps(message, separators=(",", ":")) + "\n")

    def send_hello(self) -> None:
        self._send(
            {
                "v": 1,
                "type": "hello",
                "fw": "0.1.0",
                "mode": MODE_DRY_RUN,
                "protocol": 1,
                "boot_id": self.boot_id,
                "arm_token": self.token,
                "watchdog_ms": 300,
                "state": self.state,
                "uptime_ms": 10,
            }
        )

    def _rotate_token(self) -> None:
        self.token = (self.token * 1664525 + 1013904223) & 0xFFFFFFFF
        if self.token == 0 or self.token == 0x22222222:
            self.token = (self.token + 1) & 0xFFFFFFFF or 1

    def _make_ack(self, cmd: str, ok: bool, error: str = "NONE") -> dict:
        return {
            "v": 1,
            "type": "ack",
            "cmd": cmd,
            "ok": ok,
            "error": error,
            "state": self.state,
            "boot_id": self.boot_id,
            "arm_token": self.token,
            "session": self.session if self.state == "ARMED" else 0,
            "last_session": self.last_session,
            "last_seq": self.last_seq,
            "vx": self.vx,
            "vy": self.vy,
            "w": self.w,
            "reason": self.reason,
            "uptime_ms": 20,
        }

    def _ack(self, cmd: str, ok: bool, error: str = "NONE") -> None:
        self._send(self._make_ack(cmd, ok, error))

    def release_arm_ack(self) -> None:
        if self._held_arm_ack is not None:
            ack = self._held_arm_ack
            self._held_arm_ack = None
            self._send(ack)

    def release_stop_ack(self) -> None:
        if self._held_stop_ack is not None:
            ack = self._held_stop_ack
            self._held_stop_ack = None
            self._send(ack)

    def release_drive_ack(self) -> None:
        if self._held_drive_ack is not None:
            ack = self._held_drive_ack
            self._held_drive_ack = None
            self._send(ack)

    def send_status_override(self, state: str, session: int = 0, last_seq: int = 0) -> None:
        self._send(
            {
                "v": 1,
                "type": "status",
                "fw": "0.1.0",
                "mode": MODE_DRY_RUN,
                "protocol": 1,
                "boot_id": self.boot_id,
                "arm_token": self.token,
                "state": state,
                "session": session if state == "ARMED" else 0,
                "last_session": self.last_session,
                "last_seq": last_seq,
                "vx": 100 if state == "ARMED" else 0,
                "vy": 0,
                "w": 0,
                "age_ms": 0 if state == "ARMED" else -1,
                "watchdog_ms": 300,
                "reason": "NONE" if state == "ARMED" else "STOP",
                "uptime_ms": 50,
                "dry_run": True,
            }
        )

    def _status(self) -> None:
        self._send(
            {
                "v": 1,
                "type": "status",
                "fw": "0.1.0",
                "mode": MODE_DRY_RUN,
                "protocol": 1,
                "boot_id": self.boot_id,
                "arm_token": self.token,
                "state": self.state,
                "session": self.session if self.state == "ARMED" else 0,
                "last_session": self.last_session,
                "last_seq": self.last_seq,
                "vx": self.vx,
                "vy": self.vy,
                "w": self.w,
                "age_ms": 0 if self.state == "ARMED" else -1,
                "watchdog_ms": 300,
                "reason": self.reason,
                "uptime_ms": 30,
                "dry_run": True,
            }
        )

    def on_write(self, text: str) -> None:
        parts = text.strip().split()
        if not parts:
            return
        command = parts[0]
        if command == "PING":
            self._send(
                {
                    "v": 1,
                    "type": "pong",
                    "boot_id": self.boot_id,
                    "arm_token": self.token,
                    "state": self.state,
                    "uptime_ms": 40,
                }
            )
            return
        if command == "STATUS":
            self._status()
            return
        if command in ("STOP", "DISARM"):
            self.state = "DISARMED"
            self.session = 0
            self.vx = self.vy = self.w = 0
            self.reason = "STOP" if command == "STOP" else "DISARM"
            self._rotate_token()
            ack = self._make_ack(command, True)
            if self.hold_stop_ack:
                self._held_stop_ack = ack
            else:
                self._send(ack)
            return
        if command == "ARM":
            if len(parts) != 3:
                self._ack("ARM", False, "BAD_ARGS")
                return
            try:
                session = int(parts[1])
                token = int(parts[2])
            except ValueError:
                self._ack("ARM", False, "BAD_ARGS")
                return
            if self.state == "ARMED":
                self._ack("ARM", False, "ALREADY_ARMED")
            elif self.reject_arm_token or token != self.token:
                self._ack("ARM", False, "STALE_ARM_TOKEN")
            elif session == self.last_session:
                self._ack("ARM", False, "SESSION_REUSED")
            else:
                self.state = "ARMED"
                self.session = session
                self.last_session = session
                self.last_seq = 0
                self.vx = self.vy = self.w = 0
                self.reason = "NONE"
                ack = self._make_ack("ARM", True)
                if self.hold_arm_ack:
                    self._held_arm_ack = ack
                else:
                    self._send(ack)
            return
        if command == "DRIVE":
            if len(parts) != 6:
                self._ack("DRIVE", False, "BAD_ARGS")
                return
            try:
                session = int(parts[1])
                seq = int(parts[2])
                vx = int(parts[3])
                vy = int(parts[4])
                w = int(parts[5])
            except ValueError:
                self._ack("DRIVE", False, "BAD_ARGS")
                return
            if self.state != "ARMED":
                self._ack("DRIVE", False, "NOT_ARMED")
            elif session != self.session:
                self._ack("DRIVE", False, "SESSION_MISMATCH")
            elif seq <= self.last_seq:
                self._ack("DRIVE", False, "STALE_SEQ")
            elif abs(vx) > 300 or abs(vy) > 300 or abs(w) > 180:
                self._ack("DRIVE", False, "RANGE")
            else:
                self.vx, self.vy, self.w = vx, vy, w
                self.last_seq = seq
                ack = self._make_ack("DRIVE", True)
                if self.hold_drive_ack:
                    self._held_drive_ack = ack
                else:
                    self._send(ack)
            return
        self._ack("", False, "UNKNOWN_CMD")


class UnifiedEsp32(FakeEsp32):
    def __init__(self, acknowledge=True):
        super().__init__()
        self.acknowledge = acknowledge

    def on_write(self, text):
        if text.strip() == "WHO":
            self.serial.inject("UNIFIED_ROBOT 1 mode=MOTOR_A_TEST\n")
        elif text.strip() == "MODE DRY_RUN":
            self.state = "DISARMED"
            self.session = 0
            self._rotate_token()
            self.serial.inject("MODE_OK DRY_RUN STOPPED\n" if self.acknowledge else "MODE_OK MOTOR_A_TEST STOPPED\n")
            self.send_hello()
        else:
            super().on_write(text)


class LineFramerTests(unittest.TestCase):
    def test_fragmentation_and_coalescing(self):
        framer = LineFramer()
        self.assertEqual(framer.feed(b"PI"), [])
        self.assertEqual(framer.feed(b"NG\r\nSTATUS\n"), ["PING", "STATUS"])
        self.assertEqual(framer.feed(b"\n"), [])

    def test_long_line_is_discarded_and_resyncs(self):
        framer = LineFramer(max_len=32)
        self.assertEqual(framer.feed(b"A" * 100), [])
        self.assertEqual(framer.dropped_long, 1)
        self.assertEqual(framer.feed(b"\nPING\n"), ["PING"])

    def test_binary_and_utf8_do_not_raise(self):
        framer = LineFramer()
        lines = framer.feed(b"\xff\xfe\r\nPING\n")
        self.assertEqual(len(lines), 2)
        self.assertEqual(lines[1], "PING")


class LinkClientTests(unittest.TestCase):
    def make_client(self, device: FakeEsp32, **client_kwargs):
        harness = FakeHarness(device)
        defaults = dict(
            identity_grace_s=0.2,
            probe_interval_s=0.05,
            identity_timeout_s=1.0,
            sync_retry_s=0.1,
            sync_timeout_s=1.0,
        )
        defaults.update(client_kwargs)
        client = LinkClient(serial_factory=harness.factory, **defaults)
        self.addCleanup(client.close, False)
        client.connect("FAKE")
        self.assertTrue(wait_until(lambda: harness.serial is not None))
        self.addCleanup(harness.serial.close)
        return client, device, harness.serial

    def wait_ready(self, client, serial_port):
        self.assertTrue(
            wait_until(
                lambda: client.snapshot()["connection"] == ConnectionState.READY,
                timeout=2.0,
            )
        )
        self.assertTrue(wait_until(lambda: client.can_arm()))

    def test_waits_for_identity_before_stop_status(self):
        device = FakeEsp32(auto_hello=False)
        client, _, serial_port = self.make_client(device)
        time.sleep(0.12)
        self.assertEqual(
            client.snapshot()["connection"], ConnectionState.WAITING_IDENTITY
        )
        self.assertFalse(any("STOP" in line for line in serial_port.written))
        self.assertFalse(any("STATUS" in line for line in serial_port.written))

        device.send_hello()
        self.wait_ready(client, serial_port)
        self.assertIn("STOP\n", serial_port.written)
        self.assertIn("STATUS\n", serial_port.written)
        self.assertLess(
            serial_port.written.index("STOP\n"),
            serial_port.written.index("STATUS\n"),
        )

    def test_unified_negotiation_ignores_early_hello_and_never_auto_arms(self):
        client, device, port = self.make_client(UnifiedEsp32(), select_unified_mode=True, identity_timeout_s=2)
        self.wait_ready(client, port)
        self.assertLess(port.written.index("MODE DRY_RUN\n"), port.written.index("STOP\n"))
        self.assertEqual(device.state, "DISARMED")
        self.assertFalse(any(line.startswith(("ARM ", "DRIVE ", "JOG ")) for line in port.written))
        self.assertFalse(client.send_raw("MODE MOTOR_A_TEST"))
        self.assertFalse(client.send_raw("IO\nMODE MOTOR_A_TEST"))
        self.assertFalse(client.send_raw("IO\rMODE MOTOR_A_TEST"))

    def test_unified_wrong_mode_ack_does_not_accept_hello(self):
        client, _, port = self.make_client(UnifiedEsp32(acknowledge=False), select_unified_mode=True, identity_timeout_s=2)
        self.assertTrue(wait_until(lambda: "MODE DRY_RUN\n" in port.written))
        time.sleep(0.1)
        self.assertFalse(client.can_arm())
        self.assertNotIn("STOP\n", port.written)
        self.assertEqual(client.snapshot()["connection"], ConnectionState.WAITING_IDENTITY)

    def test_unified_client_still_accepts_original_pure_dry_firmware(self):
        client, _, port = self.make_client(FakeEsp32(), select_unified_mode=True, identity_timeout_s=3)
        self.wait_ready(client, port)
        self.assertIn("PING\n", port.written)
        self.assertFalse(any(line.startswith("MODE ") for line in port.written))

    def test_auto_hello_and_armed_device_is_stopped(self):
        device = FakeEsp32(state="ARMED", auto_hello=True)
        device.session = 777
        device.last_session = 777
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)
        self.assertEqual(device.state, "DISARMED")
        self.assertIn("STOP\n", serial_port.written)

    def test_arm_drive_and_stop(self):
        device = FakeEsp32(auto_hello=True)
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)

        session = 123456
        self.assertTrue(client.arm(session))
        self.assertTrue(
            wait_until(lambda: client.snapshot()["session"] == session)
        )
        self.assertEqual(client.snapshot()["esp_state"], "ARMED")

        self.assertTrue(client.set_motion(100, -20, 10))
        self.assertTrue(
            wait_until(
                lambda: any(line.startswith("DRIVE ") for line in serial_port.written)
            )
        )
        drive_line = next(
            line for line in serial_port.written if line.startswith("DRIVE ")
        )
        parts = drive_line.strip().split()
        self.assertEqual(int(parts[1]), session)
        self.assertEqual(int(parts[2]), 1)
        self.assertEqual(parts[3:], ["100", "-20", "10"])

        client.stop()
        self.assertTrue(
            wait_until(lambda: client.snapshot()["esp_state"] == "DISARMED")
        )
        self.assertIn("STOP\n", serial_port.written)
        self.assertIsNone(client.snapshot()["session"])

    def test_motion_coalescing_and_stop_priority(self):
        device = FakeEsp32(auto_hello=True)
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)
        self.assertTrue(client.arm(999))
        self.assertTrue(
            wait_until(lambda: client.snapshot()["session"] == 999)
        )

        serial_port.gate = threading.Event()  # block writes
        self.addCleanup(serial_port.gate.set)
        self.assertTrue(client.set_motion(10, 0, 0))
        time.sleep(0.05)
        for value in range(1, 60):
            client.set_motion(value, 0, 0)
        client.stop()
        time.sleep(0.05)
        serial_port.gate.set()

        self.assertTrue(
            wait_until(lambda: client.snapshot()["esp_state"] == "DISARMED")
        )
        time.sleep(0.1)
        drives = [line for line in serial_port.written if line.startswith("DRIVE ")]
        self.assertLessEqual(len(drives), 2)
        stop_index = max(
            index
            for index, line in enumerate(serial_port.written)
            if line == "STOP\n"
        )
        self.assertFalse(
            any(
                line.startswith("DRIVE ")
                for line in serial_port.written[stop_index + 1 :]
            ),
            msg=f"written={serial_port.written}",
        )

    def test_disconnect_clears_state_and_reconnect_does_not_drive(self):
        device = FakeEsp32(auto_hello=True)
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)
        self.assertTrue(client.arm(111))
        self.assertTrue(
            wait_until(lambda: client.snapshot()["session"] == 111)
        )
        self.assertTrue(client.set_motion(20, 0, 0))
        self.assertTrue(
            wait_until(
                lambda: any(line.startswith("DRIVE ") for line in serial_port.written)
            )
        )

        serial_port.fail_reads = True
        self.assertTrue(
            wait_until(
                lambda: client.snapshot()["connection"]
                == ConnectionState.DISCONNECTED
            )
        )
        self.assertIsNone(client.snapshot()["session"])

        device2 = FakeEsp32(auto_hello=True)
        client2, _, serial2 = self.make_client(device2)
        self.wait_ready(client2, serial2)
        self.assertFalse(any(line.startswith("DRIVE ") for line in serial2.written))
        self.assertEqual(client2.snapshot()["esp_state"], "DISARMED")

    def test_reboot_requires_resync_and_no_auto_arm(self):
        device = FakeEsp32(auto_hello=True)
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)
        self.assertTrue(client.arm(222))
        self.assertTrue(
            wait_until(lambda: client.snapshot()["session"] == 222)
        )

        written_before = len(serial_port.written)
        device.boot_id += 1
        device.state = "DISARMED"
        device.session = 0
        device.send_hello()

        self.assertTrue(
            wait_until(
                lambda: client.snapshot()["connection"] == ConnectionState.READY
            )
        )
        self.assertTrue(
            wait_until(lambda: client.snapshot()["boot_id"] == device.boot_id)
        )
        self.assertIsNone(client.snapshot()["session"])
        new_lines = serial_port.written[written_before:]
        self.assertIn("STOP\n", new_lines)
        self.assertFalse(any(line.startswith("ARM ") for line in new_lines))
        self.assertEqual(client.snapshot()["esp_state"], "DISARMED")

    def test_stale_arm_token_is_not_auto_retried(self):
        device = FakeEsp32(auto_hello=True, reject_arm_token=True)
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)
        self.assertTrue(client.arm(333))
        self.assertTrue(
            wait_until(
                lambda: sum(
                    1 for line in serial_port.written if line.startswith("ARM ")
                )
                >= 1
            )
        )
        time.sleep(0.3)
        arm_lines = [
            line for line in serial_port.written if line.startswith("ARM ")
        ]
        self.assertEqual(len(arm_lines), 1)
        self.assertIsNone(client.snapshot()["session"])
        self.assertEqual(client.snapshot()["esp_state"], "DISARMED")

    def test_late_arm_ack_after_stop_is_ignored(self):
        device = FakeEsp32(auto_hello=True)
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)

        device.hold_arm_ack = True
        self.assertTrue(client.arm(444))
        self.assertTrue(
            wait_until(
                lambda: device.state == "ARMED"
                and device._held_arm_ack is not None
            )
        )
        self.assertTrue(client.snapshot()["arm_pending"])

        client.stop()
        self.assertTrue(wait_until(lambda: device.state == "DISARMED"))
        self.assertTrue(
            wait_until(lambda: client.snapshot()["esp_state"] == "DISARMED")
        )
        self.assertTrue(wait_until(lambda: client.can_arm()))

        # The ARM ack was generated before STOP; it must not revive the session.
        device.release_arm_ack()
        time.sleep(0.2)
        snapshot = client.snapshot()
        self.assertIsNone(snapshot["session"])
        self.assertEqual(snapshot["esp_state"], "DISARMED")
        self.assertFalse(snapshot["arm_pending"])

    def test_stale_armed_status_after_stop_is_ignored(self):
        device = FakeEsp32(auto_hello=True)
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)

        session = 555
        self.assertTrue(client.arm(session))
        self.assertTrue(
            wait_until(lambda: client.snapshot()["session"] == session)
        )
        client.stop()
        self.assertTrue(
            wait_until(lambda: client.snapshot()["esp_state"] == "DISARMED")
        )
        self.assertTrue(wait_until(lambda: client.can_arm()))

        # Simulate an old STATUS (generated before STOP) arriving late.
        device.send_status_override("ARMED", session=session, last_seq=1)
        time.sleep(0.2)
        snapshot = client.snapshot()
        self.assertEqual(snapshot["esp_state"], "DISARMED")
        self.assertIsNone(snapshot["session"])

    def test_stale_disarmed_status_after_arm_is_ignored(self):
        device = FakeEsp32(auto_hello=True)
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)

        self.assertTrue(client.arm(888))
        self.assertTrue(
            wait_until(lambda: client.snapshot()["session"] == 888)
        )

        # A STATUS generated before ARM can arrive just after the ARM ack.
        device.send_status_override("DISARMED")
        time.sleep(0.2)
        snapshot = client.snapshot()
        self.assertEqual(snapshot["session"], 888)
        self.assertEqual(snapshot["esp_state"], "ARMED")

    def test_late_drive_ack_after_rearm_is_ignored(self):
        device = FakeEsp32(auto_hello=True)
        client, _, serial_port = self.make_client(device)
        self.wait_ready(client, serial_port)

        self.assertTrue(client.arm(666))
        self.assertTrue(
            wait_until(lambda: client.snapshot()["session"] == 666)
        )
        device.hold_drive_ack = True
        self.assertTrue(client.set_motion(10, 0, 0))
        self.assertTrue(
            wait_until(
                lambda: device.last_seq == 1
                and device._held_drive_ack is not None
            )
        )

        client.stop()
        self.assertTrue(
            wait_until(lambda: client.snapshot()["esp_state"] == "DISARMED")
        )
        self.assertTrue(wait_until(lambda: client.can_arm()))

        self.assertTrue(client.arm(777))
        self.assertTrue(
            wait_until(lambda: client.snapshot()["session"] == 777)
        )
        self.assertTrue(wait_until(lambda: client.snapshot()["last_seq"] == 0))

        # Old DRIVE ack (session 666) must not update session 777's last_seq.
        device.release_drive_ack()
        time.sleep(0.2)
        snapshot = client.snapshot()
        self.assertEqual(snapshot["session"], 777)
        self.assertEqual(snapshot["last_seq"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
