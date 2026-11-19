"""chassis_link 的离线回归：用假固件验证握手顺序、脉冲门控、STOP 与关闭。

不需要 pyserial 或真实硬件：
    python windows/test_chassis_link.py
"""
from __future__ import annotations

import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from chassis_link import (CHASSIS_CAPS, HOLD_CAPS, HOLD_RELEASE_CAPS, MANUAL_CAPS,
                          SPEED_STREAM_CAPS, STREAM_CAPS, ChassisError, ChassisLink)  # noqa: E402


class FakeFirmware:
    """按 main.cpp 的 MECANUM_V2 行为模拟一台 ESP32。"""

    def __init__(self):
        self.sent = []
        self.closed = False
        self._rx = bytearray()
        self.layout = None
        self.running = False
        self.running_until = 0.0
        self.stop_until = 0.0

    # --- 串口接口 ---
    def write(self, data):
        for line in data.decode("ascii").splitlines():
            line = line.strip()
            if line:
                self.sent.append(line)
                self._handle(line)
        return len(data)

    def readline(self):
        now = time.monotonic()
        if self.running and now >= self.running_until:
            self.running = False
            self._emit("STOPPED timer")
            self._emit("ENC A ticks=0 invalid=0 A=0 B=0")
        index = self._rx.find(b"\n")
        if index < 0:
            return b""
        line = bytes(self._rx[:index + 1])
        del self._rx[:index + 1]
        return line

    def close(self):
        self.closed = True

    # --- 固件行为 ---
    def _emit(self, text):
        self._rx.extend(text.encode("ascii") + b"\r\n")

    def _handle(self, line):
        if line == "WHO":
            self._emit("UNIFIED_ROBOT 1 mode=DRY_RUN")
        elif line == "MODE CHASSIS":
            self.running = False
            self.layout = None
            self._emit("MODE_OK CHASSIS STOPPED")
        elif line == "CAPS":
            self._emit("CAPS MOTORS=ABCD PINS=FOUR_V1")
            self._emit(CHASSIS_CAPS)
        elif line.startswith("LAYOUT "):
            if self.running:
                self._emit("ERROR stop before changing layout")
                return
            self.layout = line.split()[1]
            self._emit(f"LAYOUT CONFIGURED=1 ORDER={self.layout} SIGNS=1,1,1,1 TYPE=X")
        elif line == "STATUS":
            self._emit("RUNNING CHASSIS" if self.running else "STOPPED ALL")
            self._emit("ENC A ticks=0 invalid=0 A=0 B=0")
            self._emit("FAULT none")
        elif line == "PING":
            self._emit("PONG CHASSIS")
        elif line.startswith("MECANUM "):
            _, fwd, left, ccw, ms = line.split()
            ms = int(ms)
            if self.running:
                self._emit("ERROR busy; STOP before next chassis motion")
            elif time.monotonic() < self.stop_until:
                self._emit("ERROR wait 1 second after stop")
            elif self.layout is None:
                self._emit("ERROR confirm X roller layout and LAYOUT first")
            else:
                self.running = True
                self.running_until = time.monotonic() + ms / 1000.0
                self._emit(f"CHASSIS_RUN A={fwd} B={fwd} C={fwd} D={fwd} duration={ms}")
        elif line == "STOP":
            self.running = False
            self.stop_until = time.monotonic()
            self._emit("STOPPED command")
            self._emit("ENC A ticks=0 invalid=0 A=0 B=0")
        elif line.startswith("ENC"):
            self._emit("ENC A ticks=0 invalid=0 A=0 B=0")


class StreamTest(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.link = ChassisLink("FAKE", clock=lambda: self.now, strict=True, require_stream=True)
        self.peer = FakeFirmware()
        self.link._device = self.peer
        self.link._phase = "CAPS"
        self.link._handle_line(STREAM_CAPS, self.now)
        self.link._handle_line(CHASSIS_CAPS, self.now)
        self.link._handle_line("LAYOUT CONFIGURED=1 ORDER=ADBC SIGNS=1,1,1,1 TYPE=X", self.now)
        self.link._handle_line("STOPPED ALL", self.now)

    def test_older_firmware_cannot_arm_autonomous_output(self):
        old = ChassisLink("FAKE", strict=True, require_stream=True)
        old._phase = "CAPS"
        with self.assertRaisesRegex(ChassisError, "CHASSIS_STREAM"):
            old._handle_line(CHASSIS_CAPS, self.now)
        self.assertFalse(old.ready)

    def test_updates_replace_unsent_target_and_renew_without_a_stop(self):
        self.assertTrue(self.link.stream(20, 2, -4, valid_until=1.35))
        self.link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "MECANUM_STREAM 20 2 -4 150")
        self.now = 1.04
        self.assertTrue(self.link.stream(18, 3, 2, valid_until=1.35))
        self.assertTrue(self.link.stream(16, 4, 2, valid_until=1.35))
        self.link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "MECANUM_STREAM 20 2 -4 150")
        self.link._handle_line("STREAM_OK", self.now)
        self.link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "MECANUM_STREAM 16 4 2 150")
        self.assertEqual(self.link.stream_outputs, ((20, 2, -4), (16, 4, 2)))
        self.link._handle_line("STREAM_OK", self.now)
        self.assertEqual(self.link.stream_outputs, ((16, 4, 2),))

    def test_pipelined_renewal_does_not_wait_for_stream_ok(self):
        link = ChassisLink("FAKE", clock=lambda: self.now, strict=True,
                           require_stream=True, stream_pipeline=True)
        link._device = self.peer
        link._phase = "CAPS"
        link._handle_line(STREAM_CAPS, self.now)
        link._handle_line(CHASSIS_CAPS, self.now)
        link._handle_line("LAYOUT CONFIGURED=1 ORDER=ADBC SIGNS=1,1,1,1 TYPE=X", self.now)
        link._handle_line("STOPPED ALL", self.now)
        self.assertTrue(link.stream(20, 0, 0, valid_until=1.35))
        link._service(self.now)
        self.assertTrue(self.peer.sent[-1].startswith("MECANUM_STREAM "))
        self.now = 1.05
        self.assertTrue(link.stream(20, 0, 0, valid_until=1.40))
        link._service(self.now)  # No STREAM_OK for the first target yet.
        self.assertTrue(self.peer.sent[-1].startswith("MECANUM_STREAM "))
        self.now = 1.30
        link._handle_line("STREAM_OK", self.now)  # A late ack must not abort.

    def test_background_stream_preserves_lease_during_planner_pause(self):
        self.link._stream_pipeline = self.link._renew_stream = True
        self.assertTrue(self.link.stream(20, 2, -4, valid_until=1.30))
        self.link._service(self.now)
        self.now = 1.06
        self.link._service(self.now)
        self.assertEqual(self.peer.sent[-2:], ["MECANUM_STREAM 20 2 -4 150"] * 2)
        self.now = 1.12
        self.link._service(self.now)
        lease_ms = int(self.peer.sent[-1].split()[-1])
        self.assertIn(lease_ms, (139, 140))
        self.now = 1.23
        self.link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "STOP")
        self.link._handle_line("STOPPED command", self.now)
        self.now = 1.32
        self.link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "STOP")

    def test_lease_shortens_and_stop_cancels_a_queued_renewal(self):
        self.link._renew_stream = True
        self.assertTrue(self.link.stream(20, 0, 0, valid_until=1.20))
        self.now = 1.05
        self.link._service(self.now)
        self.assertIn(int(self.peer.sent[-1].split()[-1]), (109, 110))
        self.assertTrue(self.link.stream(21, 0, 0, valid_until=1.30))
        self.link.stop()
        self.link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "STOP")
        self.assertFalse(self.link.stream(20, 0, 0, valid_until=1.30))
        self.link._handle_line("STOPPED command", self.now)
        self.assertEqual(self.link.stream_outputs, ())
        self.now = 1.12
        self.link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "STOP")
        self.assertTrue(self.link.stream(20, 0, 0, valid_until=1.30))

    def test_stale_queued_target_is_discarded_and_missing_ack_aborts(self):
        self.assertTrue(self.link.stream(20, 0, 0, valid_until=1.25))
        self.now = 1.22
        self.link._service(self.now)
        self.assertFalse(any(line.startswith("MECANUM_STREAM ") for line in self.peer.sent))
        self.assertTrue(self.link.stream(20, 0, 0, valid_until=1.60))
        self.link._service(self.now)
        self.now = 1.38
        with self.assertRaisesRegex(ChassisError, "租期内"):
            self.link._service(self.now)

    def test_wheel_speed_stream_checks_board_calibration_and_sends_counts(self):
        link = ChassisLink("FAKE", clock=lambda: self.now, strict=True, require_speed_stream=True)
        link._device = self.peer
        link._phase = "CAPS"
        link._handle_line(SPEED_STREAM_CAPS, self.now)
        link._handle_line(CHASSIS_CAPS, self.now)
        link._handle_line("LAYOUT CONFIGURED=1 ORDER=ADBC SIGNS=1,1,1,1 TYPE=X", self.now)
        self.assertEqual(self.peer.sent[-1], "SPEED_INFO")
        with self.assertRaisesRegex(ChassisError, "轮速标定"):
            link._handle_line("SPEED_INFO 0 0 0 0 0", self.now)
        link._handle_line("SPEED_INFO 2800 1 -1 1 -1", self.now)
        link._handle_line("STOPPED ALL", self.now)
        self.assertEqual(link.speed_info, (2800, (1, -1, 1, -1)))
        self.assertTrue(link.stream(1000, 300, -150, valid_until=1.35))
        link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "SPEED_STREAM 1000 300 -150 150")

    def test_wheel_speed_heartbeat_is_not_deferred_by_stop_or_encoder_reads(self):
        link = ChassisLink("FAKE", clock=lambda: self.now, strict=True, require_speed_stream=True)
        link._device = self.peer
        link._phase = "CAPS"
        link._handle_line(SPEED_STREAM_CAPS, self.now)
        link._handle_line(CHASSIS_CAPS, self.now)
        link._handle_line("LAYOUT CONFIGURED=1 ORDER=ADBC SIGNS=1,1,1,1 TYPE=X", self.now)
        link._handle_line("SPEED_INFO 5963 -1 -1 1 1", self.now)
        link._handle_line("STOPPED ALL", self.now)
        self.assertTrue(link.stream(-1987, 0, 0, valid_until=1.35))
        link._service(self.now)
        link._handle_line("STREAM_OK", self.now)

        self.now = 1.06
        link.stop()
        link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "STOP")
        link._handle_line("STOPPED command", self.now)
        self.now = 1.60
        link._pending, link._send_by = "ENC_SNAPSHOT", None
        link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "ENC_SNAPSHOT")

        self.now = 1.71
        link._service(self.now)
        self.assertEqual(self.peer.sent[-1], "PING")
        self.now = 2.42
        link._pending, link._send_by = "ENC_SNAPSHOT", None
        link._service(self.now)
        self.assertEqual(self.peer.sent[-2:], ["ENC_SNAPSHOT", "PING"])
        self.assertEqual([line for line in self.peer.sent if line.startswith("SPEED_STREAM ")],
                         ["SPEED_STREAM -1987 0 0 150"])


def wait_until(predicate, timeout=3.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class ChassisLinkTest(unittest.TestCase):
    def make_link(self, **kwargs):
        fake = FakeFirmware()
        options = dict(serial_factory=lambda **_: fake, heartbeat_s=0.05,
                       cooldown_s=0.05, connect_timeout_s=3.0, reply_timeout_s=2.5)
        options.update(kwargs)
        link = ChassisLink("FAKE", **options)
        return link, fake

    def test_handshake_order(self):
        link, fake = self.make_link()
        link.start()
        try:
            self.assertTrue(link.wait_ready(3.0))
            self.assertEqual(fake.sent[:5], ["WHO", "MODE CHASSIS", "CAPS",
                                             "LAYOUT ADBC 1 1 1 1", "STATUS"])
            self.assertIn("PING", fake.sent)
            self.assertTrue(link.ready)
        finally:
            link.close()

    def test_pulse_before_ready_is_refused(self):
        link, _ = self.make_link()
        self.assertFalse(link.pulse(20, 0, 0, 100))

    def test_hold_renews_and_cannot_restart_without_cooldown(self):
        class HoldFirmware(FakeFirmware):
            def _handle(self, line):
                if line == "CAPS":
                    self._emit(HOLD_CAPS)
                if line.startswith("MECANUM_HOLD "):
                    self._emit("HOLD_OK" if self.running else "CHASSIS_RUN duration=250")
                    self.running = True
                    self.running_until = time.monotonic() + .25
                    return
                super()._handle(line)

        fake = HoldFirmware()
        link = ChassisLink("FAKE", strict=True, serial_factory=lambda **_: fake, cooldown_s=.05)
        link.start()
        try:
            link.wait_ready(2)
            self.assertTrue(link.supports_hold)
            self.assertTrue(wait_until(lambda: not link.busy))
            self.assertTrue(link.hold(40, 0, 0))
            self.assertTrue(wait_until(lambda: "CHASSIS_RUN duration=250" in link.events))
            self.assertTrue(wait_until(lambda: link.hold(45, 0, 0), timeout=.15))
            self.assertTrue(wait_until(lambda: "HOLD_OK" in link.events))
            self.assertIn("MECANUM_HOLD 45 0 0", fake.sent)
            link.stop()
            self.assertTrue(wait_until(lambda: "STOP" in fake.sent))
            self.assertFalse(link.hold(45, 0, 0))
            self.assertTrue(wait_until(lambda: not link.busy))
            self.assertTrue(link.hold(40, 0, 0))
            self.assertTrue(wait_until(lambda: "STOPPED timer" in link.events))
            self.assertTrue(wait_until(lambda: not link.busy))
            self.assertFalse(link.hold(40, 0, 0, renew=True))
            self.assertTrue(link.ready)
            self.assertIsNone(link.error)
        finally:
            link.close()

    def make_manual_link(self):
        now = [1.0]
        link, fake = self.make_link(strict=True, recover_cooldown=True, cooldown_s=1.05,
                                    clock=lambda: now[0], heartbeat_s=10.0)
        link._device, link._phase = fake, "READY"
        link._ready = link._supports_hold = True
        link._last_reply = now[0]
        return link, fake, now

    def test_delayed_stop_ack_blocks_restart_and_late_start_ack_is_harmless(self):
        link, fake, now = self.make_manual_link()
        self.assertTrue(link.hold(20, 0, 0))
        link._service(now[0])
        link.stop()
        link._service(now[0])
        now[0] = 2.1
        link._handle_line("CHASSIS_RUN duration=250", now[0])
        self.assertTrue(link.busy)
        self.assertFalse(link.hold(20, 0, 0))
        link.stop()
        link._service(now[0])
        self.assertEqual(fake.sent.count("STOP"), 1)
        link._handle_line("STOPPED command", now[0])
        now[0] = 3.14
        self.assertFalse(link.hold(20, 0, 0))
        now[0] = 3.16
        self.assertTrue(link.hold(20, 0, 0))

    def test_split_stop_reply_is_parsed_only_after_newline(self):
        class SplitReplyFirmware(FakeFirmware):
            def readline(self):
                if self._rx.startswith(b"STOPPED command"):
                    first = bytes(self._rx[:11])
                    del self._rx[:11]
                    return first
                return super().readline()

        fake = SplitReplyFirmware()
        link = ChassisLink("FAKE", strict=True, serial_factory=lambda **_: fake, cooldown_s=.05)
        link.start()
        try:
            link.wait_ready(2)
            link.stop()
            self.assertTrue(wait_until(lambda: "STOPPED command" in link.events))
            self.assertTrue(wait_until(lambda: not link.busy))
            self.assertTrue(link.ready)
            self.assertIsNone(link.error)
        finally:
            link.close()

    def test_fast_release_shortens_only_the_hold_gate(self):
        link, fake, now = self.make_manual_link()
        link._phase = "CAPS"
        link._handle_line(HOLD_RELEASE_CAPS, now[0])
        link._phase = "READY"
        self.assertTrue(link.supports_hold_release)
        self.assertTrue(link.hold(20, 0, 0))
        link._service(now[0])
        link.stop(release=True)
        link._service(now[0])
        self.assertEqual(fake.sent[-1], "STOP HOLD")
        self.assertTrue(link.hold_restarting)
        now[0] = 1.01
        link._handle_line("STOPPED hold_release", now[0])
        now[0] = 1.069
        self.assertFalse(link.hold(-20, 0, 0))
        self.assertTrue(link.hold_busy)
        now[0] = 1.071
        self.assertFalse(link.hold_busy)
        self.assertFalse(link.pulse(20, 0, 0, 100))
        self.assertTrue(link.hold(-20, 0, 0))

    def test_manual_target_changes_bypass_cooldown_and_replace_pending_commands(self):
        link, fake, now = self.make_manual_link()
        link._phase = "CAPS"
        link._handle_line(MANUAL_CAPS, now[0])
        link._phase = "READY"
        link._next_ok = 10.0
        self.assertTrue(link.supports_manual)
        self.assertTrue(link.manual(20, 0, 0))
        link._service(now[0])
        link.renew_manual()
        self.assertTrue(link.manual(-20, 0, 0))
        link._service(now[0])
        self.assertTrue(link.manual(0, 0, 0))
        link._service(now[0])
        link.renew_manual()
        self.assertTrue(link.manual(0, 20, 0))
        link._service(now[0])
        link.renew_manual()
        link._service(now[0])
        self.assertEqual(fake.sent, ["MECANUM_MANUAL 20 0 0", "MECANUM_MANUAL -20 0 0",
                                     "MECANUM_MANUAL 0 0 0", "MECANUM_MANUAL 0 20 0", "MECANUM_KEEP"])

    def test_expired_manual_command_sends_neutral_instead_of_motion(self):
        link, fake, now = self.make_manual_link()
        link._supports_manual = True
        self.assertTrue(link.manual(20, 0, 0))
        now[0] += .16
        link._service(now[0])
        link._service(now[0])
        self.assertEqual(fake.sent, ["MECANUM_MANUAL 0 0 0"])
        self.assertFalse(link.manual_active)

    def test_emergency_stop_overrides_outstanding_release(self):
        link, fake, now = self.make_manual_link()
        link._supports_hold_release = True
        self.assertTrue(link.hold(20, 0, 0))
        link._service(now[0])
        link.stop(release=True)
        link._service(now[0])
        link.stop()
        link._service(now[0])
        self.assertEqual(fake.sent[-2:], ["STOP HOLD", "STOP"])
        link._handle_line("STOPPED hold_release", now[0])
        now[0] = 2.1
        self.assertFalse(link.hold_restarting)
        self.assertFalse(link.hold(20, 0, 0))
        link._handle_line("STOPPED command", now[0])
        self.assertFalse(link.hold(20, 0, 0))

    def test_old_firmware_receives_plain_stop_on_release(self):
        link, fake, now = self.make_manual_link()
        self.assertTrue(link.hold(20, 0, 0))
        link._service(now[0])
        link.stop(release=True)
        link._service(now[0])
        self.assertEqual(fake.sent[-1], "STOP")

    def test_manual_cooldown_rejection_does_not_retry_stale_input(self):
        link, fake, now = self.make_manual_link()
        self.assertTrue(link.hold(20, 0, 0))
        link._service(now[0])
        link._handle_line("ERROR wait 1 second after stop", now[0])
        self.assertFalse(link.holding)
        self.assertTrue(link.busy)
        now[0] = 2.1
        self.assertFalse(link.hold(20, 0, 0, renew=True))
        link._service(now[0])
        self.assertEqual(fake.sent, ["MECANUM_HOLD 20 0 0"])
        self.assertTrue(link.hold(0, 20, 0))
        with self.assertRaisesRegex(ChassisError, "FAULT last=stall"):
            link._handle_line("FAULT last=stall", now[0])

    def test_expired_hold_command_is_replaced_with_stop(self):
        link, fake, now = self.make_manual_link()
        self.assertTrue(link.hold(20, 0, 0))
        now[0] = 1.2
        link._service(now[0])
        self.assertFalse(link.holding)
        link._service(now[0])
        self.assertEqual(fake.sent, ["STOP"])

    def test_strict_link_distinguishes_health_report_from_a_fault(self):
        link, fake = self.make_link(strict=True, layout="CBDA 1 1 1 1")
        link.start()
        try:
            self.assertTrue(link.wait_ready(3.0))
            self.assertTrue(wait_until(lambda: "FAULT none" in link.events))
            self.assertTrue(wait_until(lambda: not link.busy))
            self.assertIsNone(link.error)
            self.assertTrue(link.pulse(20, 0, 0, 100))
            self.assertTrue(wait_until(lambda: "MECANUM 20 0 0 100" in fake.sent))
            fake._emit("FAULT last=stall")
            self.assertTrue(wait_until(lambda: fake.closed))
            self.assertEqual(link.error, "FAULT last=stall")
            self.assertFalse(link.ready)
            self.assertIn("STOP", fake.sent)
        finally:
            link.close()

    def test_pulse_sends_mecanum_and_blocks_second(self):
        link, fake = self.make_link()
        link.start()
        try:
            self.assertTrue(link.wait_ready(3.0))
            self.assertTrue(link.pulse(20, 10, 0, 120))
            self.assertTrue(wait_until(lambda: "MECANUM 20 10 0 120" in fake.sent))
            self.assertTrue(link.busy)
            self.assertFalse(link.pulse(10, 0, 0, 100))
            self.assertTrue(wait_until(lambda: not link.busy, 3.0))
            self.assertTrue(link.pulse(0, 0, 15, 100))
        finally:
            link.close()

    def test_stop_preempts_and_keeps_cooldown(self):
        link, fake = self.make_link()
        link.start()
        try:
            self.assertTrue(link.wait_ready(3.0))
            self.assertTrue(link.pulse(30, 0, 0, 400))
            self.assertTrue(wait_until(lambda: "MECANUM 30 0 0 400" in fake.sent))
            link.stop()
            self.assertTrue(wait_until(lambda: "STOP" in fake.sent))
            self.assertFalse(link.pulse(10, 0, 0, 100))
            self.assertTrue(wait_until(lambda: not link.busy, 3.0))
        finally:
            link.close()

    def test_bad_values_raise(self):
        link, _ = self.make_link()
        with self.assertRaises(ValueError):
            link.pulse(101, 0, 0, 100)
        with self.assertRaises(ValueError):
            link.pulse(10, 0, 0, 0)
        with self.assertRaises(ValueError):
            link.pulse(10.5, 0, 0, 100)

    def test_close_sends_stop_and_closes(self):
        link, fake = self.make_link()
        link.start()
        self.assertTrue(link.wait_ready(3.0))
        link.close()
        self.assertIn("STOP", fake.sent)
        self.assertTrue(fake.closed)
        self.assertFalse(link.ready)


class HandshakeFailureTest(unittest.TestCase):
    def test_unresponsive_device_times_out(self):
        class Silent:
            def write(self, data):
                return len(data)

            def readline(self):
                return b""

            def close(self):
                pass

        link = ChassisLink("FAKE", serial_factory=lambda **_: Silent(),
                           connect_timeout_s=0.2, heartbeat_s=0.05)
        link.start()
        try:
            with self.assertRaises(ChassisError):
                link.wait_ready(2.0)
        finally:
            link.close()


class IntegrationTest(unittest.TestCase):
    """链路 + 控制律：假固件在环，验证“一拍一停”的完整时序。"""

    def test_pulse_loop_reaches_target(self):
        from nav_servo import PoseServo, ServoConfig, SimChassis, plan_step

        config = ServoConfig(max_percent=30, min_pulse_ms=80, max_pulse_ms=200)
        servo = PoseServo(300, 260, 0, config)
        sim = SimChassis(150, 200, 90, config)
        fake = FakeFirmware()
        link = ChassisLink("FAKE", serial_factory=lambda **_: fake, heartbeat_s=0.05,
                           cooldown_s=0.02, connect_timeout_s=3.0)
        link.start()
        clock = 0.0
        try:
            self.assertTrue(link.wait_ready(3.0))
            for _ in range(60):
                step = plan_step(sim.pose, 0.0, clock, servo)
                if step.action != "pulse":
                    self.assertEqual(step.action, "done")
                    break
                command = step.command
                self.assertTrue(link.pulse(command.forward, command.left,
                                           command.ccw, command.ms))
                self.assertTrue(wait_until(
                    lambda: fake.sent[-1].startswith("MECANUM") and len(fake.sent) > 5, 2.0))
                sim.pulse(command.forward, command.left, command.ccw, command.ms)
                clock += command.ms / 1000.0
                self.assertTrue(wait_until(lambda: not link.busy, 2.0))
            else:
                self.fail("60 拍内未到达")
            error = math.hypot(300 - sim.pose.x_mm, 260 - sim.pose.y_mm)
            self.assertLessEqual(error, config.pos_tol_mm + 1e-6)
        finally:
            link.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
