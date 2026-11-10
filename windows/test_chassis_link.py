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

from chassis_link import CHASSIS_CAPS, ChassisError, ChassisLink  # noqa: E402


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
            self._emit("LAYOUT CONFIGURED=1 ORDER=ADBC SIGNS=1,1,1,1 TYPE=X")
        elif line == "STATUS":
            self._emit("RUNNING CHASSIS" if self.running else "STOPPED ALL")
            self._emit("ENC A ticks=0 invalid=0 A=0 B=0")
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
