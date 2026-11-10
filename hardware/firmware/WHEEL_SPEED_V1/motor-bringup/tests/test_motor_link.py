import pathlib
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from motor_link import MotorLink


def until(predicate, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class Peer:
    def __init__(self, correct=True):
        self.correct = correct
        self.buffer = bytearray()
        self.writes = []
        self.lock = threading.Lock()
        self.closed = False
        self.fail = False

    @property
    def in_waiting(self):
        if self.fail:
            raise OSError("test disconnected")
        with self.lock:
            return len(self.buffer)

    def write(self, data):
        line = data.decode().strip()
        self.writes.append(line)
        reply = ""
        if line == "PING":
            reply = "PONG MOTOR_A_TEST" if self.correct else '{"v":1,"type":"pong"}'
        elif line == "STOP":
            reply = "STOPPED command"
        elif line == "STATUS":
            reply = "STOPPED A\nENC ticks=0 invalid=0 A=0 B=0"
        elif line.startswith("JOG "):
            reply = "JOG A 15 percent 300 ms; automatic stop enabled"
        elif line == "CUSTOM":
            reply = "custom diagnostic reply"
        with self.lock:
            if reply:
                self.buffer.extend((reply + "\n").encode())
        return len(data)

    def read(self, size):
        time.sleep(0.005)
        with self.lock:
            data = bytes(self.buffer[:size])
            del self.buffer[:size]
            return data

    def close(self):
        self.closed = True


class UnifiedPeer(Peer):
    def __init__(self, acknowledge=True):
        super().__init__()
        self.acknowledge = acknowledge

    def write(self, data):
        command = data.decode().strip()
        if command == "WHO" or command.startswith("MODE "):
            self.writes.append(command)
            reply = "UNIFIED_ROBOT 1 mode=DRY_RUN\n" if command == "WHO" else (
                "MODE_OK MOTOR_A_TEST STOPPED\nMOTOR_A_TEST: unified firmware; stopped by default.\n"
                if self.acknowledge else "MODE_OK DRY_RUN STOPPED\n")
            with self.lock:
                self.buffer.extend(reply.encode())
            return len(data)
        # Deliberately send PONG before MODE_OK as well: it must not enable UI.
        return super().write(data)


class WorkerTests(unittest.TestCase):
    def link(self, peer):
        client = MotorLink(serial_factory=lambda **kwargs: peer)
        client.connect("FAKE")
        self.addCleanup(lambda: until(lambda: peer.closed))
        self.addCleanup(client.close)
        return client

    def test_connect_checks_identity_and_stops_without_jog(self):
        peer = Peer()
        client = self.link(peer)
        self.assertTrue(until(lambda: client.ready))
        self.assertIn("STOP", peer.writes)
        self.assertIn("STATUS", peer.writes)
        self.assertFalse(any(line.startswith("JOG") for line in peer.writes))

    def test_unified_mode_confirmed_before_ready_and_no_automatic_jog(self):
        peer = UnifiedPeer()
        client = self.link(peer)
        self.assertTrue(until(lambda: client.ready))
        self.assertLess(peer.writes.index("MODE MOTOR_A_TEST"), peer.writes.index("STOP"))
        self.assertEqual(peer.writes.count("MODE MOTOR_A_TEST"), 1)
        self.assertFalse(any(line.startswith("JOG") for line in peer.writes))

    def test_wrong_mode_ack_cannot_be_bypassed_by_pong(self):
        peer = UnifiedPeer(acknowledge=False)
        client = self.link(peer)
        self.assertTrue(until(lambda: "MODE MOTOR_A_TEST" in peer.writes))
        time.sleep(0.15)
        self.assertFalse(client.ready)
        self.assertFalse(client.jog(50))
        self.assertNotIn("STOP", peer.writes)

    def test_other_transport_owner_fails_with_clear_error_and_no_motion(self):
        class OwnedPeer(UnifiedPeer):
            def write(self, data):
                if data.startswith(b"MODE "):
                    self.writes.append(data.decode().strip())
                    with self.lock:
                        self.buffer.extend(b"ERROR control_owned_by_other_port; disconnect other panel first\n")
                    return len(data)
                return super().write(data)
        peer = OwnedPeer()
        client = self.link(peer)
        self.assertTrue(until(lambda: peer.closed))
        self.assertFalse(client.ready)
        events = []
        while not client.events.empty():
            events.append(client.events.get())
        self.assertTrue(any(kind == "error" and "另一个 USB/蓝牙" in value for kind, value in events))
        self.assertNotIn("STOP", peer.writes)
        self.assertFalse(any(command.startswith("JOG ") for command in peer.writes))

    def test_terminal_cannot_change_runtime_mode(self):
        client = MotorLink()
        client._ready = True
        with self.assertRaises(ValueError):
            client.send_line("MODE DRY_RUN")
        self.assertEqual(list(client._commands), [])

    def test_four_channel_identity_and_explicit_motion(self):
        class FourPeer(UnifiedPeer):
            def write(self, data):
                command = data.decode().strip()
                if command == "CAPS":
                    self.writes.append(command)
                    with self.lock:
                        self.buffer.extend(b"CAPS MOTORS=ABCD PINS=FOUR_V1\n")
                    return len(data)
                return super().write(data)
        peer = FourPeer()
        client = MotorLink(serial_factory=lambda **kwargs: peer, require_profile="FOUR_V1")
        self.addCleanup(lambda: until(lambda: peer.closed))
        self.addCleanup(client.close)
        client.connect("FAKE")
        self.assertTrue(until(lambda: client.ready))
        self.assertEqual(client.profile, "FOUR_V1")
        self.assertEqual(client.supported_channels, tuple("ABCD"))
        self.assertFalse(any(line.startswith("JOG") for line in peer.writes))
        self.assertTrue(client.send_line("JOG d -100 5000"))
        self.assertTrue(until(lambda: "JOG D -100 5000" in peer.writes))
        self.assertTrue(client.send_line("ENC C"))
        self.assertTrue(client.send_line("ENCZERO B"))
        self.assertFalse(client.send_line("JOG A 50 100"))
        client.close()
        self.assertTrue(until(lambda: peer.closed))

    def test_old_pin_profile_cannot_enable_new_workbench(self):
        peer = UnifiedPeer()
        client = MotorLink(serial_factory=lambda **kwargs: peer, require_profile="FOUR_V1")
        client.connect("FAKE")
        self.addCleanup(client.close)
        self.assertTrue(until(lambda: "CAPS" in peer.writes))
        time.sleep(0.1)
        self.assertFalse(client.ready)
        self.assertFalse(client.jog(50))
        client.close()
        self.assertTrue(until(lambda: peer.closed))

    def test_four_channel_terminal_validation_and_stop_clears_all_pending_motion(self):
        client = MotorLink()
        client._ready = True
        client.supported_channels = tuple("ABCD")
        for command in ("JOG E 50 100", "JOG AB 50 100", "ENC E", "ENCZERO E", "JOG D 0 100", "JOG C 50 2147483648"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                client.send_line(command)
        self.assertTrue(client.jog(100, 5000, channel="C"))
        self.assertTrue(client.send("STOP"))
        self.assertEqual(list(client._commands), ["STOP"])

    def test_wrong_firmware_cannot_jog(self):
        peer = Peer(correct=False)
        client = self.link(peer)
        self.assertTrue(until(lambda: "PING" in peer.writes))
        time.sleep(0.05)
        self.assertFalse(client.ready)
        self.assertFalse(client.jog(15))
        self.assertNotIn("STOP", peer.writes)

    def test_one_click_is_one_bounded_pulse_and_cooldown(self):
        peer = Peer()
        client = self.link(peer)
        self.assertTrue(until(lambda: client.ready))
        self.assertTrue(client.jog(-30))
        self.assertFalse(client.jog(30))
        self.assertTrue(until(lambda: "JOG -30 300" in peer.writes))
        client.close()
        self.assertTrue(until(lambda: peer.closed))
        self.assertEqual([s for s in peer.writes if s.startswith("JOG")], ["JOG -30 300"])
        self.assertEqual(peer.writes[-1], "STOP")

    def test_stop_clears_unsent_jog_and_close_clears_queue(self):
        client = MotorLink()
        client._ready = True
        self.assertTrue(client.jog(15))
        self.assertTrue(client.send("STOP"))
        self.assertEqual(list(client._commands), ["STOP"])
        client.close()
        self.assertEqual(list(client._commands), [])
        self.assertFalse(client.ready)

    def test_limits_and_queries_cannot_inject_motion(self):
        client = MotorLink()
        for duty in (0, 101, -101, 1000):
            with self.assertRaises(ValueError):
                client.jog(duty)
        for text in ("JOG 30 500", "ENC\nJOG 25 300", "ARM"):
            with self.assertRaises(ValueError):
                client.send(text)

    def test_disconnect_clears_ready_and_pending(self):
        peer = Peer()
        client = self.link(peer)
        self.assertTrue(until(lambda: client.ready))
        peer.fail = True
        self.assertTrue(until(lambda: peer.closed))
        self.assertFalse(client.ready)
        self.assertFalse(client.jog(15))
        self.assertEqual(list(client._commands), [])

    def test_terminal_accepts_parameters_and_applies_same_cooldown(self):
        client = MotorLink()
        client._ready = True
        self.assertTrue(client.send_line("  jog +27 450  "))
        self.assertEqual(list(client._commands), ["JOG 27 450"])
        self.assertFalse(client.send_line("JOG -27 450"))
        self.assertTrue(client.send_line("stop"))
        self.assertEqual(list(client._commands), ["STOP"])
        self.assertFalse(client.send_line("JOG 20 300"))

    def test_terminal_rejects_multiple_lines_and_bad_motion(self):
        client = MotorLink()
        client._ready = True
        for text in ("", " ", "ENC\nJOG 30 300", "STOP\r", "\x1b", "编码器", "X" * 64,
                     "JOG 101 300", "JOG -101 300", "JOG 0 300", "JOG 20 0",
                     "JOG 20 -1", "JOG 20 2147483648", "JOG 20 999999999999999",
                     "JOG 20 300 extra", "JOG 2.5 300", "STOP extra"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                client.send_line(text)
        self.assertEqual(list(client._commands), [])

    def test_terminal_boundary_values_and_disconnected_state(self):
        client = MotorLink()
        self.assertFalse(client.send_line("PING"))
        self.assertFalse(client.send_line("JOG 30 500"))
        for duty, duration in ((1, 1), (-1, 100), (31, 300), (50, 5000), (60, 60000), (100, 500), (-100, 2147483647)):
            client = MotorLink()
            client._ready = True
            self.assertTrue(client.send_line(f"JOG {duty} {duration}"))
            self.assertEqual(list(client._commands), [f"JOG {duty} {duration}"])

    def test_terminal_returns_custom_device_output(self):
        peer = Peer()
        client = self.link(peer)
        self.assertTrue(until(lambda: client.ready))
        self.assertTrue(client.send_line("custom"))
        received = []

        def got_reply():
            received.extend(client.poll())
            return ("rx", "custom diagnostic reply") in received

        self.assertTrue(until(got_reply))
        self.assertIn("CUSTOM", peer.writes)

    def test_stop_cancels_long_duration_cooldown(self):
        client = MotorLink()
        client._ready = True
        with patch("motor_link.time.monotonic", return_value=100):
            self.assertTrue(client.send_line("JOG 50 60000"))
            self.assertTrue(client.send_line("STOP"))
            self.assertLess(client._next_jog, 102)
        with patch("motor_link.time.monotonic", return_value=101.2):
            self.assertTrue(client.send_line("JOG 50 5000"))
        self.assertEqual(list(client._commands), ["STOP", "JOG 50 5000"])

    def test_panel_maintains_heartbeat_during_long_output(self):
        peer = Peer()
        client = self.link(peer)
        self.assertTrue(until(lambda: client.ready))
        self.assertTrue(client.send_line("JOG 50 5000"))
        self.assertTrue(until(lambda: "JOG 50 5000" in peer.writes))
        before = peer.writes.count("STATUS")
        self.assertTrue(until(lambda: peer.writes.count("STATUS") > before, timeout=1.3))
        self.assertEqual([line for line in peer.writes if line.startswith("JOG ")], ["JOG 50 5000"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
