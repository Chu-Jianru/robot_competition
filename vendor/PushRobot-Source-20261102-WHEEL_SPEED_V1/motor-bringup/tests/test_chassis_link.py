"""底盘握手、轮位与排队控制测试；只用假串口，不接硬件。"""
import pathlib
import sys
import time
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from chassis_link import ChassisLink
from test_motor_link import Peer, until


class ChassisPeer(Peer):
    CAPS2 = "CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI"

    def __init__(self, *, feature=True, profile="FOUR_V1", api="MECANUM_V2", cal_valid=1):
        super().__init__()
        self.feature, self.profile = feature, profile
        self.api = api
        self.cal_valid = cal_valid
        self.pid = (2.5, 1.2, 75)
        self.wheel_signs = {"A": 1, "B": -1, "C": 1, "D": 1}
        self.wheel_v100 = {"A": 4250.0, "B": 4300.0, "C": 4200.0, "D": 4280.0}
        self.wheel_deadband = {"A": 32.5, "B": 31.0, "C": 33.0, "D": 32.0}
        self.wheel_slope = {"A": 0.0160, "B": 0.0150, "C": 0.0165, "D": 0.0155}

    def cal_reply(self):
        lines = [f"CAL VALID={1 if self.cal_valid else 0} VMAX=3200.0 LIMIT=80"]
        for channel in "ABCD":
            lines.append(
                f"CALW {channel} SIGN={self.wheel_signs[channel]} "
                f"V100={self.wheel_v100[channel]:.1f} "
                f"DEADBAND={self.wheel_deadband[channel]:.1f} "
                f"SLOPE={self.wheel_slope[channel]:.4f}")
        return "\n".join(lines)

    def pid_reply(self):
        kp, ki, limit = self.pid
        return f"PID_OK KP={kp:.3f} KI={ki:.3f} LIMIT={limit}"

    def write(self, data):
        line = data.decode().strip()
        self.writes.append(line)
        reply = ""
        if line == "WHO": reply = "UNIFIED_ROBOT 1 mode=DRY_RUN"
        elif line == "MODE CHASSIS": reply = "MODE_OK CHASSIS STOPPED\nCHASSIS: stopped"
        elif line == "PING": reply = "PONG CHASSIS"
        elif line == "CAPS":
            reply = f"CAPS MOTORS=ABCD PINS={self.profile}"
            if self.feature:
                reply += f"\nCHASSIS_CAPS API={self.api} MIX=X UNITS=PERCENT"
                if self.api == "MECANUM_V2":
                    reply += " FORWARD=UNIFIED"
                    reply += "\n" + self.CAPS2
        elif line == "STOP": reply = "STOPPED command"
        elif line == "STATUS": reply = "STOPPED ALL\nENC A ticks=0 invalid=0 A=0 B=0"
        elif line.startswith("LAYOUT "):
            _, order, *signs = line.split()
            reply = f"LAYOUT CONFIGURED=1 ORDER={order} SIGNS={','.join(str(int(s)) for s in signs)} TYPE=X"
        elif line.startswith(("MECANUM ", "WHEELS ")): reply = "CHASSIS_RUN A=50 B=50 C=50 D=50 duration=500"
        elif line.startswith(("SPEED ", "SPEEDW ")): reply = "SPEED_RUN A=1600 B=-800 C=1600 D=-800 duration=500"
        elif line == "CAL": reply = "CAL START"
        elif line == "CAL?": reply = self.cal_reply()
        elif line == "CAL RESET":
            self.cal_valid = 0
            reply = "CAL RESET"
        elif line == "PID?": reply = self.pid_reply()
        elif line.startswith("PID "):
            _, kp, ki, limit = line.split()
            self.pid = (int(kp) / 1000, int(ki) / 1000, int(limit))
            reply = self.pid_reply()
        elif line == "TELEM 1": reply = "TELEM ON"
        elif line == "TELEM 0": reply = "TELEM OFF"
        elif line == "SAVE": reply = "SAVED"
        with self.lock:
            if reply: self.buffer.extend((reply+"\n").encode())
        return len(data)


class ChassisLinkTests(unittest.TestCase):
    def live(self, peer=None):
        peer = peer or ChassisPeer()
        client = ChassisLink(serial_factory=lambda **kwargs: peer)
        client.connect("FAKE")
        self.addCleanup(lambda: until(lambda: peer.closed))
        self.addCleanup(client.close)
        return client, peer

    def ready(self):
        client, peer = self.live()
        self.assertTrue(until(lambda: client.ready))
        return client, peer

    def closed_ready(self):
        client, peer = self.ready()
        self.assertTrue(until(lambda: client.cal_valid))
        self.assertTrue(until(lambda: client.pid_values is not None))
        return client, peer

    def offline(self, *, supported=True, valid=True, layout=True):
        client = ChassisLink()
        client._ready = True
        client.closed_loop_supported = supported
        client.cal_valid = valid
        client.layout_confirmed = layout
        self.addCleanup(client.close)
        return client

    def test_chassis_identity_and_no_automatic_layout_or_motion(self):
        client, peer = self.ready()
        self.assertIn("MODE CHASSIS", peer.writes)
        self.assertEqual(client.supported_channels, tuple("ABCD"))
        self.assertFalse(client.layout_confirmed)
        self.assertFalse(any(s.startswith(("LAYOUT ", "MECANUM ", "WHEELS ", "JOG ")) for s in peer.writes))

    def test_missing_feature_and_wrong_pins_cannot_enable(self):
        for peer in (ChassisPeer(feature=False), ChassisPeer(profile="OLD"), ChassisPeer(api="MECANUM_V1")):
            client, _ = self.live(peer)
            self.assertTrue(until(lambda: "STATUS" in peer.writes))
            time.sleep(0.08)
            self.assertFalse(client.ready)
            self.assertFalse(client.cal_valid)
            self.assertFalse(client.motion((10, 0, 0), 500))

    def test_mixed_motion_requires_exact_acknowledged_layout(self):
        client, peer = self.ready()
        with self.assertRaises(ValueError): client.motion((50,0,0),500)
        self.assertTrue(client.configure_layout("ADBC", (1,1,1,1)))
        self.assertTrue(until(lambda: client.layout_confirmed))
        self.assertTrue(client.motion((50,0,0),500))
        self.assertTrue(until(lambda: "MECANUM 50 0 0 500" in peer.writes))
        self.assertFalse(client.motion((0,50,0),500))

    def test_direct_four_wheel_and_global_stop(self):
        client, peer = self.ready()
        self.assertTrue(client.motion((50,-50,0,20),3000,mixed=False))
        self.assertTrue(until(lambda: "WHEELS 50 -50 0 20 3000" in peer.writes))
        self.assertTrue(client.send("STOP"))
        self.assertTrue(until(lambda: peer.writes.count("STOP")>=2))
        self.assertFalse(client.motion((50,50,50,50),100,mixed=False))

    def test_validation_and_terminal_cannot_bypass_control(self):
        client = ChassisLink()
        client._ready = True
        for order, signs in (("AAAA",(1,1,1,1)), ("ABC",(1,1,1,1)), ("ABCD",(1,1,1,0)), ("ADBC",(-1,-1,1,1))):
            with self.assertRaises(ValueError): client.configure_layout(order,signs)
        for values, duration in (((101,0,0),500), ((1,2),500), ((True,0,0),500), ((1,0,0),0), ((1,0,0),2**31)):
            with self.assertRaises(ValueError): client.motion(values,duration)
        for line in ("MECANUM 50 0 0 500", "MODE CHASSIS", "JOG A 50 500", "STOP\nMECANUM 1 0 0 10"):
            with self.assertRaises(ValueError): client.send_line(line)
        with self.assertRaises(ValueError): client.jog(50)
        self.assertFalse(client._commands)

    def test_wrong_ack_and_reset_revoke_layout(self):
        client = ChassisLink()
        client.pending_layout = ("ABCD",(1,1,1,1))
        client.handle_response("LAYOUT CONFIGURED=1 ORDER=DCBA SIGNS=1,1,1,1 TYPE=X")
        self.assertFalse(client.layout_confirmed)
        client.handle_response("LAYOUT CONFIGURED=1 ORDER=ABCD SIGNS=1,1,1,1 TYPE=X")
        self.assertTrue(client.layout_confirmed)
        client.handle_response("UNIFIED_BOOT 1 default=DRY_RUN outputs=STOPPED")
        self.assertFalse(client.layout_confirmed)
        self.assertIsNone(client.pending_layout)

    def test_stop_drops_unsent_motion_and_zero_is_stop(self):
        client = ChassisLink()
        client._ready = client.layout_confirmed = True
        self.assertTrue(client.motion((50,0,0),2147483647))
        self.assertEqual(list(client._commands), ["MECANUM 50 0 0 2147483647"])
        client.motion((0,0,0),1)
        self.assertEqual(list(client._commands), ["STOP"])
        client.close()
        self.assertFalse(client._commands)

    def test_closed_loop_capabilities_and_calibration_roundtrip(self):
        client, _ = self.closed_ready()
        self.assertTrue(client.closed_loop_supported)
        self.assertTrue(client.cal_valid)
        self.assertEqual(client.cal_vmax, 3200.0)
        self.assertEqual(client.cal_limit, 80)
        self.assertEqual(client.cal_wheels["A"]["sign"], 1)
        self.assertEqual(client.cal_wheels["B"]["sign"], -1)
        self.assertEqual(client.cal_wheels["A"]["v100"], 4250.0)
        # The firmware's DEADBAND/SLOPE fields are exposed as a/b.
        self.assertEqual(client.cal_wheels["A"]["a"], 32.5)
        self.assertEqual(client.cal_wheels["A"]["b"], 0.0160)
        self.assertEqual(client.pid_values, {"kp": 2.5, "ki": 1.2, "limit": 75})

        events = client.poll()
        kinds = [kind for kind, _ in events]
        self.assertIn("capabilities2", kinds)
        self.assertIn("cal_status", kinds)
        self.assertIn("pid", kinds)
        by_kind = dict(events)
        self.assertIs(by_kind["capabilities2"], True)
        self.assertEqual(by_kind["pid"], client.pid_values)
        self.assertEqual(by_kind["cal_status"],
                         {"valid": True, "vmax": 3200.0, "limit": 80})

    def test_speed_uses_speed_and_speedw_and_validates(self):
        client = self.offline()
        self.assertTrue(client.speed((50, 0, 0), 500, mixed=True))
        self.assertEqual(list(client._commands), ["SPEED 50 0 0 500"])
        client._commands.clear()
        client._next_jog = 0.0
        self.assertTrue(client.speed((50, 0, 50, 0), 500, mixed=False))
        self.assertEqual(list(client._commands), ["SPEEDW 50 0 50 0 500"])

        client._commands.clear()
        client._next_jog = 0.0
        self.assertTrue(client.speed((0, 0, 0), 500, mixed=True))
        self.assertEqual(list(client._commands), ["STOP"])

        for values, duration, mixed in (
                ((101, 0, 0), 500, True), ((1, 2), 500, True), ((True, 0, 0), 500, True),
                ((1, 0, 0), 0, True), ((1, 0, 0), 2 ** 31, True),
                ((1, 0, 0, 0), 500, True), ((1, 0, 0), 500, False)):
            with self.assertRaises(ValueError):
                client.speed(values, duration, mixed=mixed)

        with self.assertRaises(ValueError):
            self.offline(supported=False).speed((50, 0, 0), 500, mixed=True)
        with self.assertRaises(ValueError):
            self.offline(valid=False).speed((50, 0, 0), 500, mixed=True)
        no_layout = self.offline(layout=False)
        with self.assertRaises(ValueError):
            no_layout.speed((50, 0, 0), 500, mixed=True)
        self.assertTrue(no_layout.speed((50, 0, 50, 0), 500, mixed=False))

        cooldown = self.offline()
        cooldown._next_jog = time.monotonic() + 10
        self.assertFalse(cooldown.speed((50, 0, 0), 500, mixed=True))
        queued = self.offline()
        queued._commands.append("SPEED 10 0 0 100")
        self.assertFalse(queued.speed((50, 0, 0), 500, mixed=True))

    def test_speed_commands_reach_the_serial_link(self):
        client, peer = self.closed_ready()
        self.assertTrue(client.configure_layout("ABCD", (1, 1, 1, 1)))
        self.assertTrue(until(lambda: client.layout_confirmed))
        client._next_jog = 0.0
        self.assertTrue(client.speed((50, 0, 0), 500, mixed=True))
        self.assertTrue(until(lambda: "SPEED 50 0 0 500" in peer.writes))
        client._next_jog = 0.0
        self.assertTrue(client.speed((50, 0, 50, 0), 500, mixed=False))
        self.assertTrue(until(lambda: "SPEEDW 50 0 50 0 500" in peer.writes))

    def test_telemetry_fault_and_closed_loop_events(self):
        client = self.offline()
        client.handle_response("TLM 1000 4000 4100 4200 4300 3999 4099 4199 4299 60 61 62 63")
        client.handle_response("FAULT encoder_sign")
        client.handle_response("FAULT last=encoder_sign")  # STATUS echo is not a second event
        client.handle_response("CAL START")
        self.assertFalse(client.cal_valid)  # a calibration run invalidates the old result
        client.handle_response("CAL PROGRESS run 0 60")
        client.handle_response("CAL PROGRESS speed 1 2500.0")
        client.handle_response("CAL DONE VALID=1 VMAX=3600.0")
        client.handle_response("PID_OK KP=2.500 KI=1.200 LIMIT=75")

        events = client.poll()
        telemetry = [value for kind, value in events if kind == "telemetry"]
        self.assertEqual(telemetry, [
            (1000, 4000, 4100, 4200, 4300, 3999, 4099, 4199, 4299, 60, 61, 62, 63)])
        self.assertEqual(len(telemetry[0]), 13)
        self.assertEqual([value for kind, value in events if kind == "fault"], ["encoder_sign"])
        self.assertEqual([value for kind, value in events if kind == "cal_progress"],
                         ["start", "run 0 60", "speed 1 2500.0"])
        self.assertEqual([value for kind, value in events if kind == "cal_done"],
                         [{"valid": True, "vmax": 3600.0}])
        self.assertEqual([value for kind, value in events if kind == "pid"],
                         [{"kp": 2.5, "ki": 1.2, "limit": 75}])
        self.assertEqual(client.pid_values, {"kp": 2.5, "ki": 1.2, "limit": 75})
        self.assertTrue(client.cal_valid)
        self.assertEqual(client.cal_vmax, 3600.0)

        client.handle_response("FAULT none")
        client.handle_response("FAULT stall")
        self.assertEqual([value for kind, value in client.poll() if kind == "fault"], ["stall"])

        client.handle_response("TLM 1 2 3")
        self.assertFalse(any(kind == "telemetry" for kind, _ in client.poll()))

    def test_cal_fail_and_reset_update_validity(self):
        client = ChassisLink()
        client.cal_valid = True
        client.cal_vmax, client.cal_limit = 3200.0, 80
        client.handle_response("CAL FAIL no_feedback")
        self.assertFalse(client.cal_valid)
        self.assertIn(("cal_fail", "no_feedback"), client.poll())

        client.cal_valid = True
        client.handle_response("CAL RESET")
        self.assertFalse(client.cal_valid)
        self.assertIn(("cal_status", {"valid": False, "vmax": 3200.0, "limit": 80}), client.poll())

    def test_closed_loop_control_methods_and_terminal_whitelist(self):
        client = self.offline()
        self.assertTrue(client.calibrate())
        self.assertEqual(list(client._commands), ["CAL"])
        self.assertFalse(client.calibrate())  # calibrate honours the jog cooldown
        client._commands.clear()
        client._next_jog = 0.0

        self.assertTrue(client.cal_reset())
        self.assertTrue(client.request_cal())
        self.assertTrue(client.request_pid())
        self.assertTrue(client.save())
        self.assertEqual(list(client._commands), ["CAL RESET", "CAL?", "PID?", "SAVE"])
        client._commands.clear()

        self.assertTrue(client.set_pid(2500, 1200, 75))
        self.assertEqual(list(client._commands), ["PID 2500 1200 75"])
        client._commands.clear()
        self.assertTrue(client.telem(True))
        self.assertTrue(client.telem(False))
        self.assertEqual(list(client._commands), ["TELEM 1", "TELEM 0"])
        client._commands.clear()

        for line in ("PID?", "CAL?", "SAVE"):
            self.assertTrue(client.send_line(line))
        self.assertEqual(list(client._commands), ["PID?", "CAL?", "SAVE"])
        client._commands.clear()
        for line in ("CAL", "CAL RESET", "PID 2500 1200 75", "TELEM 1", "SPEED 50 0 0 500"):
            with self.assertRaises(ValueError):
                client.send_line(line)

        for args in ((-1, 0, 75), (10001, 0, 75), (0, -1, 75), (0, 10001, 75),
                     (0, 0, 9), (0, 0, 101), (True, 0, 75), (0, 0, "75")):
            with self.assertRaises(ValueError):
                client.set_pid(*args)
        with self.assertRaises(ValueError):
            client.telem(1)

    def test_calibration_pid_telemetry_and_save_roundtrips(self):
        client, peer = self.closed_ready()
        client.poll()  # drain handshake events

        self.assertTrue(client.calibrate())
        self.assertTrue(until(lambda: "CAL" in peer.writes))
        self.assertTrue(until(lambda: not client.cal_valid))
        self.assertTrue(until(lambda: any(
            kind == "cal_progress" and value == "start" for kind, value in client.poll())))

        self.assertTrue(client.set_pid(2500, 1200, 75))
        self.assertTrue(until(lambda: "PID 2500 1200 75" in peer.writes))
        self.assertTrue(until(lambda: any(kind == "pid" for kind, _ in client.poll())))
        self.assertEqual(client.pid_values, {"kp": 2.5, "ki": 1.2, "limit": 75})

        self.assertTrue(client.telem(True))
        self.assertTrue(until(lambda: "TELEM 1" in peer.writes))
        self.assertTrue(client.save())
        self.assertTrue(until(lambda: "SAVE" in peer.writes))
        self.assertTrue(until(lambda: any(
            kind == "rx" and value == "SAVED" for kind, value in client.poll())))

        self.assertTrue(client.cal_reset())
        self.assertTrue(until(lambda: "CAL RESET" in peer.writes))
        self.assertFalse(client.cal_valid)

    def test_boot_and_mode_ok_reset_closed_loop_state(self):
        client = ChassisLink()
        for line in ("UNIFIED_BOOT 1 default=DRY_RUN outputs=STOPPED", "MODE_OK CHASSIS STOPPED"):
            client.closed_loop_supported = True
            client.cal_valid = True
            client.cal_vmax = 3200.0
            client.cal_limit = 80
            client.cal_wheels = {"A": {"sign": 1}}
            client.pid_values = {"kp": 2.5, "ki": 1.2, "limit": 75}
            client._last_fault = "stall"
            client.handle_response(line)
            self.assertFalse(client.closed_loop_supported)
            self.assertFalse(client.cal_valid)
            self.assertEqual(client.cal_vmax, 0.0)
            self.assertEqual(client.cal_limit, 0)
            self.assertEqual(client.cal_wheels, {})
            self.assertIsNone(client.pid_values)
            self.assertIsNone(client._last_fault)

    def test_close_sends_telemetry_off_then_stop(self):
        client, peer = self.closed_ready()
        before = len(peer.writes)
        client.close()
        self.assertTrue(until(lambda: peer.closed))
        tail = peer.writes[before:]
        self.assertIn("TELEM 0", tail)
        self.assertIn("STOP", tail)
        self.assertLess(tail.index("TELEM 0"), tail.index("STOP"))

    def test_close_omits_telemetry_for_old_firmware(self):
        # Old firmware has no CHASSIS_CAPS2 and therefore no TELEM command.
        client, peer = self.live(ChassisPeer(feature=False))
        self.assertTrue(until(lambda: "CAPS" in peer.writes))
        with client._lock:
            client._ready = True
        before = len(peer.writes)
        client.close()
        self.assertTrue(until(lambda: peer.closed))
        tail = peer.writes[before:]
        self.assertIn("STOP", tail)
        self.assertNotIn("TELEM 0", tail)


if __name__ == "__main__": unittest.main()
