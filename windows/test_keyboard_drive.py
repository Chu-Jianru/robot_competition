"""Offline keyboard control and STOP checks without a window or serial device."""

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

from chassis_link import CHASSIS_CAPS, MANUAL_CAPS, SPEED_STREAM_CAPS, ChassisError, ChassisLink
from keyboard_drive import control_window, main, open_view, port_hint
from test_chassis_link import FakeFirmware


class KeyboardDriveTest(unittest.TestCase):
    def setUp(self):
        self.root = Mock()
        self.handlers = {}
        self.root.bind.side_effect = self.handlers.__setitem__
        self.root.after.return_value = "timer"
        self.tk = SimpleNamespace(Tk=lambda: self.root, ttk=Mock(), StringVar=Mock())
        self.link = Mock(spec=ChassisLink, speed_info=(6000, (-1, -1, 1, 1)),
                         ready=True, error=None)
        self.now = 1.0

    def play(self, keys, view=None, *, pwm=False):
        self.root.mainloop.side_effect = keys
        with (patch.dict(sys.modules, {"tkinter": self.tk}),
              patch("keyboard_drive.time.monotonic", side_effect=lambda: self.now)):
            control_window(self.link, view, pwm=pwm)

    def press(self, key):
        self.handlers["<KeyPress>"](SimpleNamespace(keysym=key))

    def release(self, key):
        self.handlers["<KeyRelease>"](SimpleNamespace(keysym=key))

    def test_each_direction_uses_one_axis_and_the_board_speed_limit(self):
        def keys():
            for key in "wsadqe":
                self.press(key)
                self.press(key)  # Ignore keyboard auto-repeat.

        self.play(keys)
        self.assertEqual([event.args for event in self.link.stream.call_args_list], [
            (4800, 0, 0), (-4800, 0, 0), (0, 0, 4800), (0, 0, -4800),
            (0, 4800, 0), (0, -4800, 0),
        ])
        self.link.manual.assert_not_called()
        self.link.pulse.assert_not_called()

    def test_newest_direction_wins_and_held_direction_resumes(self):
        def keys():
            self.press("W")
            self.press("S")
            self.release("S")
            self.now = 1.2
            self.release("W")

        self.play(keys)
        self.assertEqual(self.link.method_calls, [
            call.stream(4800, 0, 0, valid_until=1.25),
            call.stream(-4800, 0, 0, valid_until=1.25),
            call.stream(4800, 0, 0, valid_until=1.25), call.stop(),
        ])

    def test_speed_adjustment_renews_held_target_and_clamps_at_board_limit(self):
        def keys():
            refresh = self.root.after.call_args.args[1]
            for _ in range(20):
                self.press("z")
                self.release("z")
            self.link.stream.assert_not_called()
            self.press("W")
            self.link.stream.assert_called_once_with(6000, 0, 0, valid_until=1.25)
            self.now = 1.125
            self.press("X")
            self.link.stream.assert_called_with(5700, 0, 0, valid_until=1.375)
            self.now = 1.25
            refresh()
            self.link.stream.assert_called_with(5700, 0, 0, valid_until=1.5)
            self.release("W")
            refresh()
            self.assertEqual(self.link.stream.call_count, 3)

        self.play(keys)
        self.link.stop.assert_called_once()

    def test_short_tap_stops_without_renewing_after_release(self):
        def keys():
            refresh = self.root.after.call_args.args[1]
            self.press("W")
            self.release("W")
            delay, finish = self.root.after.call_args.args
            self.assertEqual(delay, 100)
            self.link.stop.assert_not_called()
            refresh()
            finish()
            refresh()

        self.play(keys)
        self.link.stream.assert_called_once_with(4800, 0, 0, valid_until=1.25)
        self.link.stop.assert_called_once()

    def test_space_focus_loss_and_escape_cancel_a_pending_tap(self):
        for event in ("space", "focus loss", "escape"):
            with self.subTest(event=event):
                self.setUp()

                def keys():
                    refresh = self.root.after.call_args.args[1]
                    self.press("W")
                    self.release("W")
                    if event == "focus loss":
                        self.handlers["<FocusOut>"](None)
                    else:
                        self.press(event)
                    if event == "space":
                        self.press("S")
                        self.release("space")
                    refresh()

                self.play(keys)
                self.root.after_cancel.assert_called_once_with("timer")
                self.link.stop.assert_called_once()
                self.assertEqual(self.link.stream.call_count, 1)
                self.assertEqual(self.root.quit.call_count, int(event == "escape"))

    def test_window_drives_real_link_with_stop_priority_and_waits_for_ack(self):
        self.link = ChassisLink("FAKE", clock=lambda: self.now, strict=True,
                                require_speed_stream=True, stream_pipeline=True, renew_stream=True)
        peer = FakeFirmware()
        self.link._device = peer
        self.link._phase = "CAPS"
        for line in (SPEED_STREAM_CAPS, CHASSIS_CAPS,
                     "LAYOUT CONFIGURED=1 ORDER=ADBC SIGNS=1,1,1,1 TYPE=X",
                     "SPEED_INFO 6000 -1 -1 1 1", "STOPPED ALL"):
            self.link._handle_line(line, self.now)

        def service():
            self.link._service(self.now)

        def keys():
            refresh = self.root.after.call_args.args[1]
            self.press("W")
            service()
            self.assertEqual(peer.sent[-1], "SPEED_STREAM 4800 0 0 150")
            self.now = 1.055
            service()  # Renew within the original input deadline, without a UI update.
            self.assertEqual(peer.sent[-2:], ["SPEED_STREAM 4800 0 0 150"] * 2)
            self.now = 1.125
            self.press("Z")  # Queue an update, then release before the serial thread writes it.
            self.release("W")
            self.press("S")  # No command may overtake STOP or its acknowledgement.
            service()
            self.assertEqual(peer.sent[-1], "STOP")
            refresh()
            service()
            self.assertEqual(peer.sent[-1], "STOP")
            self.link._handle_line("STOPPED command", self.now)
            self.now = 1.175
            refresh()
            service()
            self.assertEqual(peer.sent[-1], "SPEED_STREAM -5100 0 0 150")
            self.press("space")
            service()
            self.assertEqual(peer.sent[-1], "STOP")
            self.link._handle_line("STOPPED command", self.now)
            self.now = 1.30
            refresh()
            service()
            self.assertEqual(peer.sent[-1], "STOP")

        self.play(keys)

    def test_detection_view_is_attached_before_the_main_loop(self):
        view = Mock()

        def keys():
            view.attach.assert_called_once_with(self.root)
            self.press("Escape")

        self.play(keys, view)
        view.attach.assert_called_once_with(self.root)
        self.link.stop.assert_called_once()

    def test_main_requires_calibrated_speed_stream(self):
        with (patch.object(sys, "argv", ["keyboard_drive.py", "--camera"]),
              patch("keyboard_drive.ChassisLink", return_value=self.link) as link,
              patch("keyboard_drive.open_view", return_value=None),
              patch("keyboard_drive.control_window") as control):
            self.assertEqual(main(), 0)
        link.assert_called_once_with("COM6", layout="CBDA 1 1 1 1", strict=True,
                                     require_speed_stream=True, stream_pipeline=True, renew_stream=True)
        self.link.wait_ready.assert_called_once()
        control.assert_called_once_with(self.link, None, pwm=False)
        self.link.close.assert_called_once()

    def test_pwm_requires_manual_support_before_opening_window(self):
        self.link.supports_manual = False
        with (patch.object(sys, "argv", ["keyboard_drive.py", "--pwm", "--camera"]),
              patch("keyboard_drive.ChassisLink", return_value=self.link) as link,
              patch("keyboard_drive.open_view") as view,
              self.assertRaisesRegex(ChassisError, "CHASSIS_MANUAL")):
            main()
        link.assert_called_once_with("COM6", layout="CBDA 1 1 1 1", strict=True,
                                     require_speed_stream=False, stream_pipeline=False,
                                     renew_stream=False)
        view.assert_not_called()
        self.link.close.assert_called_once()

    def test_pwm_sends_full_duty_and_a_rapid_repress_cannot_overtake_stop(self):
        self.link = ChassisLink("FAKE", clock=lambda: self.now, strict=True)
        peer = FakeFirmware()
        self.link._device = peer
        self.link._phase = "CAPS"
        for line in (MANUAL_CAPS, CHASSIS_CAPS,
                     "LAYOUT CONFIGURED=1 ORDER=ADBC SIGNS=1,1,1,1 TYPE=X", "STOPPED ALL"):
            self.link._handle_line(line, self.now)

        def keys():
            refresh = self.root.after.call_args.args[1]
            self.press("W")
            self.link._service(self.now)
            self.assertEqual(peer.sent[-1], "MECANUM_MANUAL 80 0 0")
            for _ in range(5):
                self.press("Z")
                self.release("Z")
            self.link._service(self.now)
            self.assertEqual(peer.sent[-1], "MECANUM_MANUAL 100 0 0")
            self.now = 1.06
            refresh()
            self.link._service(self.now)
            self.assertEqual(peer.sent[-1], "MECANUM_KEEP")
            self.now = 1.125
            self.release("W")
            self.press("S")
            self.link._service(self.now)
            self.assertEqual(peer.sent[-1], "STOP")
            refresh()
            self.link._service(self.now)
            self.assertEqual(peer.sent[-1], "STOP")
            self.link._handle_line("STOPPED command", self.now)
            refresh()
            self.link._service(self.now)
            self.assertEqual(peer.sent[-1], "MECANUM_MANUAL -100 0 0")
            # A STOP from outside the window must not restart a previously held key.
            self.link._handle_line("STOPPED command", self.now)
            refresh()
            self.link._service(self.now)
            self.assertEqual(peer.sent[-1], "STOP")

        self.play(keys, pwm=True)

    def test_missing_camera_only_warns_and_keeps_keyboard_control(self):
        args = SimpleNamespace(scrcpy_dir=None, rotate="auto", calibration=None,
                               detector="auto")
        broken = SimpleNamespace(RemoteView=SimpleNamespace(
            open=Mock(side_effect=RuntimeError("没有找到手机"))))
        with patch.dict(sys.modules, {"remote_view": broken}):
            self.assertIsNone(open_view(args))

    def test_view_opens_with_the_camera_requested_by_the_menu(self):
        args = SimpleNamespace(scrcpy_dir="E:/scrcpy", rotate="90", calibration="E:/calib.json",
                               detector="onnx")
        opened = Mock()
        module = SimpleNamespace(RemoteView=SimpleNamespace(open=Mock(return_value=opened)))
        with patch.dict(sys.modules, {"remote_view": module}):
            self.assertIs(open_view(args), opened)
        module.RemoteView.open.assert_called_once_with(
            scrcpy_dir="E:/scrcpy", rotate="90", calibration="E:/calib.json", detector="onnx")

    def test_occupied_port_error_explains_how_to_release_it(self):
        message = ("could not open port 'COM6': PermissionError(13, '拒绝访问。', None, 5)")
        hint = port_hint(message)
        self.assertIn("ESP32", hint)
        self.assertIn("断电", hint)
        self.assertIsNone(port_hint("串口读取失败：device disconnected"))


if __name__ == "__main__":
    unittest.main()
