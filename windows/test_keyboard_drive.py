"""Offline keyboard and STOP checks without a window or serial device."""

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from chassis_link import ChassisLink
from keyboard_drive import apply_key, control_window


class KeyboardDriveTest(unittest.TestCase):
    def test_movement_keys_send_single_axis_timed_commands(self):
        commands = {
            "w": (20, 0, 0), "s": (-20, 0, 0),
            "a": (0, 0, 20), "d": (0, 0, -20),
            "q": (0, 20, 0), "e": (0, -20, 0),
        }
        for key, expected in commands.items():
            with self.subTest(key=key), patch("keyboard_drive.time.monotonic", return_value=1.):
                link = Mock(spec=ChassisLink)
                self.assertEqual(apply_key(link, key, 20), 20)
                link.pulse.assert_called_once_with(*expected, 100, send_by=1.15)

    def test_power_adjustment_does_not_send_movement(self):
        link = Mock(spec=ChassisLink)
        self.assertEqual(apply_key(link, "z", 20), 25)
        self.assertEqual(apply_key(link, "x", 20), 15)
        self.assertEqual(apply_key(link, "z", 40), 45)
        self.assertEqual(apply_key(link, "z", 100), 100)
        self.assertEqual(apply_key(link, "x", 5), 5)
        link.pulse.assert_not_called()

    def test_window_ignores_repeat_and_stops_on_space_focus_loss_and_escape(self):
        root = Mock()
        handlers = {}
        root.bind.side_effect = handlers.__setitem__
        tk = SimpleNamespace(Tk=lambda: root, ttk=Mock(), StringVar=Mock())
        link = Mock(spec=ChassisLink)
        link.supports_manual = False
        link.supports_hold = False

        def play_keys():
            press = handlers["<KeyPress>"]
            release = handlers["<KeyRelease>"]
            forward = SimpleNamespace(keysym="W")
            space = SimpleNamespace(keysym="space")
            press(forward)
            press(forward)
            release(forward)
            press(space)
            press(forward)
            self.assertEqual(link.pulse.call_count, 1)
            handlers["<FocusOut>"](None)
            press(SimpleNamespace(keysym="Escape"))

        root.mainloop.side_effect = play_keys
        with patch.dict(sys.modules, {"tkinter": tk}):
            control_window(link)
        self.assertEqual(link.stop.call_count, 3)
        root.quit.assert_called_once()
        root.destroy.assert_called_once()

    def test_hold_renews_until_release_and_updates_power(self):
        root, handlers = Mock(), {}
        root.bind.side_effect = handlers.__setitem__
        tk = SimpleNamespace(Tk=lambda: root, ttk=Mock(), StringVar=Mock())
        link = Mock(spec=ChassisLink)
        link.supports_manual = False
        link.supports_hold, link.ready, link.error = True, True, None
        now = [1.0]

        def play_keys():
            refresh = root.after.call_args.args[1]
            forward = SimpleNamespace(keysym="W")
            handlers["<KeyPress>"](forward)
            link.hold.assert_called_once_with(80, 0, 0, send_by=None)
            handlers["<KeyPress>"](SimpleNamespace(keysym="z"))
            refresh()
            now[0] += 0.2
            handlers["<KeyRelease>"](forward)
            refresh()

        root.mainloop.side_effect = play_keys
        with (patch.dict(sys.modules, {"tkinter": tk}),
              patch("keyboard_drive.time.monotonic", side_effect=lambda: now[0])):
            control_window(link)
        self.assertEqual(link.hold.call_count, 2)
        link.hold.assert_called_with(85, 0, 0, renew=True)
        link.pulse.assert_not_called()
        link.stop.assert_called_once()

    def test_short_tap_moves_without_a_refresh_and_cooldown_taps_do_not_stop_again(self):
        root, handlers = Mock(), {}
        root.bind.side_effect = handlers.__setitem__
        tk = SimpleNamespace(Tk=lambda: root, ttk=Mock(), StringVar=Mock())
        link = Mock(spec=ChassisLink)
        link.supports_manual = False
        link.supports_hold = True
        link.supports_hold_release = False
        link.hold.side_effect = [True, False, False, True]

        def play_keys():
            press = handlers["<KeyPress>"]
            release = handlers["<KeyRelease>"]
            forward = SimpleNamespace(keysym="W")
            press(forward)
            release(forward)
            link.hold.assert_called_once_with(80, 0, 0, send_by=None)
            link.stop.assert_not_called()
            delay, finish = root.after.call_args.args
            self.assertEqual(delay, 100)
            finish()
            for _ in range(2):
                press(forward)
                release(forward)
            link.stop.assert_called_once()
            press(forward)
            self.assertEqual(link.hold.call_count, 4)

        root.mainloop.side_effect = play_keys
        with (patch.dict(sys.modules, {"tkinter": tk}),
              patch("keyboard_drive.time.monotonic", return_value=1.0)):
            control_window(link)

    def test_quick_release_keeps_only_the_latest_tap_and_respects_stop(self):
        for outcome in ("resume", "expire", "space"):
            with self.subTest(outcome=outcome):
                root, handlers = Mock(), {}
                root.bind.side_effect = handlers.__setitem__
                root.after.side_effect = (f"timer-{i}" for i in range(10))
                tk = SimpleNamespace(Tk=lambda: root, ttk=Mock(), StringVar=Mock())
                link = Mock(spec=ChassisLink)
                link.supports_manual = False
                link.supports_hold = link.supports_hold_release = True
                link.hold.side_effect = [True, False, False, True]
                now = [1.0]

                def play_keys():
                    press, release = handlers["<KeyPress>"], handlers["<KeyRelease>"]
                    forward = SimpleNamespace(keysym="W")
                    press(forward)
                    now[0] += 0.2
                    if outcome != "resume":
                        release(forward)
                    link.hold_restarting = True
                    for key in ("A", "D"):
                        event = SimpleNamespace(keysym=key)
                        press(event)
                        release(event)
                    if outcome == "resume":
                        release(forward)
                    link.stop.assert_called_once_with(release=True)
                    root.after_cancel.assert_called_once_with("timer-1")
                    delay, run, *args = root.after.call_args.args
                    self.assertEqual(delay, 10)
                    if outcome == "space":
                        press(SimpleNamespace(keysym="space"))
                        root.after_cancel.assert_called_with("timer-2")
                        link.stop.assert_called_with(release=False)
                        self.assertEqual(link.hold.call_count, 3)
                        return
                    now[0] += 0.06 if outcome == "resume" else 0.15
                    run(*args)
                    if outcome == "resume":
                        self.assertEqual(link.hold.call_count, 4)
                        link.hold.assert_called_with(0, 0, -80, send_by=args[1])
                        self.assertEqual(root.after.call_args.args[0], 100)
                    else:
                        self.assertEqual(link.hold.call_count, 3)

                root.mainloop.side_effect = play_keys
                with (patch.dict(sys.modules, {"tkinter": tk}),
                      patch("keyboard_drive.time.monotonic", side_effect=lambda: now[0])):
                    control_window(link)

    def test_space_and_focus_loss_cancel_pending_short_tap_stop_immediately(self):
        for event in ("space", "focus loss"):
            with self.subTest(event=event):
                root, handlers = Mock(), {}
                root.bind.side_effect = handlers.__setitem__
                root.after.return_value = "timer"
                tk = SimpleNamespace(Tk=lambda: root, ttk=Mock(), StringVar=Mock())
                link = Mock(spec=ChassisLink)
                link.supports_manual = False
                link.supports_hold = True

                def play_keys():
                    forward = SimpleNamespace(keysym="W")
                    handlers["<KeyPress>"](forward)
                    handlers["<KeyRelease>"](forward)
                    link.stop.assert_not_called()
                    if event == "space":
                        handlers["<KeyPress>"](SimpleNamespace(keysym="space"))
                    else:
                        handlers["<FocusOut>"](None)
                    root.after_cancel.assert_called_once_with("timer")
                    link.stop.assert_called_once()

                root.mainloop.side_effect = play_keys
                with (patch.dict(sys.modules, {"tkinter": tk}),
                      patch("keyboard_drive.time.monotonic", return_value=1.0)):
                    control_window(link)


    def test_manual_direction_changes_space_and_next_press_never_wait_for_refresh(self):
        root, handlers = Mock(), {}
        root.bind.side_effect = handlers.__setitem__
        tk = SimpleNamespace(Tk=lambda: root, ttk=Mock(), StringVar=Mock())
        link = Mock(spec=ChassisLink)
        link.supports_manual = True
        now = [1.0]

        def play_keys():
            press, release = handlers["<KeyPress>"], handlers["<KeyRelease>"]
            forward = SimpleNamespace(keysym="W")
            reverse = SimpleNamespace(keysym="S")
            space = SimpleNamespace(keysym="space")
            press(forward)
            press(reverse)
            release(reverse)  # Return to the still-held W immediately.
            now[0] += .2
            release(forward)
            press(reverse)  # Start again in the same tick as the stop.
            press(space)
            press(forward)  # Space held down, so ignore this direction.
            release(space)
            release(forward)
            press(forward)
            handlers["<FocusOut>"](None)

        root.mainloop.side_effect = play_keys
        with (patch.dict(sys.modules, {"tkinter": tk}),
              patch("keyboard_drive.time.monotonic", side_effect=lambda: now[0])):
            control_window(link)
        self.assertEqual([call.args for call in link.manual.call_args_list], [
            (80, 0, 0), (-80, 0, 0), (80, 0, 0), (0, 0, 0),
            (-80, 0, 0), (0, 0, 0), (80, 0, 0), (0, 0, 0),
        ])
        root.after.assert_called_once()  # Only the normal status/renewal timer.


if __name__ == "__main__":
    unittest.main()
