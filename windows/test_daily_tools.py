"""日常菜单的离线检查：障碍开关切换和菜单 13 的启动参数。"""

import io
import json
from contextlib import redirect_stdout
import unittest
from unittest.mock import patch

import daily_tools


class ObstacleSwitchTest(unittest.TestCase):
    def launch(self, enabled):
        with (patch.object(daily_tools, "obstacles_enabled", return_value=enabled),
              patch.object(daily_tools, "set_obstacles") as setter):
            out = io.StringIO()
            with redirect_stdout(out):
                status = daily_tools.launch("obstacles")
        return status, setter, out.getvalue()

    def test_toggle_off_to_on(self):
        status, setter, text = self.launch(False)
        self.assertEqual(status, 0)
        setter.assert_called_once_with(True)
        self.assertIn("已打开", text)

    def test_toggle_on_to_off(self):
        status, setter, text = self.launch(True)
        self.assertEqual(status, 0)
        setter.assert_called_once_with(False)
        self.assertIn("已关闭", text)

    def test_dry_run_only_reports_the_next_state(self):
        with patch.object(daily_tools, "obstacles_enabled", return_value=False):
            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(daily_tools.launch("obstacles", dry_run=True), 0)
        self.assertEqual(json.loads(out.getvalue()),
                         {"action": "obstacles", "obstacles": True})

    def test_menu_lists_the_switch_and_its_state(self):
        with patch.object(daily_tools, "obstacles_enabled", return_value=False):
            out = io.StringIO()
            with redirect_stdout(out):
                daily_tools.show_menu()
        text = out.getvalue()
        self.assertIn("14  切换障碍（对方车体）检测开关", text)
        self.assertIn("障碍检测 已关闭", text)


class KeyboardLaunchTest(unittest.TestCase):
    def test_keyboard_passes_live_camera_and_rotation(self):
        args = daily_tools.command("keyboard")
        self.assertIn("--camera", args)
        self.assertEqual(args[-2:], ["--rotate", "90"])
        self.assertIn(str(daily_tools.SCRCPY), args)

    def test_keyboard_mode_selection_reaches_the_script(self):
        for choice in ("", "2", "0"):
            with (self.subTest(choice=choice), patch("builtins.input", return_value=choice),
                  patch.object(daily_tools.subprocess, "run") as run, redirect_stdout(io.StringIO())):
                run.return_value.returncode = 0
                self.assertEqual(daily_tools.launch("keyboard"), 130 if choice == "0" else 0)
                if choice == "0":
                    run.assert_not_called()
                else:
                    self.assertEqual("--pwm" in run.call_args.args[0], choice == "2")


class CameraChannelTest(unittest.TestCase):
    SERIAL = "FMR0224424000187"

    def check(self, forwards=(), reverses=(), server=False, in_use=False):
        calls = []

        def fake_adb(*args):
            calls.append(args)
            if args == ("forward", "--list"):
                return "\n".join(forwards)
            if args[-2:] == ("reverse", "--list"):
                return "\n".join(reverses)
            return ""

        with (patch.object(daily_tools, "adb_output", side_effect=fake_adb),
              patch.object(daily_tools, "scrcpy_server_running", return_value=server),
              patch.object(daily_tools, "port_in_use", return_value=in_use)):
            out = io.StringIO()
            with redirect_stdout(out):
                available = daily_tools.camera_available(self.SERIAL)
        return available, calls, out.getvalue()

    def test_no_channel_is_available(self):
        available, _calls, text = self.check()
        self.assertTrue(available)
        self.assertEqual(text, "")

    def test_live_session_still_blocks(self):
        line = f"{self.SERIAL} tcp:1478 localabstract:scrcpy_45102a19"
        available, calls, text = self.check(forwards=[line], server=True, in_use=True)
        self.assertFalse(available)
        self.assertIn("手机已有 scrcpy 通道", text)
        self.assertNotIn("--remove", [arg for call in calls for arg in call])

    def test_leftover_forward_is_removed(self):
        line = f"{self.SERIAL} tcp:1478 localabstract:scrcpy_45102a19"
        available, calls, text = self.check(forwards=[line], server=False, in_use=False)
        self.assertTrue(available)
        self.assertIn("已清理上次取图留下的 scrcpy 通道", text)
        self.assertIn(("-s", self.SERIAL, "forward", "--remove", "tcp:1478"), calls)

    def test_unknown_process_list_keeps_blocking(self):
        line = f"{self.SERIAL} tcp:1478 localabstract:scrcpy_45102a19"
        available, _calls, text = self.check(forwards=[line], server=None, in_use=False)
        self.assertFalse(available)
        self.assertIn("手机已有 scrcpy 通道", text)

    def test_connected_client_keeps_blocking(self):
        line = f"{self.SERIAL} tcp:1478 localabstract:scrcpy_45102a19"
        available, _calls, _text = self.check(forwards=[line], server=False, in_use=True)
        self.assertFalse(available)

    def test_reverse_entries_still_block(self):
        available, _calls, text = self.check(
            reverses=[f"{self.SERIAL} localabstract:scrcpy tcp:27183"], server=False)
        self.assertFalse(available)
        self.assertIn("手机已有 scrcpy 通道", text)

    def test_other_devices_are_ignored(self):
        line = "OTHERSERIAL tcp:1478 localabstract:scrcpy_45102a19"
        available, _calls, _text = self.check(forwards=[line], server=False, in_use=False)
        self.assertTrue(available)


if __name__ == "__main__":
    unittest.main()
