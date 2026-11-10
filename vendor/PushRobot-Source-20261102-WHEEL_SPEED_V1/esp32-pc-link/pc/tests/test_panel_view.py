"""UI-state tests for the redesigned panel (no serial hardware)."""

from __future__ import annotations

import sys
import tkinter as tk
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from panel import PanelApp  # noqa: E402


class PanelStateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = PanelApp(self.root)

    def tearDown(self) -> None:
        self.root.destroy()

    def fill_stale_state(self) -> None:
        app = self.app
        app.connection_var.set("已就绪")
        app.fw_var.set("0.1.0 / v1")
        app.mode_var.set("DRY_RUN")
        app.esp_state_var.set("ARMED")
        app.session_var.set("123")
        app.seq_var.set("7")
        app.target_var.set("100 / 0 / 0")
        app.vx_var.set("100")
        app.vy_var.set("0")
        app.w_var.set("0")
        app.age_var.set("age 20 ms")
        app.reason_var.set("NONE")
        app.boot_var.set("42")
        app.response_var.set("12:00:00  status")

    def test_disconnect_event_clears_stale_display(self) -> None:
        self.fill_stale_state()
        self.app._handle_event("disconnected", "test unplug")
        app = self.app
        self.assertEqual(app.connection_var.get(), "未连接")
        self.assertEqual(app.fw_var.get(), "—")
        self.assertEqual(app.mode_var.get(), "—")
        self.assertEqual(app.esp_state_var.get(), "UNKNOWN")
        self.assertEqual(app.session_var.get(), "—")
        self.assertEqual(app.seq_var.get(), "—")
        self.assertEqual(app.target_var.get(), "—")
        self.assertEqual(app.vx_var.get(), "—")
        self.assertEqual(app.vy_var.get(), "—")
        self.assertEqual(app.w_var.get(), "—")
        self.assertEqual(app.age_var.get(), "—")
        self.assertEqual(app.reason_var.get(), "—")
        self.assertEqual(app.boot_var.get(), "—")
        self.assertEqual(app.response_var.get(), "—")
        self.assertIsNone(app.client)

    def test_reboot_event_clears_session_display(self) -> None:
        self.fill_stale_state()
        self.app._handle_event(
            "reboot", {"old_boot_id": 42, "new_boot_id": 43}
        )
        app = self.app
        self.assertEqual(app.esp_state_var.get(), "UNKNOWN")
        self.assertEqual(app.session_var.get(), "—")
        self.assertEqual(app.seq_var.get(), "—")
        self.assertEqual(app.target_var.get(), "—")
        self.assertEqual(app.vx_var.get(), "—")
        self.assertEqual(app.vy_var.get(), "—")
        self.assertEqual(app.w_var.get(), "—")
        self.assertEqual(app.age_var.get(), "—")
        self.assertEqual(app.reason_var.get(), "—")

    def test_periodic_status_is_not_logged_unless_verbose(self) -> None:
        app = self.app
        app.verbose_var.set(False)
        app.clear_log()
        app._handle_event(
            "log",
            {
                "dir": "rx",
                "text": '{"v":1,"type":"status","state":"DISARMED",'
                '"vx":0,"vy":0,"w":0}',
            },
        )
        self.assertEqual(app.log_text.get("1.0", "end-1c").strip(), "")
        self.assertIn("status", app.response_var.get())
        self.assertIn("DISARMED", app.response_var.get())

        app.verbose_var.set(True)
        app._handle_event(
            "log",
            {"dir": "rx", "text": '{"v":1,"type":"pong","state":"DISARMED"}'},
        )
        self.assertIn("pong", app.log_text.get("1.0", "end-1c"))

    def test_camera_tab_exists_without_starting(self) -> None:
        tabs = [
            self.app.notebook.tab(tab, "text")
            for tab in self.app.notebook.tabs()
        ]
        self.assertIn("摄像头", tabs)
        self.assertIsNotNone(self.app.camera_panel)
        self.assertEqual(self.app.camera_panel.frame_count, 0)

    def test_key_error_is_logged_by_default(self) -> None:
        app = self.app
        app.verbose_var.set(False)
        app.clear_log()
        app._handle_event("arm_failed", {"error": "STALE_ARM_TOKEN"})
        self.assertIn("ARM 失败", app.log_text.get("1.0", "end-1c"))
        self.assertIn("STALE_ARM_TOKEN", app.message_var.get())

    def test_motor_firmware_reports_mode_mismatch_even_without_verbose_log(self) -> None:
        self.app.verbose_var.set(False)
        self.app._handle_event("log", {"dir": "rx", "text": "MOTOR_A_TEST: real GPIO outputs; stopped by default."})
        self.assertIsNone(self.app.client)
        self.assertEqual(self.app.connection_var.get(), "固件模式不匹配")
        self.assertIn("真实电机", self.app.message_var.get())
        self.assertIn("MOTOR_A_TEST", self.app.log_text.get("1.0", "end-1c"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
