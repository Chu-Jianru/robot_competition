"""GUI behaviour tests with no serial connection, camera capture or motion."""
from pathlib import Path
import sys
import tempfile
import threading
import time
import tkinter as tk
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from main import Workspace
from motor_link import MotorLink
from link_client import LinkClient


class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.root = tk.Tk()
        self.root.report_callback_exception = self.callback_error
        self.errors = []
        self.workspace = Workspace(self.root, settings_path=Path(self.folder.name) / "settings.json")
        self.root.update_idletasks()

    def callback_error(self, kind, value, traceback):
        self.errors.append(value)

    def tearDown(self):
        if self.workspace.mode == "dry":
            self.workspace.app.on_close()
        else:
            self.workspace.app.shutdown()
            self.workspace.host.destroy()
        self.root.destroy()
        self.folder.cleanup()
        self.assertFalse(self.errors)

    def fake_ready(self):
        app = self.workspace.app
        app.client = MotorLink()  # Never started: cannot open a serial port.
        app.client._ready = True
        app.data.motor_state = "已停止"
        return app

    def test_starts_disconnected_and_motion_disabled(self):
        app = self.workspace.app
        self.assertIsNone(app.client)
        self.assertIsNone(app.camera)
        self.assertFalse(app.confirm.get())
        self.assertEqual(str(app.jog_button["state"]), "disabled")
        self.assertEqual(str(app.send_button["state"]), "disabled")

    def test_both_modes_preserved_and_switch_does_not_connect(self):
        old_host = self.workspace.host
        with patch.object(MotorLink, "connect") as real_connect, patch.object(LinkClient, "connect") as dry_connect:
            self.workspace.switch("dry")
            dry = self.workspace.app
            self.assertFalse(dry.armed)
            self.assertIsNone(dry.client)
            self.assertEqual(dry.camera_panel.frame_count, 0)
            self.assertEqual(len(dry.direction_buttons), 6)
            self.assertIsNone(dry.header)
            self.assertIn("摄像头", [dry.notebook.tab(tab, "text") for tab in dry.notebook.tabs()])
            self.assertFalse(old_host.timer_ids)
            self.workspace.switch("bench")
            self.assertIsNone(self.workspace.app.client)
            self.assertFalse(self.workspace.app.confirm.get())
            real_connect.assert_not_called()
            dry_connect.assert_not_called()

    def test_rounded_buttons_do_not_invoke_when_disabled(self):
        app = self.workspace.app
        called = []
        app.jog_button.configure(command=lambda: called.append("run"), state="disabled")
        app.jog_button.invoke()
        self.assertEqual(called, [])
        app.jog_button.configure(state="normal")
        app.jog_button.invoke()
        self.assertEqual(called, ["run"])

    def test_editable_seconds_history_and_stop(self):
        app = self.fake_ready()
        app.duty.set("60")
        app.duration.set("2.5")
        app.jog_parameters()
        self.assertFalse(app.client._commands)
        app.confirm.set(True)
        app.jog_parameters()
        self.assertEqual(list(app.client._commands), ["JOG 60 2500"])
        app.stop()
        self.assertEqual(list(app.client._commands), ["STOP"])
        app.command.set("ENC")
        app.send_command()
        previous = list(app.client._commands)
        app.history_move(-1)
        self.assertEqual(app.command.get(), "ENC")
        self.assertEqual(previous, list(app.client._commands))

    def test_switch_cancels_pending_motion_and_confirmation(self):
        app = self.fake_ready()
        app.confirm.set(True)
        app.submit("JOG 50 5000")
        client = app.client
        self.workspace.switch("dry")
        self.assertFalse(client.ready)
        self.assertFalse(client._commands)
        self.assertIsNone(self.workspace.app.client)

    def test_bench_observations_survive_mode_switch_but_live_state_does_not(self):
        app = self.fake_ready()
        app.data.receive("ENC ticks=120 invalid=0 A=0 B=1", time.monotonic())
        app.confirm.set(True)
        self.workspace.switch("dry")
        self.workspace.switch("bench")
        restored = self.workspace.app
        self.assertEqual(restored.data.samples[-1].ticks, 120)
        self.assertIsNone(restored.data.latest)
        self.assertIsNone(restored.client)
        self.assertFalse(restored.confirm.get())

    def test_switch_waits_for_old_serial_worker_to_exit(self):
        app = self.fake_ready()
        worker = Mock()
        worker.is_alive.return_value = True
        app.client._thread = worker
        self.workspace.switch("dry")
        self.assertTrue(self.workspace.transitioning)
        self.assertIsNone(self.workspace.app)
        worker.is_alive.return_value = False
        deadline = time.monotonic() + 0.5
        while self.workspace.transitioning and time.monotonic() < deadline:
            self.root.update()
            time.sleep(0.01)
        self.assertFalse(self.workspace.transitioning)
        self.assertEqual(self.workspace.mode, "dry")
        self.assertIsNone(self.workspace.app.client)

    def test_changing_units_preserves_duration(self):
        app = self.workspace.app
        app.duration.set("2.5")
        app.unit.set("毫秒")
        app.convert_unit()
        self.assertEqual(app.duration.get(), "2500")
        app.unit.set("秒")
        app.convert_unit()
        self.assertEqual(app.duration.get(), "2.5")

    def test_camera_tab_does_not_open_capture(self):
        app = self.workspace.app
        app.notebook.select(app.camera_tab)
        app.on_tab()
        self.assertIsNotNone(app.camera)
        self.assertIsNone(app.camera.thread)
        self.assertIsNone(app.camera.frame)

    def test_camera_config_can_be_saved_on_a_fresh_install(self):
        import json
        app = self.workspace.app
        app.notebook.select(app.camera_tab)
        app.on_tab()
        target = Path(self.folder.name) / "fresh-data" / "camera_config.json"
        with patch("camera_view.CONFIG", target):
            app.camera.source.set("2")
            app.camera.backend.set("auto")
            app.camera.save_config()
        saved = json.loads(target.read_text(encoding="utf-8"))
        self.assertEqual((saved["source"], saved["backend"]), (2, "auto"))
        self.assertIsNone(app.camera.thread)

    def test_camera_open_runs_in_background_and_can_be_cancelled(self):
        app = self.workspace.app
        app.notebook.select(app.camera_tab)
        app.on_tab()
        release, opened, closed = threading.Event(), threading.Event(), threading.Event()

        class FakeCapture:
            def __init__(self, *args, **kwargs):
                pass

            def open(self):
                opened.set()
                release.wait(1)
                return True

            def close(self):
                closed.set()

        with patch.dict(sys.modules, {"camera_source": SimpleNamespace(CameraSource=FakeCapture)}):
            started = time.monotonic()
            app.camera.start()
            self.assertLess(time.monotonic() - started, 0.2)
            self.assertTrue(opened.wait(0.5))
            app.camera.stop()
            release.set()
            app.camera.thread.join(timeout=1)
        self.assertTrue(closed.is_set())
        self.assertIsNone(app.client)

    def test_live_data_and_disconnect_clear_stale_values(self):
        app = self.fake_ready()
        now = time.monotonic()
        app.handle_event("rx", "ENC ticks=20 invalid=0 A=1 B=0", now)
        app.poll()
        self.assertEqual(app.ticks.get(), "20")
        app.handle_event("closed", None, now+1)
        self.assertEqual(app.ticks.get(), "—")
        self.assertEqual(app.rate.get(), "—")
        self.assertIsNone(app.data.latest)

    def test_channel_switch_stops_pending_motion_and_requires_new_confirmation(self):
        app = self.fake_ready()
        app.client.supported_channels = tuple("ABCD")
        app.confirm.set(True)
        self.assertTrue(app.submit("JOG A 50 5000"))
        app.channel.set("B")
        app.change_channel()
        self.assertEqual(list(app.client._commands), ["STOP"])
        self.assertFalse(app.confirm.get())
        self.assertEqual(app.data.channel, "B")
        self.assertEqual(app.encoder_title.get(), "B 路编码器")
        self.assertFalse(app.submit("JOG B 50 1000"))
        app.client._next_jog = 0
        app.confirm.set(True)
        self.assertTrue(app.submit("JOG 50 1000"))
        self.assertEqual(list(app.client._commands), ["STOP", "JOG B 50 1000"])
        self.assertFalse(app.submit("JOG C 50 1000"))

    def test_channels_keep_display_and_calibration_separate(self):
        app = self.fake_ready()
        app.client.supported_channels = tuple("ABCD")
        now = time.monotonic()
        app.handle_event("rx", "ENC A ticks=10 invalid=0 A=0 B=0", now)
        app.handle_event("rx", "ENC B ticks=200 invalid=0 A=1 B=0", now)
        app.ppr.set("100")
        app.apply_calibration()
        app.channel.set("B")
        app.change_channel()
        self.assertEqual(app.ppr.get(), "")
        app.ppr.set("400")
        app.apply_calibration()
        app.handle_event("rx", "ENC B ticks=300 invalid=0 A=0 B=0", now+0.1)
        app.poll()
        self.assertEqual(app.ticks.get(), "300")
        app.channel.set("A")
        app.change_channel()
        self.assertEqual(float(app.ppr.get()), 100)
        self.assertEqual(app.data.channels["B"].ppr, 400)

    def test_multiline_jog_cannot_be_normalized_into_a_valid_command(self):
        app = self.fake_ready()
        app.client.supported_channels = tuple("ABCD")
        app.confirm.set(True)
        self.assertFalse(app.submit("JOG\nA\n50\n1000"))
        self.assertFalse(app.client._commands)

    def test_primary_controls_fit_initial_window(self):
        app = self.workspace.app
        self.root.update()
        bottom = self.root.winfo_rooty() + self.root.winfo_height()
        for widget in (app.jog_button, app.stop_button):
            self.assertLess(widget.winfo_rooty() + widget.winfo_height(), bottom)
        app.notebook.select(app.terminal_tab)
        self.root.update()
        self.assertLess(app.send_button.winfo_rooty() + app.send_button.winfo_height(), bottom)
        self.assertGreater(app.command_entry.winfo_width(), 200)


if __name__ == "__main__":
    unittest.main(verbosity=2)
