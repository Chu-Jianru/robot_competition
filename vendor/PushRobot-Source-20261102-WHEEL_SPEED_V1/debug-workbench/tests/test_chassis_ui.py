"""新增底盘页面的离线测试，不打开硬件端口或相机。"""
from pathlib import Path
import sys
import tempfile
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from main import Workspace
from chassis_link import ChassisLink


class ChassisUiTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.root = tk.Tk()
        self.errors = []
        self.root.report_callback_exception = lambda *args: self.errors.append(str(args[1]))
        self.workspace = Workspace(self.root, mode="chassis", settings_path=Path(self.folder.name)/"settings.json")
        self.app = self.workspace.app
        self.root.update_idletasks()

    def tearDown(self):
        if self.workspace.mode == "dry": self.workspace.app.on_close()
        else:
            self.workspace.app.shutdown()
            self.workspace.host.destroy()
        self.root.destroy()
        self.folder.cleanup()
        self.assertFalse(self.errors)

    def ready(self, *, layout=False):
        self.app.client = ChassisLink()
        self.app.client._ready = True
        self.app.client.layout_confirmed = layout
        return self.app.client

    def enable_closed_loop(self, client):
        client.handle_response("CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI")
        client.handle_response("CAL VALID=1 VMAX=4200.0 LIMIT=80")
        self.app.pump()
        self.app.x_layout.set(True)
        self.app.confirm.set(True)
        self.app.update_controls()

    def commands(self, client):
        return list(client._commands)

    def test_starts_without_connection_or_motion(self):
        self.assertIsNone(self.app.client)
        self.assertFalse(self.app.confirm.get())
        self.assertFalse(self.app.x_layout.get())
        self.assertEqual("".join(v.get() for v in self.app.channels), "CBAD")
        self.assertEqual([int(v.get()) for v in self.app.signs], [1,1,1,1])
        self.assertTrue(all(str(b["state"]) == "disabled" for b in self.app.motion_buttons))

    def test_layout_requires_explicit_x_check_and_ack(self):
        client = self.ready()
        self.app.apply_layout()
        self.assertFalse(client._commands)
        self.app.x_layout.set(True)
        self.app.apply_layout()
        self.assertEqual(list(client._commands), ["LAYOUT CBAD 1 1 1 1"])
        self.assertFalse(client.layout_confirmed)
        client.handle_response("LAYOUT CONFIGURED=1 ORDER=CBAD SIGNS=1,1,1,1 TYPE=X")
        self.assertTrue(client.layout_confirmed)

    def test_direction_requires_confirmation_and_sends_timed_vector(self):
        client = self.ready(layout=True)
        self.app.x_layout.set(True)
        self.app.direction((0,1,0))
        self.assertFalse(client._commands)
        self.app.confirm.set(True)
        self.app.direction((0,1,0))
        self.assertEqual(list(client._commands), ["MECANUM 0 50 0 500"])
        self.assertTrue(self.app.pending_run)
        self.app.direction((1,0,0))
        self.assertEqual(len(client._commands),1)
        self.app.stop()
        commands = list(client._commands)
        self.assertEqual(commands[0], "STOP")
        # STOP 优先并丢弃尚未发出的运动指令。
        self.assertNotIn("MECANUM 0 50 0 500", commands)
        # 旧固件没有 CHASSIS_CAPS2：不发送 TELEM/CAL? 等闭环命令。
        self.assertNotIn("TELEM 0", commands)
        self.assertFalse(self.app.confirm.get())

    def test_direct_wheels_do_not_assume_layout(self):
        client = self.ready()
        self.app.confirm.set(True)
        for variable, value in zip(self.app.wheels,(50,-50,0,25)): variable.set(str(value))
        self.app.run_wheels()
        self.assertEqual(list(client._commands),["WHEELS 50 -50 0 25 500"])

    def test_editing_layout_stops_and_revokes_confirmation(self):
        client = self.ready(layout=True)
        self.app.confirm.set(True)
        self.app.invalidate_layout()
        self.assertFalse(client.layout_confirmed)
        self.assertFalse(self.app.confirm.get())
        self.assertEqual(list(client._commands),["STOP"])

    def test_mode_switch_cancels_motion_and_all_three_modes_stay_available(self):
        client = self.ready()
        self.app.confirm.set(True)
        self.app.wheels[0].set("50")
        self.app.run_wheels()
        with patch.object(ChassisLink,"connect") as connect:
            for mode in ("bench","dry","chassis"):
                self.workspace.switch(mode)
                self.root.update_idletasks()
                self.assertIsNone(self.workspace.app.client)
            connect.assert_not_called()
        self.assertFalse(client.ready)
        self.assertFalse(client._commands)
        self.assertFalse(self.workspace.app.confirm.get())

    def test_pending_command_is_not_running_and_encoder_channels_are_separate(self):
        client = self.ready()
        self.app.confirm.set(True)
        self.app.wheels[0].set("50")
        self.app.run_wheels()
        self.assertFalse(self.app.device_running)
        client.events.put(("rx","CHASSIS_RUN A=50 B=0 C=0 D=0 duration=500"))
        client.events.put(("rx","ENC D ticks=-123 invalid=2 A=0 B=1"))
        self.app.pump()
        self.assertTrue(self.app.device_running)
        self.assertEqual(self.app.encoder_values,{"D":-123})
        client.events.put(("rx","STOPPED timer"))
        self.app.pump()
        self.assertFalse(self.app.device_running)

    def test_reset_and_disconnect_clear_encoder_and_confirmation(self):
        client = self.ready(layout=True)
        self.app.confirm.set(True)
        client.events.put(("rx","ENC B ticks=456 invalid=0 A=1 B=0"))
        self.app.pump()
        self.assertEqual(self.app.encoder_values,{"B":456})
        client.events.put(("reset","设备重启"))
        self.app.pump()
        self.assertFalse(self.app.encoder_values)
        self.assertIsNone(self.app.last_reply)
        self.assertFalse(self.app.confirm.get())
        self.app.encoder_values["D"]=123
        self.app.disconnect()
        self.assertFalse(self.app.encoder_values)

    def test_closed_loop_checkbox_requires_capability_and_calibration(self):
        client = self.ready(layout=True)
        self.app.confirm.set(True)
        self.app.update_controls()
        self.assertTrue(self.app.closed_loop_box.instate(["disabled"]))
        self.app.closed_loop.set(True)
        self.app.update_controls()
        self.assertFalse(self.app.closed_loop.get())
        client.handle_response("CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI")
        self.app.pump()
        self.assertTrue(self.app.closed_loop_box.instate(["disabled"]))
        self.app.closed_loop.set(True)
        self.app.update_controls()
        self.assertFalse(self.app.closed_loop.get())
        client.handle_response("CAL VALID=1 VMAX=4200.0 LIMIT=80")
        self.app.pump()
        self.assertFalse(self.app.closed_loop_box.instate(["disabled"]))
        self.app.closed_loop.set(True)
        self.app.update_controls()
        self.assertTrue(self.app.closed_loop.get())
        self.assertIn("VMAX=4200.0", self.app.closed_status.get())

    def test_closed_loop_direction_uses_speed_and_open_loop_keeps_mecanum(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        self.app.power.set("100")
        self.app.closed_loop.set(True)
        self.app.update_controls()
        self.app.direction((1, 0, 0))
        self.assertIn("SPEED 100 0 0 500", self.commands(client))
        client._commands.clear()  # 模拟串口线程已把上一条发出
        self.app.pending_run = False
        client._next_jog = 0
        self.app.closed_loop.set(False)
        self.app.update_controls()
        self.app.direction((1, 0, 0))
        self.assertEqual(self.commands(client)[-1], "MECANUM 100 0 0 500")
        client._commands.clear()
        self.app.pending_run = False
        client._next_jog = 0
        self.app.closed_loop.set(True)
        self.app.update_controls()
        for variable, value in zip(self.app.axes, ("20", "-40", "0")):
            variable.set(value)
        self.app.run_axes()
        self.assertIn("SPEED 20 -40 0 500", self.commands(client))
        client._commands.clear()
        self.app.pending_run = False
        client._next_jog = 0
        for variable, value in zip(self.app.axes, ("3", "-3", "0")):
            variable.set(value)
        self.app.run_axes()
        # 混控后的每轮目标由固件判定，面板不预判 <5% 的 x/y/z 分量。
        self.assertEqual(self.commands(client)[-1], "SPEED 3 -3 0 500")

    def test_closed_loop_rejects_below_five_percent(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        self.app.closed_loop.set(True)
        self.app.update_controls()
        self.app.power.set("4")
        self.app.direction((1, 0, 0))
        self.assertFalse(any(text.startswith(("SPEED ", "SPEEDW ")) for text in self.commands(client)))
        self.assertIn("闭环最低力度 5%", self.app.state.get())
        for variable, value in zip(self.app.wheels, (4, 0, 0, 0)):
            variable.set(str(value))
        self.app.run_wheels()
        self.assertFalse(any(text.startswith(("SPEED ", "SPEEDW ")) for text in self.commands(client)))
        self.assertIn("闭环最低力度 5%", self.app.state.get())
        self.app.wheels[0].set("5")
        self.app.run_wheels()
        self.assertIn("SPEEDW 5 0 0 0 500", self.commands(client))

    def test_calibration_requires_confirmation_and_disables_motion(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        with patch("chassis_app.messagebox.askyesno", return_value=False) as ask:
            self.app.start_calibration()
        ask.assert_called_once()
        self.assertNotIn("CAL", client._commands)
        with patch("chassis_app.messagebox.askyesno", return_value=True):
            self.app.start_calibration()
        self.assertIn("CAL", client._commands)
        self.assertTrue(self.app.calibrating)
        self.assertTrue(all(button["state"] == "disabled" for button in self.app.motion_buttons))
        client.handle_response("CAL PROGRESS measure A")
        self.app.pump()
        self.assertIn("CAL PROGRESS", self.app.cal_progress.get())
        client.handle_response("CAL FAIL encoder timeout")
        self.app.pump()
        self.assertIn("encoder timeout", self.app.cal_progress.get())
        self.assertFalse(self.app.calibrating)
        self.assertTrue(self.app.closed_loop_box.instate(["disabled"]))
        self.assertFalse(self.app.closed_loop.get())
        client._commands.clear()
        client.handle_response("CAL DONE VALID=1 VMAX=4000.0")
        self.app.pump()
        self.assertIn("CAL?", client._commands)

    def test_pid_round_trip_apply_and_save(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        self.app.kp.set("2.5")
        self.app.ki.set("1.2")
        self.app.limit.set("75")
        self.app.apply_pid()
        self.assertIn("PID 2500 1200 75", client._commands)
        client._commands.clear()
        client.handle_response("PID_OK KP=2.500 KI=1.200 LIMIT=75")
        self.app.pump()
        self.assertEqual(self.app.kp.get(), "2.500")
        self.assertEqual(self.app.ki.get(), "1.200")
        self.assertEqual(self.app.limit.get(), "75")
        self.assertIn("CAL?", client._commands)
        client._commands.clear()
        self.app.save_params()
        self.assertIn("SAVE", client._commands)
        self.app.limit.set("75.5")
        self.app.apply_pid()
        self.assertFalse(any(text.startswith("PID ") for text in client._commands))
        self.app.kp.set("abc")
        self.app.apply_pid()
        self.assertFalse(any(text.startswith("PID ") for text in client._commands))

    def test_telemetry_updates_table_without_logging(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        self.app.notebook.select(self.app.cal_tab)
        self.app.update_controls()
        self.assertIn("TELEM 1", client._commands)
        self.app.log_lines.clear()
        line = "TLM 1234 100 200 300 400 90 190 290 390 10 20 30 40"
        client.events.put(("rx", line))
        client.handle_response(line)
        self.app.pump()
        for channel, target, actual, duty in (
                ("A", 100, 90, 10), ("B", 200, 190, 20),
                ("C", 300, 290, 30), ("D", 400, 390, 40)):
            values = [str(value) for value in self.app.telemetry_table.item(channel)["values"]]
            self.assertIn(str(target), values)
            self.assertIn(str(actual), values)
            self.assertIn(f"{duty}%", values)
        self.assertFalse(any("TLM " in text for text in self.app.log_lines))
        self.assertNotIn("TLM ", self.app.log_widget.get("1.0", "end"))
        self.app.notebook.select(self.app.mecanum_tab)
        self.app.update_controls()
        self.assertIn("TELEM 0", client._commands)

    def test_fault_latches_motion_until_stop(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        client.handle_response("FAULT encoder_sign")
        self.app.pump()
        self.assertTrue(self.app.fault_latched)
        self.assertTrue(all(button["state"] == "disabled" for button in self.app.motion_buttons))
        self.assertIn("encoder_sign", self.app.state.get())
        # 0.7 s 空闲 STATUS 也回 STOPPED ALL；它不能解除故障闭锁。
        client.events.put(("rx", "STOPPED ALL"))
        client.events.put(("rx", "FAULT last=encoder_sign"))
        self.app.pump()
        self.assertTrue(self.app.fault_latched)
        self.assertTrue(all(button["state"] == "disabled" for button in self.app.motion_buttons))
        self.assertIn("encoder_sign", self.app.state.get())
        self.app.stop()
        self.assertEqual(self.commands(client)[0], "STOP")
        self.assertIn("CAL?", client._commands)
        self.assertFalse(self.app.fault_latched)
        self.app.cooldown_until = 0.0
        self.app.confirm.set(True)
        self.app.update_controls()
        self.assertTrue(all(button["state"] == "normal" for button in self.app.motion_buttons))

    def test_ready_event_does_not_duplicate_queries(self):
        # 真实顺序：先见 CHASSIS_CAPS2（客户端自动排队 CAL?/PID?），之后才 ready。
        client = ChassisLink()
        self.app.client = client
        client.handle_response("CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI")
        self.assertEqual(self.commands(client), ["CAL?", "PID?"])
        client._commands.clear()  # 串口线程已经把自动查询发出
        client._ready = True
        client.events.put(("ready", "COM9"))
        self.app.pump()
        self.assertNotIn("CAL?", client._commands)
        self.assertNotIn("PID?", client._commands)

    def test_old_firmware_receives_no_closed_loop_queries(self):
        client = self.ready()
        self.app.confirm.set(True)
        self.app.direction((0, 1, 0))
        self.app.stop()
        self.assertEqual(self.commands(client), ["STOP"])
        client.events.put(("rx", "ERROR unknown_command"))
        self.app.pump()
        self.assertNotIn("CAL?", client._commands)
        self.assertNotIn("TELEM 0", client._commands)

    def test_link_loss_clears_closed_status(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        self.assertIn("闭环已标定", self.app.closed_status.get())
        client._ready = False
        client.events.put(("closed", None))
        self.app.pump()
        self.assertEqual(self.app.closed_status.get(), "未连接 · 闭环不可用")

    def test_stop_turns_telemetry_off_and_sends_stop_first(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        self.app.closed_loop.set(True)
        self.app.update_controls()
        self.assertIn("TELEM 1", client._commands)
        self.assertTrue(self.app.telem_on)
        self.app.stop()
        commands = self.commands(client)
        self.assertEqual(commands[0], "STOP")
        self.assertIn("TELEM 0", commands)
        self.assertFalse(self.app.telem_on)
        self.app.update_controls()
        self.assertNotIn("TELEM 1", self.commands(client))

    def test_idle_stopped_all_keeps_user_messages(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        self.app.closed_loop.set(True)
        self.app.update_controls()
        self.app.power.set("4")
        self.app.direction((1, 0, 0))
        message = self.app.state.get()
        self.assertIn("闭环最低力度 5%", message)
        client.events.put(("rx", "STOPPED ALL"))  # 0.7 s idle STATUS poll
        self.app.pump()
        self.assertEqual(message, self.app.state.get())

        client.handle_response("CAL FAIL bad_line 1")
        self.app.pump()
        cal_message = self.app.state.get()
        self.assertIn("标定失败", cal_message)
        client.events.put(("rx", "STOPPED ALL"))
        self.app.pump()
        self.assertEqual(cal_message, self.app.state.get())

    def test_fault_status_lines_log_only_on_change(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        self.app.log_lines.clear()
        for _ in range(4):
            client.events.put(("rx", "FAULT last=encoder_sign"))
            self.app.pump()
        fault_lines = [text for text in self.app.log_lines if "FAULT" in text]
        self.assertEqual(len(fault_lines), 1)
        for _ in range(4):
            client.events.put(("rx", "FAULT none"))
            self.app.pump()
        fault_lines = [text for text in self.app.log_lines if "FAULT" in text]
        self.assertEqual(len(fault_lines), 2)

    def test_save_disabled_while_calibrating_or_running(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        self.app.update_controls()
        self.assertNotEqual(self.app.save_button["state"], "disabled")
        with patch("chassis_app.messagebox.askyesno", return_value=True):
            self.app.start_calibration()
        self.app.update_controls()
        self.assertEqual(self.app.save_button["state"], "disabled")
        client.handle_response("CAL FAIL bad_line 1")
        self.app.pump()
        self.app.update_controls()
        self.assertNotEqual(self.app.save_button["state"], "disabled")
        client.events.put(("rx", "SPEED_RUN A=1000 B=0 C=0 D=0 duration=500"))
        self.app.pump()
        self.assertTrue(self.app.device_running)
        self.app.update_controls()
        self.assertEqual(self.app.save_button["state"], "disabled")
        client.events.put(("rx", "STOPPED timer"))
        self.app.pump()
        self.app.cooldown_until = 0.0
        self.app.update_controls()
        self.assertNotEqual(self.app.save_button["state"], "disabled")

    def test_unrelated_error_does_not_end_calibration_and_reset_does(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        with patch("chassis_app.messagebox.askyesno", return_value=True):
            self.app.start_calibration()
        self.assertTrue(self.app.calibrating)
        client.events.put(("rx", "ERROR unknown_command"))
        self.app.pump()
        self.assertTrue(self.app.calibrating)
        client.handle_response("CAL RESET")
        self.app.pump()
        self.assertFalse(self.app.calibrating)
        self.assertIn("CAL RESET", self.app.cal_progress.get())

    def test_cal_done_refreshes_vmax_and_limit(self):
        client = self.ready(layout=True)
        self.enable_closed_loop(client)
        client._commands.clear()
        client.handle_response("CAL DONE VALID=1 VMAX=4200.0")
        self.app.pump()
        self.assertIn("CAL?", client._commands)
        client.handle_response("CAL VALID=1 VMAX=4200.0 LIMIT=75")
        self.app.pump()
        status = self.app.closed_status.get()
        self.assertIn("VMAX=4200.0", status)
        self.assertIn("LIMIT=75", status)

    def test_motion_and_layout_controls_fit_window(self):
        self.root.update()
        bottom=self.root.winfo_rooty()+self.root.winfo_height()
        right=self.root.winfo_rootx()+self.root.winfo_width()
        for widget in [*self.app.mixed_buttons, self.app.apply_button]:
            self.assertLess(widget.winfo_rooty()+widget.winfo_height(),bottom)
            self.assertLess(widget.winfo_rootx()+widget.winfo_width(),right)


if __name__ == "__main__": unittest.main()
