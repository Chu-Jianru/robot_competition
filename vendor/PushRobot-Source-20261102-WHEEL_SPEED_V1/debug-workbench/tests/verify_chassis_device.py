"""显式实机静止验收；只切换/查询/STOP，禁止任何真实运动指令。"""
from pathlib import Path
import argparse
from datetime import datetime
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / p) for p in ("debug-workbench", "motor-bringup", "esp32-pc-link/pc")]
import serial

SAFE_COMMANDS = {"WHO", "CAPS", "MODE CHASSIS", "MODE MOTOR_A_TEST", "MODE DRY_RUN", "STOP", "PING", "STATUS", "IO", "ENC", "LAYOUT?", "POLARITY"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    log = []
    def record(text):
        log.append(text)
        print(text, flush=True)
    def run(device, command, expected):
        assert command in SAFE_COMMANDS, "不允许实机测试发送运动命令"
        record("> " + command)
        device.write((command+"\n").encode("ascii"))
        deadline = time.monotonic()+5
        while time.monotonic()<deadline:
            line = device.readline().decode("utf-8",errors="replace").strip()
            if line:
                record("< " + line)
                if expected in line: return line
        raise AssertionError(f"未收到 {expected}")
    zero = "standby=0 A=0,0 B=0,0 C=0,0 D=0,0"
    with serial.Serial(args.port,115200,timeout=0.2,write_timeout=0.3) as device:
        try:
            time.sleep(2)
            device.reset_input_buffer()
            run(device,"WHO","UNIFIED_ROBOT 1")
            run(device,"CAPS","CHASSIS_CAPS API=MECANUM_V2 MIX=X UNITS=PERCENT FORWARD=UNIFIED")
            run(device,"POLARITY","MOTOR_FORWARD A=-1 B=-1 C=-1 D=-1")
            run(device,"MODE CHASSIS","MODE_OK CHASSIS STOPPED")
            run(device,"PING","PONG CHASSIS")
            run(device,"STOP","STOPPED command")
            run(device,"STATUS","STOPPED ALL")
            run(device,"ENC","ENC D ")
            run(device,"LAYOUT?","LAYOUT CONFIGURED=0")
            run(device,"IO",zero)
            run(device,"MODE MOTOR_A_TEST","MODE_OK MOTOR_A_TEST STOPPED")
            run(device,"PING","PONG MOTOR_A_TEST")
            run(device,"IO",zero)
            run(device,"MODE DRY_RUN","MODE_OK DRY_RUN STOPPED")
            run(device,"IO",zero)
        finally:
            device.write(b"STOP\nMODE DRY_RUN\n")
            device.flush()
    if args.gui:
        import tempfile
        import tkinter as tk
        from main import Workspace
        with tempfile.TemporaryDirectory() as folder:
            root = tk.Tk()
            root.withdraw()
            errors = []
            root.report_callback_exception=lambda *items:errors.append(str(items[1]))
            workspace = Workspace(root,mode="chassis",settings_path=Path(folder)/"settings.json")
            app = workspace.app
            def wait(predicate, seconds=8):
                deadline=time.monotonic()+seconds
                while time.monotonic()<deadline:
                    root.update()
                    assert not errors, errors
                    if predicate(): return
                    time.sleep(0.02)
                raise AssertionError("等待实际面板/客户端状态超时")
            try:
                app.ports={args.port:args.port}
                app.port.set(args.port)
                app.toggle_connection()
                wait(lambda:app.client and app.client.ready)
                wait(lambda:len(app.encoder_values)==4)
                assert not app.client.layout_confirmed and not app.confirm.get()
                assert not app.device_running and not app.pending_run
                assert all(str(button["state"])=="disabled" for button in app.motion_buttons)
                client=app.client
                workspace.switch("bench")
                wait(lambda:not workspace.transitioning)
                assert workspace.app.client is None
                assert not client.ready
                workspace.switch("dry")
                wait(lambda:not workspace.transitioning)
                assert workspace.app.client is None and not workspace.app.armed
                record("GUI: CHASSIS ready, four encoders, unconfigured/disarmed controls; bench/dry switches disconnected")
            finally:
                workspace.close()
                until=time.monotonic()+0.6
                while time.monotonic()<until:
                    try:root.update()
                    except tk.TclError:break
                    time.sleep(0.01)
        # 重新查 IO，结束时恢复上电默认模拟模式；不留真实底盘模式。
        with serial.Serial(args.port,115200,timeout=0.2,write_timeout=0.3) as device:
            time.sleep(2)
            device.reset_input_buffer()
            run(device,"MODE DRY_RUN","MODE_OK DRY_RUN STOPPED")
            run(device,"IO",zero)
    report = ROOT / "debug-workbench/records" / (datetime.now().strftime("chassis-static-%Y%m%d-%H%M%S")+".json")
    report.parent.mkdir(parents=True,exist_ok=True)
    report.write_text(json.dumps({"passed":True,"port":args.port,"motion_sent":False,"checks":log},ensure_ascii=False,indent=2),encoding="utf-8")
    print("PASS: static only; DRY_RUN zero outputs; port closed. Report:",report)


if __name__=="__main__":main()
