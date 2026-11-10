"""Explicit live check: simulated DRIVE only; never sends a real JOG."""
from pathlib import Path
from datetime import datetime
import argparse
import json
import sys
import time

import serial

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "esp32-pc-link" / "pc"))
sys.path.insert(0, str(ROOT / "motor-bringup"))
from link_client import LinkClient
from motor_link import MotorLink


def wait(predicate, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("Timed out waiting for device")


def verify_window(port, log):
    import tempfile
    import tkinter as tk
    sys.path.insert(0, str(HERE))
    from main import Workspace

    with tempfile.TemporaryDirectory() as folder:
        root = tk.Tk()
        root.withdraw()
        errors = []
        root.report_callback_exception = lambda *args: errors.append(str(args[1]))
        workspace = Workspace(root, "dry", Path(folder) / "settings.json")

        def pump_until(predicate):
            def step():
                root.update()
                assert not errors, errors
                return predicate()
            wait(step)

        try:
            for mode in ("dry", "bench", "dry"):
                workspace.switch(mode)
                pump_until(lambda: workspace.mode == mode and not workspace.transitioning)
                app = workspace.app
                assert app.client is None  # Switching must never auto-connect.
                if mode == "dry":
                    app.port_var.set(port)
                    app.toggle_connection()
                    pump_until(lambda: app.client is not None and app.client.can_arm())
                    assert not app.armed
                    log("PASS WINDOW DRY_RUN: connected, DISARMED, no automatic ARM")
                else:
                    app.port.set(next(name for name, device in app.ports.items() if device == port))
                    app.connect()
                    pump_until(lambda: app.client is not None and app.client.ready and app.data.latest is not None)
                    assert not app.confirm.get()
                    assert str(app.jog_button["state"]) == "disabled"
                    assert not app.data.runs
                    for channel in "ABCD":
                        app.channel.set(channel)
                        app.change_channel()
                        pump_until(lambda: app.data.channels[channel].latest is not None)
                        assert app.data.channel == channel and not app.confirm.get()
                        assert app.data.channels[channel].latest.channel == channel
                    log("PASS WINDOW FOUR_V1: A/B/C/D selectable, encoder data received, no JOG")
        finally:
            workspace.close()
            end = time.monotonic() + 0.6
            while time.monotonic() < end:
                try:
                    if not root.winfo_exists():
                        break
                    root.update()
                except tk.TclError:
                    break  # Workspace's deferred destroy has completed.
                time.sleep(0.01)
        assert not errors, errors


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--gui", action="store_true", help="Also check both actual Tk views without running a motor")
    args = parser.parse_args()
    report = []
    passed = False

    def log(text):
        report.append(text)
        print(text, flush=True)

    try:
        with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=0.3) as device:
            time.sleep(1.5)
            device.reset_input_buffer()

            def query(command, accept, timeout=2):
                log("> " + command)
                device.write((command + "\n").encode("ascii"))
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    line = device.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        log("< " + line)
                        if accept(line):
                            return line
                raise AssertionError("Missing response to " + command)

            def io(mode):
                line = query("IO", lambda s: s.startswith("IO mode="))
                assert line == f"IO mode={mode} standby=0 A=0,0 B=0,0 C=0,0 D=0,0", line

            def json_reply(command, kind):
                line = query(command, lambda s: s.startswith("{") and json.loads(s).get("type") == kind)
                return json.loads(line)

            query("WHO", lambda s: s == "UNIFIED_ROBOT 1 mode=DRY_RUN")
            query("CAPS", lambda s: s == "CAPS MOTORS=ABCD PINS=FOUR_V1")
            io("DRY_RUN")
            query("MODE DRY_RUN", lambda s: s == "MODE_OK DRY_RUN STOPPED")
            hello = json_reply("STATUS", "status")
            ack = json_reply(f"ARM 123456 {hello['arm_token']}", "ack")
            assert ack["ok"] and ack["state"] == "ARMED", ack
            ack = json_reply("DRIVE 123456 1 100 -20 10", "ack")
            assert ack["ok"] and ack["vx"] == 100, ack
            io("DRY_RUN")
            ack = json_reply("STOP", "ack")
            assert ack["ok"] and ack["state"] == "DISARMED", ack
            query("MODE MOTOR_A_TEST", lambda s: s == "MODE_OK MOTOR_A_TEST STOPPED")
            query("PING", lambda s: s == "PONG MOTOR_A_TEST")
            query("STATUS", lambda s: s == "STOPPED ALL")
            for channel in "ABCD":
                query(f"ENC {channel}", lambda s: s.startswith(f"ENC {channel} ticks="))
            io("MOTOR_A_TEST")
            query("MODE DRY_RUN", lambda s: s == "MODE_OK DRY_RUN STOPPED")
            io("DRY_RUN")
        time.sleep(0.3)

        # Use the actual PC clients, including their normal serial-open/reset.
        dry = LinkClient(select_unified_mode=True)
        try:
            dry.connect(args.port)
            wait(dry.can_arm)
            snap = dry.snapshot()
            assert snap["esp_state"] == "DISARMED", snap
            log("PASS PC DRY_RUN: " + repr(snap))
            assert dry.send_raw("IO")
            events = []

            def dry_io():
                events.extend(dry.poll())
                return any(kind == "log" and payload.get("text") == "IO mode=DRY_RUN standby=0 A=0,0 B=0,0 C=0,0 D=0,0"
                           for kind, payload in events)

            wait(dry_io)
            assert not any(kind == "log" and payload.get("dir") == "tx" and payload.get("text", "").startswith(("ARM ", "DRIVE ", "JOG "))
                           for kind, payload in events)
        finally:
            dry.close()
        time.sleep(0.3)
        motor = MotorLink(require_profile="FOUR_V1")
        try:
            motor.connect(args.port)
            wait(lambda: motor.ready)
            motor.send_line("IO")
            events = []

            def motor_io():
                events.extend(motor.poll())
                return ("rx", "IO mode=MOTOR_A_TEST standby=0 A=0,0 B=0,0 C=0,0 D=0,0") in events

            wait(motor_io)
            for kind, payload in events:
                if kind in ("rx", "tx", "tx_auto"):
                    log("MOTOR " + kind + " " + str(payload))
            assert not any(kind in ("tx", "tx_auto") and str(payload).startswith("JOG ") for kind, payload in events)
            assert motor.supported_channels == tuple("ABCD")
            log("PASS PC FOUR_V1: ready, stopped, no JOG")
        finally:
            motor.close()
            wait(lambda: not motor._thread.is_alive())
        time.sleep(0.3)
        if args.gui:
            verify_window(args.port, log)
            time.sleep(0.3)
        with serial.Serial(args.port, 115200, timeout=0.05) as device:
            time.sleep(1.2)
            device.reset_input_buffer()
            query("MODE DRY_RUN", lambda s: s == "MODE_OK DRY_RUN STOPPED")
            io("DRY_RUN")
        log("PASS: unified hardware and PC clients; final DRY_RUN outputs stopped. No real JOG sent.")
        passed = True
    except Exception as exc:
        log("FAIL: " + repr(exc))
        raise
    finally:
        # Preserve successful evidence when a later attempt has no USB device.
        records = HERE / "records"
        records.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        content = "\n".join(report) + "\n"
        (records / f"unified-{stamp}-{'pass' if passed else 'fail'}.txt").write_text(content, encoding="utf-8")
        if passed:
            (HERE / "unified-verification.txt").write_text(content, encoding="utf-8")


if __name__ == "__main__":
    main()
