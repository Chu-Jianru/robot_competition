"""Windows 键盘遥控，支持四轮轮速闭环和直接 PWM 控制。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import time

from chassis_link import ChassisError, ChassisLink


DIRECTIONS = {
    "w": (1, 0, 0),
    "s": (-1, 0, 0),
    "a": (0, 0, 1),
    "d": (0, 0, -1),
    "q": (0, 1, 0),
    "e": (0, -1, 0),
}
PULSE_MS = 100
UPDATE_MS = 50
INPUT_MAX_AGE_S = .25
MIN_PERCENT = 5
MAX_PERCENT = 100
PERCENT_STEP = 5
INITIAL_PERCENT = 80


def control_window(link: ChassisLink, view=None, *, pwm=False) -> None:
    import tkinter as tk
    from tkinter import ttk

    output_limit = 100 if pwm else link.speed_info[0]
    mode = "直接 PWM" if pwm else "轮速闭环"
    quantity = "占空比" if pwm else "轮速"
    root = tk.Tk()
    try:
        root.title(f"小车键盘遥控 · {mode}")
        frame = ttk.Frame(root, padding=18)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="W / S  前进 / 后退     A / D  左转 / 右转").pack(anchor="w")
        ttk.Label(frame, text=f"Q / E  左移 / 右移     Z / X  提高 / 降低{quantity}").pack(anchor="w")
        ttk.Label(frame, text="短按点动，长按移动").pack(anchor="w", pady=(6, 0))
        ttk.Label(frame, text="空格停车 · Esc 停车并退出").pack(anchor="w")
        if view is not None:
            ttk.Label(frame, text="旁边是检测窗口，只显示画面；按键前点一下本窗口").pack(anchor="w")
        status = tk.StringVar()
        ttk.Label(frame, textvariable=status).pack(anchor="w", pady=(12, 0))

        output_percent = INITIAL_PERCENT
        pressed = set()
        held_directions = {}  # Press time for each direction, in order.
        active_key = None
        manual_start_pending = False
        release_timer = None

        def cancel_tap():
            nonlocal release_timer
            if release_timer is not None:
                root.after_cancel(release_timer)
                release_timer = None

        def stop_motion():
            nonlocal active_key, manual_start_pending
            cancel_tap()
            active_key = None
            manual_start_pending = False
            held_directions.clear()
            link.stop()

        def send_target():
            target = max(1, round(output_limit * output_percent / 100))
            axes = tuple(target * axis for axis in DIRECTIONS[active_key])
            if pwm:
                return link.manual(*axes)
            return link.stream(*axes, valid_until=time.monotonic() + INPUT_MAX_AGE_S)

        def start_motion(key):
            nonlocal active_key, manual_start_pending
            cancel_tap()
            active_key = key
            accepted = send_target()
            manual_start_pending = pwm and not accepted

        def quit_window():
            stop_motion()
            root.quit()

        def press(event):
            nonlocal output_percent
            key = event.keysym.lower()
            if key == "escape":
                quit_window()
                return "break"
            if key in DIRECTIONS or key in ("z", "x", "space"):
                if key not in pressed:
                    pressed.add(key)
                    if key == "space":
                        stop_motion()
                    elif key in DIRECTIONS:
                        if "space" not in pressed:
                            held_directions[key] = time.monotonic()
                            start_motion(key)
                    else:
                        step = PERCENT_STEP if key == "z" else -PERCENT_STEP
                        output_percent = max(MIN_PERCENT, min(MAX_PERCENT, output_percent + step))
                        if active_key in held_directions:
                            send_target()
                return "break"
            return None

        def release(event):
            nonlocal release_timer
            key = event.keysym.lower()
            pressed.discard(key)
            started_at = held_directions.pop(key, None)
            if key != active_key or started_at is None:
                return
            if held_directions:
                start_motion(next(reversed(held_directions)))
                return
            remaining = PULSE_MS - (time.monotonic() - started_at) * 1000
            if remaining > 0:
                release_timer = root.after(math.ceil(remaining), finish_tap)
            else:
                stop_motion()

        def finish_tap():
            nonlocal release_timer
            release_timer = None
            stop_motion()

        def lost_focus(_event):
            pressed.clear()
            stop_motion()

        def refresh_status():
            nonlocal manual_start_pending
            if link.error or not link.ready:
                quit_window()
                return
            if pwm and active_key is not None and not (link.manual_active or manual_start_pending):
                stop_motion()
            elif active_key in held_directions:
                if pwm and not manual_start_pending:
                    link.renew_manual()
                else:
                    # A new press can race the preceding STOP ack. Retry only while held.
                    accepted = send_target()
                    manual_start_pending = pwm and not accepted
            state = ("长按控制中" if active_key in held_directions else
                     "点动中" if release_timer is not None else "可以操作")
            output_label = "PWM 占空比" if pwm else "轮速上限的"
            status.set(f"{output_label} {output_percent}%   ·   {state}")
            root.after(UPDATE_MS, refresh_status)

        root.bind("<KeyPress>", press)
        root.bind("<KeyRelease>", release)
        root.bind("<FocusOut>", lost_focus)
        root.bind("<Button-1>", lambda _: root.focus_set())
        root.protocol("WM_DELETE_WINDOW", quit_window)
        if view is not None:
            view.attach(root)
        root.after(0, refresh_status)
        root.mainloop()
    finally:
        root.destroy()


def port_hint(message: str):
    """端口打不开时补一句可操作的排查提示。"""
    if "could not open port" not in message and "PermissionError" not in message:
        return None
    return ("端口被占用或蓝牙串口还没释放：先关掉底盘工作台、轮速面板和任务/遥控窗口；"
            "仍不行就给 ESP32 断电重上，在 Windows 蓝牙里断开后重新连接再试；"
            "ESP32 插着 USB 时先拔掉只留蓝牙。")


def open_view(args):
    """打开检测视频窗口；取流或标定不可用时只报告原因，遥控照常可用。"""
    print("正在打开检测窗口（手机取流约需数秒）…", flush=True)
    try:
        from remote_view import RemoteView

        view = RemoteView.open(scrcpy_dir=args.scrcpy_dir, rotate=args.rotate,
                               calibration=args.calibration, detector=args.detector)
    except Exception as error:
        print(f"检测窗口未打开，仍可单独遥控：{error}", flush=True)
        return None
    print("检测窗口已打开：画面与菜单 10/12 相同，只显示不控车。", flush=True)
    return view


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM6", help="蓝牙传出串口（默认 COM6）")
    parser.add_argument("--layout", default="CBDA", help="左前、右前、左后、右后（默认 CBDA）")
    parser.add_argument("--pwm", action="store_true", help="直接控制 PWM 占空比（最高 100%%）")
    parser.add_argument("--camera", action="store_true",
                        help="同时打开检测视频窗口（需要手机 USB 和场地标定）")
    parser.add_argument("--scrcpy-dir", type=Path, help="scrcpy 5.0.1 目录（默认同任务入口）")
    parser.add_argument("--calibration", type=Path, help="场地标定（默认 windows/field_calib.json）")
    parser.add_argument("--rotate", default="90", choices=("0", "90", "180", "270", "auto"),
                        help="相机画面朝向；默认 90 与菜单 10/11/12 和现有场地标定一致，"
                             "auto 按蓝黄地面自动判断")
    parser.add_argument("--detector", choices=("auto", "hsv", "onnx"), default="auto",
                        help="红块来源；auto=有模型就用 onnx，否则 hsv")
    args = parser.parse_args()
    print("给车上电，关闭底盘工作台和自主任务后再连接。")
    port = args.port.strip().upper()
    if not re.fullmatch(r"COM[1-9]\d*", port):
        raise ValueError("请输入蓝牙传出端口，例如 COM6。")
    layout = args.layout.strip().upper()
    if sorted(layout) != list("ABCD"):
        raise ValueError("请按实物轮位输入 A、B、C、D 各一次。")

    link = ChassisLink(port, layout=f"{layout} 1 1 1 1", strict=True,
                       require_speed_stream=not args.pwm, stream_pipeline=not args.pwm,
                       renew_stream=not args.pwm)
    view = None
    try:
        print(f"正在连接 {port}，轮位 {layout}…", flush=True)
        link.start()
        link.wait_ready()
        if args.pwm:
            if not link.supports_manual:
                raise ChassisError("底盘固件缺少 CHASSIS_MANUAL，无法直接控制 PWM。")
            print(f"直接 PWM 遥控已就绪，初始占空比 {INITIAL_PERCENT}%，最高 100%。", flush=True)
        else:
            print(f"轮速闭环已就绪，上限 {link.speed_info[0]} 计数/秒，"
                  f"遥控从上限的 {INITIAL_PERCENT}% 起步。", flush=True)
        # 端口先连上再开相机：串口打不开时不必等取流，也不会留下检测窗口。
        if args.camera:
            view = open_view(args)
        control_window(link, view, pwm=args.pwm)
        if link.error or not link.ready:
            raise ChassisError(link.error or "底盘已断开。")
        return 0
    finally:
        try:
            link.close()
        finally:
            if view is not None:
                view.close()


if __name__ == "__main__":
    from daily_tools import menu_text

    try:
        raise SystemExit(main())
    except (EOFError, KeyboardInterrupt):
        raise SystemExit(130)
    except (ChassisError, ValueError) as error:
        print(menu_text(str(error), 31), flush=True)
        hint = port_hint(str(error))
        if hint:
            print(menu_text(hint, 31), flush=True)
        raise SystemExit(1)
