"""Windows 键盘控制，支持短按点动和长按移动。"""

from __future__ import annotations

import argparse
import math
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
RESTART_INPUT_S = 0.14
RESTART_POLL_MS = 10
MIN_POWER = 5
MAX_POWER = 100
POWER_STEP = 5
INITIAL_POWER = 80


def apply_key(link: ChassisLink, key: str, power: int) -> int:
    if key == "z":
        return min(MAX_POWER, power + POWER_STEP)
    if key == "x":
        return max(MIN_POWER, power - POWER_STEP)
    if key == "space":
        link.stop()
    elif key in DIRECTIONS:
        link.pulse(*(power * axis for axis in DIRECTIONS[key]), PULSE_MS,
                   send_by=time.monotonic() + .15)
    return power


def control_window(link: ChassisLink) -> None:
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    try:
        manual = link.supports_manual
        supports_hold = link.supports_hold
        quick_release = supports_hold and link.supports_hold_release
        root.title("小车键盘遥控")
        frame = ttk.Frame(root, padding=18)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="W / S  前进 / 后退     A / D  左转 / 右转").pack(anchor="w")
        ttk.Label(frame, text="Q / E  左移 / 右移     Z / X  增大 / 减小力度").pack(anchor="w")
        mode = "短按点动，长按移动"
        if manual:
            mode += " · 换向即时响应"
        elif not supports_hold:
            mode = "当前底盘固件支持点动，长按需更新固件"
        elif not quick_release:
            mode += " · 当前固件松键后需等待约 1 秒"
        ttk.Label(frame, text=mode).pack(anchor="w", pady=(6, 0))
        ttk.Label(frame, text="空格停车 · Esc 停车并退出").pack(anchor="w")
        status = tk.StringVar()
        ttk.Label(frame, textvariable=status).pack(anchor="w", pady=(12, 0))

        power = INITIAL_POWER
        pressed = set()
        manual_keys = {}  # Held directions in press order.
        active_key = None
        started_at = 0.0
        release_timer = None
        restart_timer = None

        def clear_motion():
            nonlocal active_key, release_timer
            if release_timer is not None:
                root.after_cancel(release_timer)
                release_timer = None
            active_key = None

        def cancel_restart():
            nonlocal restart_timer
            if restart_timer is not None:
                root.after_cancel(restart_timer)
                restart_timer = None

        def stop_motion(*, release=False):
            cancel_restart()
            clear_motion()
            manual_keys.clear()
            if manual:
                link.manual(0, 0, 0)
            else:
                link.stop(release=release)

        def finish_tap():
            nonlocal release_timer
            release_timer = None
            stop_motion(release=True)

        def start_motion(key, *, send_by=None):
            nonlocal active_key, started_at, release_timer
            axes = tuple(power * axis for axis in DIRECTIONS[key])
            accepted = link.manual(*axes) if manual else link.hold(*axes, send_by=send_by)
            if not accepted:
                return False
            active_key = key
            started_at = manual_keys[key] if manual else time.monotonic()
            if key not in pressed:
                release_timer = root.after(PULSE_MS, finish_tap)
            return True

        def start_after_release(key, deadline):
            nonlocal restart_timer
            restart_timer = None
            if time.monotonic() >= deadline or start_motion(key, send_by=deadline):
                return
            if link.hold_restarting:
                restart_timer = root.after(RESTART_POLL_MS, start_after_release, key, deadline)

        def request_motion(key):
            nonlocal restart_timer
            if active_key is not None:
                return
            restarting = quick_release and link.hold_restarting
            if not start_motion(key) and restarting:
                restart_timer = root.after(RESTART_POLL_MS, start_after_release, key,
                                           time.monotonic() + RESTART_INPUT_S)

        def quit_window():
            stop_motion()
            root.quit()

        def press(event):
            nonlocal power, release_timer
            key = event.keysym.lower()
            if key == "escape":
                quit_window()
                return "break"
            if key in DIRECTIONS or key in ("z", "x", "space"):
                if key not in pressed:
                    pressed.add(key)
                    if key == "space":
                        stop_motion()
                    elif manual and key in DIRECTIONS:
                        if "space" not in pressed:
                            manual_keys[key] = time.monotonic()
                            clear_motion()
                            start_motion(key)
                    elif supports_hold and key in DIRECTIONS:
                        if "space" not in pressed:
                            cancel_restart()
                            if key == active_key and release_timer is not None:
                                root.after_cancel(release_timer)
                                release_timer = None
                            elif quick_release and active_key is not None:
                                stop_motion(release=True)
                            request_motion(key)
                    elif key not in DIRECTIONS or "space" not in pressed:
                        power = apply_key(link, key, power)
                        if manual and active_key in manual_keys:
                            link.manual(*(power * axis for axis in DIRECTIONS[active_key]))
                return "break"
            return None

        def release(event):
            nonlocal release_timer
            key = event.keysym.lower()
            pressed.discard(key)
            manual_keys.pop(key, None)
            if key == active_key and release_timer is None:
                if manual_keys:
                    clear_motion()
                    start_motion(next(reversed(manual_keys)))
                    return
                remaining = PULSE_MS - (time.monotonic() - started_at) * 1000
                if remaining > 0:
                    release_timer = root.after(math.ceil(remaining), finish_tap)
                else:
                    stop_motion(release=True)

        def lost_focus(_event):
            pressed.clear()
            stop_motion()

        def refresh_status():
            if link.error or not link.ready:
                root.quit()
                return
            if manual:
                if active_key is not None and not link.manual_active:
                    clear_motion()
                    manual_keys.clear()
                if active_key in manual_keys:
                    link.renew_manual()
                busy = False
            else:
                if active_key is not None and not link.holding:
                    clear_motion()
                if active_key in pressed:
                    link.hold(*(power * axis for axis in DIRECTIONS[active_key]), renew=True)
                busy = link.hold_busy if supports_hold else link.busy
            state = ("长按控制中" if active_key in pressed else
                     "点动中" if release_timer is not None else
                     "等待松键确认" if restart_timer is not None else
                     "动作或冷却中" if busy else "可以操作")
            status.set(f"力度 {power}%   ·   {state}")
            root.after(50, refresh_status)

        root.bind("<KeyPress>", press)
        root.bind("<KeyRelease>", release)
        root.bind("<FocusOut>", lost_focus)
        root.bind("<Button-1>", lambda _: root.focus_set())
        root.protocol("WM_DELETE_WINDOW", quit_window)
        root.after(0, refresh_status)
        root.mainloop()
    finally:
        root.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM6", help="蓝牙传出串口（默认 COM6）")
    parser.add_argument("--layout", default="CBDA", help="左前、右前、左后、右后（默认 CBDA）")
    args = parser.parse_args()
    print("给车上电，关闭底盘工作台和自主任务后再连接。")
    port = args.port.strip().upper()
    if not re.fullmatch(r"COM[1-9]\d*", port):
        raise ValueError("请输入蓝牙传出端口，例如 COM6。")
    layout = args.layout.strip().upper()
    if sorted(layout) != list("ABCD"):
        raise ValueError("请按实物轮位输入 A、B、C、D 各一次。")

    link = ChassisLink(port, layout=f"{layout} 1 1 1 1", strict=True, recover_cooldown=True)
    try:
        print(f"正在连接 {port}，轮位 {layout}…", flush=True)
        link.start()
        link.wait_ready()
        control_window(link)
        if link.error or not link.ready:
            raise ChassisError(link.error or "底盘已断开。")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    from daily_tools import menu_text

    try:
        raise SystemExit(main())
    except (EOFError, KeyboardInterrupt):
        raise SystemExit(130)
    except (ChassisError, ValueError) as error:
        print(menu_text(str(error), 31), flush=True)
        raise SystemExit(1)
