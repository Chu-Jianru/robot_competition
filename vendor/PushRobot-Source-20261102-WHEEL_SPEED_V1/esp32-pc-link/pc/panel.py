"""DRY_RUN debug panel for the push-block robot link.

This panel only talks to the DRY_RUN firmware. It never imports OpenCV and
never starts a camera, so opening the panel cannot trigger vision/control.

Run:
    .venv\\Scripts\\python.exe panel.py
Self-check without hardware:
    .venv\\Scripts\\python.exe panel.py --self-test
"""

from __future__ import annotations

import argparse
import json
import secrets
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

from serial.tools import list_ports

from link_client import (
    MODE_DRY_RUN,
    WATCHDOG_MS,
    ConnectionState,
    LinkClient,
)
from panel_camera import CameraPanel
from panel_view import PanelView
from runtime_paths import camera_config_file

DIRECTION_LABELS = {
    "forward": "前进",
    "back": "后退",
    "left": "左移",
    "right": "右移",
    "ccw": "逆时针",
    "cw": "顺时针",
}

CONNECTION_LABELS = {
    ConnectionState.DISCONNECTED: "未连接",
    ConnectionState.WAITING_IDENTITY: "等待 ESP32 启动 / hello…",
    ConnectionState.SYNCING: "同步中（STOP / STATUS）",
    ConnectionState.READY: "已就绪",
    ConnectionState.ERROR: "协议或连接错误",
}


class PanelApp(PanelView):
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("推物块机器人 · DRY_RUN 调试面板")
        self.root.geometry("920x780")
        self.root.minsize(820, 700)

        self.client: LinkClient | None = None
        self.armed = False
        self.active_directions: set[str] = set()
        self.motion_timer = None
        self.last_motion_tick = 0.0
        self.status_counter = 0
        self.last_session: int | None = None
        self.port_map: dict[str, str] = {}

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value=CONNECTION_LABELS[ConnectionState.DISCONNECTED])
        self.fw_var = tk.StringVar(value="—")
        self.mode_var = tk.StringVar(value="—")
        self.esp_state_var = tk.StringVar(value="UNKNOWN")
        self.session_var = tk.StringVar(value="—")
        self.seq_var = tk.StringVar(value="—")
        self.target_var = tk.StringVar(value="—")
        self.age_var = tk.StringVar(value="—")
        self.boot_var = tk.StringVar(value="—")
        self.reason_var = tk.StringVar(value="—")
        self.message_var = tk.StringVar(value="未连接。先选择串口，再点“连接”。")
        self.verbose_var = tk.BooleanVar(value=False)
        self.translation_speed = tk.IntVar(value=100)
        self.rotation_speed = tk.IntVar(value=45)
        self.raw_var = tk.StringVar()
        self.response_var = tk.StringVar(value="—")
        self.vx_var = tk.StringVar(value="—")
        self.vy_var = tk.StringVar(value="—")
        self.w_var = tk.StringVar(value="—")
        self.autoscroll_var = tk.BooleanVar(value=True)

        self.direction_buttons: list[tuple[tk.Button, str]] = []
        self._build_ui()
        self.camera_panel = CameraPanel(
            self.notebook, camera_config_file()
        )
        self.notebook.add(self.camera_panel, text="摄像头")
        self.refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<Escape>", lambda _event: self.stop_motion())
        self.root.bind("<FocusOut>", self._on_focus_out)
        self.root.after(50, self._poll_events)
        self.root.after(200, self._refresh_snapshot)

    # ------------------------------------------------------------------ logging

    def log(self, text: str, tag: str = "info") -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{time.strftime('%H:%M:%S')} {text}\n", tag)
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 800:
            self.log_text.delete("1.0", f"{line_count - 800}.0")
        if self.autoscroll_var.get():
            self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def export_log(self) -> None:
        from tkinter import filedialog

        path = filedialog.asksaveasfilename(
            title="导出日志",
            defaultextension=".txt",
            filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")],
            initialfile=time.strftime("robot-log-%Y%m%d-%H%M%S.txt"),
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(self.log_text.get("1.0", "end-1c"))
        except OSError as exc:
            self.set_message(f"导出日志失败：{exc}")
            return
        self.set_message(f"日志已导出：{path}")

    def send_query(self, command: str) -> None:
        if self.client is None or not self.client.send_raw(command):
            self.set_message("设备未就绪，无法发送查询。")
            return
        self.log(f"查询：{command}")
        self.set_message(f"已发送 {command}，等待应答…")

    def set_message(self, text: str) -> None:
        self.message_var.set(text)

    def _clear_runtime_values(self) -> None:
        self.vx_var.set("—")
        self.vy_var.set("—")
        self.w_var.set("—")
        self.target_var.set("—")
        self.age_var.set("—")
        self.session_var.set("—")
        self.seq_var.set("—")
        self.reason_var.set("—")

    def _clear_runtime_display(self) -> None:
        self._clear_runtime_values()
        self.response_var.set("—")

    def _note_response(self, text: str) -> None:
        try:
            message = json.loads(text)
        except (ValueError, TypeError):
            return
        if not isinstance(message, dict):
            return
        label = str(message.get("type", "?"))
        if message.get("cmd"):
            label += f" · {message['cmd']}"
        if message.get("event"):
            label += f" · {message['event']}"
        if message.get("state"):
            label += f" · {message['state']}"
        self.response_var.set(f"{time.strftime('%H:%M:%S')}  {label}")

    def _reset_status_display(self) -> None:
        self.connection_var.set(CONNECTION_LABELS[ConnectionState.DISCONNECTED])
        self.fw_var.set("—")
        self.mode_var.set("—")
        self.esp_state_var.set("UNKNOWN")
        self.boot_var.set("—")
        self._clear_runtime_display()
        if hasattr(self, "connection_label"):
            self.connection_label.configure(fg="#64758A")
        if hasattr(self, "state_label"):
            self.state_label.configure(fg="#64758A")

    # ------------------------------------------------------------------ ports

    def refresh_ports(self) -> None:
        if self.client is not None:
            return
        ports = sorted(list_ports.comports(), key=lambda p: p.device)
        self.port_map = {}
        values = []
        for port in ports:
            display = f"{port.device} — {port.description}"
            self.port_map[display] = port.device
            values.append(display)
        self.port_combo["values"] = values
        current_device = self.port_var.get().split(" — ")[0].strip()
        selected = None
        for display, device in self.port_map.items():
            if device == current_device or (not current_device and device == "COM4"):
                selected = display
                break
        if selected is None and values:
            selected = values[0]
        self.port_var.set(selected or "")
        if not values:
            self.set_message("未发现串口。请检查 USB 线和 CP2102 驱动。")

    def selected_port(self) -> str | None:
        display = self.port_var.get().strip()
        if not display:
            return None
        return self.port_map.get(display, display.split(" — ")[0].strip())

    # ------------------------------------------------------------------ connect

    def toggle_connection(self) -> None:
        if self.client is not None:
            self.disconnect()
            return
        port = self.selected_port()
        if not port:
            self.set_message("没有选择串口。")
            return
        client = LinkClient(select_unified_mode=True)
        try:
            client.connect(port)
        except Exception as exc:
            try:
                client.close(send_stop=False)
            except Exception:
                pass
            self.log(f"连接失败：{exc}")
            self.set_message(f"连接 {port} 失败：{exc}")
            return
        self.client = client
        self.armed = False
        self.active_directions.clear()
        self.connect_button.configure(text="断开")
        self.port_combo.configure(state="disabled")
        self.refresh_button.state(["disabled"])
        self.log(f"已打开 {port}，等待 ESP32 启动/hello；不会在确认协议前发送 STOP。")
        self.set_message("等待 ESP32 启动并确认协议…")
        self.update_buttons(client.snapshot())

    def disconnect(self) -> None:
        self.stop_motion_loop()
        self.armed = False
        self.active_directions.clear()
        client = self.client
        self.client = None
        if client is not None:
            try:
                client.close(send_stop=True, timeout=0.4)
            except Exception as exc:
                self.log(f"关闭串口时出错：{exc}")
        self.connect_button.configure(text="连接")
        self.port_combo.configure(state="readonly")
        self.refresh_button.state(["!disabled"])
        self._reset_status_display()
        self.update_buttons(None)
        self.set_message("已断开。重新连接后必须重新 ARM。")
        self.refresh_ports()

    # ------------------------------------------------------------------ timers

    def _poll_events(self) -> None:
        client = self.client
        if client is not None:
            for kind, payload in client.poll():
                self._handle_event(kind, payload)
        self.root.after(50, self._poll_events)

    def _refresh_snapshot(self) -> None:
        client = self.client
        if client is not None:
            snapshot = client.snapshot()
            self._update_status(snapshot)
            self.update_buttons(snapshot)
            self.status_counter += 1
            if (
                self.status_counter % 3 == 0
                and snapshot["connection"] == ConnectionState.READY
            ):
                client.request_status()
        self.root.after(200, self._refresh_snapshot)

    def _handle_event(self, kind: str, payload) -> None:
        if kind == "log":
            direction = payload.get("dir", "?")
            text = payload.get("text", "")
            if direction == "rx" and text.startswith(("MOTOR_A_TEST:", "PONG MOTOR_A_TEST")) and not getattr(self.client, "select_unified_mode", False):
                self.disconnect()
                self.connection_var.set("固件模式不匹配")
                message = "ESP32 当前是 MOTOR_A_TEST 电机台架固件。请切换顶部“真实电机 · 台架测试”再连接 COM4；切换界面不会更换固件。"
                self.log(message, "error")
                self.set_message(message)
                return
            if direction == "rx" and text.startswith("{"):
                self._note_response(text)
            if not self.verbose_var.get():
                # 默认日志只保留高层事件；周期 STATUS/PONG/DRIVE 应答和
                # 启动噪声仅在“详细收发”里显示，避免淹没关键日志。
                return
            tag = "tx" if direction == "tx" else "rx"
            self.log(f"{direction.upper()}: {text}", tag)
            return

        if kind == "state_change":
            self._update_status(payload)
            self.update_buttons(payload)
            return

        if kind == "ready":
            self.log("同步完成：ESP32 处于 DISARMED，可以 ARM。")
            self.set_message("已就绪（DISARMED）。点击 ARM 开始模拟控制。")
            self._update_status(payload)
            self.update_buttons(payload)
            return

        if kind == "armed":
            self.armed = True
            self.active_directions.clear()
            self.log("ARM 成功，开始 10Hz DRIVE 流（零速也会刷新看门狗）。")
            self.set_message("已 ARM。按住方向键运动；松开后零速保持在使能状态。")
            if self.client is not None:
                self.client.set_motion(0, 0, 0)
            self.start_motion_loop()
            return

        if kind == "arm_failed":
            self.armed = False
            self.stop_motion_loop()
            error = payload.get("error", "?") if isinstance(payload, dict) else "?"
            self.log(f"ARM 失败：{error}。不会自动重试。", "error")
            self.set_message(f"ARM 失败：{error}。请查看状态后重新点击 ARM。")
            return

        if kind == "disarmed":
            self.armed = False
            self.active_directions.clear()
            self.stop_motion_loop()
            reason = ""
            if isinstance(payload, dict):
                reason = payload.get("reason", "")
                self.reason_var.set(str(reason or "—"))
            self.vx_var.set("0")
            self.vy_var.set("0")
            self.w_var.set("0")
            self.target_var.set("0 / 0 / 0")
            self.age_var.set("—")
            self.session_var.set("—")
            self.log(f"设备已 DISARMED（{reason or '原因未知'}）。")
            self.set_message("设备已撤销使能；需要重新 ARM。")
            return

        if kind == "watchdog":
            self.armed = False
            self.active_directions.clear()
            self.stop_motion_loop()
            self.reason_var.set("WATCHDOG")
            self.vx_var.set("0")
            self.vy_var.set("0")
            self.w_var.set("0")
            self.target_var.set("0 / 0 / 0")
            self.age_var.set("—")
            self.session_var.set("—")
            self.log("看门狗超时：ESP32 已撤销使能并清空目标。", "error")
            self.set_message("看门狗超时，已撤销使能；请重新 ARM。")
            return

        if kind == "drive_error":
            error = payload.get("error", "?") if isinstance(payload, dict) else "?"
            self.log(f"DRIVE 被拒绝：{error}", "error")
            self.set_message(f"DRIVE 被拒绝：{error}")
            if error in ("NOT_ARMED", "SESSION_MISMATCH"):
                self.armed = False
                self.active_directions.clear()
                self.stop_motion_loop()
            return

        if kind == "reboot":
            self.armed = False
            self.active_directions.clear()
            self.stop_motion_loop()
            self._clear_runtime_values()
            self.esp_state_var.set("UNKNOWN")
            old = payload.get("old_boot_id") if isinstance(payload, dict) else None
            new = payload.get("new_boot_id") if isinstance(payload, dict) else None
            self.log(f"ESP32 重启：boot_id {old} -> {new}。已清空会话。")
            self.set_message("ESP32 已重启；等待重新同步，不会自动 ARM。")
            return

        if kind == "error":
            self.log(f"错误：{payload}", "error")
            self.set_message(str(payload))
            return

        if kind == "disconnected":
            self.armed = False
            self.active_directions.clear()
            self.stop_motion_loop()
            client = self.client
            self.client = None
            if client is not None:
                try:
                    client.close(send_stop=False, timeout=0.1)
                except Exception:
                    pass
            self.connect_button.configure(text="连接")
            self.port_combo.configure(state="readonly")
            self.refresh_button.state(["!disabled"])
            self._reset_status_display()
            self.update_buttons(None)
            self.log(f"串口断开：{payload}", "error")
            self.set_message("串口已断开；重新连接后必须重新 ARM，旧指令不会恢复。")
            self.refresh_ports()
            return

    def _update_status(self, snapshot: dict | None) -> None:
        if snapshot is None:
            return
        connection = str(snapshot.get("connection"))
        self.connection_var.set(CONNECTION_LABELS.get(connection, connection))
        self.fw_var.set(
            f"{snapshot.get('fw') or '—'} / v{snapshot.get('protocol') or '—'}"
        )
        self.mode_var.set(snapshot.get("mode") or "—")
        state = snapshot.get("esp_state") or "UNKNOWN"
        if snapshot.get("stopping") and state != "DISARMED":
            self.esp_state_var.set("停止中…")
        else:
            self.esp_state_var.set(state)
        session = snapshot.get("session")
        self.session_var.set(str(session) if session else "—")
        self.seq_var.set(str(snapshot.get("last_seq", "—")))
        status = snapshot.get("last_status") or {}
        if status:
            vx = status.get("vx", "—")
            vy = status.get("vy", "—")
            w = status.get("w", "—")
            self.vx_var.set(str(vx))
            self.vy_var.set(str(vy))
            self.w_var.set(str(w))
            self.target_var.set(f"{vx} / {vy} / {w}")
            age = status.get("age_ms")
            if age is None or age < 0:
                self.age_var.set("—")
            else:
                remaining = max(0, WATCHDOG_MS - int(age))
                self.age_var.set(
                    f"age {age} ms（剩余约 {remaining} / {WATCHDOG_MS} ms）"
                )
            self.reason_var.set(str(status.get("reason", "—")))
        else:
            self.vx_var.set("—")
            self.vy_var.set("—")
            self.w_var.set("—")
            self.target_var.set("—")
            self.age_var.set("—")
            self.reason_var.set("—")
        self.boot_var.set(str(snapshot.get("boot_id") or "—"))
        self.mode_var.set(snapshot.get("mode") or "—")

        if snapshot.get("mode") not in (None, MODE_DRY_RUN):
            self.mode_var.set(f"{snapshot.get('mode')}（非 DRY_RUN！）")

        connection_colors = {
            ConnectionState.READY: "#087F78",
            ConnectionState.WAITING_IDENTITY: "#B26A00",
            ConnectionState.SYNCING: "#B26A00",
            ConnectionState.ERROR: "#C62828",
            ConnectionState.DISCONNECTED: "#64758A",
        }
        if hasattr(self, "connection_label"):
            self.connection_label.configure(
                fg=connection_colors.get(connection, "#64758A")
            )
        state_colors = {
            "ARMED": "#C62828",
            "DISARMED": "#087F78",
            "UNKNOWN": "#64758A",
            "停止中…": "#B26A00",
        }
        if hasattr(self, "state_label"):
            self.state_label.configure(
                fg=state_colors.get(self.esp_state_var.get(), "#64758A")
            )
        if (
            snapshot.get("connection") == ConnectionState.READY
            and state == "ARMED"
            and not self.armed
        ):
            self.set_message("设备处于 ARMED；若不是本面板发起的，请先 STOP。")

    def update_buttons(self, snapshot: dict | None) -> None:
        connected = self.client is not None
        connection = snapshot.get("connection") if snapshot else None
        can_arm = bool(snapshot and snapshot.get("can_arm")) and not self.armed
        can_stop = connected and connection in (
            ConnectionState.SYNCING,
            ConnectionState.READY,
        )
        ready = connected and connection == ConnectionState.READY

        self._set_enabled(self.arm_button, can_arm)
        self._set_enabled(self.disarm_button, can_stop)
        self._set_enabled(self.stop_button, can_stop)
        self._set_enabled(self.raw_button, ready)
        self._set_enabled(self.zero_button, self.armed)
        for button, _direction in self.direction_buttons:
            self._set_enabled(button, self.armed)

    @staticmethod
    def _set_enabled(widget, enabled: bool) -> None:
        if isinstance(widget, ttk.Widget):
            widget.state(["!disabled"] if enabled else ["disabled"])
        else:
            widget.configure(state="normal" if enabled else "disabled")

    # ------------------------------------------------------------------ control

    def arm(self) -> None:
        if self.client is None:
            self.set_message("未连接。")
            return
        session = secrets.randbelow(0x7FFFFFFF) + 1
        if session == self.last_session:
            session = session % 0x7FFFFFFF + 1
        self.last_session = session
        if not self.client.arm(session):
            self.set_message("当前状态不允许 ARM；请等待就绪且设备为 DISARMED。")
            return
        self.log(f"已发送 ARM 请求，session={session}。")
        self.set_message("ARM 请求已发送，等待设备应答…")

    def disarm(self) -> None:
        self.armed = False
        self.active_directions.clear()
        self.stop_motion_loop()
        if self.client is not None:
            self.client.disarm()
            self.log("已发送 DISARM（高优先级）。")
            self.set_message("DISARM 已发送；必须重新 ARM 才能再次运动。")

    def stop_motion(self) -> None:
        self.armed = False
        self.active_directions.clear()
        self.stop_motion_loop()
        if self.client is not None:
            self.client.stop()
            self.log("已发送 STOP（最高优先级），待发 DRIVE 已清空。")
            self.set_message("STOP 已发送；必须重新 ARM 才能再次运动。")
        else:
            self.set_message("未连接；没有可发送的 STOP。")

    def zero_motion(self) -> None:
        if not self.armed or self.client is None:
            return
        self.active_directions.clear()
        self.client.set_motion(0, 0, 0)
        self.set_message("已发送零速（仍保持 ARM）。")

    def direction_press(self, direction: str) -> None:
        if not self.armed:
            self.set_message("请先 ARM；方向键不会在未使能时发送运动。")
            return
        self.active_directions.add(direction)
        labels = [
            DIRECTION_LABELS.get(name, name)
            for name in sorted(self.active_directions)
        ]
        self.set_message("方向：" + " + ".join(labels))

    def direction_release(self, direction: str) -> None:
        if direction in self.active_directions:
            self.active_directions.discard(direction)

    def _on_focus_out(self, event) -> None:
        if self.armed:
            # Defer so focus can settle; only stop when focus left the app.
            self.root.after(10, self._check_window_focus)

    def _check_window_focus(self) -> None:
        try:
            if self.armed and self.root.focus_get() is None:
                self.stop_motion()
        except tk.TclError:
            pass

    def start_motion_loop(self) -> None:
        self.stop_motion_loop(keep_armed=True)
        self.last_motion_tick = time.monotonic()
        self.motion_timer = self.root.after(100, self._motion_tick)

    def stop_motion_loop(self, keep_armed: bool = False) -> None:
        if self.motion_timer is not None:
            try:
                self.root.after_cancel(self.motion_timer)
            except Exception:
                pass
            self.motion_timer = None
        if not keep_armed:
            self.armed = False

    def _motion_tick(self) -> None:
        if not self.armed or self.client is None:
            self.stop_motion_loop()
            return
        now = time.monotonic()
        gap = now - self.last_motion_tick
        self.last_motion_tick = now
        if gap > 0.35:
            self.log(f"界面卡顿 {gap * 1000:.0f} ms，主动 STOP；不补发旧动作。")
            self.armed = False
            self.active_directions.clear()
            self.stop_motion_loop()
            self.client.stop()
            self.set_message("界面卡顿超过 350ms，已发送 STOP；请重新 ARM。")
            return

        translation = int(self.translation_speed.get())
        rotation = int(self.rotation_speed.get())
        vx = 0
        vy = 0
        w = 0
        if "forward" in self.active_directions:
            vx += translation
        if "back" in self.active_directions:
            vx -= translation
        if "left" in self.active_directions:
            vy += translation
        if "right" in self.active_directions:
            vy -= translation
        if "ccw" in self.active_directions:
            w += rotation
        if "cw" in self.active_directions:
            w -= rotation
        self.client.set_motion(vx, vy, w)
        self.motion_timer = self.root.after(100, self._motion_tick)

    def send_raw(self) -> None:
        text = self.raw_var.get().strip()
        if not text:
            return
        first = text.split(maxsplit=1)[0].upper()
        if first == "STOP":
            self.stop_motion()
            self.raw_var.set("")
            return
        if first == "DISARM":
            self.disarm()
            self.raw_var.set("")
            return
        if first in ("ARM", "DRIVE"):
            self.set_message("原始命令不允许 ARM/DRIVE；请使用左侧控制区。")
            return
        if self.client is None or not self.client.send_raw(text):
            self.set_message("原始命令仅在已就绪时发送；未发送。")
            return
        self.log(f"RAW TX: {text}")
        self.raw_var.set("")

    # ------------------------------------------------------------------ close

    def on_close(self) -> None:
        self.stop_motion_loop()
        if hasattr(self, "camera_panel"):
            self.camera_panel.close()
        client = self.client
        self.client = None
        if client is not None:
            try:
                client.stop()
                client.close(send_stop=True, timeout=0.4)
            except Exception:
                pass
        self.root.destroy()


def run_live_test(root: tk.Tk, app: PanelApp, port: str) -> int:
    """Use the real panel widgets against a real ESP32, without mouse input."""
    app.refresh_ports()
    display = next(
        (name for name, device in app.port_map.items() if device == port), None
    )
    if display is None:
        print(f"panel live test FAIL: 未找到串口 {port}")
        root.destroy()
        return 2
    app.port_var.set(display)
    app.toggle_connection()

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        root.update()
        if app.client is not None and (
            app.client.snapshot()["connection"] == ConnectionState.READY
        ):
            break
        time.sleep(0.02)
    else:
        print("panel live test FAIL: 未在 10s 内就绪")
        if app.client is not None:
            app.disconnect()
        root.destroy()
        return 3

    app.arm()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        root.update()
        if app.armed:
            break
        time.sleep(0.02)
    if not app.armed:
        print("panel live test FAIL: ARM 未成功")
        if app.client is not None:
            print("snapshot:", app.client.snapshot())
            app.disconnect()
        root.destroy()
        return 4

    # Let the panel's 10 Hz motion loop send several zero DRIVEs.
    started = time.monotonic()
    while time.monotonic() - started < 0.6:
        root.update()
        time.sleep(0.02)
    snapshot = app.client.snapshot()
    armed_ok = snapshot["esp_state"] == "ARMED" and snapshot["session"] is not None

    # Exercise the panel's direction mapping: forward -> vx = slider value,
    # release -> vx = 0. Still zero physical motion because this is DRY_RUN.
    expected_vx = int(app.translation_speed.get())
    app.direction_press("forward")
    started = time.monotonic()
    while time.monotonic() - started < 0.35:
        root.update()
        time.sleep(0.02)
    app.client.request_status()
    direction_ok = False
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        root.update()
        status = app.client.snapshot()["last_status"]
        if status.get("vx") == expected_vx:
            direction_ok = True
            break
        time.sleep(0.02)

    app.direction_release("forward")
    started = time.monotonic()
    while time.monotonic() - started < 0.35:
        root.update()
        time.sleep(0.02)
    app.client.request_status()
    zero_ok = False
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        root.update()
        status = app.client.snapshot()["last_status"]
        if status.get("vx") == 0:
            zero_ok = True
            break
        time.sleep(0.02)

    app.stop_motion()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        root.update()
        if app.client is not None and (
            app.client.snapshot()["esp_state"] == "DISARMED"
        ):
            break
        time.sleep(0.02)
    stopped_ok = (
        app.client is not None
        and app.client.snapshot()["esp_state"] == "DISARMED"
    )

    app.disconnect()
    root.update()
    root.destroy()
    overall = armed_ok and stopped_ok and direction_ok and zero_ok
    print(
        "panel live test "
        f"{'PASS' if overall else 'FAIL'}: "
        f"armed={armed_ok} direction_vx={expected_vx}:{direction_ok} "
        f"release_zero={zero_ok} stopped={stopped_ok} "
        f"session={snapshot.get('session')}"
    )
    return 0 if overall else 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DRY_RUN 调试面板")
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="只构建界面并退出，不连接串口",
    )
    parser.add_argument(
        "--live-test",
        action="store_true",
        help="用面板自身连接真实 ESP32，执行 ARM→方向映射→零速→STOP 后退出",
    )
    parser.add_argument(
        "--camera-self-test",
        action="store_true",
        help="打开面板摄像头页读取几帧后退出（使用 camera_config.json）",
    )
    parser.add_argument("--port", default="COM4", help="--live-test 使用的串口")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = tk.Tk()
    app = PanelApp(root)
    if args.self_test:
        root.update_idletasks()
        root.update()
        root.destroy()
        print("panel self-test OK")
        return 0
    if args.live_test:
        return run_live_test(root, app, args.port)
    if args.camera_self_test:
        root.withdraw()
        app.camera_panel.start()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            root.update()
            if app.camera_panel.frame_count > 5:
                break
            time.sleep(0.02)
        frames = app.camera_panel.frame_count
        status = app.camera_panel.status_var.get()
        app.camera_panel.stop()
        root.destroy()
        print(
            f"camera self-test {'PASS' if frames > 0 else 'FAIL'}: "
            f"frames={frames} status={status}"
        )
        return 0 if frames > 0 else 6
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
