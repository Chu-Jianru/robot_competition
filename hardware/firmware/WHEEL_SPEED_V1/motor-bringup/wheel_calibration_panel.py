"""Small on-site panel for connecting the robot and calibrating wheel speed."""
from collections import deque
import re
import time
import tkinter as tk
from tkinter import messagebox, ttk

from serial.tools import list_ports

from chassis_link import ChassisLink


def connection_error(detail):
    lower = detail.lower()
    if any(word in lower for word in ("permissionerror", "access is denied", "拒绝访问", "access denied")):
        return "端口被占用或无法打开。关闭其他底盘窗口后重连。"
    if any(word in lower for word in ("could not open", "filenotfounderror", "应答超时", "未确认当前接线")):
        return "底盘未应答。确认小车上电，并选择蓝牙传出端口。"
    return detail


class WheelCalibrationPanel:
    def __init__(self, root):
        self.root = root
        self.link = None
        self.running = False
        self.cal_deadline = 0.
        self.log = deque(maxlen=60)
        self.port = tk.StringVar(value="COM6")
        self.layout = tk.StringVar(value="CBDA")
        self.status = tk.StringVar(value="等待连接底盘")
        self.calibration = tk.StringVar(value="连接后读取板载标定")
        self.raised = tk.BooleanVar()
        root.title("清扫机器人 · 轮速标定")
        root.minsize(560, 350)
        root.option_add("*Font", ("Microsoft YaHei UI", 11))
        root.configure(bg="#eef3f7")

        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("Card.TFrame", background="#ffffff")
        style.configure("Card.TLabel", background="#ffffff", foreground="#183046")
        style.configure("Muted.TLabel", background="#ffffff", foreground="#536878")
        style.configure("Title.TLabel", background="#eef3f7", foreground="#183046",
                        font=("Microsoft YaHei UI", 19, "bold"))
        style.configure("Accent.TButton", background="#00685e", foreground="#ffffff", padding=(14, 7))
        style.map("Accent.TButton", background=[("disabled", "#e5eaf0"), ("active", "#007d70")],
                  foreground=[("disabled", "#5d7080"), ("!disabled", "#ffffff")])
        style.configure("TButton", padding=(12, 7))
        style.configure("Card.TCheckbutton", background="#ffffff", foreground="#183046")
        style.map("Card.TCheckbutton", background=[("active", "#ffffff")])

        ttk.Label(root, text="轮速标定", style="Title.TLabel").pack(anchor="w", padx=24, pady=(16, 8))
        card = ttk.Frame(root, padding=16, style="Card.TFrame")
        card.pack(fill="both", expand=True, padx=24, pady=(0, 20))
        card.columnconfigure(1, weight=1)
        ttk.Label(card, text="串口", style="Card.TLabel").grid(row=0, column=0, sticky="w")
        self.port_field = ttk.Combobox(card, textvariable=self.port, width=12,
                                      postcommand=self.refresh_ports)
        self.port_field.grid(row=0, column=1, sticky="w", padx=(12, 0))
        self.connect_button = ttk.Button(card, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, sticky="e")
        ttk.Label(card, text="轮位", style="Card.TLabel").grid(row=1, column=0, sticky="w", pady=9)
        self.layout_field = ttk.Entry(card, textvariable=self.layout, width=14)
        self.layout_field.grid(row=1, column=1, sticky="w", padx=(12, 0))
        ttk.Label(card, text="左前 / 右前 / 左后 / 右后", style="Muted.TLabel").grid(row=1, column=2, sticky="e")
        ttk.Separator(card).grid(row=2, columnspan=3, sticky="ew", pady=(2, 10))
        ttk.Label(card, textvariable=self.status, style="Card.TLabel", wraplength=455).grid(
            row=3, columnspan=3, sticky="w")
        ttk.Label(card, textvariable=self.calibration, style="Muted.TLabel", wraplength=455).grid(
            row=4, columnspan=3, sticky="w", pady=(4, 10))
        self.confirm = ttk.Checkbutton(card, text="四轮已架空，轮子周围无遮挡（标定最高 100% 输出）",
                                       variable=self.raised, command=self.refresh, style="Card.TCheckbutton")
        self.confirm.grid(row=5, columnspan=3, sticky="w", pady=(0, 10))
        self.start_button = ttk.Button(card, text="开始标定", command=self.start, style="Accent.TButton")
        self.start_button.grid(row=6, column=0, sticky="w")
        self.stop_button = ttk.Button(card, text="停止 · Esc", command=self.stop)
        self.stop_button.grid(row=6, column=1, sticky="w", padx=8)
        ttk.Button(card, text="诊断日志", command=self.show_log).grid(row=6, column=2, sticky="e")

        root.bind("<Escape>", self.stop)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh_ports()
        self.refresh()
        root.after(50, self.poll)

    def refresh_ports(self):
        self.port_field.configure(values=[port.device for port in list_ports.comports()
                                          if "000000000000_" not in (port.hwid or "")])

    def refresh(self):
        link = self.link
        ready = link is not None and link.ready and link.layout_confirmed and link.closed_loop_supported
        for field in (self.port_field, self.layout_field):
            field.configure(state="disabled" if link else "normal")
        self.confirm.configure(state="normal" if ready and not self.running else "disabled")
        self.start_button.configure(state="normal" if ready and self.raised.get() and not self.running else "disabled")
        self.stop_button.configure(state="normal" if link is not None and link.ready else "disabled")
        self.connect_button.configure(text="断开" if link else "连接")

    def toggle_connection(self):
        if self.link is not None:
            self.disconnect()
            return
        port, layout = self.port.get().strip().upper(), self.layout.get().strip().upper()
        if not re.fullmatch(r"COM[1-9]\d*", port) or sorted(layout) != list("ABCD"):
            self.status.set("请输入正确的 COM 端口和四个不同的轮位通道。")
            return
        self.log.clear()
        self.port.set(port)
        self.layout.set(layout)
        self.link = ChassisLink(require_profile="FOUR_V2")
        self.link.connect(port)
        self.status.set(f"正在连接 {port} 并核对固件…")
        self.refresh()

    def start(self):
        link = self.link
        if not link or self.running or not self.raised.get() or not link.layout_confirmed:
            return
        if link.calibrate():
            self.running = True
            self.cal_deadline = time.monotonic() + 20.
            self.status.set("正在启动轮速标定…")
            self.refresh()
        else:
            self.status.set("底盘暂未就绪，请等停车确认后再试。")

    def stop(self, _event=None):
        if self.link and self.link.send_line("STOP"):
            self.raised.set(False)
            self.status.set("已请求停车，等待底盘确认…")
            self.refresh()

    def disconnect(self):
        if self.link:
            self.link.close()
            self.link = None
        self.running = False
        self.raised.set(False)
        self.status.set("已断开")
        self.calibration.set("连接后读取板载标定")
        self.refresh()

    def close(self):
        self.disconnect()
        self.root.destroy()

    def show_log(self):
        messagebox.showinfo("底盘诊断", "\n".join(list(self.log)[-14:]) or "暂无底盘应答", parent=self.root)

    def handle_event(self, link, kind, data):
        if kind == "ready":
            link.configure_layout(self.layout.get(), (1, 1, 1, 1))
            self.status.set(f"{data} 已连接，正在确认轮位…")
        elif kind == "layout":
            self.status.set("已连接，轮位已确认" if data else "轮位确认失败，请检查接线后重连。")
        elif kind == "cal_status":
            state = f"板载标定有效 · {data['vmax']:.0f} 计数/秒" if data["valid"] else "板载轮速参数待标定"
            self.calibration.set(state)
        elif kind == "cal_progress":
            self.cal_deadline = time.monotonic() + 20.
            parts = data.split()
            if len(parts) == 3 and parts[0] == "run" and parts[1] in ("0", "1", "2", "3"):
                self.status.set(f"正在标定 {'ABCD'[int(parts[1])]} 路 · 输出 {parts[2]}%")
            elif data == "start":
                self.status.set("标定已开始，等待各轮测速…")
        elif kind == "cal_done":
            self.running = False
            self.raised.set(False)
            if data["valid"]:
                self.status.set("标定已保存。断开后，可用菜单 11 做车体标定。")
                self.calibration.set(f"板载标定有效 · {data['vmax']:.0f} 计数/秒")
            else:
                self.status.set("设备未保存有效的轮速标定，可查看诊断日志。")
        elif kind in ("cal_fail", "fault"):
            self.running = False
            self.raised.set(False)
            self.status.set(f"标定已停止：{data}")
        elif kind in ("error", "reset"):
            self.log.append(str(data))
            if self.running and "应答超时" in str(data):
                self.status.set("轮速标定时底盘失联。请看诊断日志，或改用 ESP32 的 USB 串口重试。")
            else:
                self.status.set(connection_error(str(data)))
            self.running = False
            self.raised.set(False)
        elif kind == "closed":
            self.link = None
            self.running = False
            self.raised.set(False)
        elif kind == "rx" and data.startswith(("CAL", "ERROR", "FAULT", "CAPS MOTORS", "MODE_OK", "LAYOUT",
                                               "STOPPED", "UNIFIED_BOOT", "rst:", "Brownout")):
            self.log.append(data)
            if data == "STOPPED command" and self.running:
                self.running = False
                self.raised.set(False)
                self.status.set("已停车")
            elif data.startswith("ERROR") and self.running:
                self.running = False
                self.raised.set(False)
                self.status.set(f"底盘拒绝标定：{data}")

    def poll(self):
        link = self.link
        if link is not None:
            for kind, data in link.poll():
                self.handle_event(link, kind, data)
            if self.running and time.monotonic() > self.cal_deadline:
                self.stop()
                self.running = False
                self.status.set("标定没有继续返回进度，已请求停车。")
            self.refresh()
        self.root.after(50, self.poll)


if __name__ == "__main__":
    root = tk.Tk()
    WheelCalibrationPanel(root)
    root.mainloop()
