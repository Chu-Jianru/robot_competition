"""Tk layout and styling, kept separate from serial and motion logic."""
from __future__ import annotations

import tkinter as tk
from tkinter import scrolledtext, ttk
from ui_theme import BG, INK, MUTED, LINE, TEAL, FONT, RoundedButton, RoundedCard, apply_theme


class PanelView:
    def _build_ui(self) -> None:
        self.root.configure(bg=BG)
        self.root.option_add("*Font", (FONT, 10))
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background="white")
        style.configure("TLabel", background="white", foreground=INK)
        style.configure("Muted.TLabel", foreground=MUTED)
        style.configure("Title.TLabel", font=(FONT, 11, "bold"))
        style.configure("TButton", padding=(12, 7), background="#EAF0F5", foreground=INK, borderwidth=0)
        style.map("TButton", background=[("active", "#DCE6EF")], foreground=[("disabled", "#9AA6B2")])
        style.configure("Accent.TButton", background=TEAL, foreground="white")
        style.map("Accent.TButton", background=[("disabled", "#DCE6E8"), ("active", "#066B65")], foreground=[("disabled", "#849A9C")])
        style.configure("TCheckbutton", background="white", foreground=MUTED)
        style.configure("TNotebook", background="white", borderwidth=0)
        style.configure("TNotebook.Tab", padding=(14, 8), background="#EDF2F6", foreground=MUTED)
        style.map("TNotebook.Tab", background=[("selected", "white")], foreground=[("selected", TEAL)])
        style.configure("TCombobox", padding=6, fieldbackground="white")
        apply_theme(self.root)

        self.header = None
        if not getattr(self.root, "embedded", False):
            self.header = tk.Frame(self.root, bg="white", padx=22, pady=15)
            self.header.pack(fill="x")
            tk.Label(self.header, text="机器人调试台", font=(FONT, 17, "bold"), bg="white", fg=INK).pack(side="left")
            tk.Label(self.header, text="DRY_RUN  ·  模拟模式", bg="#E7F4F0", fg=TEAL, padx=12, pady=6).pack(side="right")

        content = tk.Frame(self.root, bg=BG)
        content.pack(fill="both", expand=True, padx=20, pady=(10, 0))
        content.columnconfigure(0, weight=0, minsize=315)
        content.columnconfigure(1, weight=1)
        content.rowconfigure(2, weight=1)

        conn = self._card(content)
        conn.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        conn.columnconfigure(1, weight=1)
        ttk.Label(conn, text="设备连接", style="Title.TLabel").grid(row=0, column=0, padx=(14, 12), pady=12)
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, state="readonly", width=36)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(0, 8))
        self.refresh_button = ttk.Button(conn, text="刷新", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=(0, 8))
        self.connect_button = ttk.Button(conn, text="连接设备", style="Accent.TButton", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=(0, 14))

        summary = tk.Frame(content, bg=BG)
        summary.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        for col, (title, variable) in enumerate((
            ("连接状态", self.connection_var),
            ("控制状态", self.esp_state_var),
            ("最近有效应答", self.response_var),
        )):
            summary.columnconfigure(col, weight=1, uniform="status")
            card = self._card(summary)
            card.grid(row=0, column=col, sticky="nsew", padx=(0 if col == 0 else 8, 0))
            ttk.Label(card, text=title, style="Muted.TLabel").pack(anchor="w", padx=14, pady=(9, 3))
            label = tk.Label(card, textvariable=variable, bg="white", fg=INK, font=(FONT, 12, "bold"), anchor="w")
            label.pack(fill="x", padx=14, pady=(0, 10))
            if col == 0:
                self.connection_label = label
            elif col == 1:
                self.state_label = label
            else:
                self.response_label = label

        left = self._card(content)
        left.grid(row=2, column=0, sticky="nsew", padx=(0, 12))
        left.columnconfigure(0, weight=1)
        ttk.Label(left, text="手动控制", style="Title.TLabel").grid(row=0, column=0, sticky="w", padx=16, pady=(12, 2))
        ttk.Label(left, text="先使能，再按住方向；松开即零速", style="Muted.TLabel").grid(row=1, column=0, sticky="w", padx=16, pady=(0, 9))
        arms = ttk.Frame(left, style="Card.TFrame")
        arms.grid(row=2, column=0, sticky="ew", padx=14)
        arms.columnconfigure(0, weight=1)
        arms.columnconfigure(1, weight=1)
        self.arm_button = ttk.Button(arms, text="模拟使能", style="Accent.TButton", command=self.arm)
        self.arm_button.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        self.disarm_button = ttk.Button(arms, text="撤销使能", command=self.disarm)
        self.disarm_button.grid(row=0, column=1, sticky="ew")

        pad = ttk.Frame(left, style="Card.TFrame")
        pad.grid(row=3, column=0, pady=(10, 4))
        for col in range(3):
            pad.columnconfigure(col, weight=1)
        self._make_hold_button(pad, "↶ 逆时针", "ccw", 0, 0)
        self._make_hold_button(pad, "↑ 前进", "forward", 0, 1)
        self._make_hold_button(pad, "顺时针 ↷", "cw", 0, 2)
        self._make_hold_button(pad, "← 左移", "left", 1, 0)
        self.zero_button = RoundedButton(pad, text="零速\n保持使能", command=self.zero_motion, **self._pad_style())
        self.zero_button.grid(row=1, column=1, padx=3, pady=3)
        self._make_hold_button(pad, "右移 →", "right", 1, 2)
        self._make_hold_button(pad, "↓ 后退", "back", 2, 1)

        self._speed_slider(left, 4, "平移速度", self.translation_speed, 300, "mm/s")
        self._speed_slider(left, 5, "旋转速度", self.rotation_speed, 180, "°/s")
        self.stop_button = RoundedButton(
            left, text="停止并撤销使能    Esc", command=self.stop_motion,
            tone="danger", height=44, font=(FONT, 11, "bold"),
        )
        self.stop_button.grid(row=6, column=0, sticky="ew", padx=14, pady=(8, 8))
        ttk.Label(left, text="失去窗口焦点时停止并撤销使能", style="Muted.TLabel").grid(row=7, column=0, sticky="w", padx=16, pady=(0, 8))
        left.rowconfigure(8, weight=1)

        right = tk.Frame(content, bg=BG)
        right.grid(row=2, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)
        metrics = self._card(right)
        metrics.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        ttk.Label(metrics, text="设备回传目标", style="Title.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", padx=14, pady=(10, 3))
        for col, (title, variable, unit) in enumerate((
            ("前进  vx", self.vx_var, "mm/s"), ("左移  vy", self.vy_var, "mm/s"), ("逆时针  ω", self.w_var, "°/s"),
        )):
            metrics.columnconfigure(col, weight=1, uniform="metric")
            ttk.Label(metrics, text=title, style="Muted.TLabel").grid(row=1, column=col, sticky="w", padx=16)
            ttk.Label(metrics, textvariable=variable, font=("Segoe UI", 25, "bold"), foreground=TEAL).grid(row=2, column=col, sticky="w", padx=16)
            ttk.Label(metrics, text=unit, style="Muted.TLabel").grid(row=3, column=col, sticky="w", padx=16, pady=(0, 10))

        notebook = ttk.Notebook(right)
        notebook.grid(row=1, column=0, sticky="nsew")
        self.notebook = notebook
        log_frame = self._card(notebook)
        detail_frame = self._card(notebook)
        tools_frame = self._card(notebook)
        notebook.add(log_frame, text="通信日志")
        notebook.add(detail_frame, text="设备详情")
        notebook.add(tools_frame, text="协议工具")
        log_frame.rowconfigure(1, weight=1)
        log_frame.columnconfigure(0, weight=1)
        toolbar = ttk.Frame(log_frame, style="Card.TFrame")
        toolbar.grid(row=0, column=0, sticky="ew", padx=12, pady=10)
        ttk.Checkbutton(toolbar, text="详细收发", variable=self.verbose_var).pack(side="left")
        ttk.Checkbutton(toolbar, text="自动滚动", variable=self.autoscroll_var).pack(side="left", padx=6)
        ttk.Button(toolbar, text="导出", command=self.export_log).pack(side="right", padx=(4, 0))
        ttk.Button(toolbar, text="清空", command=self.clear_log).pack(side="right")
        self.log_text = scrolledtext.ScrolledText(
            log_frame, height=8, state="disabled", wrap="word", font=("Consolas", 10),
            bg="#172337", fg="#DBE6F2", insertbackground="white", relief="flat", borderwidth=0,
            padx=16, pady=14, selectbackground="#31516B",
        )
        self.log_text.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))
        for tag, color in (("info", "#DBE6F2"), ("tx", "#A5C8FC"), ("rx", "#7FD9C3"), ("error", "#FFA5A8")):
            self.log_text.tag_configure(tag, foreground=color)

        fields = (
            ("固件 / 协议", self.fw_var), ("运行模式", self.mode_var),
            ("目标 vx / vy / ω", self.target_var), ("设备报告的指令年龄", self.age_var),
            ("停止原因", self.reason_var), ("会话编号", self.session_var),
            ("最后指令序号", self.seq_var), ("启动编号", self.boot_var),
        )
        detail_frame.columnconfigure(1, weight=1)
        for row, (title, variable) in enumerate(fields):
            ttk.Label(detail_frame, text=title, style="Muted.TLabel").grid(row=row, column=0, sticky="nw", padx=14, pady=7)
            ttk.Label(detail_frame, textvariable=variable, wraplength=300).grid(row=row, column=1, sticky="w", padx=8, pady=7)

        ttk.Label(tools_frame, text="常用查询", style="Title.TLabel").pack(anchor="w", padx=14, pady=(14, 8))
        query = ttk.Frame(tools_frame, style="Card.TFrame")
        query.pack(fill="x", padx=14)
        ttk.Button(query, text="PING 连通测试", command=lambda: self.send_query("PING")).pack(side="left", padx=(0, 8))
        ttk.Button(query, text="STATUS 刷新状态", command=lambda: self.send_query("STATUS")).pack(side="left")
        ttk.Label(tools_frame, text="单行命令", style="Title.TLabel").pack(anchor="w", padx=14, pady=(20, 6))
        ttk.Label(tools_frame, text="运动与使能请使用左侧控制区。", style="Muted.TLabel").pack(anchor="w", padx=14)
        raw_row = ttk.Frame(tools_frame, style="Card.TFrame")
        raw_row.pack(fill="x", padx=14, pady=10)
        self.raw_entry = ttk.Entry(raw_row, textvariable=self.raw_var)
        self.raw_entry.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.raw_entry.bind("<Return>", lambda _event: self.send_raw())
        self.raw_button = ttk.Button(raw_row, text="发送", command=self.send_raw)
        self.raw_button.pack(side="left")

        footer = tk.Frame(self.root, bg=BG, padx=22, pady=9)
        footer.pack(fill="x")
        tk.Label(footer, textvariable=self.message_var, bg=BG, fg=MUTED, anchor="w", wraplength=1000).pack(fill="x")
        self.update_buttons(None)

    @staticmethod
    def _card(parent):
        return RoundedCard(parent)

    @staticmethod
    def _pad_style():
        return dict(width=91, height=55, font=(FONT, 9), tone="secondary", takefocus=False)

    def _make_hold_button(self, parent, text, direction, row, column):
        button = RoundedButton(parent, text=text, **self._pad_style())
        button.grid(row=row, column=column, padx=3, pady=3)
        button.bind("<ButtonPress-1>", lambda _event, d=direction: self.direction_press(d) if button.cget("state") != "disabled" else None)
        button.bind("<ButtonRelease-1>", lambda _event, d=direction: self.direction_release(d))
        button.bind("<Leave>", lambda _event, d=direction: (button.hover(False), self.direction_release(d)))
        self.direction_buttons.append((button, direction))

    @staticmethod
    def _speed_slider(parent, row, title, variable, maximum, unit):
        frame = tk.Frame(parent, bg="white")
        frame.grid(row=row, column=0, sticky="ew", padx=16, pady=2)
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text=title, style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=variable).grid(row=0, column=1, padx=4)
        ttk.Label(frame, text=unit, style="Muted.TLabel").grid(row=0, column=2)
        scale = ttk.Scale(frame, variable=variable, from_=0, to=maximum, orient="horizontal")
        scale.configure(command=lambda value: variable.set(round(float(value))))
        scale.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(4, 6))
