"""Shared visual theme and small vector widgets; no device or motion logic."""
import tkinter as tk
from tkinter import ttk

BG = "#F4F6FA"
INK = "#1B2940"
MUTED = "#7B889C"
LINE = "#E5EAF1"
TEAL = "#008C80"
FONT = "Microsoft YaHei UI"


def rounded(canvas, box, radius, **kwargs):
    x1, y1, x2, y2 = box
    r = min(radius, max(0, (x2-x1)/2), max(0, (y2-y1)/2))
    points = (x1+r, y1, x2-r, y1, x2, y1, x2, y1+r,
              x2, y2-r, x2, y2, x2-r, y2, x1+r, y2,
              x1, y2, x1, y2-r, x1, y1+r, x1, y1)
    return canvas.create_polygon(points, smooth=True, splinesteps=24, **kwargs)


class RoundedCard(tk.Frame):
    def __init__(self, parent, **kwargs):
        try:
            background = parent.cget("bg")
        except tk.TclError:
            background = BG
        super().__init__(parent, bg=background, bd=0, **kwargs)
        self.surface = tk.Canvas(self, bg=background, highlightthickness=0, bd=0)
        self.surface.place(x=0, y=0, relwidth=1, relheight=1)
        self.surface.tk.call("lower", self.surface._w)
        self.bind("<Configure>", self.draw)

    def cget(self, key):
        return "white" if key in ("bg", "background") else super().cget(key)

    def draw(self, event=None):
        self.surface.delete("all")
        rounded(self.surface, (1, 1, self.winfo_width()-1, self.winfo_height()-1), 12,
                fill="white", outline=LINE, width=1)


class RoundedButton(tk.Canvas):
    """Keyboard-accessible rounded button with explicit disabled handling."""
    def __init__(self, parent, text="", command=None, tone="primary", width=148, height=40, font=None, **kwargs):
        self.text, self.command, self.tone = text, command, tone
        self.button_state = kwargs.pop("state", "normal")
        self.hovered = self.focused = False
        self.fill_override = None
        self.color_override = None
        self.text_font = font or (FONT, 10, "bold")
        try:
            background = parent.cget("bg")
        except tk.TclError:
            background = "white"
        takefocus = kwargs.pop("takefocus", True)
        super().__init__(parent, width=width, height=height, bg=background, highlightthickness=0, bd=0, takefocus=takefocus, **kwargs)
        self.bind("<Configure>", self.draw)
        self.bind("<Enter>", lambda event: self.hover(True))
        self.bind("<Leave>", lambda event: self.hover(False))
        self.bind("<ButtonRelease-1>", lambda event: self.invoke())
        self.bind("<Return>", lambda event: self.invoke())
        self.bind("<space>", lambda event: self.invoke())
        self.bind("<FocusIn>", lambda event: self.focus(True))
        self.bind("<FocusOut>", lambda event: self.focus(False))

    def hover(self, value):
        self.hovered = value
        self.draw()

    def focus(self, value):
        self.focused = value
        self.draw()

    def invoke(self):
        if self.button_state != "disabled" and self.command:
            self.command()
        return "break"

    def configure(self, cnf=None, **kwargs):
        if isinstance(cnf, dict):
            kwargs = {**cnf, **kwargs}
            cnf = None
        for name in ("text", "command", "tone"):
            if name in kwargs:
                setattr(self, name, kwargs.pop(name))
        if "state" in kwargs:
            self.button_state = kwargs.pop("state")
        if "bg" in kwargs:
            self.fill_override = kwargs.pop("bg")
        if "fg" in kwargs:
            self.color_override = kwargs.pop("fg")
        result = super().configure(cnf, **kwargs) if cnf is not None or kwargs else None
        self.draw()
        return result

    config = configure

    def cget(self, key):
        if key == "state":
            return self.button_state
        if key == "text":
            return self.text
        return super().cget(key)

    __getitem__ = cget

    def draw(self, event=None):
        self.delete("all")
        palettes = {
            "primary": (TEAL, "#00766C", "white", "#E6EEEF", "#91A5AC"),
            "danger": ("#DC4A5D", "#C53449", "white", "#FBECEF", "#BC7380"),
            "secondary": ("#EDF2F7", "#E1EAF2", "#4A5D73", "#F0F3F7", "#9CA9B9"),
        }
        normal, hover, color, disabled, disabled_color = palettes[self.tone]
        if self.button_state == "disabled":
            fill, color = disabled, disabled_color
        else:
            fill = self.fill_override or (hover if self.hovered else normal)
            color = self.color_override or color
        self.tk.call(self._w, "configure", "-cursor", "arrow" if self.button_state == "disabled" else "hand2")
        w, h = self.winfo_width(), self.winfo_height()
        rounded(self, (1, 1, w-1, h-1), 9, fill=fill,
                outline="#59B7AE" if self.focused else fill, width=2 if self.focused else 0)
        self.create_text(w/2, h/2, text=self.text, fill=color, font=self.text_font)


def apply_theme(root):
    style = ttk.Style(root)
    style.theme_use("clam")
    root.option_add("*Font", (FONT, 10))
    style.configure("TFrame", background=BG)
    style.configure("Card.TFrame", background="white")
    style.configure("TLabel", background="white", foreground=INK)
    style.configure("Muted.TLabel", foreground=MUTED)
    style.configure("Title.TLabel", font=(FONT, 11, "bold"), foreground=INK)
    style.configure("TButton", padding=(13, 8), background="#EDF2F7", foreground="#4A5D73", borderwidth=0, focusthickness=0, font=(FONT, 9))
    style.layout("TButton", [("Button.padding", {"sticky": "nswe", "children": [("Button.label", {"sticky": "nswe"})]})])
    style.map("TButton", background=[("disabled", "#F0F3F7"), ("active", "#E1EAF2")], foreground=[("disabled", "#A1ADBC")])
    for name in ("Accent.TButton", "BenchAccent.TButton"):
        style.configure(name, background=TEAL, foreground="white", padding=(16, 9), font=(FONT, 10, "bold"))
        style.map(name, background=[("disabled", "#E6EEEF"), ("active", "#00766C")], foreground=[("disabled", "#91A5AC")])
    style.configure("Bench.TButton", padding=(13, 8))
    for name in ("TEntry", "TCombobox", "TSpinbox"):
        style.configure(name, fieldbackground="white", background="white", foreground=INK, bordercolor="#DDE5EF", lightcolor="white", darkcolor="white", padding=7, arrowsize=12, arrowcolor="#8191A5", borderwidth=1)
        style.map(name, fieldbackground=[("disabled", "#F3F5F8"), ("readonly", "white")], background=[("readonly", "white"), ("active", "#F0F5F8")], foreground=[("disabled", "#9CAABD")], bordercolor=[("focus", "#4CAEA5"), ("active", "#B5CDC9")], selectbackground=[("readonly", "white")], selectforeground=[("readonly", INK)])
    for name in ("TCheckbutton", "TRadiobutton"):
        style.configure(name, background="white", foreground="#64758A", padding=3, indicatorbackground="white", indicatorforeground=TEAL)
        style.map(name, background=[("active", "white"), ("selected", "white")], indicatorbackground=[("selected", TEAL), ("disabled", "#F0F3F7"), ("active", "#EDF7F5")])
    style.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(0, 4, 0, 0))
    style.configure("TNotebook.Tab", padding=(18, 10), background=BG, foreground=MUTED, borderwidth=0, font=(FONT, 10))
    style.map("TNotebook.Tab", background=[("selected", "white"), ("active", "#EAF2F5")], foreground=[("selected", TEAL), ("active", INK)], lightcolor=[("selected", "white")])
    style.layout("TNotebook.Tab", [("Notebook.tab", {"sticky": "nswe", "children": [("Notebook.padding", {"side": "top", "sticky": "nswe", "children": [("Notebook.label", {"side": "top", "sticky": ""})]})]})])
    style.configure("Horizontal.TProgressbar", troughcolor="#EDF2F6", background=TEAL, bordercolor="#EDF2F6", lightcolor=TEAL, darkcolor=TEAL, borderwidth=0, thickness=5)
    style.configure("Horizontal.TScale", background="white", troughcolor="#E5EDF2", borderwidth=0, sliderlength=18, sliderthickness=14, lightcolor="white", darkcolor="white")
    style.configure("Vertical.TScrollbar", background="#D6DFE9", troughcolor="#F4F6FA", borderwidth=0, arrowsize=10)
    style.configure("Treeview", rowheight=33, fieldbackground="white", background="white", foreground=INK, borderwidth=0, font=(FONT, 9))
    style.configure("Treeview.Heading", padding=(8, 10), background="#F4F7FA", foreground="#687B91", borderwidth=0, font=(FONT, 9, "bold"))
    style.map("Treeview", background=[("selected", "#E5F4F0")], foreground=[("selected", "#087C71")])
