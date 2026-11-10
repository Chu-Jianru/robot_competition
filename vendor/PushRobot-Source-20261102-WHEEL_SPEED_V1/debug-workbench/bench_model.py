"""编码器观测、运行记录与参数保存；不依赖窗口或真实串口，便于离线测试。"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta, timezone
from decimal import Decimal, InvalidOperation
import csv
import json
from pathlib import Path
import re

MAX_MS = 2_147_483_647
LOCAL_TIME = timezone(timedelta(hours=8))


def timestamp():
    return datetime.now(LOCAL_TIME).isoformat(timespec="milliseconds")


def duration_ms(text, unit):
    # 用十进制数换算，避免 0.001 秒等输入因浮点误差被误判；固件使用整数毫秒。
    try:
        value = Decimal(str(text)) * (1000 if unit == "秒" else 1)
        if not value.is_finite() or value != value.to_integral_value() or not 1 <= value <= MAX_MS:
            raise ValueError
        return int(value)
    except (InvalidOperation, ValueError):
        raise ValueError("时间必须大于零，精确到毫秒；例如 2.5 秒或 2500 毫秒") from None


def load_settings(path):
    defaults = dict(duty="50", duration="1", unit="秒", ppr="", ppr_by_channel={}, port="COM4", history=[])
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            return defaults
        duty = int(data.get("duty", 50))
        unit = data.get("unit", "秒")
        if not 1 <= duty <= 100 or unit not in ("秒", "毫秒"):
            return defaults
        duration_ms(data.get("duration", "1"), unit)
        defaults.update(duty=str(duty), duration=str(data.get("duration", "1")), unit=unit)
        ppr = str(data.get("ppr", ""))
        if ppr and Decimal(ppr).is_finite() and Decimal(ppr) > 0:
            defaults["ppr"] = ppr
        calibration = data.get("ppr_by_channel", {"A": defaults["ppr"]})
        if isinstance(calibration, dict):
            for channel in "ABCD":
                try:
                    value = str(calibration.get(channel, ""))
                    if value and Decimal(value).is_finite() and 0 < Decimal(value) <= 1e12:
                        defaults["ppr_by_channel"][channel] = value
                except (InvalidOperation, ValueError):
                    pass
        defaults["port"] = str(data.get("port", "COM4"))[:40]
        history = data.get("history", [])
        if isinstance(history, list):
            defaults["history"] = [s for s in history if isinstance(s, str) and s and len(s) <= 63 and all(32 <= ord(c) <= 126 for c in s)][-50:]
    except (OSError, ValueError, TypeError, InvalidOperation):
        pass
    return defaults


def save_settings(path, settings):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # 先写临时文件再替换，避免程序中断后留下半份 JSON。
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(settings, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


@dataclass
class Sample:
    timestamp: str
    elapsed_s: float
    segment: int
    channel: str
    ticks: int
    invalid: int
    a: int
    b: int
    counts_s: float | None
    rpm: float | None
    counts_per_rev: float | None


@dataclass
class Run:
    timestamp: str
    channel: str
    duty: int
    requested_ms: int
    observed_s: float
    ticks_delta: int | None
    invalid_delta: int | None
    result: str
    final_sample: bool


@dataclass
class ChannelData:
    """每路单独维护计数差分、清零状态和标定，防止切通道时曲线串台。"""
    latest: Sample | None = None
    last_sample_at: float | None = None
    rate_anchor: tuple | None = None
    ppr: float | None = None
    segment: int = 0
    zero_pending: bool = False


def per_channel(name):
    """Public selected-channel values, preserving the original panel API."""
    return property(lambda self: getattr(self.channels[self.channel], name),
                    lambda self, value: setattr(self.channels[self.channel], name, value))


class BenchData:
    latest = per_channel("latest")
    last_sample_at = per_channel("last_sample_at")
    rate_anchor = per_channel("rate_anchor")
    ppr = per_channel("ppr")
    segment = per_channel("segment")
    zero_pending = per_channel("zero_pending")

    def __init__(self, now=0.0, limit=20000):
        self.channel = "A"
        self.channels = {name: ChannelData() for name in "ABCD"}
        self.started = now
        self.samples = deque(maxlen=limit)
        self.runs = deque(maxlen=500)
        self.logs = deque(maxlen=5000)
        self.dropped = 0
        self.segment = 0
        self.latest = None
        self.last_sample_at = None
        self.rate_anchor = None
        self.last_reply_at = None
        self.ppr = None
        self.motor_state = "未连接"
        self.active = None
        self.zero_pending = False
        self.revision = 0

    def log(self, kind, text):
        self.logs.append((timestamp(), kind, text))

    def select_channel(self, channel):
        if channel not in self.channels:
            raise ValueError("Unknown motor channel")
        self.channel = channel
        self.revision += 1

    def clear_baseline(self, zero=False, channel=None):
        # 清零/重连时切断差分基准，不能把计数重置误当成一次高速反转。
        state = self.channels[channel or self.channel]
        state.segment += 1
        state.last_sample_at = None
        state.rate_anchor = None
        state.zero_pending = zero

    def disconnect(self, now, reason="断开，停止状态未确认"):
        if self.active:
            self._finish(now, reason, False)
        for channel, state in self.channels.items():
            self.clear_baseline(channel=channel)
            state.latest = None
        self.last_reply_at = None
        self.motor_state = "未连接"

    def receive(self, line, now):
        match = re.fullmatch(r"ENC (?:([A-D]) )?ticks=(-?\d+) invalid=(\d+) A=([01]) B=([01])", line)
        if match:
            channel = match[1] or "A"
            state = self.channels[channel]
            ticks, invalid, a, b = map(int, match.groups()[1:])
            rate = None
            if state.zero_pending:
                if ticks == 0:
                    state.zero_pending = False
                state.rate_anchor = None
            elif state.rate_anchor is not None:
                dt = now - state.rate_anchor[0]
                if 0.1 <= dt <= 3:
                    rate = (ticks - state.rate_anchor[1]) / dt
            if not state.zero_pending and (state.rate_anchor is None or now - state.rate_anchor[0] >= 0.1):
                state.rate_anchor = (now, ticks)
            state.latest = Sample(timestamp(), round(now - self.started, 4), state.segment, channel, ticks, invalid, a, b,
                                  rate, rate * 60 / state.ppr if rate is not None and state.ppr else None, state.ppr)
            state.last_sample_at = now
            if len(self.samples) == self.samples.maxlen:
                self.dropped += 1
            self.samples.append(state.latest)
            if self.active and self.active["channel"] == channel and self.active.get("stop_at") is not None:
                # 停止后只用本次运行通道的末次 ENC 完成记录，其他三路不能代替。
                self._finish(now, self.active["reason"], True)
            self.last_reply_at = now
            self.revision += 1
            return True
        zero = re.fullmatch(r"ZEROED ([A-D]|ALL)", line)
        if zero:
            for channel in ("ABCD" if zero[1] == "ALL" else zero[1]):
                self.clear_baseline(zero=True, channel=channel)
            self.last_reply_at = now
            return True
        jog = re.fullmatch(r"JOG ([A-D]) (-?\d+) percent (\d+) ms;.*", line)
        if jog:
            initial = self.channels[jog[1]].latest
            self.active = dict(timestamp=timestamp(), channel=jog[1], duty=int(jog[2]), duration=int(jog[3]), start=now,
                               start_ticks=initial.ticks if initial else None,
                               start_invalid=initial.invalid if initial else None, stop_at=None)
            self.motor_state = "运行中"
        elif re.fullmatch(r"RUNNING [A-D]", line):
            self.motor_state = "运行中"
        elif line.startswith("STOPPED"):
            self.motor_state = "已停止"
            if self.active and self.active.get("stop_at") is None:
                self.active.update(stop_at=now, reason=line)
        elif not line.startswith(("PONG MOTOR_A_TEST", "ERROR ", "MOTOR_A_TEST limits:", "Link timeout:")):
            return False
        self.last_reply_at = now
        return True

    def tick(self, now):
        if self.active and self.active.get("stop_at") is not None and now - self.active["stop_at"] > 1:
            self._finish(now, self.active["reason"] + "（末次编码器读数缺失）", False)

    def _finish(self, now, reason, final_sample):
        # 断连或缺少末次读数时保留“未确认”，不把观测缺失伪造为零位移。
        active = self.active
        delta = invalid = None
        latest = self.channels[active["channel"]].latest
        if final_sample and latest:
            if active["start_ticks"] is not None:
                delta = latest.ticks - active["start_ticks"]
            if active["start_invalid"] is not None:
                invalid = latest.invalid - active["start_invalid"]
        stopped_at = active.get("stop_at") or now
        self.runs.append(Run(active["timestamp"], active["channel"], active["duty"], active["duration"], round(stopped_at - active["start"], 3), delta, invalid, reason, final_sample))
        self.active = None
        self.revision += 1

    def export_csv(self, path, runs=False):
        rows = self.runs if runs else self.samples
        cls = Run if runs else Sample
        with Path(path).open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(cls.__dataclass_fields__))
            writer.writeheader()
            writer.writerows(asdict(row) for row in rows)
