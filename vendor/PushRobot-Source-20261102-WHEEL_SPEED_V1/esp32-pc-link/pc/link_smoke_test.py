"""Physical acceptance test for the DRY_RUN firmware.

WARNING: This script talks to real hardware over a real COM port. It never
commands motors, but it does verify that the ESP32 rejects motion before ARM,
enforces the 300 ms watchdog, and does not resume old motion after reconnect.

Run:
    .venv\\Scripts\\python.exe link_smoke_test.py --port COM4
    .venv\\Scripts\\python.exe link_smoke_test.py --port COM4 --skip-manual

The crash/reconnect test spawns a child process that exits with os._exit()
while armed, then reconnects and checks that the device is DISARMED.
"""

from __future__ import annotations

import argparse
import json
import os
import secrets
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

import serial

from link_client import LineFramer, MODE_DRY_RUN, PROTOCOL_VERSION

WATCHDOG_MS = 300


def new_session() -> int:
    return secrets.randbelow(0x7FFFFFFF) + 1


class RawLink:
    """Minimal request/response helper used only by this acceptance script."""

    def __init__(self, port: str, baud: int = 115200, verbose: bool = False) -> None:
        self.port = port
        self.verbose = verbose
        self.ser = serial.Serial(
            port=port, baudrate=baud, timeout=0.05, write_timeout=0.3
        )
        self.framer = LineFramer()
        self.queue: deque = deque()
        self.boot_id: int | None = None
        self.arm_token: int | None = None
        self.state: str | None = None

    def send(self, text: str) -> None:
        self.ser.write((text + "\n").encode("ascii"))
        self.ser.flush()

    def send_slow(self, text: str, delay: float = 0.05) -> None:
        for char in text:
            self.ser.write(char.encode("ascii"))
            self.ser.flush()
            time.sleep(delay)
        self.ser.write(b"\n")
        self.ser.flush()
        time.sleep(delay)

    def _update_from(self, message: dict) -> None:
        if isinstance(message.get("boot_id"), int):
            self.boot_id = message["boot_id"]
        if isinstance(message.get("arm_token"), int):
            self.arm_token = message["arm_token"]
        if message.get("state") in ("ARMED", "DISARMED"):
            self.state = message["state"]

    def read_message(self, timeout: float = 1.0) -> dict | None:
        deadline = time.monotonic() + timeout
        while True:
            if self.queue:
                return self.queue.popleft()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                raise
            if not data:
                continue
            for line in self.framer.feed(data):
                if not line.startswith("{"):
                    if self.verbose:
                        print(f"  [boot/raw] {line[:160]}")
                    continue
                try:
                    message = json.loads(line)
                except (ValueError, TypeError):
                    continue
                if not isinstance(message, dict):
                    continue
                self._update_from(message)
                self.queue.append(message)

    def wait_message(
        self,
        *,
        kind: str | None = None,
        cmd: str | None = None,
        event: str | None = None,
        timeout: float = 2.0,
    ) -> dict | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.read_message(min(0.1, max(0.01, deadline - time.monotonic())))
            if message is None:
                continue
            if kind is not None and message.get("type") != kind:
                continue
            if cmd is not None and message.get("cmd") != cmd:
                continue
            if event is not None and message.get("event") != event:
                continue
            return message
        return None

    def close(self, send_stop: bool = True) -> None:
        try:
            if send_stop:
                self.send("STOP")
                time.sleep(0.05)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


def open_link(
    port: str, *, verbose: bool = False, identify_timeout: float = 5.0
) -> RawLink:
    link = RawLink(port, verbose=verbose)
    started = time.monotonic()
    last_ping = 0.0
    identity = None
    try:
        while time.monotonic() - started < identify_timeout:
            message = link.read_message(0.2)
            if message is not None:
                if message.get("v") != PROTOCOL_VERSION:
                    raise RuntimeError(
                        f"协议版本不匹配: {message.get('v')!r} != {PROTOCOL_VERSION}"
                    )
                if message.get("type") in ("hello", "pong", "status"):
                    identity = message
                    break
            elapsed = time.monotonic() - started
            if elapsed >= 1.5 and time.monotonic() - last_ping >= 0.3:
                last_ping = time.monotonic()
                link.send("PING")
        if identity is None:
            raise RuntimeError("等待 hello/PONG/STATUS 超时")
        if identity.get("mode") != MODE_DRY_RUN:
            raise RuntimeError(f"设备模式不是 DRY_RUN: {identity.get('mode')!r}")
        if link.arm_token is None:
            raise RuntimeError("身份消息缺少 arm_token")
        return link
    except Exception:
        link.close(send_stop=False)
        raise


def sync_disarmed(link: RawLink, timeout: float = 2.0) -> dict:
    for _attempt in range(2):
        link.send("STOP")
        link.wait_message(kind="ack", cmd="STOP", timeout=timeout)
        link.send("STATUS")
        status = link.wait_message(kind="status", timeout=timeout)
        if status is not None and status.get("state") == "DISARMED":
            return status
    raise RuntimeError("设备没有进入 DISARMED")


def query_status(link: RawLink, timeout: float = 2.0) -> dict:
    link.send("STATUS")
    status = link.wait_message(kind="status", timeout=timeout)
    if status is None:
        raise RuntimeError("STATUS 超时")
    return status


def arm(link: RawLink, session: int, timeout: float = 2.0) -> dict:
    if link.arm_token is None:
        raise RuntimeError("缺少当前 arm_token")
    link.send(f"ARM {session} {link.arm_token}")
    ack = link.wait_message(kind="ack", cmd="ARM", timeout=timeout)
    if ack is None or not ack.get("ok"):
        raise RuntimeError(f"ARM 失败: {ack}")
    status = query_status(link)
    if status.get("state") != "ARMED":
        raise RuntimeError(f"ARM 后状态不是 ARMED: {status}")
    return status


def expect_ack(
    link: RawLink, cmd: str, *, ok: bool, error: str | None = None, timeout: float = 2.0
) -> dict:
    ack = link.wait_message(kind="ack", cmd=cmd, timeout=timeout)
    if ack is None:
        raise RuntimeError(f"等待 {cmd} ack 超时")
    if bool(ack.get("ok")) != ok:
        raise RuntimeError(f"{cmd} ack ok 不符: {ack}")
    if error is not None and ack.get("error") != error:
        raise RuntimeError(f"{cmd} ack 错误码不符: {ack}")
    return ack


def poll_until_disarmed(link: RawLink, timeout: float = 1.2) -> tuple[dict | None, float]:
    started = time.monotonic()
    last = None
    while time.monotonic() - started < timeout:
        last = query_status(link)
        if last.get("state") == "DISARMED":
            return last, (time.monotonic() - started) * 1000.0
        time.sleep(0.02)
    return last, (time.monotonic() - started) * 1000.0


class Recorder:
    def __init__(self) -> None:
        self.results: list[dict] = []

    def record(
        self, name: str, ok: bool, detail: str = "", skipped: bool = False
    ) -> None:
        self.results.append(
            {
                "name": name,
                "ok": bool(ok) if not skipped else True,
                "skipped": bool(skipped),
                "detail": detail,
            }
        )
        status = "SKIP" if skipped else ("PASS" if ok else "FAIL")
        print(f"[{status}] {name} {detail}")

    def ok(self) -> bool:
        return all(
            item["ok"] for item in self.results if not item.get("skipped")
        )

    def counts(self) -> tuple[int, int, int]:
        passed = sum(
            1
            for item in self.results
            if item["ok"] and not item.get("skipped")
        )
        failed = sum(
            1
            for item in self.results
            if not item["ok"] and not item.get("skipped")
        )
        skipped = sum(1 for item in self.results if item.get("skipped"))
        return passed, failed, skipped


def run_crash_child(port: str) -> int:
    """Child process: arm, drive, then exit without closing the port."""
    link = open_link(port)
    sync_disarmed(link)
    session = new_session()
    arm(link, session)
    link.send(f"DRIVE {session} 1 40 0 0")
    ack = link.wait_message(kind="ack", cmd="DRIVE", timeout=2.0)
    if ack is None or not ack.get("ok"):
        print(f"CHILD_FAIL ack={ack}", flush=True)
        link.close(send_stop=False)
        return 4
    print(f"CHILD_SESSION={session}", flush=True)
    print(f"CHILD_TOKEN={link.arm_token}", flush=True)
    print("CHILD_ARMED_AND_DRIVING_EXIT_NOW", flush=True)
    os._exit(0)


def run_smoke(args: argparse.Namespace) -> int:
    recorder = Recorder()
    link: RawLink | None = None
    port = args.port

    try:
        link = open_link(port, verbose=args.verbose)
        recorder.record(
            "identity",
            link.arm_token is not None,
            f"boot_id={link.boot_id} arm_token={link.arm_token}",
        )
        status = sync_disarmed(link)
        recorder.record(
            "sync_disarmed",
            status.get("state") == "DISARMED",
            f"fw={status.get('fw')} mode={status.get('mode')} reason={status.get('reason')}",
        )

        link.send_slow("PING", delay=0.04)
        pong = link.wait_message(kind="pong", timeout=2.0)
        recorder.record("slow_fragmented_ping", pong is not None, "逐字节 40ms 间隔输入")

        link.send("A" * 300)
        too_long = link.wait_message(kind="ack", cmd="", timeout=2.0)
        recorder.record(
            "overlong_line_rejected",
            too_long is not None and too_long.get("error") == "TOO_LONG",
            f"ack={too_long}",
        )
        link.send("PING")
        resync = link.wait_message(kind="pong", timeout=2.0)
        recorder.record("overlong_resync", resync is not None)

        link.send("DRIVE 1 1 0 0 0")
        not_armed = expect_ack(link, "DRIVE", ok=False, error="NOT_ARMED")
        recorder.record(
            "drive_before_arm_rejected", not_armed.get("error") == "NOT_ARMED"
        )

        session = new_session()
        arm_status = arm(link, session)
        recorder.record(
            "arm_with_token",
            arm_status.get("state") == "ARMED",
            f"session={session}",
        )
        link.send(f"DRIVE {session} 1 50 -20 10")
        expect_ack(link, "DRIVE", ok=True)
        status = query_status(link)
        recorder.record(
            "drive_accepted",
            status.get("state") == "ARMED"
            and status.get("session") == session
            and status.get("last_seq") == 1
            and status.get("vx") == 50
            and status.get("vy") == -20
            and status.get("w") == 10,
            f"target={status.get('vx')},{status.get('vy')},{status.get('w')} last_seq={status.get('last_seq')}",
        )

        link.send(f"DRIVE {session} 1 0 0 0")
        stale = expect_ack(link, "DRIVE", ok=False, error="STALE_SEQ")
        target_after = query_status(link)
        recorder.record(
            "duplicate_seq_rejected",
            target_after.get("vx") == 50 and target_after.get("last_seq") == 1,
            f"target_after={target_after.get('vx')},{target_after.get('vy')},{target_after.get('w')}",
        )

        # Zero-speed DRIVE refreshes the watchdog; then let it expire.
        token_before = link.arm_token
        session_before = session
        link.send(f"DRIVE {session} 2 0 0 0")
        expect_ack(link, "DRIVE", ok=True)
        disarmed, elapsed_ms = poll_until_disarmed(link, timeout=1.2)
        recorder.record(
            "zero_drive_refresh_and_watchdog",
            disarmed is not None
            and disarmed.get("reason") == "WATCHDOG"
            and 250.0 <= elapsed_ms <= 500.0,
            f"检测到 DISARMED 用时约 {elapsed_ms:.0f} ms（轮询分辨率约 20ms）",
        )
        new_token = link.arm_token

        link.send(f"ARM {new_session()} {token_before}")
        stale_token = expect_ack(link, "ARM", ok=False, error="STALE_ARM_TOKEN")
        recorder.record(
            "old_token_rejected_after_watchdog",
            stale_token.get("error") == "STALE_ARM_TOKEN",
        )

        link.send(f"ARM {session_before} {new_token}")
        reused = expect_ack(link, "ARM", ok=False, error="SESSION_REUSED")
        recorder.record(
            "old_session_rejected_after_watchdog",
            reused.get("error") == "SESSION_REUSED",
        )

        session2 = new_session()
        arm(link, session2)
        link.send(f"DRIVE {session2} 1 25 0 0")
        expect_ack(link, "DRIVE", ok=True)
        link.send("STOP")
        expect_ack(link, "STOP", ok=True)
        status = query_status(link)
        recorder.record(
            "stop_disarms",
            status.get("state") == "DISARMED" and status.get("reason") == "STOP",
            f"reason={status.get('reason')}",
        )
        link.send(f"DRIVE {session2} 2 10 0 0")
        stopped_drive = expect_ack(link, "DRIVE", ok=False, error="NOT_ARMED")
        recorder.record(
            "drive_after_stop_rejected",
            stopped_drive.get("error") == "NOT_ARMED",
        )
        link.send(f"ARM {new_session()} {new_token}")
        stale_after_stop = expect_ack(link, "ARM", ok=False, error="STALE_ARM_TOKEN")
        recorder.record(
            "old_token_rejected_after_stop",
            stale_after_stop.get("error") == "STALE_ARM_TOKEN",
        )

        # PC stall without closing the port: no commands for 500 ms.
        session3 = new_session()
        arm(link, session3)
        link.send(f"DRIVE {session3} 1 30 0 0")
        expect_ack(link, "DRIVE", ok=True)
        time.sleep(0.5)
        stalled_status = query_status(link)
        recorder.record(
            "stall_watchdog",
            stalled_status.get("state") == "DISARMED"
            and stalled_status.get("reason") == "WATCHDOG",
            f"state={stalled_status.get('state')} reason={stalled_status.get('reason')}",
        )

        # Child-process crash while armed.
        link.close(send_stop=True)
        link = None
        child = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--port",
                port,
                "--phase",
                "crash-child",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )
        child_out = child.stdout + child.stderr
        child_session = None
        child_token = None
        for line in child_out.splitlines():
            if line.startswith("CHILD_SESSION="):
                child_session = int(line.split("=", 1)[1])
            elif line.startswith("CHILD_TOKEN="):
                child_token = int(line.split("=", 1)[1])
        recorder.record(
            "child_process_armed_then_crashed",
            "CHILD_ARMED_AND_DRIVING_EXIT_NOW" in child_out,
            child_out.strip().replace("\n", " | ")[:300],
        )

        link = None
        for _attempt in range(25):
            try:
                link = open_link(port, verbose=args.verbose)
                break
            except Exception:
                time.sleep(0.2)
        if link is None:
            raise RuntimeError("子进程退出后无法重新打开串口")
        crashed_status = sync_disarmed(link)
        recorder.record(
            "crash_reconnect_is_disarmed",
            crashed_status.get("state") == "DISARMED",
            f"boot_id={link.boot_id} reason={crashed_status.get('reason')}",
        )
        if child_session is not None:
            link.send(f"DRIVE {child_session} 2 10 0 0")
            old_drive = expect_ack(link, "DRIVE", ok=False, error="NOT_ARMED")
            recorder.record(
                "old_session_drive_after_crash_rejected",
                old_drive.get("error") == "NOT_ARMED",
            )
        if child_token is not None:
            link.send(f"ARM {new_session()} {child_token}")
            old_token_ack = expect_ack(
                link, "ARM", ok=False, error="STALE_ARM_TOKEN"
            )
            recorder.record(
                "old_token_after_crash_rejected",
                old_token_ack.get("error") == "STALE_ARM_TOKEN",
            )

        if not args.skip_manual:
            old_boot = link.boot_id
            link.close(send_stop=True)
            link = None
            input(
                "请拔下 ESP32 的 USB 线，等待 2 秒后重新插上，然后按回车继续..."
            )
            link = open_link(port, verbose=args.verbose)
            manual_status = sync_disarmed(link)
            recorder.record(
                "manual_usb_replug_is_disarmed",
                manual_status.get("state") == "DISARMED",
                f"boot_id {old_boot} -> {link.boot_id}",
            )

            old_boot = link.boot_id
            link.close(send_stop=True)
            link = None
            input("请按一下 ESP32 的 EN 复位键，然后按回车继续...")
            link = open_link(port, verbose=args.verbose)
            reset_status = sync_disarmed(link)
            recorder.record(
                "manual_en_reset_is_disarmed",
                reset_status.get("state") == "DISARMED",
                f"boot_id {old_boot} -> {link.boot_id}",
            )
        else:
            recorder.record(
                "manual_usb_replug_is_disarmed",
                True,
                "未执行（--skip-manual）",
                skipped=True,
            )
            recorder.record(
                "manual_en_reset_is_disarmed",
                True,
                "未执行（--skip-manual）",
                skipped=True,
            )

    except Exception as exc:
        recorder.record("exception", False, f"{type(exc).__name__}: {exc}")
    finally:
        if link is not None:
            link.close(send_stop=True)

    print("\n==== 验收结果 ====")
    for item in recorder.results:
        status = (
            "SKIP"
            if item.get("skipped")
            else ("PASS" if item["ok"] else "FAIL")
        )
        print(f"{status}  {item['name']}  {item['detail']}")
    passed, failed, skipped = recorder.counts()
    print(f"\n通过 {passed}，失败 {failed}，跳过 {skipped}（跳过不计入失败）")

    output = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "port": port,
        "results": recorder.results,
    }
    out_path = Path(args.json_out)
    out_path.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"结果已写入 {out_path}")
    return 0 if recorder.ok() else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DRY_RUN 实物串口验收")
    parser.add_argument("--port", default="COM4", help="ESP32 串口，例如 COM4")
    parser.add_argument("--skip-manual", action="store_true", help="跳过拔插 USB/按 EN")
    parser.add_argument("--verbose", action="store_true", help="打印启动噪声和原始行")
    parser.add_argument("--json-out", default="smoke_results.json")
    parser.add_argument(
        "--phase",
        choices=["smoke", "crash-child"],
        default="smoke",
        help=argparse.SUPPRESS,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.phase == "crash-child":
        return run_crash_child(args.port)
    return run_smoke(args)


if __name__ == "__main__":
    raise SystemExit(main())
