"""Read a scrcpy 5.0.1 USB camera stream into Python; no ROS or screen capture."""
from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
import re
import secrets
import socket
import subprocess
import sys
import threading
import time

from scene_vision import inspect_image
from red_blocks import calibrate_red_config, load_red_config, save_red_config


class LatestFrame:
    """A single decoded frame, replaced rather than queued for processing."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.received_at = 0.0
        self.sequence = 0

    def put(self, frame):
        with self.lock:
            self.frame = frame
            self.received_at = time.monotonic()
            self.sequence += 1

    def get(self):
        with self.lock:
            return self.frame, self.received_at, self.sequence


class H264Decoder:
    def __init__(self, publish):
        import av
        self.codec = av.CodecContext.create("h264", "r")
        self.codec.thread_count = 1  # Avoid frame-threading latency.
        self.publish = publish

    def feed(self, data):
        for packet in self.codec.parse(data):
            for frame in self.codec.decode(packet):
                self.publish(frame)


class UsbCamera:
    def __init__(self, folder, serial=None, max_size=1280, camera_fps=30):
        self.folder = Path(folder).resolve()
        self.adb = self.folder / ("adb.exe" if sys.platform == "win32" else "adb")
        self.scrcpy = self.folder / ("scrcpy.exe" if sys.platform == "win32" else "scrcpy")
        self.server_file = self.folder / "scrcpy-server"
        self.serial = serial
        self.max_size, self.camera_fps = max_size, camera_fps
        self.port = None
        self.process = None
        self.sock = None
        self.reader = None
        self.log_reader = None
        self.remote_file = None
        self.stop = threading.Event()
        self.latest = LatestFrame()
        self.error = None
        self.logs = deque(maxlen=80)

    def run_adb(self, *args, device=True, check=True):
        prefix = [str(self.adb)]
        if device and self.serial:
            prefix += ["-s", self.serial]
        result = subprocess.run(prefix + list(args), capture_output=True, text=True,
                                encoding="utf-8", errors="replace", timeout=30)
        if check and result.returncode:
            raise RuntimeError((result.stderr or result.stdout).strip())
        return result.stdout.strip()

    def start(self):
        for file in (self.adb, self.scrcpy, self.server_file):
            if not file.is_file():
                raise RuntimeError(f"Missing {file.name}; --scrcpy-dir must point to the extracted scrcpy folder.")
        result = subprocess.run([str(self.scrcpy), "--version"], capture_output=True,
                                text=True, encoding="utf-8", errors="replace", timeout=15)
        match = re.search(r"scrcpy\s+(\d+\.\d+(?:\.\d+)?)", result.stdout + result.stderr)
        if not match or match.group(1) != "5.0.1":
            raise RuntimeError("This client was checked against scrcpy 5.0.1. Keep its matching scrcpy-server file.")
        # 新起的 adb server 要过一会儿才枚举完 USB；立刻查会返回空表，
        # 不能据此判成“没插手机”。给 10 s 轮询，命中目标就继续。
        devices = []
        usb_devices = []
        states = {}
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            states = {}
            for line in self.run_adb("devices", device=False).splitlines():
                parts = line.split()
                if len(parts) >= 2 and parts[1] in ("device", "unauthorized", "offline"):
                    states[parts[0]] = parts[1]
            devices = [serial for serial, state in states.items() if state == "device"]
            usb_devices = [serial for serial in devices
                           if ":" not in serial and not serial.startswith("emulator-")]
            if self.serial and self.serial in devices:
                break
            if not self.serial and len(usb_devices) == 1:
                break
            time.sleep(0.25)
        if self.serial:
            if self.serial not in devices:
                state = states.get(self.serial)
                if state == "unauthorized":
                    raise RuntimeError(
                        "手机已连接但未授权：在手机上点“允许 USB 调试”（可勾“始终允许”）后重试。")
                if state == "offline":
                    raise RuntimeError(
                        "手机处于 offline：先 adb kill-server，再拔插；仍不行就重启手机。")
                raise RuntimeError("没有找到指定设备；先运行 adb devices 看是否为空。")
        else:
            if len(usb_devices) != 1:
                raise RuntimeError("Connect one authorized USB phone, or specify --serial.")
            self.serial = usb_devices[0]
        if int(self.run_adb("shell", "getprop", "ro.build.version.sdk")) < 31:
            raise RuntimeError("Camera capture needs Android API 31 or newer.")

        scid = f"{secrets.randbelow(0x7fffffff):08x}"
        self.remote_file = f"/data/local/tmp/scrcpy-vision-{scid}.jar"
        try:
            self.run_adb("push", str(self.server_file), self.remote_file)
        except RuntimeError:
            # USB 偶发 “failed to read copy response: EOF”，重推一次再判失败。
            time.sleep(0.5)
            self.run_adb("push", str(self.server_file), self.remote_file)
        self.port = int(self.run_adb("forward", "tcp:0", f"localabstract:scrcpy_{scid}"))
        options = [f"scid={scid}", "log_level=info", "video_source=camera",
                   "camera_facing=back", f"max_size={self.max_size}",
                   f"camera_fps={self.camera_fps}", "video_codec=h264",
                   "video_bit_rate=8000000", "audio=false", "control=false",
                   "tunnel_forward=true", "raw_stream=true", "send_dummy_byte=true"]
        command = [str(self.adb), "-s", self.serial, "shell",
                   f"CLASSPATH={self.remote_file}", "app_process", "/",
                   "com.genymobile.scrcpy.Server", "5.0.1"] + options
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True,
                                        encoding="utf-8", errors="replace")
        self.log_reader = threading.Thread(target=self._read_logs, daemon=True)
        self.log_reader.start()
        self.sock = self._connect_ready()
        self.sock.settimeout(15)
        self.reader = threading.Thread(target=self._receive, daemon=True)
        self.reader.start()

    def _read_logs(self):
        for line in self.process.stdout:
            self.logs.append(line.rstrip())
            print(line.rstrip(), flush=True)

    def _connect_ready(self):
        # ADB may accept TCP before the Android socket exists. The dummy byte
        # confirms the device-side server, not merely the ADB forward, is ready.
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError("scrcpy server exited; see server messages above.")
            candidate = socket.socket()
            try:
                candidate.settimeout(1)
                candidate.connect(("127.0.0.1", self.port))
                while time.monotonic() < deadline:
                    try:
                        dummy = candidate.recv(1)
                    except socket.timeout:
                        if self.process.poll() is not None:
                            break
                        continue
                    if dummy == b"\x00":
                        return candidate
                    break
            except OSError:
                pass
            candidate.close()
            time.sleep(0.1)
        raise RuntimeError("USB video socket did not start within 20 seconds.")

    def _receive(self):
        try:
            decoder = H264Decoder(self.latest.put)
            while not self.stop.is_set():
                data = self.sock.recv(65536)
                if not data:
                    raise RuntimeError("USB video stream closed.")
                decoder.feed(data)
        except Exception as exc:
            if not self.stop.is_set():
                self.error = str(exc)
        finally:
            self.stop.set()

    def close(self):
        self.stop.set()
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sock.close()
        if self.reader:
            self.reader.join(timeout=2)
        if self.process:
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
        if self.log_reader:
            self.log_reader.join(timeout=1)
        # Remove only this session's forward and uploaded file.
        if self.port:
            try:
                self.run_adb("forward", "--remove", f"tcp:{self.port}", check=False)
            except (OSError, subprocess.SubprocessError):
                pass
        if self.remote_file:
            try:
                self.run_adb("shell", "rm", "-f", self.remote_file, check=False)
            except (OSError, subprocess.SubprocessError):
                pass


def prepare_image(image, rotation):
    import cv2
    import numpy as np
    if rotation:
        codes = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
        image = cv2.rotate(image, codes[rotation])
    height, width = image.shape[:2]
    scale = min(640 / width, 480 / height)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    canvas = np.zeros((480, 640, 3), dtype=np.uint8)
    x, y = (640 - size[0]) // 2, (480 - size[1]) // 2
    canvas[y:y + size[1], x:x + size[0]] = resized
    return canvas


def annotate_detections(image, result):
    """Draw a perimeter without crossing diagonals; keep the input unmarked."""
    import cv2
    import numpy as np
    annotated = image.copy()
    if result["homography"] is not None:
        perimeter = cv2.convexHull(np.asarray(result["field_px"], np.float32))
        cv2.polylines(annotated, [np.rint(perimeter).astype(np.int32)],
                      True, (0, 255, 0), 2)
        for point in result["block_px"]:
            cv2.circle(annotated, tuple(round(v) for v in point), 5, (0, 0, 255), 1)
        if result["robot_px"]:
            center, front = result["robot_px"]
            cv2.arrowedLine(annotated, tuple(round(v) for v in center),
                            tuple(round(v) for v in front), (255, 0, 255), 2)
        field_text = "FIELD OK"
        if result["red_enabled"]:
            field_text += f" | own groups={result['own_count']}"
    else:
        field_text = "FIELD NOT FOUND" if result["field_enabled"] else "FIELD OFF"
    if result["red_enabled"]:
        for x, y, width, height in result["red_candidate_boxes"]:
            cv2.rectangle(annotated, (x, y), (x + width - 1, y + height - 1), (0, 255, 0), 1)
            cv2.putText(annotated, "block?", (x, max(75, y - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
        # Calibration only adds coordinates; pixel boxes remain independent.
        hint = ("COLOR + SHAPE | pixel detections; field calibration OFF/unavailable"
                if result["homography"] is None else
                "Green block? = shape candidates | red dots = field-mapped")
        cv2.putText(annotated, hint, (8, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.40, (0, 200, 255), 1)
        text = f"RED ON: block candidates={len(result['red_candidate_boxes'])} | {field_text}"
    else:
        text = f"RED OFF | {field_text}"
    cv2.putText(annotated, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (255, 255, 255), 1)
    return annotated


def save_snapshot(output, original, annotated):
    """Save matching raw/annotated PNGs, including Unicode Windows paths."""
    import cv2
    stamp = time.time_ns()
    name = f"field_{time.strftime('%Y%m%d_%H%M%S', time.localtime(stamp / 1e9))}_{stamp % 1000000000:09d}"
    encoded_images = []
    for label, pixels in (("raw", original), ("annotated", annotated)):
        ok, encoded = cv2.imencode(".png", pixels)
        if not ok:
            raise RuntimeError(f"Could not encode {label} snapshot.")
        encoded_images.append((Path(output) / f"{name}_{label}.png", encoded))
    Path(output).mkdir(parents=True, exist_ok=True)
    for path, encoded in encoded_images:
        encoded.tofile(str(path))
    return tuple(path for path, _ in encoded_images)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scrcpy-dir", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--serial")
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=90)
    parser.add_argument("--field", action="store_true", help="Enable experimental HSV field detection")
    parser.add_argument("--no-blocks", action="store_true", help="Start with independent red detection disabled")
    parser.add_argument("--red-profile", type=Path,
                        default=Path(__file__).resolve().with_name("red_block_profile.json"),
                        help="Red-block color profile; K selects a true block top face to recalibrate")
    args = parser.parse_args()
    import cv2
    import av  # Fail before changing phone state if dependencies are missing.
    cv2.setNumThreads(1)
    camera = UsbCamera(args.scrcpy_dir, args.serial)
    red_config = load_red_config(args.red_profile)
    rotation, field_enabled, red_enabled = args.rotate, args.field, not args.no_blocks
    field_lock = None
    output = Path(__file__).resolve().parent / "captures"
    window = "USB Camera | B red | F field | K calibrate red | S save | Q quit"
    print("Close the existing scrcpy window and other camera apps first.")
    print("Controls: B toggle red detection, F toggle field/robot, S save raw + annotated,")
    print("          Q quit, R rotate, C reset field lock. Red detection starts ON unless --no-blocks.")
    print(f"Red color profile: {args.red_profile} (exists={args.red_profile.exists()})")
    print("K calibrates: select INSIDE a true red block TOP FACE; avoid background and dark sides.")
    print(f"Snapshots: {output}")
    print("Red detection is independent of field detection. Green block? boxes pass color + shape filters.")
    print("Preview/processing: 640x480, at most 10 Hz. No robot commands are sent.")
    print("PC age is time since decoding, NOT phone-to-PC latency.")
    try:
        camera.start()
        last_sequence = 0
        last_console = 0.0
        while not camera.stop.is_set():
            loop_start = time.monotonic()
            frame, received_at, sequence = camera.latest.get()
            if frame is None:
                time.sleep(0.02)
                continue
            if sequence == last_sequence:
                if time.monotonic() - received_at > 0.35:
                    raise RuntimeError("No fresh decoded frame for 0.35 s. Restart the camera; do not use old detections.")
                if cv2.waitKey(1) & 0xff in (27, ord("q"), ord("Q")):
                    break
                time.sleep(0.01)
                continue
            last_sequence = sequence
            image = prepare_image(frame.to_ndarray(format="bgr24"), rotation)
            original = image.copy()
            if field_enabled and field_lock is None:
                from field_vision import FieldLock
                field_lock = FieldLock()
            result = inspect_image(image, field_lock, red_enabled=red_enabled,
                                   field_enabled=field_enabled, red_config=red_config)
            if result["field_rejected"]:
                field_lock = None
            image = annotate_detections(image, result)
            age_ms = (time.monotonic() - received_at) * 1000
            cv2.putText(image, f"PC age={age_ms:.0f} ms | rotate={rotation}", (8, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.imshow(window, image)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord("r"), ord("R")):
                rotation = (rotation + 90) % 360
                field_lock = None
            elif key in (ord("c"), ord("C")):
                field_lock = None
            elif key in (ord("f"), ord("F")):
                field_enabled = not field_enabled
                field_lock = None
            elif key in (ord("b"), ord("B")):
                red_enabled = not red_enabled
            elif key in (ord("k"), ord("K")):
                calibration_window = "Select INSIDE red block TOP FACE | Enter confirm | Esc cancel"
                try:
                    x, y, width, height = map(int, cv2.selectROI(
                        calibration_window, original, showCrosshair=True, fromCenter=False))
                finally:
                    cv2.destroyWindow(calibration_window)
                if width and height:
                    try:
                        calibrated = calibrate_red_config(original, roi=(x, y, width, height))
                    except ValueError as exc:
                        print(f"Calibration unchanged: {exc}")
                    else:
                        save_red_config(args.red_profile, calibrated)
                        red_config = calibrated
                        print(f"Saved red-block color calibration: {args.red_profile}")
            elif key in (ord("s"), ord("S")):
                for file in save_snapshot(output, original, image):
                    print(f"Saved: {file}")
            if result is not None and time.monotonic() - last_console >= 1:
                last_console = time.monotonic()
                print(f"field={result['homography'] is not None}, own_groups={result['own_count']}, "
                      f"robot={result['robot']}, red_candidates={len(result['red_candidate_boxes'])}, "
                      f"field_rejected={result['field_rejected']}")
            time.sleep(max(0, 0.1 - (time.monotonic() - loop_start)))
        if camera.error:
            raise RuntimeError(camera.error)
        return 0
    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
