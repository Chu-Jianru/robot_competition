"""Offline checks: run with Python after installing requirements."""
import io
from fractions import Fraction
import socket
import sys
from pathlib import Path
from subprocess import CompletedProcess
import tempfile
import threading
import unittest
from unittest.mock import patch

import av
import cv2
import numpy as np

from usb_camera import (H264Decoder, LatestFrame, UsbCamera, annotate_detections,
                        inspect_image, prepare_image, save_snapshot)


def sample_stream():
    output = io.BytesIO()
    container = av.open(output, mode="w", format="h264")
    stream = container.add_stream("libx264", rate=30)
    stream.width, stream.height, stream.pix_fmt = 128, 96, "yuv420p"
    stream.options = {"preset": "ultrafast", "tune": "zerolatency", "bf": "0"}
    for index in range(18):
        image = np.zeros((96, 128, 3), np.uint8)
        image[:, :, 0] = index * 10
        frame = av.VideoFrame.from_ndarray(image, format="bgr24")
        frame.pts, frame.time_base = index, Fraction(1, 30)
        for packet in stream.encode(frame):
            container.mux(packet)
    for packet in stream.encode():
        container.mux(packet)
    container.close()
    return output.getvalue()


class CameraChecks(unittest.TestCase):
    def test_selects_physical_phone_when_an_emulator_is_running(self):
        camera = UsbCamera(".")

        def adb(*args, **kwargs):
            if args == ("devices",):
                return "List of devices attached\nemulator-5554\tdevice\nPHONE\tdevice\n"
            if args == ("shell", "getprop", "ro.build.version.sdk"):
                self.assertEqual(camera.serial, "PHONE")
                return "31"
            raise OSError("stop before uploading the server")

        with (patch.object(Path, "is_file", return_value=True),
              patch("usb_camera.subprocess.run",
                    return_value=CompletedProcess([], 0, "scrcpy 5.0.1", "")),
              patch.object(camera, "run_adb", side_effect=adb),
              self.assertRaisesRegex(OSError, "stop before uploading")):
            camera.start()

    def test_fragmented_h264_and_latest_frame(self):
        slot = LatestFrame()
        decoder = H264Decoder(slot.put)
        data = sample_stream()
        # Splits cross NAL start codes, packet boundaries and frame boundaries.
        sizes = (1, 3, 17, 2, 509, 7)
        offset, index = 0, 0
        while offset < len(data):
            size = sizes[index % len(sizes)]
            decoder.feed(data[offset:offset + size])
            offset += size
            index += 1
        frame, received, sequence = slot.get()
        # The incremental parser may hold the final access unit until more data.
        self.assertGreaterEqual(sequence, 16)
        self.assertGreater(received, 0)
        self.assertEqual((frame.width, frame.height), (128, 96))
        self.assertGreater(frame.to_ndarray(format="bgr24")[:, :, 0].mean(), 120)

    def test_rotation_and_letterbox(self):
        image = np.zeros((20, 40, 3), np.uint8)
        image[:, :20] = (0, 0, 255)
        image[:, 20:] = (255, 0, 0)
        result = prepare_image(image, 90)
        self.assertEqual(result.shape, (480, 640, 3))
        self.assertTrue(np.all(result[:, :190] == 0))
        self.assertGreater(result[80, 320, 2], 240)
        self.assertGreater(result[400, 320, 0], 240)

    def test_usb_ready_byte_is_consumed(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        def serve():
            conn, _ = listener.accept()
            with conn:
                conn.sendall(b"\x00\x00\x00\x00\x01\x67")
            listener.close()
        thread = threading.Thread(target=serve)
        thread.start()
        camera = UsbCamera(".")
        camera.port = listener.getsockname()[1]
        camera.process = type("Running", (), {"poll": lambda self: None})()
        conn = camera._connect_ready()
        with conn:
            self.assertEqual(conn.recv(5), b"\x00\x00\x00\x01\x67")
        thread.join(timeout=1)

    def test_field_outline_has_no_crossing_diagonals(self):
        image = np.full((480, 640, 3), 40, np.uint8)
        # This is the existing homography pairing order, not a perimeter order.
        result = {"homography": np.eye(3), "field_px": np.float32(
            [[80, 80], [80, 400], [560, 80], [560, 400]]),
            "block_px": [], "robot_px": None, "blocks": [], "own_count": 0,
            "red_enabled": True, "field_enabled": True, "red_candidate_boxes": []}
        annotated = annotate_detections(image, result)
        np.testing.assert_array_equal(annotated[240, 320], image[240, 320])
        np.testing.assert_array_equal(annotated[80, 320], [0, 255, 0])
        self.assertTrue(np.all(image == 40))

    def test_sparse_color_is_not_a_field_and_red_survives(self):
        image = np.full((480, 640, 3), 150, np.uint8)
        # Closing joins thin colored wires into an apparently large field.
        for x in range(180, 421, 12):
            cv2.line(image, (x, 100), (x, 240), (230, 64, 13), 2)
            cv2.line(image, (x, 241), (x, 380), (13, 204, 242), 2)
        cv2.rectangle(image, (480, 200), (510, 230), (13, 13, 242), -1)
        result = inspect_image(image, field_enabled=True)
        self.assertTrue(result["field_rejected"])
        self.assertIsNone(result["homography"])
        self.assertIsNone(result["robot"])
        self.assertEqual(result["blocks"], [])
        self.assertTrue(any(x <= 495 < x + w and y <= 215 < y + h
                            for x, y, w, h in result["red_candidate_boxes"]))

    def test_real_ground_still_passes_preview_validation(self):
        image = np.full((480, 640, 3), 150, np.uint8)
        image[80:240, 140:500] = (13, 204, 242)
        image[240:400, 140:500] = (230, 64, 13)
        cv2.rectangle(image, (300, 270), (308, 278), (13, 13, 242), -1)
        result = inspect_image(image, field_enabled=True)
        self.assertFalse(result["field_rejected"])
        self.assertIsNotNone(result["homography"])
        self.assertEqual(len(result["blocks"]), 1)
        self.assertEqual(result["own_count"], 1)

    def test_red_runs_without_importing_the_field_module(self):
        image = np.full((160, 240, 3), 150, np.uint8)
        cv2.rectangle(image, (80, 60), (110, 90), (0, 0, 240), -1)
        with patch.dict(sys.modules, {"field_vision": None}):
            result = inspect_image(image)
        self.assertEqual(len(result["red_detections"]), 1)
        self.assertFalse(result["field_enabled"])
        self.assertIsNone(result["homography"])
        self.assertEqual(result["blocks"], [])

    def test_field_switch_does_not_change_red_pixel_results(self):
        image = np.full((480, 640, 3), 150, np.uint8)
        image[80:240, 140:500] = (13, 204, 242)
        image[240:400, 140:500] = (230, 64, 13)
        cv2.rectangle(image, (300, 270), (308, 278), (13, 13, 242), -1)
        # Outside-field red must remain visible in independent pixel results.
        cv2.rectangle(image, (50, 120), (70, 140), (13, 13, 242), -1)
        red_only = inspect_image(image)
        both = inspect_image(image, field_enabled=True)
        field_only = inspect_image(image, red_enabled=False, field_enabled=True)
        self.assertEqual(red_only["red_candidate_boxes"], both["red_candidate_boxes"])
        self.assertEqual(len(red_only["red_detections"]), 2)
        self.assertEqual(len(both["blocks"]), 1)
        self.assertIsNotNone(field_only["homography"])
        self.assertEqual(field_only["red_detections"], [])
        self.assertEqual(field_only["blocks"], [])
        self.assertEqual(field_only["own_count"], -1)

    def test_snapshot_pair_preserves_pixels_and_unicode_path(self):
        original = np.full((80, 100, 3), 140, np.uint8)
        annotated = original.copy()
        cv2.rectangle(annotated, (10, 10), (30, 30), (0, 255, 0), 2)
        with tempfile.TemporaryDirectory() as folder:
            paths = save_snapshot(Path(folder) / "标注照片", original, annotated)
            self.assertTrue(paths[0].name.endswith("_raw.png"))
            self.assertTrue(paths[1].name.endswith("_annotated.png"))
            self.assertEqual(paths[0].name.removesuffix("_raw.png"),
                             paths[1].name.removesuffix("_annotated.png"))
            for path, expected in zip(paths, (original, annotated)):
                actual = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
                np.testing.assert_array_equal(actual, expected)


if __name__ == "__main__":
    unittest.main()
