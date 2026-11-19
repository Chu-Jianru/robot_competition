"""菜单 13 检测视频窗口的离线检查：只画图、不连接相机或底盘。"""

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from remote_view import DISPLAY_PX, RemoteView, compose_view, view_caption


def scene_of(*, blocks=(), obstacles=(), valid=True, home_side="blue", bias=0.0, stamp=None,
             tag_side_mm=50.0):
    return {
        "valid": valid, "home_side": home_side, "seam_bias": bias,
        "stamp": time.monotonic() if stamp is None else stamp,
        "blocks": list(blocks), "obstacles": list(obstacles),
        "tag_side_mm": tag_side_mm,
    }


class ComposeViewTest(unittest.TestCase):
    def test_obstacle_box_uses_the_task_mapping(self):
        marked = np.zeros((960, 960, 3), np.uint8)
        display = compose_view(marked, scene_of(obstacles=[(0.1, 0.05, 0.02, 0.02)]))
        self.assertEqual(display.shape, (DISPLAY_PX, DISPLAY_PX, 3))
        # scale = 500 px/m; x=0.6+0.1, y=0.6-0.05, half 0.02 m.
        self.assertEqual(tuple(display[265, 340]), (255, 100, 0))
        self.assertEqual(tuple(display[275, 350]), (0, 0, 0))  # 只画框，不填充

    def test_yellow_side_mirrors_the_obstacle(self):
        display = compose_view(np.zeros((960, 960, 3), np.uint8),
                               scene_of(obstacles=[(0.1, 0.05, 0.02, 0.02)],
                                        home_side="yellow"))
        # 黄区起跑时策略 x 取反：x=0.6-0.1，y=0.6+0.05。
        self.assertEqual(tuple(display[315, 240]), (255, 100, 0))

    def test_invalid_or_missing_scene_is_still_shown(self):
        marked = np.zeros((960, 960, 3), np.uint8)
        for scene in (None, scene_of(valid=False)):
            display = compose_view(marked, scene)
            self.assertEqual(display.shape, (DISPLAY_PX, DISPLAY_PX, 3))
            self.assertEqual(int(display.sum()), 0)


class CaptionTest(unittest.TestCase):
    def test_valid_scene_reports_side_blocks_obstacles_and_age(self):
        scene = scene_of(stamp=100.0, tag_side_mm=50.4,
                         blocks=[(0.1, -0.1, .01, .01, True), (-0.2, -0.2, .01, .01, False)],
                         obstacles=[(0.0, 0.0, .02, .02), (0.2, 0.2, .02, .02)])
        text = view_caption(scene, "vision ready", 100.05, .35)
        for part in ("己方蓝区", "tag 50 mm", "红块 2 个（己方 1）", "障碍 2 个", "帧龄 50 ms"):
            self.assertIn(part, text)
        self.assertNotIn("画面过期", text)

    def test_stale_scene_is_labelled(self):
        text = view_caption(scene_of(stamp=100.0), "vision ready", 100.5, .35)
        self.assertIn("画面过期", text)

    def test_invalid_scene_keeps_the_vision_reason(self):
        text = view_caption({"valid": False, "stamp": 100.0}, "ID 7: not found", 100.05, .35)
        self.assertIn("ID 7: not found", text)
        self.assertIn("帧龄 50 ms", text)

    def test_no_scene_yet(self):
        self.assertEqual(view_caption(None, None, 100.0, .35), "正在等待手机画面…")


class OpenTest(unittest.TestCase):
    def fake_run_task(self, vision, error=None):
        options = lambda: SimpleNamespace(
            scrcpy_dir=Path("default-scrcpy"), calibration=Path("default-calib.json"),
            max_size=1280, detector="auto", rotate="auto", onnx_model=Path("model.onnx"),
            onnx_device="auto", no_track=False)
        return SimpleNamespace(
            MAX_AGE=.35,
            build_parser=lambda: SimpleNamespace(parse_args=lambda _argv: options()),
            load_calibration=Mock(return_value="calib"),
            task_vision=Mock(return_value=vision))

    def vision(self, error=None):
        camera = SimpleNamespace(error=None, start=Mock(side_effect=error), close=Mock())
        return SimpleNamespace(camera=camera, thread=SimpleNamespace(start=Mock()),
                               latest=None, error=None, stop=Mock())

    def test_open_starts_camera_and_vision_thread_with_task_defaults(self):
        vision = self.vision()
        fake = self.fake_run_task(vision)
        with patch.dict(sys.modules, {"run_task": fake}):
            view = RemoteView.open(rotate="90", detector="hsv")
        self.assertIsInstance(view, RemoteView)
        vision.camera.start.assert_called_once_with()
        vision.thread.start.assert_called_once_with()
        options, calib = fake.task_vision.call_args.args
        self.assertEqual(calib, "calib")
        self.assertEqual(options.rotate, 90)
        self.assertEqual(options.detector, "hsv")
        self.assertEqual(options.scrcpy_dir, Path("default-scrcpy"))
        self.assertEqual(options.calibration, Path("default-calib.json"))
        self.assertEqual(fake.load_calibration.call_args.args[0], Path("default-calib.json"))

    def test_obstacles_follow_the_shared_switch_or_an_override(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                vision = self.vision()
                fake = self.fake_run_task(vision)
                flags = SimpleNamespace(obstacles_enabled=lambda: enabled)
                with patch.dict(sys.modules, {"run_task": fake, "detection_flags": flags}):
                    RemoteView.open()
                self.assertIs(fake.task_vision.call_args.kwargs["detect_obstacles"], enabled)
        vision = self.vision()
        fake = self.fake_run_task(vision)
        with patch.dict(sys.modules, {"run_task": fake}):
            RemoteView.open(detect_obstacles=True)
        self.assertIs(fake.task_vision.call_args.kwargs["detect_obstacles"], True)

    def test_auto_rotation_stays_undecided_and_paths_can_be_overridden(self):
        vision = self.vision()
        fake = self.fake_run_task(vision)
        with patch.dict(sys.modules, {"run_task": fake}):
            RemoteView.open(scrcpy_dir="E:/scrcpy", calibration="E:/calib.json", max_size=1920)
        options, _ = fake.task_vision.call_args.args
        self.assertIsNone(options.rotate)
        self.assertEqual(options.scrcpy_dir, Path("E:/scrcpy"))
        self.assertEqual(options.calibration, Path("E:/calib.json"))
        self.assertEqual(options.max_size, 1920)

    def test_camera_failure_closes_what_it_opened_and_propagates(self):
        vision = self.vision(error=RuntimeError("没有找到手机"))
        fake = self.fake_run_task(vision)
        with patch.dict(sys.modules, {"run_task": fake}):
            with self.assertRaises(RuntimeError):
                RemoteView.open()
        vision.camera.close.assert_called_once_with()
        vision.thread.start.assert_not_called()


class WindowTest(unittest.TestCase):
    def setUp(self):
        self.marked = np.zeros((960, 960, 3), np.uint8)
        self.scene = scene_of(blocks=[(0.1, -0.1, .01, .01, True)])
        self.vision = SimpleNamespace(
            latest=(self.scene, self.marked, "vision ready"), error=None,
            camera=SimpleNamespace(error=None, close=Mock()),
            stop=Mock(), thread=SimpleNamespace(is_alive=lambda: False, join=Mock()))

    def fake_window(self):
        top = Mock()
        top.winfo_exists.return_value = True
        canvas = Mock()
        canvas.create_image.return_value = "frame"
        canvas.create_text.return_value = "status"
        tk = SimpleNamespace(Toplevel=Mock(return_value=top), Frame=Mock(), Label=Mock(),
                             Canvas=Mock(return_value=canvas), StringVar=Mock(),
                             PhotoImage=Mock(), TclError=RuntimeError)
        return tk, canvas

    def test_attach_keeps_keyboard_focus_and_draws_into_the_canvas(self):
        tk, canvas = self.fake_window()
        root = Mock()
        view = RemoteView(self.vision, stale_s=.35)
        with patch.dict(sys.modules, {"tkinter": tk}):
            view.attach(root)
        tk.Toplevel.assert_called_once_with(root)
        root.focus_force.assert_called_once_with()
        # 第一帧：把图片放到画布上，并清空中间的等待文字。
        canvas.itemconfigure.assert_any_call("frame", image=tk.PhotoImage.return_value)
        canvas.itemconfigure.assert_any_call("status", text="")
        self.assertIn("己方蓝区", view.caption.set.call_args.args[0])
        root.after.assert_called_once_with(100, view.refresh)
        # 视频窗口的任何点击都把焦点还给遥控窗口。
        for widget in (view.top, view.canvas):
            handler = widget.bind.call_args.args[1]
            handler(None)
        self.assertGreaterEqual(root.focus_force.call_count, 3)

    def test_no_frame_yet_is_explained_inside_the_window(self):
        tk, canvas = self.fake_window()
        self.vision.latest = None
        root = Mock()
        view = RemoteView(self.vision, stale_s=.35)
        with patch.dict(sys.modules, {"tkinter": tk}):
            view.attach(root)
        status = canvas.itemconfigure.call_args.kwargs["text"]
        self.assertIn("正在等待手机画面", status)
        self.assertIn("检查手机取景", status)
        self.assertIn("正在等待手机画面", view.caption.set.call_args.args[0])
        root.after.assert_called_once_with(100, view.refresh)

    def test_refresh_keeps_polling_and_close_stops_detection(self):
        tk, _ = self.fake_window()
        root = Mock()
        view = RemoteView(self.vision, stale_s=.35)
        with patch.dict(sys.modules, {"tkinter": tk}):
            view.attach(root)
            view.refresh()
        self.assertEqual(root.after.call_count, 2)
        view.close()
        self.vision.stop.set.assert_called_once_with()
        self.vision.camera.close.assert_called_once_with()
        self.assertIsNone(view.top)
        root.after.assert_called_with(100, view.refresh)
        count = root.after.call_count
        view.refresh()
        self.assertEqual(root.after.call_count, count)

    def test_camera_error_is_reported_without_new_polls(self):
        self.vision.error = "USB video stream closed."
        tk, canvas = self.fake_window()
        root = Mock()
        view = RemoteView(self.vision, stale_s=.35)
        with patch.dict(sys.modules, {"tkinter": tk}):
            view.attach(root)
        self.assertIn("检测已停止", view.caption.set.call_args.args[0])
        self.assertIn("USB video stream closed.", view.caption.set.call_args.args[0])
        self.assertIn("遥控仍可用", canvas.itemconfigure.call_args.kwargs["text"])
        self.vision.stop.set.assert_called_once_with()
        self.vision.camera.close.assert_called_once_with()
        self.assertEqual(root.after.call_count, 0)

    def test_display_failure_stops_detection_but_keeps_the_control_window(self):
        tk, canvas = self.fake_window()
        root = Mock()
        view = RemoteView(self.vision, stale_s=.35)
        with (patch.dict(sys.modules, {"tkinter": tk}),
              patch("remote_view.compose_view", side_effect=RuntimeError("bad frame"))):
            view.attach(root)
        self.assertIn("检测窗口显示失败", view.caption.set.call_args.args[0])
        self.assertIn("bad frame", canvas.itemconfigure.call_args.kwargs["text"])
        self.vision.camera.close.assert_called_once_with()
        self.assertEqual(root.after.call_count, 0)

    def test_closed_video_window_releases_the_camera(self):
        tk, _ = self.fake_window()
        root = Mock()
        view = RemoteView(self.vision, stale_s=.35)
        with patch.dict(sys.modules, {"tkinter": tk}):
            view.attach(root)
        view.top.winfo_exists.return_value = False
        view.refresh()
        self.vision.camera.close.assert_called_once_with()
        self.assertEqual(root.after.call_count, 1)


if __name__ == "__main__":
    unittest.main()
