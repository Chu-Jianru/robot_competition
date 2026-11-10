"""验证解压运行后的可写路径与首次保存，不访问硬件。"""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "esp32-pc-link" / "pc"))
sys.path.insert(0, str(ROOT / "debug-workbench"))
import runtime_paths
from bench_model import save_settings, load_settings


class RuntimePathTests(unittest.TestCase):
    def test_source_layout_stays_compatible(self):
        with patch.dict(os.environ, {"ROBOT_WORKBENCH_DATA_DIR": ""}), patch.object(sys, "frozen", False, create=True):
            self.assertEqual(runtime_paths.settings_file(), ROOT / "debug-workbench" / "settings.json")
            self.assertEqual(runtime_paths.camera_config_file(), ROOT / "esp32-pc-link" / "pc" / "camera_config.json")

    def test_frozen_data_is_next_to_exe_not_inside_resources(self):
        with tempfile.TemporaryDirectory() as folder:
            exe = Path(folder) / "app" / "RobotDebugWorkbench.exe"
            with patch.dict(os.environ, {"ROBOT_WORKBENCH_DATA_DIR": ""}), patch.object(sys, "frozen", True, create=True), patch.object(sys, "executable", str(exe)):
                self.assertEqual(runtime_paths.settings_file(), exe.parent / "data" / "settings.json")
                self.assertEqual(runtime_paths.camera_config_file().parent, exe.parent / "data")
                save_settings(runtime_paths.settings_file(), {"duty": "60", "duration": "1"})
                self.assertEqual(load_settings(runtime_paths.settings_file())["duty"], "60")

    def test_explicit_test_data_directory(self):
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {"ROBOT_WORKBENCH_DATA_DIR": folder}):
            self.assertEqual(runtime_paths.records_directory(), Path(folder).resolve() / "records")


if __name__ == "__main__":
    unittest.main()
