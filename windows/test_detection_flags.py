"""障碍检测开关的离线检查：只读写 JSON，不接相机或底盘。"""

import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from detection_flags import DEFAULTS, load, obstacles_enabled, save, set_obstacles


class FlagsTest(unittest.TestCase):
    def setUp(self):
        self.folder = TemporaryDirectory()
        self.path = Path(self.folder.name) / "detection_flags.json"

    def tearDown(self):
        self.folder.cleanup()

    def test_missing_file_uses_defaults(self):
        self.assertFalse(self.path.exists())
        self.assertTrue(obstacles_enabled(self.path))
        self.assertEqual(load(self.path), DEFAULTS)

    def test_corrupt_file_uses_defaults(self):
        self.path.write_text("{ not json", encoding="utf-8")
        self.assertTrue(obstacles_enabled(self.path))
        self.path.write_text("[1, 2]", encoding="utf-8")
        self.assertTrue(obstacles_enabled(self.path))

    def test_toggle_writes_only_the_obstacle_key(self):
        save({"obstacles": False, "future_flag": True}, self.path)
        self.assertTrue(set_obstacles(True, self.path))
        self.assertTrue(obstacles_enabled(self.path))
        saved = json.loads(self.path.read_text(encoding="utf-8"))
        self.assertEqual(saved, {"obstacles": True, "future_flag": True})
        self.assertFalse(set_obstacles(False, self.path))
        self.assertFalse(obstacles_enabled(self.path))
        self.assertEqual(json.loads(self.path.read_text(encoding="utf-8")),
                         {"obstacles": False, "future_flag": True})


if __name__ == "__main__":
    unittest.main()
