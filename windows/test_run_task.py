"""Offline checks for the Windows task adapter and expiring Bluetooth pulses."""
import math
import unittest

import cv2
import numpy as np

from chassis_link import ChassisLink
from field_view import FieldCalibration
from run_task import (DriveConfig, HybridCompetitionPolicy, MAX_AGE, can_start,
                      fresh, latest_scene, observe)
from tag_detector import TAG_ID
from test_chassis_link import FakeFirmware


class HoldVisionTest(unittest.TestCase):
    @staticmethod
    def scene(stamp, valid):
        return {"stamp": stamp, "valid": valid, "robot": None, "blocks": []}

    def test_invalid_frame_holds_the_last_valid_scene(self):
        held = self.scene(1.0, True)
        latest, held_after = latest_scene(self.scene(1.05, False), held)
        self.assertIs(latest, held)
        self.assertIs(held_after, held)

    def test_hold_expires_after_max_age(self):
        held = self.scene(1.0, True)
        stale = self.scene(1.0 + MAX_AGE + 0.01, False)
        latest, _ = latest_scene(stale, held)
        self.assertIs(latest, stale)

    def test_valid_frame_replaces_the_held_scene(self):
        held = self.scene(1.0, True)
        fresh_scene = self.scene(1.05, True)
        latest, held_after = latest_scene(fresh_scene, held)
        self.assertIs(latest, fresh_scene)
        self.assertIs(held_after, fresh_scene)

    def test_invalid_first_frame_is_still_reported(self):
        first = self.scene(1.0, False)
        latest, held_after = latest_scene(first, None)
        self.assertIs(latest, first)
        self.assertIsNone(held_after)


class TaskSceneTest(unittest.TestCase):
    def setUp(self):
        self.calib = FieldCalibration(
            np.float32([[20, 20], [980, 20], [980, 980], [20, 980]]),
            source_size=(1000, 1000),
        )
        self.image = np.zeros((1000, 1000, 3), np.uint8)
        # The colour seam is 15 mm south of the geometric centre.
        self.image[20:512, 20:980] = (10, 230, 240)
        self.image[512:980, 20:980] = (230, 50, 30)
        self.image[370:386, 500:516] = (0, 0, 220)
        self.image[620:636, 500:516] = (0, 0, 220)
        self.image[740:800, 460:520] = (255, 255, 255)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, TAG_ID, 40)
        self.image[750:790, 470:510] = marker[:, :, None]

    def test_rectified_scene_feeds_hybrid_policy(self):
        scene, _, reason = observe(self.image, self.calib, 4.)
        self.assertTrue(scene["valid"], reason)
        self.assertTrue(can_start(scene))
        self.assertAlmostEqual(scene["robot"][2], math.pi / 2, delta=.04)
        self.assertAlmostEqual(scene["seam_bias"], -.015, delta=.002)
        self.assertEqual(sorted(block[4] for block in scene["blocks"]), [False, True])
        policy = HybridCompetitionPolicy()
        policy.update(scene)
        self.assertTrue(policy.valid)
        self.assertEqual(len(policy.pending()), 1)
        self.assertTrue(fresh(scene, 4.34))
        self.assertFalse(fresh(scene, 4.36))

    def test_missing_colour_boundary_blocks_policy(self):
        self.image[512:980, 20:980] = (10, 230, 240)
        scene, _, _ = observe(self.image, self.calib, 4.)
        self.assertFalse(scene["valid"])

    def test_wheel_mix_is_bounded_before_transmission(self):
        drive = DriveConfig("COM6", "CBAD", 3., 2., 1., 30)
        output, physical = drive.convert((.20, -.10, .5))
        self.assertLessEqual(sum(map(abs, output)), 30)
        self.assertGreater(physical[0], 0)
        self.assertLess(physical[1], 0)
        self.assertGreater(physical[2], 0)


class PulseLeaseTest(unittest.TestCase):
    def test_expired_queued_pulse_is_not_written_and_stop_still_is(self):
        now = [1.]
        link = ChassisLink("FAKE", strict=True, clock=lambda: now[0])
        fake = FakeFirmware()
        # Drive the actual serial service directly so expiry between enqueue
        # and write is deterministic and does not depend on thread scheduling.
        link._device = fake
        link._ready = True
        link._phase = "READY"
        link._last_reply = now[0]
        self.assertTrue(link.pulse(20, 0, 0, 150, send_by=1.05))
        now[0] = 1.1
        link._service(now[0])
        self.assertFalse(any(line.startswith("MECANUM") for line in fake.sent))
        link.stop()
        link._service(now[0])
        self.assertEqual(fake.sent[-1], "STOP")


if __name__ == "__main__":
    unittest.main()
