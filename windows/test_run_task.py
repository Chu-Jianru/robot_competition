"""Offline checks for the Windows task adapter and expiring Bluetooth pulses."""
import math
import unittest
from collections import deque
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np

from chassis_link import ChassisLink
from calibrate_drive import (ObstacleGate, Pose, fit_gains, measure_drive, motion_fits, obstacles_near_car,
                             save_obstacle_images, stable_pose, trial_gain)
from field_view import FieldCalibration
from pose_tag import detect
from run_task import (DriveConfig, HybridCompetitionPolicy, MAX_AGE, VisionWorker,
                      can_start, fresh, latest_scene, observe)
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

    def test_worker_publishes_scene_with_its_frame_and_label(self):
        camera = SimpleNamespace()
        worker = VisionWorker(camera, None, 0)
        image = np.zeros((8, 8, 3), np.uint8)
        valid_frame, invalid_frame = image.copy(), image.copy()
        observations = [
            (self.scene(1., True), valid_frame, "vision ready"),
            (self.scene(1.1, False), invalid_frame, "tag lost"),
            (self.scene(1.4, False), invalid_frame, "tag lost"),
        ]
        published = []

        def get_frame():
            published.append(worker.latest)
            sequence = len(published)
            if sequence == len(observations):
                worker.stop.set()
            return SimpleNamespace(to_ndarray=lambda **_: image), observations[sequence-1][0]["stamp"], sequence

        camera.latest = SimpleNamespace(get=get_frame)
        with patch("run_task.observe", side_effect=observations):
            worker.run()

        self.assertIsNone(worker.error)
        scene, frame, label = published[1]
        self.assertIs(scene, observations[0][0])
        self.assertIs(frame, valid_frame)
        self.assertEqual(label, "vision ready")
        self.assertIs(published[2], published[1])
        scene, frame, label = worker.latest
        self.assertIs(scene, observations[2][0])
        self.assertIs(frame, invalid_frame)
        self.assertEqual(label, "tag lost")


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

    def test_empty_field_mode_skips_obstacle_vision_but_keeps_pose_and_blocks(self):
        with patch("run_task._obstacles") as obstacle_detector:
            scene, _, reason = observe(self.image, self.calib, 4., detect_obstacles=False)
        obstacle_detector.assert_not_called()
        self.assertTrue(scene["valid"], reason)
        self.assertIsNotNone(scene["robot"])
        self.assertEqual(len(scene["blocks"]), 2)
        self.assertEqual(scene["obstacles"], [])

    def test_excludes_the_wheel_contour_and_keeps_the_adjacent_block(self):
        pose, _ = detect(self.calib.warp(self.image), self.calib)
        cx, cy = pose.center_px
        # The synthetic tag faces north. Above is forward and left is x-negative.
        # The first contour is over the rear left wheel. The second extends
        # outside the wheel strip and is a potential block next to the chassis.
        contours = [np.float32([[cx-64, cy+87], [cx-55, cy+87], [cx-55, cy+102], [cx-64, cy+102]]),
                    np.float32([[cx-92, cy+87], [cx-70, cy+87], [cx-70, cy+102], [cx-92, cy+102]])]
        inverse = np.linalg.inv(self.calib.homography())
        contours = [cv2.perspectiveTransform(c.reshape(1, 4, 2), inverse)[0] for c in contours]
        detections = [{"center_px": c.mean(axis=0), "contour_px": c} for c in contours]
        detector = SimpleNamespace(detect=lambda _: detections)
        scene, _, reason = observe(self.image, self.calib, 4., detector=detector)
        self.assertTrue(scene["valid"], reason)
        self.assertEqual(len(scene["blocks"]), 1)
        self.assertLess(scene["blocks"][0][0], (cx - 75) * self.calib.mm_per_px / 1000 - .6)

    def test_strips_floor_coloured_reflection_and_keeps_solid_obstacles(self):
        # A bent white highlight can make a large rectangle despite being thin.
        tint = tuple(map(int, cv2.cvtColor(np.uint8([[[112, 40, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]))
        cv2.rectangle(self.image, (315, 558), (409, 652), tint, -1)
        cv2.rectangle(self.image, (348, 564), (358, 627), (255, 255, 255), -1)
        cv2.rectangle(self.image, (348, 620), (391, 630), (255, 255, 255), -1)
        scene, _, reason = observe(self.image, self.calib, 1.)
        self.assertTrue(scene["valid"], reason)
        self.assertEqual(scene["obstacles"], [])

        for value in (28, 215):
            with self.subTest(value=value):
                image = self.image.copy()
                cv2.rectangle(image, (342, 582), (395, 634), (value,) * 3, -1)
                scene, _, reason = observe(image, self.calib, 1.)
                self.assertTrue(scene["valid"], reason)
                self.assertEqual(len(obstacles_near_car(scene)), 1)

    def test_small_specular_dot_on_floor_is_ignored(self):
        cv2.rectangle(self.image, (600, 240), (627, 267), (255, 255, 255), -1)
        scene, _, reason = observe(self.image, self.calib, 4.)
        self.assertTrue(scene["valid"], reason)
        self.assertEqual(scene["obstacles"], [])

    def test_ignores_large_white_floor_reflections_and_keeps_a_tagged_dark_robot(self):
        cv2.rectangle(self.image, (600, 560), (700, 630), (250, 251, 253), -1)
        cv2.rectangle(self.image, (340, 390), (440, 470), (255, 255, 255), -1)
        scene, _, reason = observe(self.image, self.calib, 4.)
        self.assertTrue(scene["valid"], reason)
        self.assertEqual(scene["obstacles"], [])

        cv2.rectangle(self.image, (300, 550), (425, 675), (35, 40, 45), -1)
        self.image[570:628, 326:384] = 255
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, (TAG_ID + 1) % 50, 40)
        self.image[579:619, 335:375] = marker[:, :, None]
        scene, _, reason = observe(self.image, self.calib, 4.)
        self.assertTrue(scene["valid"], reason)
        self.assertEqual(len(scene["obstacles"]), 1)
        (x, _, hx, hy), = obstacles_near_car(scene)
        self.assertLess(x, scene["robot"][0])
        self.assertGreater(min(hx, hy), .065)

    def test_filters_pale_blue_reflection_only_on_matching_floor(self):
        # The recorded lamp reflection is tinted blue and too dim to clip white.
        pale_blue = cv2.cvtColor(np.uint8([[[114, 51, 202]]]), cv2.COLOR_HSV2BGR)[0, 0]
        self.image[570:645, 315:375] = pale_blue
        self.image[390:465, 315:375] = pale_blue
        scene, _, reason = observe(self.image, self.calib, 4.)
        self.assertTrue(scene["valid"], reason)
        (_, y, _, _), = scene["obstacles"]
        self.assertGreater(y, 0.)

    def test_masks_own_chassis_edge_without_dropping_an_adjacent_robot(self):
        # A projected wheel/bottom edge protrudes beyond the roof-tag outline.
        self.image[792:865, 551:585] = (40, 45, 50)
        with patch("run_task.OBSTACLE_OUTLINE_PAD_MM", 0):
            scene, _, reason = observe(self.image, self.calib, 4.)
        self.assertTrue(scene["valid"], reason)
        self.assertEqual(len(scene["obstacles"]), 1)

        scene, _, reason = observe(self.image, self.calib, 4.)
        self.assertTrue(scene["valid"], reason)
        self.assertEqual(scene["obstacles"], [])

        # A dark opponent meets the residual edge but extends well beyond it.
        self.image[802:880, 581:661] = (30, 35, 40)
        scene, _, reason = observe(self.image, self.calib, 4.)
        self.assertTrue(scene["valid"], reason)
        (x, _, hx, _), = scene["obstacles"]
        self.assertGreater(x, scene["robot"][0])
        self.assertGreater(hx, .045)

    def test_yellow_start_maps_to_the_same_planner_orientation(self):
        blue_start, _, _ = observe(self.image, self.calib, 4., home_side=None)
        self.assertEqual(blue_start["home_side"], "blue")
        image = cv2.rotate(self.image, cv2.ROTATE_180)
        blue = np.all(image == (230, 50, 30), axis=2)
        yellow = np.all(image == (10, 230, 240), axis=2)
        image[blue] = (10, 230, 240)
        image[yellow] = (230, 50, 30)
        yellow_start, _, reason = observe(image, self.calib, 4., home_side=None)
        self.assertTrue(yellow_start["valid"], reason)
        self.assertEqual(yellow_start["home_side"], "yellow")
        self.assertTrue(can_start(yellow_start))
        np.testing.assert_allclose(yellow_start["robot"], blue_start["robot"], atol=.003)
        self.assertAlmostEqual(yellow_start["seam_bias"], blue_start["seam_bias"], delta=.003)
        policy = HybridCompetitionPolicy()
        policy.update(yellow_start)
        self.assertTrue(policy.valid)
        self.assertEqual(len(policy.pending()), 1)

    def test_locked_yellow_ownership_persists_after_robot_crosses(self):
        scene, _, reason = observe(self.image, self.calib, 4., home_side="yellow")
        self.assertTrue(scene["valid"], reason)
        self.assertEqual(scene["home_side"], "yellow")
        self.assertGreater(scene["robot"][1], 0)
        self.assertEqual([(y < 0, own) for _, y, _, _, own in sorted(scene["blocks"], key=lambda b: b[1])],
                         [(True, True), (False, False)])

    def test_wheel_mix_is_bounded_before_transmission(self):
        drive = DriveConfig("COM6", "CBAD", 10., 12., 100., 2500, (1, 1, 1, 1))
        output, physical = drive.convert((.20, -.10, .5))
        self.assertLessEqual(sum(map(abs, output)), 2500)
        self.assertGreater(physical[0], 0)
        self.assertLess(physical[1], 0)
        self.assertGreater(physical[2], 0)
        self.assertEqual(drive.convert((.1, 0., 0.))[0], (1000, 0, 0))
        with self.assertRaisesRegex(ValueError, "重新运行"):
            drive.verify_board((3600, (1, -1, 1, 1)))

    def test_encoder_calibration_corrects_roof_scale_and_layout(self):
        start = Pose(0., 0., math.pi / 2, 55.)
        end = Pose(0., .132, math.pi / 2, 55.)
        raw_signs = (-1, 1, 1, -1)
        self.assertAlmostEqual(trial_gain(0, 1, start, end, (0,) * 4,
                                         (-1200, 1200, 1200, -1200), "CBDA", raw_signs), 10.)
        left = Pose(-.132, 0., math.pi / 2, 55.)
        self.assertAlmostEqual(trial_gain(1, 1, start, left, (0,) * 4,
                                         (1440, 1440, -1440, -1440), "CBDA", raw_signs), 12.)

    def test_encoder_calibration_unwraps_turn_and_rejects_inconsistent_runs(self):
        start = Pose(0., 0., math.radians(170), 50.)
        end = Pose(0., 0., math.radians(-145), 50.)
        self.assertAlmostEqual(trial_gain(2, 1, start, end, (0,) * 4,
                                         (-4500, 4500, -4500, 4500), "ABCD", (1,) * 4), 100.)
        self.assertEqual(fit_gains(([10., 10.1, 9.9], [11., 11.1, 10.9], [15., 15.3, 14.7])),
                         (10., 11., 15.))
        with self.assertRaisesRegex(ValueError, "相差过大"):
            fit_gains(([10., 10.1, 7.], [11.], [15.]))

    def test_encoder_calibration_reports_the_failed_measurement(self):
        start = Pose(0., 0., 0., 50.)
        before = (10, 20, 30, 40)
        signs = (1, 1, -1, -1)
        normal = (1210, 1220, -1170, -1160)
        cases = (
            (Pose(.210, 0., 0., 50.), normal, "有效位移 210.0 mm 超出 90～192 mm"),
            (Pose(.120, 0., 0., 54.), normal, "tag 投影边长变化超过 3.5 mm"),
            (Pose(.120, 0., 0., 50.), before, "沿指令方向的编码器计数应为正"),
        )
        for end, after, reason in cases:
            with self.subTest(reason=reason):
                with self.assertRaisesRegex(ValueError, reason) as raised:
                    trial_gain(0, 1, start, end, before, after, "CBDA", signs)
                message = str(raised.exception)
                self.assertIn(f"tag 投影边长 50.0 → {end.side_mm:.1f} mm", message)
                self.assertIn(f"编码器原始增量 A={after[0] - before[0]:+d}", message)
                self.assertIn("轮位 CBDA（FL/FR/RL/RR）", message)


class ObstacleGateTest(unittest.TestCase):
    """The calibration gate holds a candidate just long enough to tell a
    stable object from a flickering floor reflection."""

    @staticmethod
    def feed(gate, candidates, stop=.6, step=.1):
        blocking, pending = [], False
        stamp = 0.
        while stamp <= stop + 1e-9:
            blocking, pending = gate.update(round(stamp, 6), candidates(stamp))
            stamp += step
        return blocking, pending

    def test_stable_candidate_confirms_and_blocks(self):
        gate = ObstacleGate()
        blocking, pending = self.feed(gate, lambda stamp: [(-.12, -.02, .09, .04)], stop=.2)
        self.assertEqual(len(blocking), 1)
        self.assertFalse(pending)

    def test_candidate_is_pending_until_it_can_be_judged(self):
        gate = ObstacleGate()
        blocking, pending = gate.update(0., [(-.12, -.02, .09, .04)])
        self.assertEqual(blocking, [])
        self.assertTrue(pending)

    def test_blinking_but_stable_candidate_stays_blocking(self):
        # A real object can be missed for a frame; do not treat stable size and
        # position as flicker just because detection blinked.
        gate = ObstacleGate()
        blocking, pending = self.feed(
            gate, lambda stamp: [(-.12, -.02, .09, .04)] if round(stamp * 10) % 2 == 0 else [],
            stop=1.2)
        self.assertEqual(len(blocking), 1)
        self.assertFalse(pending)

    def test_position_jitter_is_rejected_as_flicker(self):
        gate = ObstacleGate()
        blocking, pending = self.feed(
            gate, lambda stamp: [(-.12 + .015 * (round(stamp * 10) % 2), -.02, .09, .04)],
            stop=1.2)
        self.assertEqual(blocking, [])
        self.assertFalse(pending)

    def test_size_jitter_is_rejected_as_flicker(self):
        gate = ObstacleGate()
        blocking, pending = self.feed(
            gate, lambda stamp: [(-.12, -.02, .09 + .04 * (round(stamp * 10) % 2), .04)],
            stop=1.2)
        self.assertEqual(blocking, [])
        self.assertFalse(pending)

    def test_candidate_that_drifts_across_the_field_stays_blocking(self):
        gate = ObstacleGate()
        blocking, _ = self.feed(
            gate, lambda stamp: [(-.20 + .04 * round(stamp * 10), .0, .06, .06)], stop=.5)
        self.assertEqual(len(blocking), 1)

    def test_confirmed_candidate_is_held_briefly_then_released(self):
        gate = ObstacleGate()
        self.feed(gate, lambda stamp: [(-.12, -.02, .09, .04)], stop=.5)
        blocking, _ = gate.update(.6, [])
        self.assertEqual(len(blocking), 1)
        blocking, _ = gate.update(1.2, [])
        self.assertEqual(blocking, [])


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


class DriveWizardTest(unittest.TestCase):
    def test_saves_matching_input_and_obstacle_overlay(self):
        source = np.full((20, 20, 3), 80, dtype=np.uint8)
        boxed = source.copy()
        cv2.rectangle(boxed, (3, 3), (12, 12), (255, 100, 0), 2)
        with TemporaryDirectory() as folder, patch("builtins.print"):
            log_dir = Path(folder) / "drive_calibration"
            save_obstacle_images({"source_image": source}, boxed, 3, log_dir)
            for label, expected in (("input", source), ("boxed", boxed)):
                path, = log_dir.glob(f"*_step03_{label}.png")
                image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
                np.testing.assert_array_equal(image, expected)

    def test_checks_the_near_edge_of_an_obstacle_and_ignores_distant_wall(self):
        distant_wall = (-.12, -.56, .15, .043)
        nearby = (.44, .10, .12, .03)
        scene = {"robot": (.024, .10, 0.), "obstacles": [distant_wall, nearby]}
        self.assertEqual(obstacles_near_car(scene), [nearby])

    def test_stable_pose_tolerates_one_noisy_tag_frame(self):
        samples = [(i * .1, Pose(0., 0., 0., 50.)) for i in range(4)]
        samples.append((.4, Pose(.02, .02, 0., 50.)))
        self.assertIsNotNone(stable_pose(samples))

        samples = [(i * .1, Pose(0., 0., 0., 50.)) for i in range(3)]
        samples += [(.3, Pose(.02, 0., 0., 50.)), (.4, Pose(.03, 0., 0., 50.))]
        self.assertIsNone(stable_pose(samples))

    def test_motion_clearance_allows_return_but_blocks_outward_motion(self):
        pose = Pose(.29, 0., 0., 55.)
        self.assertTrue(motion_fits(pose, 0, -1))
        self.assertFalse(motion_fits(pose, 0, 1))
        self.assertTrue(motion_fits(pose, 1, 1))
        self.assertFalse(motion_fits(Pose(.39, 0., 0., 55.), 0, -1))

    def test_runs_all_axes_with_delayed_vision_and_coasting(self):
        self.run_drive_wizard(check_obstacles=True)

    def test_empty_field_calibration_ignores_near_car_candidates(self):
        self.run_drive_wizard(check_obstacles=False)

    def run_drive_wizard(self, *, check_obstacles):
        # A legal off-centre start leaves the first endpoint outside the
        # starting area. At 25 Hz the last five frames span only 160 ms.
        now, x, y, heading = 0., .11, 0., 0.
        dt = .04
        coast_time = .32
        limit = 5963
        expected_gains = (21.5, 25., 60.)
        wheel_counts = [0.] * 4
        target = (0, 0, 0)
        coast_target = target
        axes_moved = set()
        frames = deque(maxlen=4)
        snapshots = []
        snapshot_positions = []
        stops = []
        saw_new_candidate = False
        freeze_until = None
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        camera = SimpleNamespace(error=None, start=Mock(), close=Mock())
        vision = SimpleNamespace(camera=camera, error=None, stop=Mock(), latest=None,
                                 thread=SimpleNamespace(start=Mock(), is_alive=lambda: False))

        def publish():
            nonlocal saw_new_candidate
            # A flickering near-car candidate must not block the wizard; it is
            # rejected as local floor flicker after the observation window.
            obstacle = (-.02 + .02 * (int(now * 10) % 2), -.02,
                        .09 + .02 * (int(now * 10) % 3), .04)
            obstacles = [obstacle]
            if any(target) and not saw_new_candidate:
                obstacles += [(.20, .06, .035, .035)]
                saw_new_candidate = True
            frames.append(dict(stamp=now, valid=True, robot=(1.1*x, 1.1*y, heading),
                               seam_bias=0., tag_side_mm=55., blocks=[], obstacles=obstacles))
            # Deliver the pose from three ticks ago to simulate image latency.
            if len(frames) == frames.maxlen and (freeze_until is None or now >= freeze_until):
                vision.latest = (frames[0], image, "")

        def stream(x_ticks, y_ticks, turn_ticks, *, valid_until):
            nonlocal target
            self.assertGreater(valid_until, now)
            self.assertLessEqual(valid_until, vision.latest[0]["stamp"] + .35)
            self.assertGreater(vision.latest[0]["stamp"], snapshots[-1])
            target = (x_ticks, y_ticks, turn_ticks)
            axes_moved.add(target)
            return True

        def stop():
            nonlocal target, coast_target
            coast_target = target
            target = (0, 0, 0)
            stops.append(now)

        def snapshot():
            self.assertFalse(any(target))
            if stops:
                self.assertGreaterEqual(now - stops[-1], coast_time)
            snapshots.append(now)
            snapshot_positions.append(1.1 * x)
            return tuple(round(count) for count in wheel_counts)

        def tick(_):
            nonlocal now, x, y, heading, freeze_until
            forward, left, ccw = target
            if check_obstacles and any(target) and len(stops) == 1 and freeze_until is None:
                freeze_until = now + .43
            # Model residual travel at the rate of the preceding command.
            if stops and not any(target) and now - stops[-1] < coast_time - 1e-9:
                forward, left, ccw = coast_target
            advance = dt * forward / (1000 * expected_gains[0])
            side = dt * left / (1000 * expected_gains[1])
            x += math.cos(heading) * advance - math.sin(heading) * side
            y += math.sin(heading) * advance + math.cos(heading) * side
            heading += math.radians(dt * ccw / expected_gains[2])
            wheels = (forward-left-ccw, forward+left+ccw, forward+left-ccw, forward-left+ccw)
            for index, ticks in enumerate(wheels):
                wheel_counts[index] += dt * ticks
            now += dt
            self.assertLess(now, 150, "标定流程没有结束")
            publish()

        link = SimpleNamespace(error=None, ready=True, speed_info=(limit, (1,) * 4), start=Mock(),
                               close=Mock(), stream=stream, stop=stop, encoder_snapshot=snapshot)
        publish()
        with patch("calibrate_drive.time", SimpleNamespace(monotonic=lambda: now, sleep=tick)), \
                patch("calibrate_drive.cv2", Mock(waitKey=lambda _: 32, getWindowProperty=lambda *_: 1.)), \
                patch("builtins.print"):
            gains, cap, _ = measure_drive(vision, link, "ABCD", .35, .10, check_obstacles=check_obstacles)
        self.assertEqual(cap, limit)
        self.assertLess(snapshot_positions[0], .24)
        self.assertGreater(snapshot_positions[1], .24)
        np.testing.assert_allclose(gains[:2], expected_gains[:2], rtol=.004)
        # The stable-pose median can include a frame from the end of the coast.
        self.assertAlmostEqual(gains[2], expected_gains[2], delta=1.5)
        self.assertEqual(axes_moved, {(1987, 0, 0), (-1987, 0, 0), (0, 1987, 0),
                                      (0, -1987, 0), (0, 0, 1987), (0, 0, -1987)})
        self.assertTrue(saw_new_candidate)
        # Only the first case exercises an obstacle pause and a frozen-frame pause.
        self.assertEqual(len(stops), 20 if check_obstacles else 18)
        link.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
