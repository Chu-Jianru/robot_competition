"""nav_servo 的离线回归：坐标换算、限幅、安全状态机、仿真收敛。

不需要相机、底盘或 pyserial：
    python windows/test_nav_servo.py
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from nav_servo import (PoseServo, ServoConfig, SimChassis,  # noqa: E402
                       body_error, plan_step, wrap_deg)


class AngleTest(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(wrap_deg(0), 0)
        self.assertAlmostEqual(wrap_deg(370), 10)
        self.assertAlmostEqual(wrap_deg(-370), -10)
        self.assertAlmostEqual(wrap_deg(180), 180)
        self.assertAlmostEqual(wrap_deg(-180), 180)
        self.assertAlmostEqual(wrap_deg(540), 180)


class BodyFrameTest(unittest.TestCase):
    def test_east_heading(self):
        fwd, left = body_error(0, 0, 0, 100, 50)
        self.assertAlmostEqual(fwd, 100)
        self.assertAlmostEqual(left, 50)

    def test_north_heading(self):
        # 朝北时，东在右手边，场地 +y 是正前方。
        fwd, left = body_error(0, 0, 90, 0, 100)
        self.assertAlmostEqual(fwd, 100)
        self.assertAlmostEqual(left, 0)
        fwd, left = body_error(0, 0, 90, 100, 0)
        self.assertAlmostEqual(fwd, 0, places=6)
        self.assertAlmostEqual(left, -100, places=6)

    def test_west_heading(self):
        fwd, left = body_error(0, 0, 180, -80, 0)
        self.assertAlmostEqual(fwd, 80, places=6)
        self.assertAlmostEqual(left, 0, places=6)

    def test_diagonal(self):
        fwd, left = body_error(0, 0, 45, 100, 100)
        self.assertAlmostEqual(fwd, math.hypot(100, 100))
        self.assertAlmostEqual(left, 0, places=6)


class ServoTest(unittest.TestCase):
    class Pose:
        def __init__(self, x, y, h):
            self.x_mm, self.y_mm, self.heading_deg = x, y, h

    def test_already_there(self):
        servo = PoseServo(100, 100, 0)
        self.assertIsNone(servo.command(self.Pose(102, 98, 1)))

    def test_turn_first(self):
        servo = PoseServo(100, 100, 90)
        command = servo.command(self.Pose(100, 100, 0))
        self.assertEqual(command.kind, "turn")
        self.assertGreater(command.ccw, 0)
        self.assertEqual((command.forward, command.left), (0, 0))

    def test_turn_backwards(self):
        servo = PoseServo(100, 100, 0)
        command = servo.command(self.Pose(100, 100, 90))
        self.assertEqual(command.kind, "turn")
        self.assertLess(command.ccw, 0)

    def test_move_forward_east(self):
        servo = PoseServo(300, 100, 0)
        command = servo.command(self.Pose(100, 100, 0))
        self.assertEqual(command.kind, "move")
        self.assertGreater(command.forward, 0)
        self.assertEqual(command.left, 0)

    def test_move_left_when_facing_north(self):
        # 朝北停车时，场地 -x（西）在车体左侧。
        servo = PoseServo(0, 100, 90)
        command = servo.command(self.Pose(100, 100, 90))
        self.assertGreater(command.left, 0)
        self.assertEqual(command.forward, 0)

    def test_percent_is_limited(self):
        config = ServoConfig(max_percent=40)
        servo = PoseServo(1000, 1000, 0, config)
        command = servo.command(self.Pose(0, 0, 0))
        self.assertLessEqual(math.hypot(command.forward, command.left), 40)
        self.assertLessEqual(command.ms, config.max_pulse_ms)

    def test_start_percent_breaks_deadband(self):
        config = ServoConfig(start_percent=12, pos_tol_mm=5)
        servo = PoseServo(12, 0, 0, config)
        command = servo.command(self.Pose(0, 0, 0))
        self.assertGreaterEqual(abs(command.forward), 12)

    def test_pulse_length_is_bounded(self):
        config = ServoConfig(min_pulse_ms=150, max_pulse_ms=600)
        servo = PoseServo(5000, 0, 0, config)
        command = servo.command(self.Pose(0, 0, 0))
        self.assertGreaterEqual(command.ms, 150)
        self.assertLessEqual(command.ms, 600)


class PlanStepTest(unittest.TestCase):
    class Pose:
        def __init__(self, x, y, h):
            self.x_mm, self.y_mm, self.heading_deg = x, y, h

    def setUp(self):
        self.servo = PoseServo(300, 300, 0)

    def test_deadline_stops(self):
        step = plan_step(self.Pose(0, 0, 0), 0.0, 90.0, self.servo)
        self.assertEqual(step.action, "stop")
        self.assertEqual(step.reason, "deadline")

    def test_missing_pose_stops(self):
        self.assertEqual(plan_step(None, 0.0, 1.0, self.servo).reason, "no_pose")

    def test_stale_pose_stops(self):
        step = plan_step(self.Pose(0, 0, 0), 0.5, 1.0, self.servo,
                         pose_timeout_s=0.35)
        self.assertEqual(step.action, "stop")
        self.assertEqual(step.reason, "stale_pose")

    def test_fresh_pose_pulses(self):
        step = plan_step(self.Pose(0, 0, 0), 0.1, 1.0, self.servo)
        self.assertEqual(step.action, "pulse")

    def test_reached(self):
        step = plan_step(self.Pose(300, 300, 0), 0.1, 1.0, self.servo)
        self.assertEqual(step.action, "done")


def drive(sim, servo, deadline_s=90.0, max_steps=200):
    clock = 0.0
    for _ in range(max_steps):
        step = plan_step(sim.pose, 0.0, clock, servo, deadline_s=deadline_s)
        if step.action != "pulse":
            return step, clock
        command = step.command
        sim.pulse(command.forward, command.left, command.ccw, command.ms)
        clock += command.ms / 1000.0
    return None, clock


class SimulationTest(unittest.TestCase):
    def test_converges_to_target(self):
        config = ServoConfig()
        servo = PoseServo(400, 300, 0, config)
        sim = SimChassis(150, 200, 90, config)
        step, _ = drive(sim, servo)
        self.assertIsNotNone(step)
        self.assertEqual(step.action, "done")
        error = math.hypot(400 - sim.pose.x_mm, 300 - sim.pose.y_mm)
        self.assertLessEqual(error, config.pos_tol_mm + 1e-6)
        self.assertLessEqual(abs(wrap_deg(0 - sim.pose.heading_deg)),
                             config.heading_tol_deg + 1e-6)
        self.assertLess(sim.pulses, 40)

    def test_first_pulse_is_a_turn(self):
        config = ServoConfig()
        servo = PoseServo(400, 300, 0, config)
        sim = SimChassis(150, 200, 90, config)
        step = plan_step(sim.pose, 0.0, 0.0, servo)
        self.assertEqual(step.command.kind, "turn")

    def test_converges_with_gain_error(self):
        # 车辆比模型慢 15%：会多走几拍，但仍应收敛，不允许发散。
        config = ServoConfig()
        servo = PoseServo(400, 300, 0, config)
        sim = SimChassis(150, 200, 90, config, gain_error=0.85, noise_mm=1.0, seed=3)
        step, _ = drive(sim, servo, max_steps=400)
        self.assertIsNotNone(step)
        self.assertEqual(step.action, "done")
        self.assertLessEqual(math.hypot(400 - sim.pose.x_mm, 300 - sim.pose.y_mm),
                             config.pos_tol_mm + 2.0)

    def test_slow_vehicle_still_bounded(self):
        config = ServoConfig()
        servo = PoseServo(250, 250, 0, config)
        sim = SimChassis(200, 200, 0, config, gain_error=0.6)
        step, _ = drive(sim, servo)
        self.assertEqual(step.action, "done")
        self.assertLess(sim.pulses, 40)


if __name__ == "__main__":
    unittest.main(verbosity=2)
