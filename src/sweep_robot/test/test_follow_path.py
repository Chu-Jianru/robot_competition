"""不启动 Gazebo，检查自动控制最容易回归的运动约束。"""

import math
from types import SimpleNamespace

from sweep_robot.follow_path import (
    FollowPath,
    _contact_ok,
    _lead_face,
    _opening,
    _opening_modes,
    advance_pose,
    limit_block_contact,
)


def test_opening_actions_match_goals():
    route = _opening()
    modes = _opening_modes()
    assert len(route) == len(modes)
    assert route[1][:2] == route[0][:2]
    assert modes[1] == "turn"
    assert modes[3] == "push_east"
    assert modes[6] == "push_east"
    assert modes[9] == "turn"
    assert modes[11] == "push_north"
    assert route[7][0] <= 0.20 and route[8][0] <= 0.20
    assert route[8][1] <= -0.48 and route[10][1] <= -0.48


def test_transit_cannot_touch_block_with_front():
    pose = (0.0, 0.0, 0.0)
    block = [(0.08, 0.0)]
    assert _contact_ok(pose, (0.20, 0.0, 0.0), block, True)
    assert not _contact_ok(pose, (0.20, 0.0, 0.0), block, False)


def test_captured_block_can_move_forward_but_not_during_transit():
    pose = (0.0, 0.0, 0.0)
    captured = [(0.0, 0.0)]
    assert _contact_ok(pose, (0.20, 0.0, 0.0), captured, True)
    assert not _contact_ok(pose, (0.0, 0.0, 1.0), captured, False)


def test_front_corner_guide_plate_is_valid_push_contact():
    pose = (0.5292, 0.0779, 1.5746)
    wall_block = [(0.5931, 0.1158)]
    assert _contact_ok(pose, (0.20, 0.0, 0.0), wall_block, True)
    assert not _contact_ok(pose, (0.0, 0.0, 1.0), wall_block, False)


def test_long_front_guide_keeps_wall_block_out_of_wheel_side():
    pose = (0.0, 0.0, 0.0)
    assert _contact_ok(pose, (0.08, 0.0, 0.0), [(0.016, -0.078)], True)
    assert not _contact_ok(pose, (0.08, 0.0, 0.0), [(-0.010, -0.070)], True)


def test_turn_backs_away_from_near_block_then_turns():
    pose = (-0.423, 0.221, 1.588)
    blocks = [(-0.419, 0.308)]
    previous = (0.0, 0.0, 0.0)
    backed_up = False
    for _ in range(30):
        cmd = limit_block_contact(
            pose,
            (0.0, 0.0, -2.2),
            blocks,
            previous,
            False,
        )
        assert _contact_ok(pose, cmd, blocks, False)
        backed_up |= cmd[0] < -0.05
        pose = advance_pose(pose, cmd, 0.05)
        previous = cmd
        if abs(cmd[2]) > 0.40:
            break
    assert backed_up
    assert abs(cmd[2]) > 0.40


def test_corner_pile_does_not_slow_a_push_from_the_south():
    pose = (0.53, -0.32, math.pi / 2)
    face = _lead_face(
        pose,
        [(0.58, -0.22), (0.55, 0.55)],
        [],
        0.53,
        0.50,
        math.pi / 2,
    )
    assert face is not None and face < 0.0
    assert _contact_ok(pose, (0.08, 0.0, 0.02), [(0.588, -0.226)], True)


def test_align_does_not_strafe_until_facing_the_lane():
    node = SimpleNamespace(
        pose=(0.20, -0.50, 1.90),
        goals=[(0.51, -0.50, math.pi / 2)],
        index=0,
        blocks=[],
        extents=[],
        _current_mode=lambda: "align",
    )
    FollowPath._steer(node)
    assert abs(node.target[0]) < 0.02
    assert abs(node.target[1]) < 0.02
    assert node.target[2] < 0.0
    straight = SimpleNamespace(
        pose=(0.20, -0.50, math.pi / 2),
        goals=[(0.51, -0.50, math.pi / 2)],
        index=0,
        blocks=[],
        extents=[],
        _current_mode=lambda: "align",
    )
    FollowPath._steer(straight)
    assert math.hypot(straight.target[0], straight.target[1]) <= 0.22


def test_align_starts_the_north_push_when_the_brush_meets_a_block():
    completed = []
    node = SimpleNamespace(
        pose=(0.46, -0.50, math.pi / 2),
        goals=[(0.53, -0.50, math.pi / 2)],
        index=0,
        wall_hits=0,
        blocks=[(0.52, -0.45)],
        _current_mode=lambda: "align",
        _against_wall=lambda *_args: False,
        _complete_goal=lambda: completed.append(True) or False,
    )
    FollowPath._advance(node)
    assert completed


def test_cleanup_starts_with_the_east_column():
    node = SimpleNamespace(
        pose=(0.50, 0.48, math.pi / 2),
        phase="route",
        current_x=None,
        current_anchor_x=None,
        goals=[],
        goal_modes=[],
        index=0,
        _own_targets=lambda: [(-0.49, -0.15), (0.53, -0.40)],
        last_detected_x=None,
        own_repeats=0,
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None),
    )
    FollowPath._begin_own(node)
    assert node.current_x > 0.40
    assert math.isclose(node.goals[-1][1], 0.50)


def test_wall_lane_reaches_blocks_against_the_east_wall():
    node = SimpleNamespace(
        blocks=[(0.58, -0.05), (0.20, 0.10)],
        goals=[(0.0, 0.0, 0.0)] * 12,
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None),
    )
    FollowPath._apply_wall_lane(node)
    assert 0.515 <= node.goals[10][0] <= 0.530
    assert node.goals[11][0] == node.goals[10][0]


def test_long_merged_blob_does_not_fake_wall_contact():
    pose = (0.20, 0.05, 0.0)
    face = _lead_face(
        pose,
        [(0.40, 0.05)],
        [(0.20, 0.01)],
        0.51,
        0.05,
        0.0,
    )
    assert face is not None and face < 0.585


def test_control_pose_uses_latest_measurement_without_overshoot():
    node = SimpleNamespace(
        measured=(0.0, 0.0, 0.0),
        pose=(9.0, 9.0, 1.0),
        pose_time=0.95,
        vision_velocity=(0.2, -0.1, 0.4),
        _now=lambda: 1.0,
    )
    FollowPath._predict(node, 0.05)
    assert math.isclose(node.pose[0], 0.0)
    assert math.isclose(node.pose[1], 0.0)


def test_push_completes_after_crossing_target_line():
    completed = []
    node = SimpleNamespace(
        pose=(-0.42, 0.33, math.pi / 2),
        goals=[(-0.42, 0.24, math.pi / 2)],
        index=0,
        wall_hits=0,
        _current_mode=lambda: "push_north",
        _against_wall=lambda *_args: False,
        _complete_goal=lambda: completed.append(True) or False,
    )
    FollowPath._advance(node)
    assert completed


def test_turn_completion_does_not_require_original_position():
    completed = []
    node = SimpleNamespace(
        pose=(-0.42, -0.09, 0.02),
        goals=[(-0.42, 0.24, 0.0)],
        index=0,
        wall_hits=0,
        _current_mode=lambda: "turn",
        _against_wall=lambda *_args: False,
        _complete_goal=lambda: completed.append(True) or False,
    )
    FollowPath._advance(node)
    assert completed


def test_remaining_block_route_leaves_wall_pile_before_going_south():
    node = SimpleNamespace(
        pose=(0.50, 0.52, math.pi / 2),
        phase="route",
        current_x=None,
        current_anchor_x=None,
        goals=[],
        goal_modes=[],
        index=0,
        _own_targets=lambda: [(0.52, -0.08)],
        last_detected_x=None,
        own_repeats=0,
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None),
    )
    FollowPath._begin_own(node)
    assert node.goal_modes[:2] == ["align", "retreat"]
    assert node.goals[0][0] <= 0.38
    assert node.goals[1][1] <= -0.42
    assert math.isclose(node.goals[-1][1], 0.50)


def test_east_column_stages_south_of_its_tail():
    node = SimpleNamespace(
        pose=(0.50, 0.52, math.pi / 2),
        phase="route",
        current_x=None,
        current_anchor_x=None,
        goals=[],
        goal_modes=[],
        index=0,
        _own_targets=lambda: [(0.50, -0.40), (0.52, -0.06)],
        last_detected_x=None,
        own_repeats=0,
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None),
    )
    FollowPath._begin_own(node)
    assert node.goals[1][1] <= -0.50
    assert node.goals[2][1] <= -0.50
    assert math.isclose(node.goals[-1][1], 0.50)


def test_far_remaining_block_uses_clear_field_diagonal():
    node = SimpleNamespace(
        pose=(0.50, 0.48, math.pi / 2),
        phase="route",
        current_x=None,
        current_anchor_x=None,
        goals=[],
        goal_modes=[],
        index=0,
        _own_targets=lambda: [(-0.52, -0.08)],
        last_detected_x=None,
        own_repeats=0,
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None),
    )
    FollowPath._begin_own(node)
    assert node.goal_modes == ["retreat", "transit", "approach", "push_north"]
    assert node.goals[1][0] == -0.52
    assert node.goals[1][1] <= -0.42


def test_cavity_block_can_be_dumped_in_reverse():
    pose = (0.0, 0.0, 0.0)
    captured = [(0.0, 0.0)]
    assert _contact_ok(pose, (-0.12, 0.0, 0.0), captured, False)
    dumped = limit_block_contact(pose, (-0.30, -0.10, 0.0), captured, (0.0, 0.0, 0.0), False)
    assert dumped[0] < -0.05


def test_avoidance_does_not_drive_opposite_the_command():
    pose = (0.0, 0.05, 0.0)
    # 南侧贴着车，往南对准会被挡住。避碰不能改成往东横穿。
    blocks = [(0.02, -0.07)]
    cmd = limit_block_contact(pose, (-0.20, -0.25, 0.0), blocks, (0.0, 0.0, 0.0), False)
    assert cmd[0] <= 0.02


def test_old_wall_pile_does_not_slow_new_row_from_west_side():
    node = SimpleNamespace(
        pose=(-0.50, -0.04, 0.0),
        goals=[(0.51, -0.04, 0.0)],
        index=0,
        blocks=[(0.58, 0.02)],
        extents=[(0.01, 0.01)],
        target=(0.0, 0.0, 0.0),
        _current_mode=lambda: "push_east",
    )
    FollowPath._steer(node)
    assert node.target[0] > 0.25


if __name__ == "__main__":
    test_opening_actions_match_goals()
    test_transit_cannot_touch_block_with_front()
    test_captured_block_can_move_forward_but_not_during_transit()
    test_front_corner_guide_plate_is_valid_push_contact()
    test_long_front_guide_keeps_wall_block_out_of_wheel_side()
    test_turn_backs_away_from_near_block_then_turns()
    test_long_merged_blob_does_not_fake_wall_contact()
    test_control_pose_uses_latest_measurement_without_overshoot()
    test_push_completes_after_crossing_target_line()
    test_turn_completion_does_not_require_original_position()
    test_remaining_block_route_leaves_wall_pile_before_going_south()
    test_far_remaining_block_uses_clear_field_diagonal()
    test_cavity_block_can_be_dumped_in_reverse()
    test_avoidance_does_not_drive_opposite_the_command()
    test_old_wall_pile_does_not_slow_new_row_from_west_side()
    print("运动控制测试通过")
