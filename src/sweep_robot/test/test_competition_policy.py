"""不依赖 ROS：场景变化、遮挡、对手占位、时限和运动约束。"""
import math
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sweep_robot.competition_policy import CompetitionPolicy, SmoothVelocity, Object, plan_path as bounded_plan_path, segment_clear, body_gap


def plan_path(start,goal,objects):
    # 几何测试不依赖测试机负载；比赛默认仍有25ms搜索预算。
    return bounded_plan_path(start,goal,objects,budget=None)


def scene(t, blocks=(), obstacles=(), robot=(-0.42, -0.48, math.pi / 2), valid=True):
    return dict(stamp=t, valid=valid, robot=robot, blocks=list(blocks), obstacles=list(obstacles))


def test_stale_and_invalid_cannot_move_or_finish():
    p = CompetitionPolicy()
    p.update(scene(0, [(0, -.06, .01, .01, True)]))
    for t in (0, .05, .1): p.step(t)
    p.update(scene(.15, valid=False))
    assert p.step(.15) == (0, 0, 0)
    assert p.phase != 'ended'
    p.update(scene(.2, [(0, -.06, .01, .01, True)]))
    assert p.step(.8) == (0, 0, 0)
    p.update(scene(.9, robot=(float('nan'), 0, 0)))
    assert p.step(.9) == (0, 0, 0)


def test_returned_blocks_restart_after_empty_field():
    p = CompetitionPolicy()
    p.update(scene(0)); p.step(0)
    assert p.phase == 'guard'
    p.update(scene(.5, [(.15, -.08, .01, .01, True)])); p.step(.5)
    assert p.phase == 'transit' and p.lane is not None
    assert p.step(90) == (0, 0, 0) and p.phase == 'ended'
    p.update(scene(91, [(.1, -.1, .01, .01, True)]))
    assert p.step(91) == (0, 0, 0)


def test_single_frame_miss_preserves_blocks():
    p = CompetitionPolicy()
    p.update(scene(1, [(0, -.06, .01, .01, True)]))
    p.update(scene(1.1))
    assert len(p.pending()) == 1
    p.update(scene(1.6))
    assert not p.pending()


def test_blocking_opponent_changes_lane():
    p = CompetitionPolicy()
    p.update(scene(0, [(-.3, -.06, .01, .01, True), (.3, -.06, .01, .01, True)],
                   [(-.3, .04, .07, .07)]))
    p.step(0)
    assert p.lane > .2, p.events
    p.phase = 'push'; p.pose = (.3, -.1, math.pi / 2)
    p.update(scene(.1, [(.3, -.02, .01, .01, True)], [(.3, .12, .07, .07)], robot=p.pose))
    command = p.step(.1)
    assert command[0] < 0 and p.safe(command,.1)
    assert p.phase == 'evade'


def test_safe_adjacent_lane_does_not_trigger_repeated_evasion():
    p = CompetitionPolicy()
    for i in range(10):
        p.update(scene(i*.1, [(.129,-.048,.01,.01,True)],
                       [(.282,.016,.062,.068)], robot=(.083,-.182,math.pi/2)))
    assert not p.corridor_blocked(.083,-.182,.37)
    assert not p.conflict()
    p.phase='align';p.lane=.083;p.finish_y=.28
    command=p.step(.9)
    assert p.phase=='push' and command[0]>0 and p.safe(command,.9)
    p.update(scene(1, [(.129,-.048,.01,.01,True)],
                   [(.17,.016,.062,.068)], robot=(.083,-.12,math.pi/2)))
    assert p.conflict()
    assert p.step(1)[0]<=0
    assert p.obstacles[0].still_for==0


def test_release_route_checks_all_guide_and_block_axes():
    # 真实失败局面；完整分离轴检查能识别此前被误报成导板角碰撞的出口。
    objects=[Object(-.0345,.3586,.0143,.0207),Object(.0368,.4436,.0092,.0092),
             Object(.0598,.4436,.0092,.0092),Object(.1386,.4472,.0227,.0139),
             Object(.2823,.0161,.077,.083,kind='obstacle')]
    start=(.0828,.3433);goal=(.48,-.18)
    assert segment_clear(start,(start[0],-.22),objects,escaping=True)
    path=bounded_plan_path(start,goal,objects,budget=.1)
    assert path
    previous=start
    for i,point in enumerate(path):
        assert segment_clear(previous,point,objects,escaping=i==0)
        previous=point


def test_stationary_and_crossing_obstacles_are_retained():
    p = CompetitionPolicy()
    p.update(scene(0, obstacles=[(.3, 0, .06, .06)]))
    p.update(scene(.1, obstacles=[(.25, 0, .06, .06)]))
    assert p.obstacles[0].vx < -.2
    predicted = p.forecast_obstacles()[0]
    assert predicted.x < .25 and predicted.hx > .06
    p.update(scene(.2)); assert p.obstacles
    p.update(scene(1)); assert not p.obstacles


def test_grid_route_avoids_obstacle_and_corners():
    objects = [Object(0, -.2, .08, .09)]
    path = plan_path((-.4, -.2), (.4, -.2), objects)
    assert len(path) > 1
    current = (-.4, -.2)
    for point in path:
        assert segment_clear(current, point, objects)
        current = point
    barrier = [Object(0, -.1, .01, .7)]
    assert not plan_path((-.4, -.2), (.4, -.2), barrier)


def test_smoother_limits_acceleration_and_jerk():
    smooth = SmoothVelocity()
    last = (0, 0, 0); acceleration = (0, 0, 0)
    for i in range(60):
        current = smooth.step((.3 if i < 25 else -.3, -.2, .5), .05)
        acc = tuple((current[j] - last[j]) / .05 for j in range(3))
        assert max(abs(a) for a in acc[:2]) <= .75001
        # At a target clamp the acceleration can become zero immediately;
        # away from that saturation, the jerk is explicitly bounded.
        for j in range(3):
            if current[j] != ((.3 if i < 25 else -.3), -.2, .5)[j]:
                assert abs(acc[j] - acceleration[j]) <= (5 if j == 2 else 3.5) * .05 + 1e-6
        last, acceleration = current, acc


def test_applied_velocity_is_checked_after_smoothing():
    p = CompetitionPolicy(); p.start_time = 0
    p.update(scene(1, obstacles=[(0, 0, .07, .07)], robot=(0, -.2, math.pi/2)))
    p.phase = 'push'; p.lane = 0
    p.smoother.velocity = [.28, 0, 0]
    assert p.step(1) == (0, 0, 0)


def test_close_target_not_replaced_every_camera_frame():
    p = CompetitionPolicy()
    p.update(scene(0, [(0, -.06, .01, .01, True)])); p.step(0)
    lane = p.lane
    for i in range(1, 6):
        t = i * .05
        p.update(scene(t, [(.002*i, -.06, .01, .01, True)])); p.step(t)
        assert p.lane == lane


def test_smooth_reverse_can_release_a_touching_pile():
    p = CompetitionPolicy()
    p.update(scene(0, [(0,.08,.01,.015,False)], robot=(0,0,math.pi/2)))
    p.phase = 'transit'
    # 第一拍加速度斜坡只走几十分之一毫米，不能要求立即退出固定3mm。
    command = p.smoother.step((-.2,0,0),.05)
    assert p.safe(command,0)
    assert not p.safe((.05,0,0),0)
    assert p.safe((0,0,0),0)


def test_route_uses_rear_clearance_between_two_rows():
    objects = [Object(0,.09,.014,.02),Object(0,-.12,.01,.01)]
    path = plan_path((0,0),(.3,-.3),objects)
    assert path


def test_pushed_heading_recovers_by_making_room_before_turning():
    p = CompetitionPolicy()
    p.update(scene(0,[(.07,.02,.01,.01,False)],robot=(0,0,.45)))
    p.step(0)
    assert p.phase == 'evade'
    assert p.escape_target[0] < 0 and p.escape_target[2] == 0


def test_shallow_corner_uncertainty_does_not_trap_the_robot():
    objects = [Object(-.1396,.2005,.0104,.0104),Object(-.0689,.2535,.0104,.0104),
               Object(-.0463,.2525,.007,.0093),Object(.0317,.2664,.0116,.0116),
               Object(.0562,.0476,.0104,.0104),Object(-.1622,.0556,.0093,.0093)]
    for y in [.1598,.1631,.167]:
        start = (-.0462,y)
        assert plan_path(start,(.30,-.19),objects)
    assert not plan_path(start,(-.1396,.2005),objects)


def test_dense_midline_returns_through_a_cleared_column():
    objects = [Object(x,-.05,.011,.011,True) for x in [-.234,.036,.133,.259,.318,.444]]
    objects += [Object(x,.05,.011,.011) for x in [-.258,-.162,.056,.15,.24,.34,.44]]
    objects += [Object(x,.37,.012,.012) for x in [-.43,-.35,-.124,-.069,-.046]]
    start=(-.062,.273)
    path=plan_path(start,(.12,-.18),objects)
    assert path
    previous=start
    for i,point in enumerate(path):
        assert segment_clear(previous,point,objects,escaping=i==0)
        previous=point


def test_rotated_body_and_heading_recovery_without_a_route():
    assert abs(body_gap((0,0,0),Object(-.07,0))-body_gap((0,0,math.pi/2),Object(0,-.07)))<1e-9
    p=CompetitionPolicy()
    p.update(scene(0,robot=(0,-.3,2.28)))
    p.phase='yield'
    command=p.raw_command(0)
    assert command[:2]==(0,0) and command[2]<0
    assert p.safe(command,0)


def test_real_20hz_push_start_does_not_deadlock_at_the_brush():
    p=CompetitionPolicy()
    p.update(scene(0,[(0,.08,.01,.01,True)],robot=(0,0,math.pi/2)))
    p.phase='push'
    command=p.smoother.step((.28,0,0),.05)
    assert 0<command[0]<.025
    assert p.safe(command,0)
    p.lane=.02
    raw=p.raw_command(0)
    p.smoother.reset()
    command=p.smoother.step(raw,.05)
    assert p.safe(command,0)
    p.phase='transit'
    assert not p.safe(command,0)


def test_incoming_rear_block_can_trigger_a_legal_forward_escape():
    p=CompetitionPolicy()
    for t,y in [(0,-.20),(.1,-.18),(.2,-.16),(.3,-.14)]:
        p.update(scene(t,[(0,.08,.01,.01,False),(0,y,.01,.01,True)],robot=(0,0,math.pi/2)))
    p.phase='guard'
    assert p.blocks[-1].vy>.1 and p.conflict()
    command=p.step(.3)
    assert p.phase=='evade' and command[0]>0
    assert p.safe(command,.3)


def test_cavity_payload_is_allowed_but_not_a_side_collision():
    p=CompetitionPolicy()
    p.update(scene(0,[(0,-.04,.01,.01,True)],robot=(0,0,math.pi/2)))
    p.phase='push'
    assert not p.conflict() and p.safe((.12,0,0),0)
    p.phase='transit'
    assert p.safe((-.12,0,0),0)


def test_background_path_is_rechecked_against_a_new_obstacle():
    p=CompetitionPolicy()
    p.update(scene(0,[(.3,-.06,.01,.01,True)],robot=(0,-.4,math.pi/2)))
    p.start_planning(0,.3,(.3,-.2),.28)
    deadline=time.monotonic()+1
    while p.planning_result is None and time.monotonic()<deadline:time.sleep(.005)
    assert p.planning_result
    p.update(scene(.1,[(.3,-.06,.01,.01,True)],[(.15,-.3,.05,.05)],robot=(0,-.4,math.pi/2)))
    p.finish_planning(.1)
    assert p.phase=='yield' and not p.path


def test_background_result_cannot_restart_an_expired_match():
    p=CompetitionPolicy();p.start_time=0
    p.update(scene(89.9,[(.3,-.06,.01,.01,True)]))
    p.start_planning(89.9,.3,(.3,-.2),.28)
    assert p.step(90)==(0,0,0) and p.phase=='ended'
    assert p.step(90.5)==(0,0,0) and p.phase=='ended'


def test_brush_front_air_gap_is_not_a_solid_body_corner():
    p=CompetitionPolicy()
    block=Object(0,.112,.017,.023)
    assert body_gap((0,0,math.pi/2),block)>.004
    p.update(scene(0,[(0,.112,.017,.023,False)],robot=(0,0,math.pi/2)))
    p.phase='transit'
    assert p.safe((0,.12,0),0)


def test_lane_centers_the_blocks_in_the_physical_brush():
    p=CompetitionPolicy()
    blocks=[(-.429,-.05,.01,.01,True),(-.348,-.05,.01,.01,True),
            (-.44,.05,.01,.01,False),(-.34,.05,.01,.01,False)]
    p.update(scene(0,blocks,robot=(-.43,-.5,math.pi/2)));p.choose(0)
    assert p.lane is not None
    assert all(abs(b[0]-p.lane)+b[2]<.070 for b in blocks[:2])


def test_alignment_moves_continuously_then_brakes_before_pushing():
    p=CompetitionPolicy();p.update(scene(0,robot=(0,-.2,math.pi/2)))
    p.phase='align';p.lane=.10;p.smoother.velocity[1]=-.08
    assert p.raw_command(0)[1]<-.05 and p.phase=='align'
    p.pose=(.099,-.2,math.pi/2);p.smoother.velocity[1]=-.02
    command=p.raw_command(.1)
    assert command[:2]==(0,0) and p.phase=='align'


def test_trapped_robot_can_clear_red_blocks_without_ramming_opponent():
    blocks=[(x,.23,.01,.01,False) for x in [-.229,-.161,-.138]]
    blocks += [(x,.05,.01,.01,False) for x in [-.262,-.060,-.046]]
    blocks += [(.3,-.05,.01,.01,True)]
    p=CompetitionPolicy();p.update(scene(0,blocks,robot=(-.142,.071,math.pi/2)))
    assert p.try_clearance(0) and p.phase=='align' and p.finish_y>.25
    q=CompetitionPolicy();q.update(scene(0,blocks,[(-.18,.18,.09,.09)],robot=(-.142,.071,math.pi/2)))
    assert not q.try_clearance(0)


def crossing_ready():
    p=CompetitionPolicy();p.pose=(0,-.20,math.pi/2)
    for t,x in ((0,-.18),(.1,-.16)):
        p.update(scene(t,[(0,-.05,.01,.01,True)],[(x,.04,.06,.06)],robot=p.pose))
    p.phase='push';p.lane=0;p.finish_y=.28
    return p


def test_crossing_window_preserves_lane_then_resumes_without_a_detour():
    p=crossing_ready()
    assert p.conflict() and p.raw_command(.1)==(0,0,0)
    assert p.phase=='push' and p.lane==0
    p.update(scene(1,[(0,-.05,.01,.01,True)],[(.45,.04,.06,.06)],robot=p.pose))
    assert p.raw_command(1)[0]>0 and p.lane==0 and p.window_wait_started is None


def test_window_wait_is_bounded_and_rejects_incoming_or_rear_threats():
    p=crossing_ready();assert p.wait_for_crossing_window(.1)
    assert p.wait_for_crossing_window(2.7) and p.phase=='yield' and p.lane is None
    p=crossing_ready()
    p.update(scene(.2,[(0,-.05,.01,.01,True)],[(-.16,.02,.06,.06)],robot=p.pose))
    assert not p.wait_for_crossing_window(.2)
    p=crossing_ready();p.blocks.append(Object(0,-.29,.01,.01,vx=0,vy=.2))
    assert not p.wait_for_crossing_window(.1)


def test_lateral_patrol_uses_shorter_push_beyond_its_swept_band():
    p=CompetitionPolicy()
    for t,x in ((0,.25),(.1,.27)):
        p.update(scene(t,[(-.3,-.05,.01,.01,True)],[(x,.04,.06,.06)]))
    assert p.choose(.1) and p.finish_y==.12


def main():
    tests = [(name, value) for name, value in globals().copy().items() if name.startswith('test_')]
    for name, test in tests: test(); print(name, '通过')
    print(f'{len(tests)} 组自适应策略测试通过')

if __name__ == '__main__': main()
