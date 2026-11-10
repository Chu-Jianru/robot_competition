"""整排任务的选择、合法车头接触、任务中断和时效约束。"""
import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sweep_robot.row_policy import HybridCompetitionPolicy


def scene(t=0,robot=(-.42,-.54,math.pi/2),obstacles=(),blocks=None):
    if blocks is None:
        blocks=[(x,y,.01,.01,y<0) for x in [-.42+i*.092 for i in range(10)] for y in (-.05,.05)]
    return dict(stamp=t,valid=True,robot=robot,blocks=blocks,obstacles=obstacles)


def begin():
    p=HybridCompetitionPolicy();p.start_time=0;p.update(scene());assert p.start_rows(0)
    return p


def test_regular_rows_use_two_sweeps_then_north_wall_clear():
    p=begin()
    pushes=[leg for leg in p.legs if leg.mode=='push']
    assert len(pushes)==3
    assert pushes[0].y>0 and pushes[1].y<0
    assert pushes[0].yaw==pushes[1].yaw==0 and pushes[2].yaw==math.pi/2
    assert pushes[0].x>.45 and pushes[1].x>.45


def test_disrupted_rows_and_blocked_corridors_skip_opening():
    for changed in (scene(obstacles=[(.3,.03,.06,.06)]),
                    scene(blocks=[(0,-.3,.01,.01,True)]+scene()['blocks'])):
        p=HybridCompetitionPolicy();p.start_time=0;p.update(changed)
        assert not p.start_rows(0)
        assert not p.row_active
    p=HybridCompetitionPolicy();p.start_time=-.5;p.update(scene(blocks=[(0,-.05,.01,.01,True)]))
    p.step(0)
    assert not p.row_active and p.phase=='transit'


def test_east_sweep_allows_front_but_not_side_contact():
    p=begin();p.phase='row_push';p.row_front_contact=True
    p.update(scene(.1,robot=(0,.07,0),blocks=[(.09,.05,.01,.01,False)]))
    assert p.safe((.2,0,0),.1)
    p.update(scene(.2,robot=(0,.07,0),blocks=[(0,.14,.01,.01,False)]))
    assert not p.safe((0,.15,0),.2)


def test_west_entry_requires_east_heading_and_no_turn_in_recovery():
    p=begin();p.update(scene(.1,robot=(-.55,-.25,0),blocks=[]))
    assert p.safe((0,.1,0),.1)
    p.finish_rows(.1,'通道受阻')
    assert p.row_active and p.row_recovery
    assert all(leg.x>=-.515 for leg in p.legs if leg.mode=='turn')
    assert p.legs[0].yaw==0
    p.row_active=False
    assert not p.safe((0,.1,0),.1)


def test_new_obstacle_stops_sweep_and_releases_to_local_policy():
    p=begin();p.leg_index=4;p.pose=(0,.065,0)
    p.update(scene(.1,robot=p.pose,obstacles=[(.12,.065,.06,.06)]))
    assert p.row_command(.1)==(0,0,0)
    assert p.row_active
    assert p.row_command(1)==(0,0,0)
    assert not p.row_active


def test_visual_outage_resumes_leg_but_never_extends_match():
    p=begin();p.leg_index=4;p.pose=(0,.065,0)
    p.row_last_tick=1;p.row_progress=p.pose;p.row_progress_at=1;p.leg_started=1
    p.update(scene(3.1,robot=p.pose,blocks=[]))
    assert p.step(3.1)[0]>0 and p.row_active
    assert p.step(3.6)==(0,0,0)
    p.update(scene(90,robot=p.pose,blocks=[]))
    assert p.step(90)==(0,0,0) and p.phase=='ended'
    p.update(scene(91,robot=p.pose))
    assert p.step(91)==(0,0,0)


def test_own_row_is_refitted_before_second_sweep():
    p=begin();p.leg_index=5
    p.update(scene(10,robot=(-.55,.07,0),blocks=[(x,-.075,.01,.01,True) for x in (-.4,-.2,0,.2)]))
    p.next_leg(10)
    assert abs(p.legs[6].y+.087)<1e-6 and p.legs[7].y==p.legs[6].y


def test_scattered_second_row_does_not_force_old_route():
    p=begin();p.leg_index=5
    p.update(scene(10,robot=(-.55,.07,0),blocks=[(-.3,-.03,.01,.01,True),(.2,-.135,.01,.01,True)]))
    p.next_leg(10)
    assert p.row_recovery
    assert '队形已散开' in p.reason


def test_connected_train_stops_at_wall_without_compressing_it():
    p=begin();p.leg_index=4
    train=[(x,.065,.02,.01,False) for x in (.32,.37,.42,.47,.52,.57)]
    p.update(scene(10,robot=(.20,.065,0),blocks=train))
    p.row_command(10)
    assert p.leg_index==5
    p=begin();p.leg_index=4
    p.update(scene(10,robot=(.20,.065,0),blocks=[(.57,.065,.02,.01,False)]))
    assert p.row_command(10)[0]>0 and p.leg_index==4


def test_packed_front_pile_can_release_from_first_smoothing_tick():
    p=HybridCompetitionPolicy();p.phase='evade'
    p.update(scene(1,robot=(.512,.181,math.pi/2),blocks=[
        (.513,.288,.083,.019,False),(.419,.265,.012,.012,False)]))
    for speed in (-.00125,-.01,-.05,-.18):
        assert p.safe((speed,0,0),1)
    assert not p.safe((0,.1,0),1)
    assert not p.safe((-.00125,0,.2),1)


def test_west_gap_must_be_reached_before_crossing_between_rows():
    for index in (2,5):
        for x in (-.525,-.547):
            p=begin();p.leg_index=index
            leg=p.legs[index]
            p.update(scene(5,robot=(x,leg.y,0),blocks=[]))
            p.row_progress=p.pose;p.row_progress_at=5
            assert p.row_command(5)[0]<0 and p.leg_index==index


def test_wall_approach_stages_from_current_deepest_block():
    for y in (-.05,-.34):
        p=begin();p.leg_index=8
        p.update(scene(25,blocks=[(.48,y,.01,.01,True)]))
        p.next_leg(25)
        assert abs(p.legs[9].y-(y-.18))<1e-6
        assert p.legs[10].y==p.legs[11].y==p.legs[13].y==p.legs[9].y


def test_retreat_releases_straight_before_lateral_correction():
    p=begin();p.leg_index=13
    p.update(scene(60,robot=(.512,.13,math.pi/2-.01),blocks=[(.513,.23,.083,.019,False)]))
    command=p.row_command(60)
    assert command[0]<0 and command[1:]==(0.,0.)
    for _ in range(4):
        assert p.safe(p.smoother.step(command,.05),60)


def test_train_slows_before_the_wall_and_before_leg_completion():
    p=begin();p.leg_index=4
    p.update(scene(10,robot=(.20,.065,0),blocks=[
        (x,.065,.02,.01,False) for x in (.32,.37,.42,.47,.50)]))
    command=p.row_command(10)
    assert 0<command[0]<.10 and p.leg_index==4


def test_second_sweep_keeps_guide_clear_of_the_compacted_opponent_row():
    p=begin();p.leg_index=5
    own=[(x,-.05,.01,.01,True) for x in (-.4,-.2,0,.2)]
    p.update(scene(15,robot=(-.55,.07,0),blocks=own+[(.47,.05,.10,.015,False)]))
    p.next_leg(15)
    assert .05-.015-p.legs[6].y>=.105-1e-9
    assert all(abs(y-p.legs[6].y)+hy<=.062 for x,y,hx,hy,own in own)


def test_measured_west_corner_gap_remains_clear_through_both_rows():
    p=begin()
    blocks=[(-.4436,.046,.0092,.0069,False),(-.416,-.0506,.0092,.0069,True)]
    p.update(scene(1,robot=(-.552,-.035,0),blocks=blocks))
    assert p.safe((0,.30,0),1)
    p.pose=(-.548,-.035,0)
    assert not p.safe((0,.30,0),1)


def test_opening_observes_opponent_motion_before_committing():
    p=HybridCompetitionPolicy();p.start_time=0
    for i in range(5):
        t=i*.1
        p.update(scene(t,obstacles=[(.4-.1*t,.34-.2*t,.06,.06)]))
        p.step(t)
        assert not p.row_active
    assert p.opening_checked and any(abs(o.vy)>.04 for o in p.obstacles)
    p=HybridCompetitionPolicy();p.start_time=0
    for i in range(5):
        t=i*.1;p.update(scene(t));p.step(t)
        if i<4:assert not p.row_active
    assert p.row_active


def test_future_row_is_rechecked_before_entering_the_west_gap():
    p=begin()
    p.update(scene(.1,robot=(-.45,-.35,math.pi/2),obstacles=[(.3,.07,.06,.06)]))
    assert not p.conflict()
    assert p.row_command(.1)==(0,0,0) and p.row_active
    p.row_command(.4)
    assert not p.row_active and '提前切换' in p.reason


def test_repeated_blockage_keeps_west_recovery_until_safe_exit():
    p=begin();p.update(scene(.1,robot=(-.552,.06,0)))
    p.finish_rows(.1,'首次受阻');p.finish_rows(1,'退出时再次受阻')
    assert p.row_active and p.row_recovery
    assert all(leg.x>=-.515 for leg in p.legs if leg.mode=='turn')


if __name__=='__main__':
    tests=[(name,fn) for name,fn in list(globals().items()) if name.startswith('test_') and callable(fn)]
    for name,test in tests:test();print(name,'通过')
    print(len(tests),'组整排策略测试通过')
