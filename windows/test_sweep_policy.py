"""Offline route selection and front-push execution for the Windows task."""

import math
import unittest

from run_task import SweepPolicy, can_start
from sweep_policy import NORTH, ZERO, Leg, Route, cluster_priority, clusters, local, world, wrap
from sweep_robot.competition_policy import Object


def scene(pose, blocks, stamp=0., obstacles=(), seam_bias=0.):
    return dict(valid=True, robot=pose, stamp=stamp, seam_bias=seam_bias,
                blocks=[(x, y, .01, .01, y < 0.) for x, y in blocks], obstacles=obstacles)


class SynchronousSweep(SweepPolicy):
    def start_search(self, now):
        self.job_started = now
        self.planning_result = (self.plan(now),)


class SweepPolicyTest(unittest.TestCase):
    def test_start_accepts_either_half_and_any_heading_inside_survey(self):
        for pose in ((-.50, -.05, -NORTH), (.25, .24, -1.), (.02, 0., math.pi)):
            self.assertTrue(can_start(scene(pose, [])))
        self.assertFalse(can_start(scene((0., .59, 0.), [], seam_bias=.02)))

    def test_compact_group_outweighs_equal_count_spread_group(self):
        tight = [Object(-.26, -.15), Object(-.24, -.16), Object(-.22, -.17)]
        loose = [Object(.19, -.15), Object(.24, -.18), Object(.29, -.21)]
        self.assertEqual(clusters(tight + loose), [tight, loose])
        self.assertGreater(cluster_priority(tight), cluster_priority(loose))

    def test_dense_group_has_priority_until_its_push_corridor_is_blocked(self):
        p = SweepPolicy()
        p.stage = 'clusters'
        blocks = [(-.25, -.12), (-.23, -.155), (-.205, -.18), (.21, -.15)]
        p.update(scene((.32, -.23, 0.), blocks))
        route = p.plan(0.)
        self.assertEqual(set(route.focus), set(blocks[:3]))
        self.assertGreater(next(leg for leg in route.legs if leg.mode == 'push').point[1], .15)
        p.update(scene((.32, -.23, 0.), blocks, 1., [(-.22, .15, .07, .08)]))
        alternative = p.plan(1.)
        self.assertEqual(alternative.focus, (blocks[-1],))

    def test_plans_a_push_when_the_only_corridor_contains_an_obstacle(self):
        p = SweepPolicy()
        p.stage = 'clusters'
        p.update(scene((.32, -.23, 0.), [(-.23, -.15)], obstacles=[(-.23, .10, .07, .08)]))
        self.assertIsNone(p.plan_route(0., [p.pending()], p.forecast_obstacles(), math.inf))
        route = p.plan(0.)
        self.assertEqual(route.focus, ((-.23, -.15),))

    def test_new_obstacle_allows_turn_and_motion_but_wall_still_limits_motion(self):
        p = SweepPolicy()
        p.update(scene((0., -.22, 0.), [], obstacles=[(.07, -.22, .06, .07)]))
        p.route = Route((Leg(p.pose[:2], NORTH, 'turn'),), (), 0, (), 1.)
        p.progress_pose, p.progress_time = p.pose, 0.
        self.assertGreater(p.step(0.)[2], 0.)
        self.assertTrue(p.safe((.2, 0., 0.), 0.))
        p.update(scene((.505, -.22, 0.), [], stamp=.1))
        self.assertFalse(p.safe((.2, 0., 0.), .1))

    def test_turn_near_wall_or_red_block_is_rejected(self):
        p = SweepPolicy()
        p.update(scene((.50, -.25, NORTH), [(.05, -.14)]))
        self.assertFalse(p.turn_clear(p.pose[:2], NORTH, math.pi, []))
        self.assertFalse(p.turn_clear((.03, -.24), NORTH, 0., []))
        self.assertTrue(p.turn_clear((-.20, -.32), NORTH, 0., []))

    def test_stale_pose_and_match_timeout_stop_an_active_route(self):
        p = SynchronousSweep(duration=1.)
        data = scene((.1, -.30, 0.), [(-.3, -.14), (-.22, -.14)])
        p.update(data)
        p.step(0.)
        p.step(.05)
        self.assertIsNotNone(p.route)
        self.assertNotEqual(p.step(.1), ZERO)
        self.assertEqual(p.step(.36), ZERO)
        p.update({**data, 'stamp': 1.})
        self.assertEqual(p.step(1.), ZERO)
        self.assertEqual(p.phase, 'ended')

    def test_front_push_stages_from_other_half_with_scattered_blocks(self):
        p = SynchronousSweep()
        pose = (.25, .24, -1.)
        blocks = [(-.30, -.14), (-.23, -.14), (-.15, -.14), (-.09, -.14),
                  (.21, -.28), (.235, -.30), (.218, -.27)]
        pushes = []
        dt = .05
        for tick in range(900):
            now = tick * dt
            p.update(scene(pose, blocks, now))
            vx, vy, wz = p.step(now)
            dx, dy = world(vx * dt, vy * dt, pose[2])
            if p.row_front_contact and vx > 0:
                if not pushes or p.phase != pushes[-1]:
                    pushes.append(p.phase)
                if p.stage == 'wall':
                    self.assertGreater(abs(pose[0]), .46)
                # Ideal front contact model. Move only the train touching the brush.
                projected = [(*local(x - pose[0], y - pose[1], pose[2]), i)
                             for i, (x, y) in enumerate(blocks)]
                reach = .09 + vx * dt
                for forward, _, i in sorted(v for v in projected if v[0] >= 0. and abs(v[1]) < .068):
                    if forward > reach:
                        break
                    blocks[i] = (blocks[i][0] + dx, blocks[i][1] + dy)
                    reach = max(reach, forward + .024)
            pose = (pose[0] + dx, pose[1] + dy, wrap(pose[2] + wz * dt))
            if all(y > .04 for _, y in blocks):
                break
        self.assertEqual(pushes, ['opening_push', 'wall_push', 'clusters_push'])
        self.assertTrue(all(y > .04 for _, y in blocks), blocks)


if __name__ == '__main__':
    unittest.main()
