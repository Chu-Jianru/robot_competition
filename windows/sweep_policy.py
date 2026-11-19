"""Windows 实车的首轮沿墙清扫和红块集群清理。坐标以己方为 y < 0。"""

from copy import copy
from dataclasses import dataclass, replace
import math
import threading
import time

from sweep_robot.competition_policy import CompetitionPolicy, NORTH, body_gap, clamp, gap, plan_path, wrap


# Fiducial-relative outline in (forward, left) metres, also used by vision.
OUTLINE = ((.058, -.053), (.058, .058), (-.012, .058), (-.012, .103),
           (-.155, .103), (-.155, -.077), (-.012, -.077), (-.012, -.053))
WALL_LANES = {-1: -.487, 1: .505}
BRUSH_HALF = .067
ZERO = (0., 0., 0.)


def local(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x + s * y, -s * x + c * y


def world(forward, left, yaw):
    return local(forward, left, -yaw)


def weight(block):
    return clamp(block.hx * block.hy / .0001, 1., 6.)


def clusters(blocks):
    """Group connected red regions by the gap between their bounding boxes."""
    remaining = list(blocks)
    groups = []
    while remaining:
        group = [remaining.pop(0)]
        for a in group:
            for b in remaining[:]:
                dx = max(0., abs(a.x - b.x) - a.hx - b.hx)
                dy = max(0., abs(a.y - b.y) - a.hy - b.hy)
                if math.hypot(dx, dy) <= .055:
                    group.append(b)
                    remaining.remove(b)
        groups.append(group)
    return groups


def cluster_priority(group):
    left = min(b.x - b.hx for b in group)
    right = max(b.x + b.hx for b in group)
    south = min(b.y - b.hy for b in group)
    north = max(b.y + b.hy for b in group)
    count = sum(map(weight, group))
    footprint = max(.04, right - left) * max(.04, north - south)
    return count * (1 + min(1., .0025 * count / footprint))


def footprint_gap(pose, block):
    x, y, yaw = pose
    forward, left = local(block.x - x, block.y - y, yaw)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    hf = c * block.hx + s * block.hy
    hl = s * block.hx + c * block.hy
    # Also check the real rear chassis, which extends behind the legacy brush model.
    rear = max(-.155 - forward - hf, forward + .012 - hf,
               -.077 - left - hl, left - .103 - hl)
    return min(rear, body_gap(pose, block))


@dataclass(frozen=True)
class Leg:
    point: tuple[float, float]
    yaw: float
    mode: str


@dataclass(frozen=True)
class Route:
    legs: tuple[Leg, ...]
    key: tuple
    wall: int
    focus: tuple[tuple[float, float], ...]
    count: float


class SweepPolicy(CompetitionPolicy):
    def __init__(self, duration=90.):
        super().__init__(duration)
        self.stage = 'opening'
        self.route = None
        self.leg_index = 0
        self.wall = 0
        self.focus = ()
        self.seam_bias = 0.
        self.after_scene = -math.inf
        self.retry_at = -math.inf
        self.job_started = None
        self.row_front_contact = False

    def update(self, scene):
        super().update(scene)
        if self.valid and scene['stamp'] == self.scene_time:
            self.seam_bias = scene.get('seam_bias', 0.)

    def pending(self):
        return [block for block in self.blocks if block.own]

    def wall_violation(self, pose):
        x, y, yaw = pose
        limit = .594
        corners = (world(forward, left, yaw) for forward, left in OUTLINE)
        return max(0., abs(x) - .515, -.548 - y, y - .505,
                   *(max(abs(x + dx), abs(y + self.seam_bias + dy)) - limit for dx, dy in corners))

    def occupied_gap(self, pose, obstacles):
        return min((gap(pose[:2], o, .105, .105) for o in obstacles), default=1.)

    def transit_clear(self, start, end, yaw, obstacles, *, allow_exit=False):
        first = (*start, yaw)
        wall = self.wall_violation(first)
        old_gaps = [footprint_gap(first, block) for block in self.blocks]
        old_obstacle = self.occupied_gap(first, obstacles)
        steps = max(1, math.ceil(math.dist(start, end) / .035))
        for i in range(1, steps + 1):
            t = i / steps
            pose = (start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]), yaw)
            margin = self.wall_violation(pose)
            if margin > 0 and (not allow_exit or margin >= wall):
                return False
            if self.occupied_gap(pose, obstacles) < .006 and (
                    not allow_exit or self.occupied_gap(pose, obstacles) <= old_obstacle):
                return False
            for block, initial in zip(self.blocks, old_gaps):
                clearance = footprint_gap(pose, block)
                if clearance < .004 and (not allow_exit or clearance <= initial + 1e-6):
                    return False
        return True

    def turn_clear(self, point, old_yaw, yaw, obstacles):
        error = wrap(yaw - old_yaw)
        if abs(error) < .045:
            return True
        if self.occupied_gap((*point, yaw), obstacles) < .015:
            return False
        steps = math.ceil(abs(error) / .16)
        return all(self.wall_violation((*point, old_yaw + error * i / steps)) == 0 for i in range(steps + 1)) and all(
            math.dist(point, (b.x, b.y)) > .165 + max(b.hx, b.hy) for b in self.blocks)

    def approach_legs(self, point, yaw, obstacles, deadline):
        x, y, old_yaw = self.pose
        start = (x, y)
        stages = [start, (clamp(x, -.40, .40), clamp(y, -.40, .40)), point]
        stages.extend((x + dx, y + dy) for distance in (.14, .23)
                      for dx, dy in ((distance, 0.), (-distance, 0.), (0., distance), (0., -distance)))
        stages = sorted(set(stages), key=lambda p: (math.dist(start, p) + math.dist(p, point), p != start))
        eligible = None
        for stage in stages:
            if time.monotonic() >= deadline:
                break
            if not self.turn_clear(stage, old_yaw, yaw, obstacles):
                continue
            if stage != start and not self.transit_clear(start, stage, old_yaw, obstacles, allow_exit=True):
                continue
            entry = [Leg(stage, old_yaw, 'transit')] if stage != start else []
            if abs(wrap(yaw - old_yaw)) >= .045:
                entry.append(Leg(stage, yaw, 'turn'))
            if eligible is None:
                eligible = stage, entry
            approaches = [(), ((stage[0], point[1]),), ((point[0], stage[1]),)]
            for middle in approaches:
                points = (stage, *middle, point)
                legs = self.path_legs(points, yaw, obstacles, start)
                if legs is not None:
                    return entry + legs
        budget = min(.022, deadline - time.monotonic())
        if eligible is not None and budget > 0:
            stage, entry = eligible
            path = plan_path(stage, point, self.blocks + obstacles, budget=budget)
            if path:
                legs = self.path_legs((stage, *path), yaw, obstacles, start)
                if legs is not None:
                    return entry + legs
        return None

    def path_legs(self, points, yaw, obstacles, start):
        legs = []
        for a, b in zip(points, points[1:]):
            if a != b:
                if not self.transit_clear(a, b, yaw, obstacles, allow_exit=a == start):
                    return None
                legs.append(Leg(b, yaw, 'transit'))
        return legs

    def push_candidates(self, group, yaw, now, *, wall=0):
        projected = []
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        for block in group:
            forward, left = local(block.x, block.y, yaw)
            projected.append((block, forward, left, c * block.hx + s * block.hy,
                              s * block.hx + c * block.hy))
        lateral = [entry[2] for entry in projected]
        lanes = {round(clamp(left + offset, -.505, .487), 3) for left in lateral
                 for offset in (0., -.032, .032)}
        lanes.update(round((a + b) / 2, 3) for a in lateral for b in lateral
                     if abs(a - b) < .125 and -.505 <= (a + b) / 2 <= .487)
        if wall and yaw == NORTH:
            lanes = {-WALL_LANES[wall], -WALL_LANES[wall] + wall * .025}
        for lane in lanes:
            if yaw != NORTH and world(0., lane, yaw)[1] > -.025:
                continue
            finish = .26 if yaw == NORTH else abs(WALL_LANES[wall]) - .036
            covered = [(b, forward - hf, overlap) for b, forward, left, hf, hl in projected
                       if forward <= finish + .075 and (overlap :=
                           (min(lane + BRUSH_HALF, left + hl) - max(lane - BRUSH_HALF, left - hl))
                           / max(.005, 2 * hl)) >= .40]
            if not covered:
                continue
            start = min(edge for _, edge, _ in covered) - .112
            if yaw == NORTH:
                start = min(start, -.135)
            entry, exit_point = world(start, lane, yaw), world(finish, lane, yaw)
            if start + .08 > finish or self.wall_violation((*entry, yaw)) or self.wall_violation((*exit_point, yaw)):
                continue
            benefit = sum(weight(block) * overlap ** 2 for block, _, overlap in covered)
            key = (self.stage, wall, round(yaw, 2), round(lane / .065))
            if now - self.failures.get(key, -math.inf) < 3.:
                continue
            cost = .5 * math.dist(self.pose[:2], entry) + .15 * (finish - start)
            score = benefit if self.stage == 'opening' else benefit / (1 + cost)
            yield (round(score, 3), -cost), benefit, key, entry, exit_point, yaw, wall

    def push_clear(self, start, end, yaw, obstacles):
        length, _ = local(end[0] - start[0], end[1] - start[1], yaw)
        steps = max(1, math.ceil(length / .037))
        side_blocks = []
        for block in self.blocks:
            forward, left = local(block.x - start[0], block.y - start[1], yaw)
            if not (forward >= .06 and abs(left) <= BRUSH_HALF + .005):
                side_blocks.append(block)
        for i in range(steps + 1):
            dx, dy = world(length * i / steps, 0., yaw)
            pose = (start[0] + dx, start[1] + dy, yaw)
            if self.occupied_gap(pose, obstacles) < .012:
                return False
            if any(footprint_gap(pose, block) < .004 for block in side_blocks):
                return False
        return True

    def plan(self, now):
        pending = self.pending()
        if not pending:
            return None
        started = time.monotonic()
        obstacles = self.forecast_obstacles()
        focus_groups = [pending]
        if self.stage == 'clusters':
            focus_groups = sorted(clusters(pending), key=lambda group: (
                any(math.dist((b.x, b.y), seed) < .085 for b in group for seed in self.focus),
                cluster_priority(group), -min(math.dist(self.pose[:2], (b.x, b.y)) for b in group)), reverse=True)
        if obstacles:
            route = self.plan_route(now, focus_groups, obstacles, started + .14)
            if route is not None:
                return route
        return self.plan_route(now, focus_groups, [], started + .28)

    def plan_route(self, now, focus_groups, obstacles, deadline):
        for group in focus_groups:
            if self.stage == 'opening':
                headings = ((0., 1), (math.pi, -1))
            elif self.stage == 'wall':
                headings = ((NORTH, self.wall),) if self.wall else ((NORTH, -1), (NORTH, 1))
            else:
                headings = ((NORTH, 0),)
            options = [entry for yaw, wall in headings for entry in self.push_candidates(group, yaw, now, wall=wall)]
            for _, benefit, key, start, end, yaw, wall in sorted(options, reverse=True):
                if time.monotonic() >= deadline:
                    return None
                if not self.push_clear(start, end, yaw, obstacles):
                    continue
                legs = self.approach_legs(start, yaw, obstacles, deadline)
                if legs is None:
                    continue
                legs.append(Leg(end, yaw, 'push'))
                dx, dy = world(-.15, 0., yaw)
                back = (end[0] + dx, end[1] + dy)
                if not self.wall_violation((*back, yaw)):
                    legs.append(Leg(back, yaw, 'retreat'))
                return Route(tuple(legs), key, wall, tuple((b.x, b.y) for b in group), benefit)
        return None

    def start_search(self, now):
        snapshot = copy(self)
        snapshot.failures = self.failures.copy()
        self.planning_token += 1
        token = self.planning_token
        self.planning_result = None
        self.job_started = now
        self.phase = self.stage + '_planning'

        def search():
            route = snapshot.plan(now)
            if self.planning_token == token:
                self.planning_result = (route,)

        threading.Thread(target=search, daemon=True).start()

    def finish(self, now, reason, *, failed=False):
        if self.route is not None:
            if failed:
                self.failures[self.route.key] = now
            elif self.stage == 'opening':
                self.wall = self.route.wall
            elif self.stage == 'clusters':
                self.focus = self.route.focus
        if self.stage == 'opening':
            self.stage = 'wall'
        elif self.stage == 'wall':
            self.stage = 'clusters'
        self.route = None
        self.phase = 'observe'
        self.after_scene = self.scene_time
        self.retry_at = now + .18
        self.blocked_since = None
        self.smoother.reset()
        self.event(now, reason)

    def abort(self, now, reason):
        self.finish(now, reason, failed=True)

    def safe(self, cmd, now, escape_push=False):
        if not super().safe(cmd, now, escape_push=escape_push, check_obstacles=False):
            return False
        if not any(cmd):
            return True
        x, y, yaw = self.pose
        vx, vy, wz = cmd
        dx, dy = world(vx, vy, yaw)
        old_wall = self.wall_violation(self.pose)
        pushing = self.row_front_contact and vx > 1e-5 and abs(vy) <= max(.003, .4 * vx) and abs(wz) < .25
        for t in (.08, .16, .25):
            pose = (x + dx * t, y + dy * t, yaw + wz * t)
            margin = self.wall_violation(pose)
            if margin > 0 and (margin >= old_wall or abs(wz) > .12):
                return False
            for block in self.blocks:
                forward, left = local(block.x - x, block.y - y, yaw)
                if pushing and forward > -.035 and abs(left) <= .078:
                    continue
                predicted = copy(block)
                predicted.x += block.vx * t
                predicted.y += block.vy * t
                clearance = footprint_gap(pose, predicted)
                if clearance < .003 and clearance < footprint_gap(self.pose, block) + 1e-6:
                    return False
        return True

    def pile_clearance(self, leg):
        """Distance to the wall for the train of blocks connected to the brush."""
        front, lane = local(*self.pose[:2], leg.yaw)
        intervals = []
        for block in self.blocks:
            forward, left = local(block.x, block.y, leg.yaw)
            if abs(lane - left) < BRUSH_HALF and forward >= front:
                intervals.append((forward - block.hx, forward + block.hx))
        reach = front + .075
        for low, high in sorted(intervals):
            if low > reach + .021:
                break
            reach = max(reach, high)
        # Keep the leading edge within the northbound brush's reach at this wall.
        return abs(WALL_LANES[self.route.wall]) + BRUSH_HALF - .008 - reach

    def retreat_from_contact(self, leg):
        if self.leg_index + 1 == len(self.route.legs):
            return
        next_leg = self.route.legs[self.leg_index + 1]
        if next_leg.mode != 'retreat':
            return
        dx, dy = world(-.15, 0., leg.yaw)
        goal = (self.pose[0] + dx, self.pose[1] + dy)
        if not self.wall_violation((*goal, leg.yaw)):
            legs = (*self.route.legs[:self.leg_index + 1], replace(next_leg, point=goal))
            self.route = replace(self.route, legs=legs)

    def raw_command(self, now):
        self.row_front_contact = False
        if self.route is None:
            if self.job_started is not None:
                if self.planning_result is None and now - self.job_started < 1.0:
                    return ZERO
                self.planning_token += 1
                self.job_started = None
                self.route = self.planning_result[0] if self.planning_result else None
                if self.route is None:
                    self.finish(now, '当前无可达推送路线，刷新分布后再选')
                    return ZERO
                self.leg_index = 0
                self.progress_pose = self.pose
                self.progress_time = now
                self.event(now, f'{self.stage} 选定路线，预计覆盖 {self.route.count:.1f} 块')
            elif now >= self.retry_at and self.scene_time > self.after_scene:
                if self.pending():
                    self.start_search(now)
                else:
                    self.phase = 'guard'
            return ZERO

        leg = self.route.legs[self.leg_index]
        self.phase = self.stage + '_' + leg.mode
        error = wrap(leg.yaw - self.pose[2])
        delta = (leg.point[0] - self.pose[0], leg.point[1] - self.pose[1])
        distance = math.hypot(*delta)
        forward, left = local(*delta, leg.yaw)
        wall_space = self.pile_clearance(leg) if self.stage == 'opening' and leg.mode == 'push' else math.inf
        if leg.mode == 'turn':
            complete = abs(error) < .055
        elif leg.mode == 'push':
            complete = forward < .011 or wall_space <= .008
        else:
            complete = distance < .012
        if complete:
            if leg.mode == 'push':
                self.retreat_from_contact(leg)
            self.smoother.reset()
            self.leg_index += 1
            self.progress_pose, self.progress_time = self.pose, now
            if self.leg_index == len(self.route.legs):
                self.finish(now, '推送结束，按新画面复查剩余红块')
            return ZERO
        if (math.dist(self.pose[:2], self.progress_pose[:2]) > .025 or
                abs(wrap(self.pose[2] - self.progress_pose[2])) > .09):
            self.progress_pose, self.progress_time = self.pose, now
        elif now - self.progress_time > 2.6:
            self.abort(now, '路线进度停滞，停车并重新选择')
            return ZERO
        if leg.mode == 'turn':
            if not self.turn_clear(self.pose[:2], self.pose[2], leg.yaw, ()):
                self.abort(now, '转向净空不足，停车并重新选择')
                return ZERO
            return (0., 0., clamp(2. * error, -.70, .70))
        if leg.mode == 'push':
            if abs(error) > .17:
                self.abort(now, '推送航向偏离，停车并重新选择')
                return ZERO
            self.row_front_contact = True
            speed = clamp(2. * min(forward, wall_space), .035, .22)
            return (speed, clamp(left, -.014, .014), clamp(1.6 * error, -.20, .20))
        speed = min(.24, 2. * distance)
        vx, vy = local(*delta, self.pose[2])
        return (speed * vx / distance, speed * vy / distance, clamp(1.5 * error, -.18, .18))
