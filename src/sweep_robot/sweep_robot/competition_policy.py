"""视觉闭环的比赛策略。纯 Python，不依赖 ROS、Gazebo 或里程计。"""

import heapq
import math
import time
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

Point = Tuple[float, float]
Pose = Tuple[float, float, float]
Command = Tuple[float, float, float]
NORTH = math.pi / 2


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def clamp(value, low, high):
    return max(low, min(high, value))


def brush_coverage(x,block):
    overlap=max(0,min(x+.067,block.x+block.hx)-max(x-.067,block.x-block.hx))
    return min(1.0,overlap/max(.005,2*block.hx))


@dataclass
class Object:
    x: float
    y: float
    hx: float = 0.01
    hy: float = 0.01
    own: bool = False
    stamp: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    kind: str = 'block'
    still_for: float = 0.0


def gap(point: Point, obj: Object, margin_x=None, margin_y=None):
    """点到膨胀矩形的有符号距离，负数为侵入。"""
    if margin_x is None and margin_y is None:
        if obj.kind!='obstacle':return body_gap((point[0],point[1],NORTH),obj)
        margin_x=margin_y=.105
    if margin_x is None:margin_x=.089
    if margin_y is None:margin_y=.085 if obj.y>=point[1] else .053
    dx = abs(point[0] - obj.x) - obj.hx - margin_x
    dy = abs(point[1] - obj.y) - obj.hy - margin_y
    return math.hypot(max(0, dx), max(0, dy)) if max(dx, dy) > 0 else max(dx, dy)


def body_gap(pose: Pose, obj: Object):
    """底盘、滚刷和两块斜导板的投影并集，不把刷前空隙当实心车身。"""
    x,y,yaw=pose
    c,s=math.cos(yaw),math.sin(yaw)
    fx=c*(obj.x-x)+s*(obj.y-y)
    fy=-s*(obj.x-x)+c*(obj.y-y)
    hx=abs(c)*obj.hx+abs(s)*obj.hy
    hy=abs(s)*obj.hx+abs(c)*obj.hy
    def rectangle(px,py,cx,cy,sx,sy,ex,ey):
        dx=abs(px-cx)-sx-ex;dy=abs(py-cy)-sy-ey
        return math.hypot(max(0,dx),max(0,dy)) if max(dx,dy)>0 else max(dx,dy)
    # 约5mm皮肤，覆盖标定误差和执行迟滞；红块外接尺寸另算。
    values=[rectangle(fx,fy,.004,0,.057,.059,hx,hy),
            rectangle(fx,fy,.064,0,.017,.075,hx,hy)]
    for side in (-1,1):
        angle=side*.42;ca,sa=math.cos(angle),math.sin(angle)
        dx,dy=fx-.050,fy-side*.062
        local=rectangle(ca*dx+sa*dy,-sa*dx+ca*dy,0,0,.045,.008,
                        abs(ca)*hx+abs(sa)*hy,abs(sa)*hx+abs(ca)*hy)
        # 两个导板轴和两个块外接框轴都须重叠才算相交。
        # 只检查导板轴会把西侧实际存在的空隙误报成导板端部碰撞。
        axis_x=abs(dx)-(abs(ca)*.045+abs(sa)*.008)-hx
        axis_y=abs(dy)-(abs(sa)*.045+abs(ca)*.008)-hy
        values.append(max(local,axis_x,axis_y))
    return min(values)


def segment_clear(a: Point, b: Point, objects: List[Object], escaping=False):
    distance = math.dist(a, b)
    count = max(1, int(math.ceil(distance / 0.018)))
    initial = [gap(a, obj) for obj in objects]
    for i in range(1, count + 1):
        t = i / count
        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        for obj, old_gap in zip(objects, initial):
            value = gap(p, obj)
            if value < 0.003:
                improving = value > old_gap + 0.001 * t
                # 视觉和外形保守膨胀造成的毫米级重叠，允许平行退出，禁止加深。
                nose_only=obj.y-a[1]-obj.hy>=.05
                shallow_exit = nose_only and old_gap > -.005 and value >= old_gap - 1e-6
                if not (escaping and old_gap < 0.003 and (improving or shallow_exit)):
                    return False
    return True


def corridor_centers(objects):
    """将真实空隙中心加入搜索，避免粗网格错过可通行的窄车道。"""
    xs=[];ys=[]
    for i,a in enumerate(objects):
        for b in objects[i+1:]:
            left,right=sorted((a,b),key=lambda o:o.x)
            width=(right.x-right.hx)-(left.x+left.hx)
            if abs(a.y-b.y)<.14 and .18<width<.65:
                xs.append(((left.x+left.hx)+(right.x-right.hx))/2)
            south,north=sorted((a,b),key=lambda o:o.y)
            height=(north.y-north.hy)-(south.y+south.hy)
            if abs(a.x-b.x)<.14 and .16<height<.65:
                ys.append(((south.y+south.hy)+(north.y-north.hy))/2-.020)
    return xs,ys


def plan_path(start: Point, goal: Point, objects: List[Object], budget=0.025) -> List[Point]:
    """小场地 A* 加直线可见性简化；不允许穿过块带或障碍。"""
    deadline = None if budget is None else time.monotonic()+budget
    if any(gap(goal,obj)<.003 for obj in objects): return []
    if segment_clear(start, goal, objects, escaping=True):
        return [goal]
    portal_xs,portal_ys=corridor_centers(objects)
    # 刚完成北推的这条道已被实测清空，优先沿它退回南侧空区。
    # 仍逐段检查净空；对手回推堵住时才改用全局搜索。
    if start[1]>-.12 and goal[1]<-.12:
        stage = (start[0],min(-.22,goal[1]))
        if all(gap(stage,o)>=.003 for o in objects) and segment_clear(start,stage,objects,escaping=True) and segment_clear(stage,goal,objects):
            return [stage,goal]
        # 斜导板两侧的块可能只封住直退中的一个角；比较小幅斜退后再南退，
        # 终点必须完全脱离接触，每一段仍按同一车身轮廓检查。
        for off in (-.005,.005,-.01,.01,-.015,.015):
            release=(clamp(start[0]+off,-.515,.515),max(stage[1],start[1]-.10))
            south=(release[0],stage[1])
            if all(gap(release,o)>=.003 for o in objects) and segment_clear(start,release,objects,escaping=True) and segment_clear(release,south,objects) and segment_clear(south,goal,objects):
                return [release,south,goal]
        release = (start[0],max(stage[1],start[1]-.10))
        # 后方有散块时，也比较先侧向离堆再南退的可见通道。
        exits=[clamp(start[0]+off,-.515,.515) for off in (-.03,.03,-.06,.06,-.09,.09)]
        exits += [-.48,-.42,-.35,-.25,0,.25,.35,.42,.48]+[x for x in portal_xs if -.515<x<.515]
        for sx in sorted(set(exits),key=lambda value:abs(value-start[0])):
            if deadline is not None and time.monotonic()>deadline:return []
            side=(sx,start[1]);south=(sx,stage[1])
            if all(gap(side,o)>=.003 for o in objects) and segment_clear(start,side,objects,escaping=True) and segment_clear(side,south,objects) and segment_clear(south,goal,objects):
                return [side,south,goal]
        if segment_clear(start,release,objects,escaping=True):
            exits = [clamp(start[0]+off,-.515,.515) for off in (-.03,.03,-.06,.06,-.09,.09)]
            exits += [-.48,-.42,-.35,-.25,0,.25,.35,.42,.48]
            for sx in sorted(set(exits),key=lambda value:abs(value-start[0])):
                side = (sx,release[1]); south = (sx,stage[1])
                if all(segment_clear(a,b,objects) for a,b in [(release,side),(side,south),(south,goal)]):
                    return [release,side,south,goal]
    step = 0.04
    xs = [-0.515 + step * i for i in range(27)]
    ys = [-0.53 + step * i for i in range(27)]
    xs[-1] = 0.515
    # 保留当前坐标轴，窄出口可先直退；不能让4cm网格强迫车斜着穿块。
    xs = sorted(set(xs + [clamp(start[0],-.515,.515),clamp(goal[0],-.515,.515)]))
    ys = sorted(set(ys + [clamp(start[1],-.53,.51),clamp(goal[1],-.53,.51)]))
    xs=sorted(set(xs+[x for x in sorted(set(portal_xs),key=lambda x:min(abs(x-start[0]),abs(x-goal[0])))[:16] if -.515<x<.515]))
    ys=sorted(set(ys+[y for y in sorted(set(portal_ys),key=lambda y:min(abs(y-start[1]),abs(y-goal[1])))[:16] if -.53<y<.51]))
    def nearest(p):
        return min(range(len(xs)), key=lambda i: abs(xs[i] - p[0])), min(range(len(ys)), key=lambda i: abs(ys[i] - p[1]))
    def point(key): return xs[key[0]], ys[key[1]]
    root = (-1, -1)
    frontier = [(math.dist(start, goal), 0.0, root)]
    cost = {root: 0.0}
    parent = {}
    closed = set()
    while frontier:
        if deadline is not None and time.monotonic()>deadline: return []
        _, score, key = heapq.heappop(frontier)
        if key in closed:
            continue
        closed.add(key)
        here = start if key == root else point(key)
        if math.dist(here, goal) < 0.10 and segment_clear(here, goal, objects, escaping=key == root):
            path = [goal]
            while key != root:
                path.append(point(key)); key = parent[key]
            path.reverse()
            # 只去掉有净空的中间点，不用切进障碍的圆角样条。
            simple = []
            anchor = start
            while path:
                farthest = 0
                for i, p in enumerate(path):
                    if segment_clear(anchor, p, objects, escaping=not simple): farthest = i
                anchor = path[farthest]
                simple.append(anchor)
                path = path[farthest + 1:]
            return simple
        center = nearest(start) if key == root else key
        reach = 3 if key == root else 1
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                nxt = center[0] + dx, center[1] + dy
                if nxt == key or not (0 <= nxt[0] < len(xs) and 0 <= nxt[1] < len(ys)):
                    continue
                p = point(nxt)
                if any(gap(p,o)<.003 for o in objects): continue
                if not segment_clear(here, p, objects, escaping=key == root):
                    continue
                proximity = min((gap(p, o) for o in objects), default=1.0)
                new_cost = score + math.dist(here, p) * (1 + 0.08 / max(0.025, proximity))
                if new_cost < cost.get(nxt, float('inf')):
                    cost[nxt] = new_cost; parent[nxt] = key
                    heapq.heappush(frontier, (new_cost + math.dist(p, goal), new_cost, nxt))
    return []


class SmoothVelocity:
    """限加速度和加加速度。紧急停车由独立安全层覆盖。"""
    def __init__(self):
        self.velocity = [0.0] * 3
        self.acceleration = [0.0] * 3

    def reset(self):
        self.velocity = [0.0] * 3
        self.acceleration = [0.0] * 3

    def step(self, target: Command, dt: float):
        for i in range(3):
            maximum = 1.5 if i == 2 else 0.75
            jerk = 5.0 if i == 2 else 3.5
            desired = clamp((target[i] - self.velocity[i]) / dt, -maximum, maximum)
            acceleration = self.acceleration[i] + clamp(desired - self.acceleration[i], -jerk * dt, jerk * dt)
            proposed = self.velocity[i] + acceleration * dt
            if (target[i] - self.velocity[i]) * (target[i] - proposed) <= 0:
                proposed = target[i]; acceleration = 0.0
            self.velocity[i] = proposed; self.acceleration[i] = acceleration
        return tuple(self.velocity)


class CompetitionPolicy:
    def __init__(self, duration=90.0):
        self.duration = duration
        self.start_time = None
        self.scene_time = -float('inf')
        self.valid = False
        self.pose: Optional[Pose] = None
        self.blocks: List[Object] = []
        self.obstacles: List[Object] = []
        self.phase = 'observe'
        self.reason = '等待有效视觉'
        self.path: List[Point] = []
        self.lane = None
        self.approach = None
        self.finish_y = 0.16
        self.plan_time = -float('inf')
        self.last_select = -float('inf')
        self.blocked_since = None
        self.progress_time = 0.0
        self.progress_pose = None
        self.failures = {}
        self.smoother = SmoothVelocity()
        self.last_now = None
        self.guard_since = None
        self.events = []
        self.escape_until = 0.0
        self.escape_target = (0.0,0.0,0.0)
        self.planning_token=0
        self.planning_result=None
        self.planning_job=None
        self.last_clearance=-float('inf')
        self.window_wait_started=None

    def event(self, now, reason):
        self.reason = reason
        self.events.append((round(now, 2), self.phase, reason))
        self.events = self.events[-100:]

    def update(self, scene):
        stamp = float(scene.get('stamp', -float('inf')))
        if not math.isfinite(stamp) or stamp < self.scene_time:
            return
        self.scene_time = stamp
        robot = scene.get('robot')
        self.valid = bool(scene.get('valid')) and robot is not None
        if not self.valid:
            return
        if len(robot) != 3 or not all(math.isfinite(float(v)) for v in robot):
            self.valid = False; return
        self.pose = tuple(float(v) for v in robot)
        new_blocks = []
        matched_blocks = set()
        for values in scene.get('blocks', []):
            if len(values) < 5 or not all(math.isfinite(float(v)) for v in values[:4]):
                self.valid = False; return
            x, y, hx, hy = map(float, values[:4])
            if max(abs(x), abs(y)) > 0.70 or not (0 < hx < 0.5 and 0 < hy < 0.5):
                self.valid = False; return
            obj=Object(x,y,hx,hy,bool(values[4]),stamp)
            candidates=[(math.dist((x,y),(o.x,o.y)),i,o) for i,o in enumerate(self.blocks) if i not in matched_blocks]
            if candidates:
                distance,i,old=min(candidates,key=lambda entry:entry[0])
                dt=stamp-old.stamp
                if distance<.10 and .02<=dt<=.35:
                    matched_blocks.add(i)
                    obj.vx=.6*old.vx+.4*clamp((x-old.x)/dt,-.55,.55)
                    obj.vy=.6*old.vy+.4*clamp((y-old.y)/dt,-.55,.55)
            new_blocks.append(obj)
        # 短时遮挡保留目标，不用单帧空结果宣告清空。
        for old in self.blocks:
            if stamp - old.stamp < 0.45 and not any(math.dist((old.x, old.y), (b.x, b.y)) < 0.12 for b in new_blocks):
                new_blocks.append(old)
        self.blocks = new_blocks
        new_obstacles = []
        used = set()
        for values in scene.get('obstacles', []):
            if len(values) < 4 or not all(math.isfinite(float(v)) for v in values[:4]):
                self.valid = False; return
            x, y, hx, hy = map(float, values[:4])
            if max(abs(x), abs(y)) > 0.7 or not (0 < hx <= 0.3 and 0 < hy <= 0.3):
                self.valid = False; return
            obj = Object(x, y, hx, hy, stamp=stamp)
            choices = [(math.dist((x, y), (o.x, o.y)), i, o) for i, o in enumerate(self.obstacles) if i not in used]
            if choices:
                distance, i, old = min(choices, key=lambda item: item[0])
                dt = stamp - old.stamp
                if distance < 0.20 and 0.02 <= dt <= 0.8:
                    used.add(i)
                    obj.vx = 0.5 * old.vx + 0.5 * clamp((x - old.x) / dt, -0.6, 0.6)
                    obj.vy = 0.5 * old.vy + 0.5 * clamp((y - old.y) / dt, -0.6, 0.6)
                    if distance<.004 and math.hypot(obj.vx,obj.vy)<.025:
                        obj.still_for=old.still_for+dt
            new_obstacles.append(obj)
        new_obstacles.extend(o for i, o in enumerate(self.obstacles) if i not in used and stamp - o.stamp < 0.7
                             and not any(math.dist((o.x, o.y), (p.x, p.y)) < 0.12 for p in new_obstacles))
        self.obstacles = new_obstacles

    def occupied(self):
        return self.blocks + self.forecast_obstacles()

    def forecast_obstacles(self):
        result = []
        for o in self.obstacles:
            # 留出接近一秒的退让提前量，不在冲突点等对手撞上来。
            dx, dy = o.vx * 0.80, o.vy * 0.80
            result.append(Object(o.x + dx / 2, o.y + dy / 2,
                                 o.hx + abs(dx) / 2 + 0.015, o.hy + abs(dy) / 2 + 0.015,kind='obstacle'))
        return result

    def pending(self):
        return [b for b in self.blocks if b.own or b.y - b.hy < -0.008]

    def corridor_blocked(self, x, south, north):
        return any(abs(o.x - x) < o.hx + 0.105 and o.y + o.hy + 0.11 > south and o.y - o.hy - 0.11 < north
                   for o in self.forecast_obstacles())

    def choose(self, now):
        """选择单位时间可清除量高、可到达、对手风险低的车道。"""
        planning_deadline = time.monotonic()+.045
        pending = self.pending()
        if not pending:
            danger = min((gap(self.pose[:2], o, .10, .10) for o in self.forecast_obstacles()), default=1.0)
            if self.pose[1] > -0.12 or danger < .08:
                homes = [(clamp(self.pose[0],-.35,.35),-.28),(-.36,-.35),(.36,-.35),(0,-.42)]
                homes.sort(key=lambda pt: math.dist(pt,self.pose[:2]) + 3 * max(0,.15-min((gap(pt,o) for o in self.forecast_obstacles()),default=1)))
                for home in homes:
                    if math.dist(home,self.pose[:2]) < .06: continue
                    path = plan_path(self.pose[:2],home,self.occupied())
                    if path:
                        self.phase = 'standby'; self.path = path; self.lane = None
                        self.plan_time = now; self.progress_pose = self.pose; self.progress_time = now
                        self.event(now,'己方暂时清空，退回空区待命并监视回推')
                        return True
            entering = self.phase != 'guard'
            self.phase = 'guard'; self.path = []; self.lane = None
            if self.guard_since is None: self.guard_since = now
            if entering: self.event(now, '己方暂时清空，继续监视回推物块')
            return False
        self.guard_since = None
        xs = [b.x for b in pending]
        for i, a in enumerate(pending):
            xs.extend((a.x + a.hx - 0.055, a.x - a.hx + 0.055, a.x - 0.065, a.x + 0.065))
            xs.extend((a.x-.025,a.x+.025))
            xs.extend((a.x + b.x) / 2 for b in pending[i + 1:] if abs(a.x - b.x) < 0.12)
        candidates = sorted(set(round(clamp(x, -0.515, 0.515), 3) for x in xs))
        ranked = []
        for x in candidates:
            column = [b for b in pending if brush_coverage(x,b)>=.35]
            if not column: continue
            south = min(b.y - b.hy for b in column)
            approach = (x, clamp(min(-0.18, south - 0.125), -0.53, -0.18))
            # 从检测到的堆南端后面进场，不能把车横移进物块。
            if south - approach[1] < 0.09: continue
            # 越过中线后留出整车退让空间，避免夹在两排之间再横移。
            lateral_patrol=any(abs(o.vx)>.04 and abs(o.vy)<.04 and -.04<o.y<.18 for o in self.obstacles)
            finish = .12 if lateral_patrol else .28
            if self.corridor_blocked(x, approach[1], finish + 0.09): continue
            key = round(x / 0.1)
            failed = self.failures.get(key)
            if failed and now - failed < 4.0: continue
            # 路径只对最有希望的候选求解，先用距离估算排序。
            benefit = sum(min(6,max(1,4*b.hx*b.hy/.0004))*brush_coverage(x,b)**2 for b in column)
            travel = math.dist(self.pose[:2], approach)
            score = benefit / (1.0 + travel / 0.32 + (finish - approach[1]) / 0.27)
            grazing=sum(.9-brush_coverage(x,b) for b in self.blocks
                        if approach[1]<b.y<finish+.10 and abs(b.x-x)<.108 and brush_coverage(x,b)<.80)
            score/=1+2*grazing
            # 有块正在蓝区深处时，先处理它，避免只围着中线转。
            score *= 1 + max(0, -south - 0.15)
            ranked.append((score, x, approach, finish))
        # 候选若不可达，封存短时再选别列，而不是在原地随机挪车。
        for _, x, approach, finish in sorted(ranked, reverse=True)[:8]:
            if time.monotonic()>planning_deadline: break
            path = plan_path(self.pose[:2], approach, self.occupied())
            if path:
                self.lane = x; self.approach = approach; self.finish_y = finish
                self.path = path; self.phase = 'transit'; self.plan_time = now
                self.progress_pose = self.pose; self.progress_time = now
                self.event(now, f'选择北推车道 x={x:.3f}，接近点 y={approach[1]:.3f}')
                return True
            self.failures[round(x / 0.1)] = now
        if ranked:
            _,x,approach,finish=max(ranked)
            self.start_planning(now,x,approach,finish)
            return False
        entering = self.phase != 'yield'
        self.phase = 'yield'; self.path = []; self.lane = None
        if entering: self.event(now, '车道被占或暂不可达，等待后重选')
        return False

    def start_planning(self,now,x,approach,finish):
        """完整搜索在后台继续，控制周期与看门狗不等待它。"""
        self.planning_token+=1
        token=self.planning_token
        self.planning_result=None
        self.planning_job=(now,x,approach,finish)
        self.lane=x
        self.phase='planning';self.path=[]
        start=self.pose[:2];objects=self.occupied()
        def solve():
            result=plan_path(start,approach,objects,budget=.7)
            if token==self.planning_token:self.planning_result=result
        threading.Thread(target=solve,daemon=True).start()
        self.event(now,'快速通道搜索未完成，停车继续全局规划')

    def finish_planning(self,now):
        if self.planning_result is None and now-self.planning_job[0]<1.2:return
        _stamp,x,approach,finish=self.planning_job
        path=self.planning_result or []
        previous=self.pose[:2]
        clear=bool(path) and any(abs(b.x-x)<.11 for b in self.pending())
        for i,point in enumerate(path):
            if not segment_clear(previous,point,self.occupied(),escaping=i==0):clear=False;break
            previous=point
        if clear:
            self.lane=x;self.approach=approach;self.finish_y=finish;self.path=path
            self.phase='transit';self.plan_time=now;self.progress_pose=self.pose;self.progress_time=now
            self.event(now,'全局规划完成，沿已复查的净空通道接近目标')
        elif not self.try_clearance(now):self.abort(now,'全局通道暂不可用，比较其他车道')
        self.planning_token+=1

    def try_clearance(self,now):
        """被红块围住时，先用车头腾出退路；不把可推动物永远当固定墙。"""
        if now-self.last_clearance<4 or not self.pending() or abs(wrap(self.pose[2]-NORTH))>.10:return False
        x,y,_yaw=self.pose
        finish=min(.40,y+.20)
        if finish<y+.08:return False
        front=[b for b in self.blocks if y+.06<b.y<y+.26]
        if not front or any(b.y+b.hy>.565 for b in front):return False
        candidates=[x]+[b.x for b in front]
        candidates += [(a.x+b.x)/2 for i,a in enumerate(front) for b in front[i+1:]]
        options=[]
        for lane in candidates:
            if abs(lane-x)>.06 or not(-.51<lane<.51) or self.corridor_blocked(lane,y,finish+.10):continue
            if not segment_clear((x,y),(lane,y),self.occupied(),escaping=True):continue
            covered=[b for b in front if brush_coverage(lane,b)>.80]
            if not covered:continue
            grazing=any(abs(b.x-lane)<.108 and brush_coverage(lane,b)<.60 for b in front)
            if grazing:continue
            options.append((sum(brush_coverage(lane,b) for b in covered)-2*abs(lane-x),lane))
        if not options:return False
        _score,lane=max(options)
        self.lane=lane;self.approach=(lane,y);self.finish_y=finish;self.path=[]
        self.phase='align';self.last_clearance=now;self.plan_time=now
        self.progress_pose=self.pose;self.progress_time=now
        self.event(now,'退路被物块封住，先对准并直推腾出通道')
        return True

    def abort(self, now, reason):
        self.planning_token+=1
        if self.lane is not None: self.failures[round(self.lane / 0.1)] = now
        self.phase = 'yield'; self.path = []; self.lane = None
        self.last_select = now
        self.blocked_since = None
        self.event(now, reason)

    def escape(self, now):
        """从预测冲突中向净空增加的方向退让，不靠随机挪车。"""
        x,y,yaw = self.pose
        c,s = math.cos(yaw),math.sin(yaw)
        choices = []
        obstacles = self.forecast_obstacles()
        moving_blocks=[Object(b.x+b.vx*.45,b.y+b.vy*.45,b.hx,b.hy) for b in self.blocks]
        objects = moving_blocks + obstacles
        current = min((body_gap(self.pose,o) for o in objects),default=1.0)
        for vx,vy in [(-.18,0),(0,.18),(0,-.18),(.28,0),(-.13,.13),(-.13,-.13)]:
            cmd = (vx,vy,0.0)
            if not self.safe(cmd,now,escape_push=vx>0): continue
            dx,dy = c*vx-s*vy,s*vx+c*vy
            target = (x+dx*.65,y+dy*.65)
            if not (-.515 <= target[0] <= .515 and -.54 <= target[1] <= .48): continue
            evaluated=objects
            if vx>0 and abs(wrap(yaw-NORTH))<.14:
                evaluated=[o for o in objects if o in obstacles or
                           not(c*(o.x-x)+s*(o.y-y)>=-.035 and abs(-s*(o.x-x)+c*(o.y-y))<=.090)]
            clearance = min((body_gap((target[0],target[1],yaw),o) for o in evaluated),default=1.0)
            threat = min((gap(target,o,.10,.105) for o in obstacles),default=.2)
            # 优先退入己方空区；只有提高净空的候选才参与选择。
            score = min(.15,clearance)-min(.15,current) + min(.2,threat) - .30*dy
            if self.phase == 'evade' and cmd == self.escape_target: score += .015
            choices.append((score,cmd))
        if not choices: return None
        return max(choices,key=lambda item:item[0])[1]

    def conflict(self):
        # 连续静止确认后才用邻接通道净空。移动/刚掉头的对手仍保留
        # 原有提前退让量，不能把静态通行优化泛化为缩小动态安全距离。
        for tracked,forecast in zip(self.obstacles,self.forecast_obstacles()):
            threshold=.015 if tracked.still_for>=.7 else .07
            if gap(self.pose[:2],forecast,.10,.105)<threshold:return True
        return self.block_conflict()

    def block_conflict(self):
        x,y,yaw=self.pose;c,s=math.cos(yaw),math.sin(yaw)
        for b in self.blocks:
            fx=c*(b.x-x)+s*(b.y-y);fy=-s*(b.x-x)+c*(b.y-y)
            captured=-.045<=fx<=.055 and abs(fy)<=.035
            if captured or (fx>=-.035 and abs(fy)<=.090): continue
            current=body_gap(self.pose,b)
            if current<.003: return True
            if math.hypot(b.vx,b.vy)>.04:
                for t in (.25,.5,.75,1.):
                    future=Object(b.x+b.vx*t,b.y+b.vy*t,b.hx,b.hy)
                    if body_gap(self.pose,future)<.08 and body_gap(self.pose,future)<current-.01: return True
        return False

    def wait_for_crossing_window(self,now):
        """在横穿扫掠带以南保留就绪车道；冲向自己或侧后有块则退让。"""
        if self.phase not in ('transit','align','push') or abs(wrap(self.pose[2]-NORTH))>.10 or self.block_conflict():
            self.window_wait_started=None;return False
        threats=[(o,f) for o,f in zip(self.obstacles,self.forecast_obstacles()) if gap(self.pose[:2],f,.10,.105)<.07]
        waiting=bool(threats) and all(
            abs(o.vx)>.04 and abs(o.vy)<.04 and -.04<o.y<.18
            and self.pose[1]+.10 < f.y-f.hy-.02
            and gap(self.pose[:2],o,.10,.105)>.025 for o,f in threats)
        if not waiting:
            self.window_wait_started=None;return False
        if self.window_wait_started is None:
            self.window_wait_started=now
            self.event(now,'横穿对手将经过，保留当前车道等待空档')
        self.progress_pose=self.pose;self.progress_time=now
        if now-self.window_wait_started>2.5:
            self.window_wait_started=None
            self.abort(now,'横穿空档等待超时，重新选择车道')
        return True

    def safe(self, cmd, now, escape_push=False):
        if self.pose is None: return False
        x, y, yaw = self.pose
        vx, vy, wz = cmd
        if max(abs(vx), abs(vy), abs(wz)) < 1e-10: return True
        # 平滑起步的第一拍速度很小，但仍是合法车头推送；不能要求先越过速度门槛。
        push_mode=self.phase=='push' or getattr(self,'row_front_contact',False) or ((self.phase=='evade' or escape_push) and abs(wrap(yaw-NORTH))<.14)
        pushing = push_mode and vx > 1e-5 and abs(vy) <= max(0.003, 0.4 * vx) and abs(wz) < 0.25
        dumping=vx<-.05 and abs(vy)<=.4*abs(vx) and abs(wz)<.25
        horizon = max(0.16, math.hypot(vx, vy) / 1.3 + 0.10)
        for t in (0.05, horizon / 2, horizon):
            c, s = math.cos(yaw), math.sin(yaw)
            px, py = x + (c * vx - s * vy) * t, y + (s * vx + c * vy) * t
            # 朝东扫排时，车尾短于侧向导板，允许从西侧空隙进入。
            # 此扩展仅用于已对正的扫排任务；北向通行仍保持原边界。
            west=-.553 if getattr(self,'row_active',False) and abs(wrap(yaw))<.12 else -.515
            if not (west <= px <= 0.515 and -0.548 <= py <= 0.505):
                # 标记在车顶，出发贴墙时有少量视差；只允许向场内离墙。
                old_violation = max(0, west-x,x-.515) + max(0, -0.548 - y, y - 0.505)
                violation = max(0, west-px,px-.515) + max(0, -0.548 - py, py - 0.505)
                if violation >= old_violation: return False
            for o in self.blocks:
                dx, dy = o.x - x, o.y - y
                forward, left = c * dx + s * dy, -s * dx + c * dy
                captured=-.045<=forward<=.055 and abs(left)<=.035
                if (pushing and forward >= -0.035 and abs(left) <= 0.090) or (captured and (pushing or dumping)):
                    continue
                old_gap = body_gap((x,y,yaw),o)
                predicted=Object(o.x+o.vx*t,o.y+o.vy*t,o.hx,o.hy)
                new_gap = body_gap((px,py,yaw+wz*t),predicted)
                half_forward=abs(c)*o.hx+abs(s)*o.hy
                nose_only=forward-half_forward>=.05
                # 允许从已经贴住车头的整排堆直退。导板斜面与合并视觉框
                # 可能重叠超过5mm，不能因此阻断离开该接触的第一拍小速度。
                straight_release = vx < 0 and abs(vy) <= .1*abs(vx) and abs(wz)<.08
                parallel_exit = nose_only and (old_gap > -.005 or straight_release) and new_gap >= old_gap-1e-7
                if new_gap < 0.003 and new_gap <= old_gap + 1e-5 * t and not parallel_exit:
                    return False
                if abs(wz) > 0.12 and math.hypot(dx, dy) < 0.125 + max(o.hx, o.hy): return False
            for o in self.forecast_obstacles():
                old_gap = gap((x, y), o, 0.10, 0.105)
                new_gap = gap((px, py), o, 0.10, 0.105)
                if new_gap < 0.005 and new_gap <= old_gap + 1e-5 * t: return False
        return True

    def raw_command(self, now):
        x, y, yaw = self.pose
        error = wrap(NORTH - yaw)
        wz = clamp(1.6 * error, -0.65, 0.65)
        turn_needed = abs(error) > (.20 if self.phase == 'push' else .10)
        misaligned_near_blocks = turn_needed and any(math.dist((x,y),(b.x,b.y))<.135+max(b.hx,b.hy) for b in self.blocks)
        if self.wait_for_crossing_window(now):return (0.,0.,0.)
        if self.conflict() or misaligned_near_blocks or self.phase == 'evade':
            if self.phase == 'evade' and not self.conflict() and not misaligned_near_blocks and now >= self.escape_until:
                self.phase = 'observe'; self.last_select = now
                self.event(now,'脱离冲突区域，重新选择车道')
                return (0.0,0.0,0.0)
            command = self.escape(now)
            if command:
                if self.phase != 'evade':
                    self.planning_token+=1
                    self.phase = 'evade'; self.path = []; self.lane = None
                    self.escape_until = now + .55
                    self.event(now,'预测到近距离冲突，主动向安全空区退让')
                self.escape_target = command
                return command
            return (0.0,0.0,0.0)
        if self.phase in ('observe','guard','yield') and abs(error)>.10:
            return (0.0,0.0,wz)
        if self.phase=='planning':
            self.finish_planning(now)
            return (0.0,0.0,0.0)
        if self.phase in ('observe', 'guard', 'yield'):
            if now - self.last_select >= 0.35:
                self.last_select = now; self.choose(now)
            return (0.0, 0.0, 0.0)
        if self.phase in ('transit', 'standby'):
            if self.phase == 'standby' and self.pending():
                self.choose(now)
                return (0.0,0.0,0.0)
            if abs(error) > 0.16: return (0.0, 0.0, wz)
            if not self.path:
                self.phase = 'guard' if self.phase == 'standby' else 'align'
                return (0.0, 0.0, wz)
            while len(self.path) > 1 and math.dist((x, y), self.path[0]) < 0.035:
                self.path.pop(0)
            goal = self.path[0]
            if not segment_clear((x, y), goal, self.occupied(), escaping=True):
                if now - self.plan_time > 0.35:
                    self.abort(now, '通行路径发生变化，重新评估车道')
                return (0.0, 0.0, 0.0)
            dist = math.dist((x, y), goal)
            if len(self.path) == 1 and dist < 0.024:
                self.phase = 'guard' if self.phase == 'standby' else 'align'
                self.path = []; return (0.0, 0.0, wz)
            remaining = dist if len(self.path) == 1 else dist + 0.10
            speed = min(0.36, 1.8 * remaining, math.sqrt(0.9 * remaining))
            vxw, vyw = speed * (goal[0] - x) / max(dist, 0.001), speed * (goal[1] - y) / max(dist, 0.001)
            return (math.cos(yaw) * vxw + math.sin(yaw) * vyw,
                    -math.sin(yaw) * vxw + math.cos(yaw) * vyw, wz)
        if self.phase == 'align':
            if abs(error) > 0.10: return (0.0, 0.0, wz)
            if abs(x - self.lane) > 0.008:
                return (0.0, clamp(1.5 * (x - self.lane), -0.14, 0.14), wz)
            if abs(self.smoother.velocity[1]) > .006 or abs(self.smoother.velocity[2]) > .06:
                return (0.0,0.0,wz)
            self.phase = 'push'; self.progress_time = now; self.progress_pose = self.pose
            self.event(now, '对准完成，沿车道直推')
        if self.phase == 'push':
            if self.corridor_blocked(self.lane, y, self.finish_y + 0.09):
                self.abort(now, '对手进入推送走廊，让行并选择其他车道')
                return (0.0, 0.0, 0.0)
            if y >= self.finish_y - 0.025:
                self.phase = 'observe'; self.last_select = now
                self.event(now, '完成一铲，等待新画面评估剩余和回推')
                return (0.0, 0.0, 0.0)
            if abs(error) > 0.20:
                return (0.0, 0.0, clamp(wz, -0.2, 0.2))
            speed = min(0.28, max(0.075, 1.6 * (self.finish_y - y)))
            # 微小纠偏，不在块带里更换车道。
            lateral_limit=min(.018,max(.002,.25*max(0,self.smoother.velocity[0])))
            correction = clamp(1.0 * (x - self.lane), -lateral_limit, lateral_limit)
            return (speed, correction, clamp(wz, -0.20, 0.20))
        return (0.0, 0.0, 0.0)

    def step(self, now):
        if self.start_time is None: self.start_time = now
        dt = clamp(0.05 if self.last_now is None else now - self.last_now, 0.01, 0.10)
        self.last_now = now
        if now - self.start_time >= self.duration:
            if self.phase != 'ended':
                self.phase = 'ended'; self.event(now, '90 秒比赛结束，禁止继续移动')
            self.smoother.reset(); return (0.0, 0.0, 0.0)
        if not self.valid or self.pose is None or not (-0.02 <= now - self.scene_time <= 0.35):
            self.smoother.reset(); return (0.0, 0.0, 0.0)
        if self.phase == 'ended': return (0.0, 0.0, 0.0)
        target = self.raw_command(now)
        if self.phase in ('transit', 'standby', 'align', 'push'):
            if self.progress_pose is None or math.dist(self.pose[:2], self.progress_pose[:2]) > 0.035 or abs(wrap(self.pose[2] - self.progress_pose[2])) > 0.10:
                self.progress_pose = self.pose; self.progress_time = now
            elif now - self.progress_time > 2.2:
                self.abort(now, '目标进度停滞，换列规划，不随机侧撞脱困')
                target = (0.0, 0.0, 0.0)
        candidate = self.smoother.step(target, dt)
        # 平滑之后检查实际要发出的速度，避免“先安全后斜坡”重新引入危险速度。
        if not self.safe(candidate, now):
            self.smoother.reset()
            if self.blocked_since is None: self.blocked_since = now
            if now - self.blocked_since > 0.65 and self.phase not in ('guard', 'yield'):
                self.abort(now, '预测净空不足，停车后重新规划')
            return (0.0, 0.0, 0.0)
        self.blocked_since = None
        return candidate
