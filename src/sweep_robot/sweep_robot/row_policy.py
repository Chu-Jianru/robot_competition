"""整排连续推送优先，队形失效或受阻时回到视觉局部清理。"""
import math
import statistics
from dataclasses import dataclass

from sweep_robot.competition_policy import CompetitionPolicy, NORTH, clamp, wrap


@dataclass
class SweepLeg:
    x: float
    y: float
    yaw: float
    mode: str
    name: str


class HybridCompetitionPolicy(CompetitionPolicy):
    def __init__(self, duration=90):
        super().__init__(duration)
        self.row_active=False
        self.row_front_contact=False
        self.opening_checked=False
        self.legs=[]
        self.leg_index=0
        self.leg_started=0.
        self.row_blocked=None
        self.row_progress=None
        self.row_progress_at=0.
        self.row_recovery=False
        self.row_last_tick=None

    def start_rows(self, now):
        """只在实测的两排队形下启动；不为散块或深处回推块空跑。"""
        own=self.pending()
        other=[b for b in self.blocks if .012<b.y<.14 and not b.own]
        if len(own)<6 or len(other)<6 or any(b.y<-.14 for b in own):return False
        def regular(row):
            center=statistics.median(b.y for b in row)
            return max(b.x for b in row)-min(b.x for b in row)>.60 and all(abs(b.y-center)+b.hy<.045 for b in row)
        if not regular(own) or not regular(other):return False
        # 长整排任务不能凭一帧把尚未判断运动方向的对手当作空场。
        if any(math.hypot(o.vx,o.vy)>.04 or o.still_for<.3 for o in self.obstacles):return False
        own_y=min(statistics.median(b.y for b in own)-.012,
                  min(b.y-b.hy for b in other)-.105)
        if any(abs(b.y-own_y)+b.hy>.062 for b in own):return False
        # 从另一排外侧略偏离，给斜导板留出净空；仍须完整罩住目标排。
        other_y=max(statistics.median(b.y for b in other)+.006,max(b.y+b.hy for b in own)+.105)
        if any(abs(b.y-other_y)+b.hy>.062 for b in other):return False
        if any(abs(o.y-other_y)<o.hy+.11 or abs(o.y-own_y)<o.hy+.11 for o in self.forecast_obstacles()):return False
        west=-.552
        self.legs=[
            SweepLeg(-.47,-.25,NORTH,'transit','西侧空区接近'),
            SweepLeg(-.47,-.25,0.,'turn','空区转向东'),
            SweepLeg(west,-.25,0.,'align','从西侧留出刷前空隙'),
            SweepLeg(west,other_y,0.,'align','对准对方整排'),
            SweepLeg(.505,other_y,0.,'push','连续横推对方一排'),
            SweepLeg(west,other_y,0.,'retreat','沿已清空排返回西侧'),
            SweepLeg(west,own_y,0.,'align','对准己方整排'),
            SweepLeg(.505,own_y,0.,'push','连续横推己方一排'),
            SweepLeg(.20,own_y,0.,'retreat','退出东墙积木堆'),
            SweepLeg(.20,-.53,0.,'transit','南侧空区准备转向'),
            SweepLeg(.20,-.53,NORTH,'turn','空区转向北'),
            SweepLeg(.510,-.53,NORTH,'align','对准东墙集中块'),
            SweepLeg(.510,.14,NORTH,'push','沿墙一次推过中线'),
            SweepLeg(.510,-.53,NORTH,'retreat','退出集中块，复查漏块'),
        ]
        self.row_active=True;self.leg_index=0;self.leg_started=now
        self.row_progress=self.pose;self.row_progress_at=now
        self.phase='row_transit'
        self.event(now,'两排队形有效，启动整排连续推送')
        return True

    def finish_rows(self,now,reason):
        if self.pose[0]<-.515:
            # 西侧空隙内不能直接转向北；先沿已走过的空隙退出，再向东收回。
            self.row_recovery=True;self.row_front_contact=False
            self.legs=[SweepLeg(self.pose[0],-.25,0.,'retreat','退出西侧整排入口'),
                       SweepLeg(-.47,-.25,0.,'transit','返回可转向空区'),
                       SweepLeg(-.47,-.25,NORTH,'turn','恢复北向局部清理')]
            self.leg_index=0;self.leg_started=now;self.row_progress=self.pose;self.row_progress_at=now
            self.blocked_since=None;self.row_blocked=None
            self.phase='row_retreat';self.event(now,reason+'；先退出西侧窄入口')
            return
        self.row_active=False;self.row_front_contact=False
        self.legs=[];self.phase='observe';self.path=[];self.lane=None
        self.smoother.reset();self.last_select=now-.4
        self.planning_token+=1
        self.event(now,reason)

    def next_leg(self,now):
        self.leg_index+=1
        self.row_blocked=None;self.blocked_since=None
        self.row_progress=self.pose;self.row_progress_at=now;self.leg_started=now
        if self.leg_index>=len(self.legs):
            self.finish_rows(now,'整排推送完成，视觉复查漏块并监视回推')
            return
        leg=self.legs[self.leg_index]
        # 到达西侧后再从新画面确认己方排，不沿开局时的旧坐标盲推。
        if self.leg_index==6:
            row=[b for b in self.pending() if b.x<.42 and -.14<b.y<-.01]
            if row:
                lane=statistics.median(b.y for b in row)-.012
                upper=[b.y-b.hy for b in self.blocks if b.y>0 and not b.own]
                if upper:lane=min(lane,min(upper)-.105)
                if any(abs(b.y-lane)+b.hy>.062 for b in row):
                    self.finish_rows(now,'己方队形已散开，切换局部清理');return
                for i in (6,7,8):self.legs[i].y=lane
        if self.leg_index==9:
            east=[b for b in self.pending() if b.x>.33]
            south=clamp(min((b.y-b.hy for b in east),default=-.04)-.17,-.53,-.20)
            for i in (9,10,11,13):self.legs[i].y=south
        if self.leg_index==11:
            east=[b for b in self.pending() if b.x>.38]
            if east:
                lane=clamp(max(b.x+b.hx for b in east)-.065,.49,.515)
                for i in (11,12,13):self.legs[i].x=lane
        self.phase='row_'+leg.mode
        self.event(now,leg.name)

    def row_command(self,now):
        leg=self.legs[self.leg_index]
        x,y,yaw=self.pose
        if self.row_last_tick is not None and now-self.row_last_tick>.35:
            self.row_progress=self.pose;self.row_progress_at=now
        self.row_last_tick=now
        error=wrap(leg.yaw-yaw)
        self.phase='row_'+leg.mode
        self.row_front_contact=leg.mode=='push' and abs(error)<.14
        distance=math.hypot(leg.x-x,leg.y-y)
        wall_speed=.30
        if leg.mode=='turn':
            complete=abs(error)<.065 and abs(self.smoother.velocity[2])<.15
        elif leg.mode=='push':
            forward=(leg.x-x)*math.cos(leg.yaw)+(leg.y-y)*math.sin(leg.yaw)
            complete=forward<.025 and abs(error)<.15
            # 堆到墙即结束这一铲，不能为了凑坐标持续挤墙。
            if abs(leg.yaw)<.1:
                # 检查从刷前连到墙边的队列，不等车心也贴近墙才停车。
                # 不把远处孤立的墙边块当成本铲已经到墙。
                train=sorted((b for b in self.blocks if b.x+b.hx>x+.02 and abs(b.y-y)<.075),key=lambda b:b.x-b.hx)
                end=x+.125
                connected=False
                for b in train:
                    if b.x-b.hx>end+.025:break
                    end=max(end,b.x+b.hx);connected=True
                if connected:
                    # 先减速再换段，为视觉间隔和制动行程保留空间。
                    wall_speed=clamp(1.6*(.56-end),.025,.30)
                    complete=complete or end>.55
        else:
            precise=leg.mode=='align' or leg.x<-.53
            tolerance=.0015 if leg.x<-.53 else .007 if precise else .030
            complete=distance<tolerance and abs(error)<.08
            if precise:
                complete=complete and math.hypot(*self.smoother.velocity[:2])<.025 and abs(self.smoother.velocity[2])<.10
        if complete:
            self.next_leg(now)
            return (0.,0.,0.)
        # 持续复查尚未执行的整排通道，不能等对手靠近车身才取消长任务。
        upcoming=[item for item in self.legs[self.leg_index:] if item.mode=='push' and abs(item.yaw)<.1]
        threat=not self.row_recovery and any(
            -.56<o.x+o.hx and o.x-o.hx<.60 and abs(o.y-item.y)<o.hy+.11
            for item in upcoming for o in self.forecast_obstacles())
        if threat:
            if self.row_blocked is None:self.row_blocked=now
            if now-self.row_blocked>.25:self.finish_rows(now,'下一整排通道出现对手，提前切换局部清理')
            return (0.,0.,0.)
        # 动态块接近侧后方或对手占位时先停车，短时等待，持续受阻才退出整排任务。
        if self.conflict():
            if self.row_blocked is None:self.row_blocked=now
            if now-self.row_blocked>.8:self.finish_rows(now,'整排通道被对手/动态块占据，切换局部清理')
            return (0.,0.,0.)
        self.row_blocked=None
        if self.row_progress is None or math.dist(self.pose[:2],self.row_progress[:2])>.025 or abs(wrap(yaw-self.row_progress[2]))>.08:
            self.row_progress=self.pose;self.row_progress_at=now
        elif now-self.row_progress_at>2.0:
            self.finish_rows(now,'整排任务进度停滞，保留已完成清场并重新规划')
            return (0.,0.,0.)
        # 第一排受阻也不能无限消耗比赛时间。
        budget=62 if self.leg_index<8 else 80
        if not self.row_recovery and now-self.start_time>budget:
            self.finish_rows(now,'整排任务预算到期，优先处理当前己方残余')
            return (0.,0.,0.)
        wz=clamp(1.6*error,-.75 if leg.mode=='turn' else -.30,.75 if leg.mode=='turn' else .30)
        if leg.mode=='turn' or abs(error)>.12:return (0.,0.,wz)
        if leg.mode=='push':
            speed=min(wall_speed,max(.07,1.6*distance))
            lateral=-(leg.x-x)*math.sin(yaw)+(leg.y-y)*math.cos(yaw)
            limit=min(.018,max(.002,.25*max(0,self.smoother.velocity[0])))
            return (speed,clamp(lateral,-limit,limit),wz)
        tolerance=.0015 if leg.x<-.53 else .007
        if (leg.mode=='align' or leg.x<-.53) and distance<tolerance:return (0.,0.,wz)
        speed=min(.38,1.8*distance)
        forward=(leg.x-x)*math.cos(yaw)+(leg.y-y)*math.sin(yaw)
        if leg.mode=='retreat' and forward<-.03 and abs(error)<.04:
            # 先沿车头轴线脱离接触，再在空区校正横向误差；独立平滑
            # 会让起步时极小的横向纠偏占比过大，阻断合法直退。
            return (-speed,0.,0.)
        vx=speed*(leg.x-x)/max(distance,.001);vy=speed*(leg.y-y)/max(distance,.001)
        return (math.cos(yaw)*vx+math.sin(yaw)*vy,-math.sin(yaw)*vx+math.cos(yaw)*vy,wz)

    def abort(self,now,reason):
        if self.row_active:
            self.finish_rows(now,'整排路径安全检查未通过，转入局部清理：'+reason)
        else:super().abort(now,reason)

    def raw_command(self,now):
        if not self.opening_checked:
            if not self.row_active and now-self.start_time<.40:return (0.,0.,0.)
            self.opening_checked=True
            if now-self.start_time<2:self.start_rows(now)
        if self.row_active:return self.row_command(now)
        self.row_front_contact=False
        return super().raw_command(now)
