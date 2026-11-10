"""启动单次真实 Gazebo 检查，控制只用视觉；真实位姿仅用于验收。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import hashlib
import shutil
import re

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--controller', default='adaptive_controller')
    parser.add_argument('--rows-off',action='store_true',help='同一时钟/视觉条件下比较逐列任务')
    parser.add_argument('--world')
    parser.add_argument('--opponent', choices=('block', 'cross', 'return'))
    parser.add_argument('--dropout', action='store_true', help='在第12到14秒暂停视觉检测节点')
    parser.add_argument('--contacts-binary')
    parser.add_argument('--require-front-contact',action='store_true')
    parser.add_argument('--wall-timeout',type=float,default=480,help='慢速渲染时允许仿真花更多墙钟时间；比赛仍为90仿真秒')
    args = parser.parse_args()
    out = Path(args.output).resolve(); out.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[3]
    sources=list((root/'src/sweep_robot/sweep_robot').glob('*.py'))
    sources += [root/'src/sweep_robot/launch/sim.launch.py',root/'src/sweep_robot/config/gazebo.yaml',
                root/'competition_robot/models/competition_robot/build_robot.py',Path(__file__).with_name('contact_audit.cpp')]
    manifest = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (out/'source_sha256.json').write_text(json.dumps(manifest, indent=2))
    (out/'source').mkdir(exist_ok=True)
    for p in (root/'src/sweep_robot/sweep_robot').glob('*.py'):
        shutil.copy2(p,out/'source'/p.name)
    rclpy.init(); n = Node('simulation_audit')
    t = [0.0]; first_clock=[None]; latest = {}; last_image = [-100.]; bridge = CvBridge()
    def clock(msg):
        t[0] = msg.clock.sec + msg.clock.nanosec * 1e-9
        if first_clock[0] is None:first_clock[0]=t[0]
    def update(key, value): latest[key] = value
    def image(msg):
        if t[0] - last_image[0] >= 5:
            cv2.imwrite(str(out / f'camera_{t[0]:05.1f}.png'), bridge.imgmsg_to_cv2(msg, 'bgr8'))
            last_image[0] = t[0]
    n.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    n.create_subscription(String, '/perception/scene', lambda m: update('scene', json.loads(m.data)), 1)
    n.create_subscription(String, '/strategy/status', lambda m: update('strategy', json.loads(m.data)), 1)
    n.create_subscription(Twist, '/cmd_vel', lambda m: update('command',[m.linear.x,m.linear.y,m.angular.z]),1)
    n.create_subscription(Odometry, '/odom', lambda m: update('truth_robot', [m.pose.pose.position.x, m.pose.pose.position.y]), 1)
    n.create_subscription(Odometry, '/opponent/odom', lambda m: update('truth_opponent', [m.pose.pose.position.x, m.pose.pose.position.y]), 1)
    n.create_subscription(Image, '/overhead/camera/image_raw', image, qos_profile_sensor_data)
    cmd = ['ros2','launch','sweep_robot','sim.launch.py','gui:=false','autostart:=true','show_window:=false',f'controller:={args.controller}']
    if args.world: cmd.append('world:='+str(Path(args.world).resolve()))
    if args.rows_off:cmd.append('sweep_rows:=false')
    (out/'launch_command.json').write_text(json.dumps(cmd,indent=2))
    if args.world: shutil.copy2(Path(args.world).resolve(),out/'world.sdf')
    log = (out/'launch.log').open('w')
    proc = subprocess.Popen(cmd, cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    opponent = subprocess.Popen(['python3', str(Path(__file__).with_name('opponent_fixture.py')), args.opponent],
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True) if args.opponent else None
    deadline = time.monotonic() + args.wall_timeout
    startup_deadline=time.monotonic()+20
    last = -1
    detector_pid = None
    paused_detector = False
    dropout_done = False
    contacts = None
    contact_log = None
    try:
        with (out/'samples.jsonl').open('w') as f:
            while t[0] < 94 and time.monotonic() < deadline and proc.poll() is None:
                rclpy.spin_once(n, timeout_sec=.1)
                if first_clock[0] is not None and first_clock[0]>2:
                    raise RuntimeError('收到已有仿真的时钟；请使用独立ROS_DOMAIN_ID和GAZEBO_MASTER_URI，不能把旧场地当本场验收')
                if t[0]==0 and time.monotonic()>startup_deadline:
                    raise RuntimeError('20秒内没有仿真时钟，检查 launch.log')
                if args.contacts_binary and contacts is None and t[0]>.5:
                    contact_log=(out/'contacts.jsonl').open('w')
                    contacts=subprocess.Popen([str(Path(args.contacts_binary).resolve()),str(out/'truth_poses.jsonl')],stdout=contact_log,stderr=log)
                if args.dropout and not dropout_done and t[0]>=12:
                    if detector_pid is None:
                        matches=re.findall(r'\[hsv_detector-\d+\]: process started with pid \[(\d+)\]',(out/'launch.log').read_text())
                        if matches: detector_pid=int(matches[-1])
                    if detector_pid and not paused_detector:
                        os.kill(detector_pid,signal.SIGSTOP); paused_detector=True
                    if detector_pid and paused_detector and t[0]>=14:
                        os.kill(detector_pid,signal.SIGCONT);paused_detector=False;dropout_done=True
                if t[0] - last >= .1:
                    f.write(json.dumps(dict(t=t[0], **latest))+'\n'); f.flush(); last = t[0]
            if t[0] < 90: raise RuntimeError(f'未完成90秒仿真，停在 {t[0]}')
            if not all(key in latest for key in ('truth_robot','scene','command')):
                raise RuntimeError('没有完整的场景、车辆和指令记录，禁止输出验收比分')
        if args.contacts_binary:
            # 持续诊断订阅保留94秒附近的真实位置，避免反复新建Gazebo查询
            # 连接引发其 transport 断言崩溃。该订阅不向比赛控制发布任何数据。
            records=[]
            for line in (out/'truth_poses.jsonl').read_text().splitlines():
                try:record=json.loads(line)
                except json.JSONDecodeError:continue
                if 93.8<=record['t']<=94.2 and len(record['poses'])==21:records.append(record)
            if not records:raise RuntimeError('没有94秒附近完整的21模型诊断位置，禁止输出验收比分')
            snapshot=min(records,key=lambda row:abs(row['t']-94))
            poses=snapshot['poses']
            (out/'score_stamp.json').write_text(json.dumps(dict(t=snapshot['t'],source='gazebo pose/info diagnostic subscription')))
        else:
            subprocess.run(['gz','world','-w','competition','-p','1'],timeout=8,check=True)
            poses = {}
            for name in ['competition_robot'] + [f'block_{i:02d}' for i in range(20)]:
                result = subprocess.run(['gz','model','-w','competition','-m',name,'-p'],capture_output=True,text=True,timeout=8,check=True)
                poses[name] = list(map(float, result.stdout.split()))
        (out/'final_poses.json').write_text(json.dumps(poses,indent=2))
        remaining = {name:p for name,p in poses.items() if name.startswith('block_') and p[1] < 0}
        print(json.dumps(dict(time=t[0], remaining=len(remaining), blocks=remaining, output=str(out)),ensure_ascii=False),flush=True)
        if args.require_front_contact and (out/'contacts.jsonl').exists():
            allowed={'brush','plate_left','plate_right','wall_left','wall_right','wall_rear'}
            forbidden=[]
            for line in (out/'contacts.jsonl').read_text().splitlines():
                record=json.loads(line)
                if record['t']>90.1 or not ('block_' in record['a'] or 'block_' in record['b']): continue
                name=record['a'] if 'competition_robot' in record['a'] else record['b']
                if name.split('::')[-1] not in allowed:forbidden.append(record)
            if forbidden:raise RuntimeError(f'真实接触审计失败：{len(forbidden)}条非允许部件碰块采样')
    finally:
        if contacts and contacts.poll() is None:
            contacts.send_signal(signal.SIGINT);contacts.wait(timeout=8)
        if contact_log: contact_log.close()
        if paused_detector: os.kill(detector_pid,signal.SIGCONT)
        if opponent and opponent.poll() is None:
            os.killpg(opponent.pid, signal.SIGINT)
            opponent.wait(timeout=5)
        # launch 会转发信号；只通知父进程，避免子节点收到两次 SIGINT。
        if proc.poll() is None: proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM); proc.wait(timeout=5)
        log.close(); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
