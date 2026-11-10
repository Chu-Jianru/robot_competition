"""串行运行场景矩阵，避免多个 Gazebo 实例/模型查询争用。"""
import argparse
import subprocess
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('--revision',required=True)
parser.add_argument('--modes',nargs='+',default=['seed2','cross','return','seed1','block'])
parser.add_argument('--contacts-binary')
parser.add_argument('--require-front-contact',action='store_true')
args=parser.parse_args()
root=Path(__file__).resolve().parents[3]
for mode in args.modes:
    print(f'{args.revision}: {mode}',flush=True)
    world='seed1' if mode=='dropout' else mode
    opponent_mode='return' if mode=='return2' else mode
    world_path=root/f'log/adaptive_20261021/world_{world}/competition.world'
    if not world_path.exists():
        build=['python3',str(root/'competition_robot/models/competition_robot/build_robot.py'),'--seed',
               '20261021' if mode in ('seed2','return2') else '20260922','--output-dir',str(world_path.parent)]
        if opponent_mode in ('cross','return','block'):build+=['--opponent',opponent_mode]
        subprocess.run(build,cwd=root,check=True)
    cmd=['python3',str(Path(__file__).with_name('run_sim_audit.py')),'--output',str(root/f'log/adaptive_20261021/{args.revision}_{mode}'),
         '--world',str(root/f'log/adaptive_20261021/world_{world}/competition.world')]
    if opponent_mode in ('cross','return','block'):cmd+=['--opponent',opponent_mode]
    if mode=='dropout':cmd+=['--dropout']
    if args.contacts_binary:cmd+=['--contacts-binary',args.contacts_binary]
    if args.require_front_contact:cmd+=['--require-front-contact']
    subprocess.run(cmd,cwd=root,check=True)
