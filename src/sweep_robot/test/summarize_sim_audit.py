"""汇总逐块真值、车身接触与场景时效，并生成可独立查看的轨迹图。"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def summarize(folder, root):
    samples=read_rows(folder/'samples.jsonl')
    models=json.loads((folder/'final_poses.json').read_text())
    states=[r['strategy'] for r in samples if r.get('strategy')]
    own=sum(v[1]<0 for k,v in models.items() if k.startswith('block_'))
    positive_seen=False; clear=None
    for r in samples:
        state=r.get('strategy',{})
        if state.get('own',0)>0:positive_seen=True
        if positive_seen and state.get('own')==0 and state.get('phase') in ('standby','guard'):
            clear=r['t'];break
    contacts=read_rows(folder/'contacts.jsonl') if (folder/'contacts.jsonl').exists() else []
    match_contacts=[r for r in contacts if r['t']<=90.1]
    block_contacts=[r for r in match_contacts if 'block_' in r['a'] or 'block_' in r['b']]
    opponent_contacts=[r for r in match_contacts if 'test_opponent' in r['a'] or 'test_opponent' in r['b']]
    pieces=Counter((r['a'] if 'competition_robot' in r['a'] else r['b']).split('::')[-1] for r in block_contacts)
    allowed={'brush','plate_left','plate_right','wall_left','wall_right','wall_rear'}
    violations={k:v for k,v in pieces.items() if k not in allowed}
    pushes=[s for s in states if s['phase'] in ('push','row_push')]
    stop_fraction=sum(math.hypot(*s['command'][:2])<.005 for s in pushes)/max(1,len(pushes))
    ended=[s for s in states if s['phase']=='ended']
    assert ended and all(max(map(abs,s['command']))==0 for s in ended),'到时必须持续零速'
    processing=[r['scene']['processing_ms'] for r in samples if r.get('scene',{}).get('processing_ms') is not None]
    age=[s['scene_age'] for s in states if s.get('scene_age') is not None and math.isfinite(s['scene_age'])]
    stationary_errors=[math.dist(r['truth_robot'],r['scene']['robot'][:2]) for r in samples
                       if r.get('truth_robot') and r.get('scene',{}).get('robot') and r.get('strategy',{}).get('phase')=='guard']
    manifest=json.loads((folder/'source_sha256.json').read_text())
    core={name:manifest['src/sweep_robot/sweep_robot/'+name]==hashlib.sha256((root/'src/sweep_robot/sweep_robot'/name).read_bytes()).hexdigest()
          for name in ('competition_policy.py','row_policy.py','adaptive_controller.py','field_vision.py','hsv_detector.py')}
    core['sim.launch.py']=manifest['src/sweep_robot/launch/sim.launch.py']==hashlib.sha256((root/'src/sweep_robot/launch/sim.launch.py').read_bytes()).hexdigest()
    for name in ('competition_robot/models/competition_robot/build_robot.py','src/sweep_robot/config/gazebo.yaml','src/sweep_robot/test/contact_audit.cpp'):
        core[name]=manifest.get(name)==hashlib.sha256((root/name).read_bytes()).hexdigest()
    result=dict(case=folder.name,own_actual=own,opponent_actual=20-own,clear_observed_at=clear,
                block_contact_parts=dict(pieces),disallowed_block_contacts=violations,
                opponent_contact_samples=len(opponent_contacts),push_zero_fraction=stop_fraction,
                duration=samples[-1]['t'],ended_at=ended[0]['t'],core_source_matches=core,
                processing_ms_median=statistics.median(processing) if processing else None,
                scene_age_median=statistics.median(age) if age else None,
                stationary_error_cm_median=100*statistics.median(stationary_errors) if stationary_errors else None)
    seen=False;first_clear=None
    for row in samples:
        own_seen=row.get('strategy',{}).get('own')
        if own_seen is not None and own_seen>0:seen=True
        if seen and own_seen==0:
            first_clear=row['t'];break
    result['first_visual_clear_at']=first_clear
    result['row_opening_used']=any(s['phase'].startswith('row_') for s in states)
    result['row_events']=list(dict.fromkeys(s['reason'] for s in states if s['phase'].startswith('row_')))
    result['stale_scene_fraction']=sum(a>.35 for a in age)/max(1,len(age))
    xy=[r['truth_robot'] for r in samples if r.get('truth_robot') and r['t']<=90]
    result['travel_m']=sum(math.dist(a,b) for a,b in zip(xy,xy[1:]))
    truth_path=folder/'truth_poses.jsonl'
    truth=read_rows(truth_path) if truth_path.exists() else []
    counts=[(row['t'],sum(p[1]<0 for k,p in row['poses'].items() if k.startswith('block_')))
            for row in truth if len(row['poses'])==21 and row['t']<=90]
    result['first_actual_clear_at']=next((t for t,n in counts if n==0),None)
    last_nonempty=max((t for t,n in counts if n>0),default=0)
    result['stable_actual_clear_from']=next((t for t,n in counts if t>last_nonempty and n==0),None)
    result['score_stamp']=json.loads((folder/'score_stamp.json').read_text()) if (folder/'score_stamp.json').exists() else None
    result['near_midline_blocks']=sum(abs(v[1])<.0175 for k,v in models.items() if k.startswith('block_'))
    if folder.name.endswith('dropout'):
        zeros=[r for r in samples if 12.4<=r['t']<14 and 'command' in r]
        result['dropout_zero_verified']=bool(zeros) and all(max(map(abs,r['command']))==0 for r in zeros)
        resumed=[r['t'] for r in samples if r['t']>=14 and math.hypot(*r.get('command',[0,0])[:2])>.01]
        result['resumed_at']=resumed[0] if resumed else None
        assert result['dropout_zero_verified'],'中断视觉时必须停车'
    return result,samples,models


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--directory',default='log/adaptive_20261021')
    parser.add_argument('--revision',required=True)
    parser.add_argument('--baseline',help='同源码、同默认世界，关闭整排任务的对照目录名')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[3]
    out=(root/args.directory).resolve()
    modes=['seed2','return','cross','block','dropout']
    labels=['Random layout','Return attacker','Crossing opponent','Stationary blocker','2s vision outage']
    summary=[];fig,axes=plt.subplots(1,5,figsize=(17,3.8))
    for mode,label,ax in zip(modes,labels,axes):
        folder=out/f'{args.revision}_{mode}'
        result,samples,models=summarize(folder,root);summary.append(result)
        ax.axhspan(-.6,0,color='#c9def8');ax.axhspan(0,.6,color='#fff3ae')
        xy=[r['truth_robot'] for r in samples if r.get('truth_robot')]
        ax.plot([p[0] for p in xy],[p[1] for p in xy],lw=1.2,color='#2463b4')
        opp=[r['truth_opponent'] for r in samples if r.get('truth_opponent')]
        if opp:ax.plot([p[0] for p in opp],[p[1] for p in opp],lw=.8,color='#666666',ls='--')
        blocks=[p for k,p in models.items() if k.startswith('block_')]
        ax.scatter([p[0] for p in blocks],[p[1] for p in blocks],s=15,c='#ce3232',marker='s',zorder=3)
        ax.axhline(0,color='white',lw=1)
        ax.set(xlim=(-.62,.62),ylim=(-.62,.62),aspect='equal',xlabel='x (m)',
               title=label+f"\nFinal own: {result['own_actual']}/20")
    axes[0].set_ylabel('y (m)')
    fig.tight_layout();fig.savefig(out/'trajectories.png',dpi=160);plt.close(fig)
    extra_labels=[]
    for mode,label in [('return2','连续回推（另一布局种子20261021）'),('seed1','默认场地，无对手')]:
        repeat=out/f'{args.revision}_{mode}'
        if (repeat/'final_poses.json').exists():
            result,_,_=summarize(repeat,root);summary.append(result);extra_labels.append(label)
    baseline=out/(args.baseline or f'{args.revision}_north_only')
    if (baseline/'final_poses.json').exists():
        result,_,_=summarize(baseline,root);summary.append(result);extra_labels.append('同场地关闭横推的逐列对照')
    assert all(all(row['core_source_matches'].values()) for row in summary),'测试快照必须对应当前核心源码'
    assert all(not row['disallowed_block_contacts'] for row in summary),'存在非允许部件碰块，不能作为通过的接触验收'
    early=baseline/'samples.jsonl'
    if early.exists():
        fig,axes=plt.subplots(2,1,figsize=(10,4.8),sharex=True)
        for ax,label,path in zip(axes,['North-only control','Row sweeps + local cleanup'],[early,out/f'{args.revision}_seed1/samples.jsonl']):
            states=[r['strategy'] for r in read_rows(path) if r.get('strategy')]
            ax.plot([s['t'] for s in states],[math.hypot(*s['command'][:2]) for s in states],lw=.8,color='#2463b4')
            ax.set(title=label,ylabel='speed (m/s)',ylim=(-.01,.43))
            ax.grid(alpha=.2)
        axes[-1].set_xlabel('Simulation time (s)')
        fig.tight_layout();fig.savefig(out/'motion_profile.png',dpi=150);plt.close(fig)
    (out/'验证汇总.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    lines=['# 仿真验收汇总（2026-10-21）','',
           f'当前版本：`{args.revision}`。默认采用横向整排开局、沿墙处理和视觉局部清理。对照目录：`{baseline.name}`。','',
           '双方均在90秒截止后停车。独立Gazebo观察器记录20个块和己方车辆的真实位置，以94秒附近完整快照计数，采样时间见JSON。诊断数据不进入比赛控制。实际剩余按块心y<0计，视觉区域数不作为真实木块数；跨中线物块的赛场判罚口径需另行确认，靠近中线的数量见JSON。', '',
           '| 场景 | 己方实际剩余 | 对方实际剩余 | 首次真实清空(s) | 待命开始(s) | 非允许部件碰块采样 |',
           '| --- | ---: | ---: | ---: | ---: | ---: |']
    chinese=['随机布局种子20261021','连续回推','横向拦截','中线右侧堵路','原布局+12至14秒视觉中断']
    chinese+=extra_labels
    for title,row in zip(chinese,summary):
        c=f"{row['clear_observed_at']:.1f}" if row['clear_observed_at'] is not None else '未记录待命'
        first=f"{row['first_actual_clear_at']:.1f}" if row['first_actual_clear_at'] is not None else '未清空'
        lines.append(f"| {title} | {row['own_actual']} | {row['opponent_actual']} | {first} | {c} | {sum(row['disallowed_block_contacts'].values())} |")
    lines += ['', '首次真实清空由连续逐块位置记录确定，待命为策略阶段。之后仍可能被回推，需要再次作业；比赛结束前持续清空的起点另见JSON stable_actual_clear_from。最终比分以94秒快照为准。', '',
              '每场策略与感知核心SHA256均与当前源文件比较，结果见JSON。接触采样不是独立碰撞次数；日志只用于确认接触部件。', '',
              '对手仍会主动撞入，车车接触采样单独记录，不能宣称完全避撞或对任意对手必胜。未知外观、不同尺寸与速度、真实动力学和无线链路不在这组验证内。', '',
              '状态进入ended后全部指令为零；视觉中断12.4至14秒的实际指令须全部为零，恢复时间见JSON。', '',
              '轨迹图中蓝线为己方实际轨迹，灰虚线为脚本对手，红方块为最终真实木块位置。', '',
              '![轨迹](trajectories.png)', '',
              f'本表只使用{args.revision}_*以及指定的同源码逐列对照目录{baseline.name}。其他目录是历史或迭代中间结果，不能混作当前最终版本。每局源码快照和SHA保留作复查。']
    if (out/'motion_profile.png').exists():
        lines += ['', '下图比较相同默认布局下关闭横推与启用横推的指令速度，使用相同版本、传感器和时钟设置。仅关闭整排任务。安全停车保留；速度采样不等同于实车机械顿挫评价。', '', '![速度曲线](motion_profile.png)']
    (out/'验证汇总.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
