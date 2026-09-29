"""Сопоставление команд и телеметрии записи; не управляет роботом."""
import argparse
import json
import math
from collections import Counter
from pathlib import Path
import numpy as np


def rows(path):
    with path.open(encoding='utf-8') as f:
        return [json.loads(line) for line in f if line.strip()]


def audit(folder):
    events = rows(folder/'events.jsonl')
    telemetry = rows(folder/'telemetry.jsonl')
    camera = rows(folder/'camera.jsonl')
    by_topic = {}
    for row in telemetry:
        by_topic.setdefault(row['topic'], []).append(row)
    odom = by_topic.get('ROBOTODOM', [])
    poses = []
    for row in odom:
        p = row['message']['data']['pose']; q = p['orientation']
        yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
        poses.append([row['elapsed_s'], p['position']['x'], p['position']['y'], yaw])
    poses = np.array(poses)
    moves = [e for e in events if e.get('event')=='command_sent' and e.get('command')=='Move']
    stops = [e for e in events if e.get('event')=='command_sent' and e.get('command')=='StopMove']
    groups = []
    for e in moves:
        if not groups or e['elapsed_s']-groups[-1][-1]['elapsed_s']>.4 or e['parameter']!=groups[-1][-1]['parameter']:
            groups.append([])
        groups[-1].append(e)
    segments = []
    for group in groups:
        a,b=group[0]['elapsed_s'],group[-1]['elapsed_s']
        if b-a<.5 or not len(poses): continue
        # Последняя команда могла действовать до StopMove, максимум heartbeat .35 с.
        after=[s['elapsed_s'] for s in stops if b<=s['elapsed_s']<=b+.35]
        end=min(after) if after else b+.1
        i=int(np.argmin(abs(poses[:,0]-a))); j=int(np.argmin(abs(poses[:,0]-end)))
        x,y=poses[j,1:3]-poses[i,1:3]; yaw=poses[i,3]
        forward=x*math.cos(yaw)+y*math.sin(yaw)
        lateral=-x*math.sin(yaw)+y*math.cos(yaw)
        dyaw=math.atan2(math.sin(poses[j,3]-yaw), math.cos(poses[j,3]-yaw))
        segments.append({'start_s':a,'end_s':end,'duration_s':end-a,'command':group[0]['parameter'],
                         'odom_forward_m':forward,'odom_lateral_m':lateral,'odom_yaw_change_deg':math.degrees(dyaw),
                         'odom_distance_m':math.hypot(x,y)})
    rates={k:round((len(v)-1)/(v[-1]['elapsed_s']-v[0]['elapsed_s']),2) for k,v in by_topic.items() if len(v)>1}
    stamps={k:len({json.dumps(r['message']['data'].get('stamp',r['message']['data'].get('header',{}).get('stamp')),sort_keys=True) for r in v}) for k,v in by_topic.items()}
    result={'scope':'recorded_commands_vs_reported_odometry_not_physical_calibration',
            'session':folder.name,'move_requests':len(moves),'segments':segments,'observed_receive_hz':rates,
            'unique_timestamps':stamps,'stop_reasons':dict(Counter(e.get('reason') for e in events if e.get('event')=='stop_requested')),
            'autonomous_motion_ready':False}
    if len(poses):
        result['odom_bounds_xy_m']={'min':poses[:,1:3].min(axis=0).tolist(),'max':poses[:,1:3].max(axis=0).tolist()}
    if len(camera)>1:
        from fractions import Fraction
        result['camera_receive_span_s']=camera[-1]['elapsed_s']-camera[0]['elapsed_s']
        result['camera_pts_span_s']=float(camera[-1]['pts']*Fraction(camera[-1]['time_base'])-camera[0]['pts']*Fraction(camera[0]['time_base']))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);args=p.parse_args()
    report=audit(args.folder)
    (args.folder/'motion-audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
