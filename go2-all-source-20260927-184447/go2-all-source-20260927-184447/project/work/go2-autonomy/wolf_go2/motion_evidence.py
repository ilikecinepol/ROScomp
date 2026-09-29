"""Разбор отдельного импульса: устойчивое смещение после остановки, не ACK."""
import hashlib
import json
import math
import statistics
from pathlib import Path


def settled_pose(rows, phase):
    samples = []
    for row in rows:
        if row.get('topic') != 'ROBOTODOM' or row.get('phase') != phase:
            continue
        try:
            pose = row['message']['data']['pose']
            p, q = pose['position'], pose['orientation']
            yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
            values = (row['elapsed_s'], p['x'], p['y'], yaw)
            if all(math.isfinite(v) for v in values): samples.append(values)
        except (KeyError, TypeError, ValueError):
            continue
    if not samples:
        raise ValueError('Нет координат фазы '+phase)
    end = max(s[0] for s in samples)
    tail = [s for s in samples if end-1 <= s[0] <= end]
    if len(tail) < 5 or max(s[0] for s in tail)-min(s[0] for s in tail) < .5:
        raise ValueError('Слишком короткий участок координат '+phase)
    yaw = math.atan2(sum(math.sin(s[3]) for s in tail), sum(math.cos(s[3]) for s in tail))
    center = (statistics.median(s[1] for s in tail), statistics.median(s[2] for s in tail), yaw)
    spread = max(math.dist(center[:2], s[1:3]) for s in tail)
    return center, spread


def analyze_pulse(directory):
    directory = Path(directory)
    summary_path, samples_path = directory/'summary.json', directory/'samples.jsonl'
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    rows = [json.loads(s) for s in samples_path.read_text(encoding='utf-8').splitlines() if s.strip()]
    before, before_spread = settled_pose(rows, 'before')
    after, after_spread = settled_pose(rows, 'after')
    dx, dy = after[0]-before[0], after[1]-before[1]
    co, si = math.cos(before[2]), math.sin(before[2])
    forward, left = co*dx+si*dy, -si*dx+co*dy
    yaw = math.atan2(math.sin(after[2]-before[2]), math.cos(after[2]-before[2]))
    plan = summary['plan']
    vx, vy = plan['vx_m_s'], plan['vy_m_s']
    commanded = math.hypot(vx, vy)*plan['duration_s']
    along = (forward*vx+left*vy)/max(math.hypot(vx, vy), 1e-9)
    stopped = summary.get('final_stop_acknowledged') is True and summary.get('pulse',{}).get('stop_acknowledged') is True
    stable = max(before_spread, after_spread) <= .03
    # Это критерий конкретного прямолинейного импульса, не нижняя гарантированная
    # скорость и не проверка локализации или всех направлений движения.
    response = (commanded > 0 and along >= max(.05, commanded*.25) and stable
                and stopped and not summary.get('errors'))
    expected_yaw = plan.get('vyaw_rad_s', 0)*plan['duration_s']
    rotation = (abs(expected_yaw) > 0 and yaw*math.copysign(1,expected_yaw) >= max(.05,abs(expected_yaw)*.25)
                and stable and stopped and not summary.get('errors'))
    return {'scope':'single_motion_pulse', 'robot_profile_verified':False,
            'plan':plan, 'net_forward_m':forward, 'net_left_m':left, 'net_yaw_rad':yaw,
            'settled_position_spread_m':max(before_spread,after_spread),
            'stop_acknowledged':stopped, 'translation_observed':response,
            'rotation_observed':rotation,
            'errors':summary.get('errors',[]),
            'evidence_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (summary_path,samples_path)},
            'limitation':'Одно измерение одометрии после остановки; не подтверждает другие скорости, направления и препятствия'}
