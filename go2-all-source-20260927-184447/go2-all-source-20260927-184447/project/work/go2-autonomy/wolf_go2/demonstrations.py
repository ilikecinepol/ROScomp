"""Импорт ручных примеров. Данные не разрешают автоматическое движение."""
from pathlib import Path
import hashlib
import json
import math


def read_rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8-sig').splitlines() if line.strip()]


def import_run(root, session, run):
    base = Path(root).resolve()
    folder = (base / session / run).resolve()
    if not folder.is_relative_to(base):
        raise ValueError('Фрагмент вне каталога записей')
    required = [folder / name for name in ('recording.json', 'camera.jsonl', 'telemetry.jsonl')]
    required.append(folder.parent / 'events.jsonl')
    if not all(p.is_file() for p in required):
        raise ValueError('Фрагмент недокачан: ' + str(folder))
    meta = json.loads(required[0].read_text(encoding='utf-8'))
    if meta.get('complete') is not True:
        raise ValueError('Запись не завершена')
    camera = read_rows(required[1])
    telemetry = read_rows(required[2])
    events = read_rows(required[3])
    if len(camera) < 2 or not telemetry:
        raise ValueError('Недостаточно кадров или телеметрии')
    for c in camera:
        if not math.isfinite(c['elapsed_s']):
            raise ValueError('Некорректное время кадра')
        target = (folder / c['file']).resolve()
        if not target.is_relative_to(folder) or not target.is_file():
            raise ValueError('Нет кадра или путь выходит из фрагмента')
    a, b = camera[0]['elapsed_s'], camera[-1]['elapsed_s']
    if not math.isfinite(a) or not math.isfinite(b) or b <= a:
        raise ValueError('Некорректное время записи')
    if any(y['elapsed_s'] <= x['elapsed_s'] for x, y in zip(camera, camera[1:])):
        raise ValueError('Нарушен порядок кадров')
    poses = []
    states = []
    for t in telemetry:
        if not a <= t['elapsed_s'] <= b:
            continue
        d = t['message']['data']
        if t['topic'] == 'ROBOTODOM':
            p = d['pose']['position']; q = d['pose']['orientation']
            yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
            values = [t['elapsed_s']-a, p['x'], p['y'], p['z'], yaw]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('Нечисловая одометрия')
            poses.append(values)
        if t['topic'] == 'LF_SPORT_MOD_STATE':
            states.append({'t': t['elapsed_s']-a, 'velocity': d.get('velocity'),
                'rpy': d.get('imu_state', {}).get('rpy'), 'mode': d.get('mode'),
                'gait_type': d.get('gait_type')})
    if not poses or not states:
        raise ValueError('Нет одометрии или состояния движения')
    origin = poses[0]
    co, si = math.cos(origin[4]), math.sin(origin[4])
    relative = []
    for t, x, y, z, yaw in poses:
        dx, dy = x-origin[1], y-origin[2]
        relative.append([t, co*dx+si*dy, -si*dx+co*dy, z-origin[3],
                         math.atan2(math.sin(yaw-origin[4]), math.cos(yaw-origin[4]))])
    selected = [e for e in events if a <= e['elapsed_s'] <= b and e.get('event') in
                ('marker', 'command_sent', 'profile_changed', 'stop_requested')]
    active_profile = None
    for e in events:
        if e['elapsed_s'] > a:
            break
        if e.get('event') in ('record_start', 'profile_changed'):
            active_profile = e.get('profile', active_profile)
    return {'session': session, 'run': run, 'duration_s': b-a,
        'session_start_s': a, 'profile_at_start': active_profile,
        'reference_only': True, 'autonomous_skill_validated': False,
        'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in required},
        'pose_columns': ['t_s', 'forward_m', 'left_m', 'height_delta_m', 'yaw_rad'],
        'coordinate_warning': 'Локальная одометрия фрагмента; дрейф и физическая точность не проверены. Фрагменты не сшиты.',
        'poses': relative, 'states': states,
        'events': [{**e, 't': e['elapsed_s']-a} for e in selected],
        'frames': [{'t': c['elapsed_s']-a, 'file': c['file']} for c in camera]}


def build_library(root, manifest):
    items = manifest.get('examples', [])
    if not items:
        raise ValueError('Нет примеров')
    examples = []
    for item in items:
        if item['kind'] not in ('slalom', 'aframe', 'teeter', 'platforms'):
            raise ValueError('Неизвестный тип препятствия')
        examples.append({**item, 'segments': [import_run(root, x['session'], x['run'])
                                              for x in item['segments']]})
    return {'schema_version': 1, 'purpose': 'offline_reference_not_motion_replay',
        'autonomous_ready': False, 'examples': examples}
