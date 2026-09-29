"""Сравнение возвратов около корпуса без изменения карты и разрешения движения."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def read_frame(path):
    meta = json.loads(path.read_text(encoding='utf-8-sig'))
    pose = meta['pose']
    resolution = float(meta['resolution'])
    with np.load(path.with_suffix('.npz'), allow_pickle=False) as data:
        world = np.asarray(data['points'], dtype=float)
    world = world[np.isfinite(world).all(axis=1)]
    # Сравниваем центры вокселей, а не гипотетические поверхности опоры.
    world = world + resolution / 2
    delta = world - [pose['x'], pose['y'], pose['z']]
    co, si = np.cos(pose['yaw']), np.sin(pose['yaw'])
    body = np.column_stack((co*delta[:, 0]+si*delta[:, 1],
                            -si*delta[:, 0]+co*delta[:, 1], delta[:, 2]))
    # Только возвраты выше центра корпуса: пол не должен давать ложное сходство.
    selected = ((np.abs(body[:, 0]) < .5) & (np.abs(body[:, 1]) < .35)
                & (body[:, 2] > .1) & (body[:, 2] < 1.2))
    return meta, world[selected], body[selected]


def similarity(a, b, tolerance=.09):
    if len(a) == 0 or len(b) == 0:
        return None
    ab = cKDTree(b).query(a)[0]
    ba = cKDTree(a).query(b)[0]
    return {'fraction_within_9cm': float((np.mean(ab <= tolerance)+np.mean(ba <= tolerance))/2),
            'median_distance_m': float(np.median(np.concatenate((ab, ba))))}


def compare(paths):
    frames = [read_frame(p) for p in paths]
    rows = []
    for i, (ma, wa, ba) in enumerate(frames):
        for j in range(i+1, len(frames)):
            mb, wb, bb = frames[j]
            same_map = (ma['pi_boot_id'] == mb['pi_boot_id'] and
                        ma['frame_epoch'] == mb['frame_epoch'] and
                        ma['point_frame'] == mb['point_frame'])
            distance = float(np.hypot(ma['pose']['x']-mb['pose']['x'],
                                      ma['pose']['y']-mb['pose']['y'])) if same_map else None
            rows.append({'a': i, 'b': j, 'same_map': same_map,
                         'pose_distance_m': distance,
                         'independent_position': distance is not None and distance > .4,
                         'body': similarity(ba, bb),
                         'world': similarity(wa, wb) if same_map else None})
    return {'note': 'Сходство не доказывает собственное отражение. Привязка и время съёмки не проверены; фильтр точек не создаётся.',
            'frames': [{'path': str(p), 'pose': m['pose'], 'elevated_points': len(b)}
                       for p, (m, w, b) in zip(paths, frames)],
            'pairs': rows, 'map_modified': False, 'motion_authorized': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frames', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = compare(args.frames)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
