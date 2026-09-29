"""Повторная обработка записанных датчиков. Соединения и управления роботом нет."""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import scipy

from ramp_vision import detect_ramp, render_overlay
from voxel_geometry import analyze_voxels, load_capture_pairs
from calibration_contract import camera_detection_to_metric_target


def _thermal():
    """Температура процессора, если Linux предоставляет её без привилегий."""
    path = Path('/sys/class/thermal/thermal_zone0/temp')
    return float(path.read_text()) / 1000 if path.exists() else None


def process_capture(capture: Path, output: Path, camera_width: int, allow_legacy_order=False):
    output.mkdir(parents=True, exist_ok=True)
    summary = json.loads((capture / 'summary.json').read_text(encoding='utf-8'))
    result = {'capture': capture.name, 'camera': [], 'voxel': [], 'errors': [],
              'actuation': False, 'camera_lidar_fusion': False,
              'sensor_clock_synchronization_verified': False,
              'capture_observed_hz': summary.get('observed_hz'),
              'camera_input_width': camera_width or 'original',
              'notes': ['Работа только с записью. Время обработки не является задержкой живого управления.',
                        'Поза подобрана по времени приёма среди сохранённых сообщений; это не синхронизация датчиков.',
                        'Кандидаты камеры и карты не сопоставлены друг с другом; подход не разрешён.']}
    # Неполную запись отвергаем целиком: индексы нельзя незаметно сдвигать.
    for key, pattern in [('camera_files', 'camera_*.jpg'), ('lidar_files', 'lidar_*.npz')]:
        expected = summary.get(key, [])
        actual = sorted(p.name for p in capture.glob(pattern))
        if not expected or sorted(expected) != actual or any(Path(name).name != name for name in expected):
            result['errors'].append(f'{key}: отсутствуют файлы либо набор не совпадает с summary')
    if result['errors']:
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        return result
    try:
        pairs = load_capture_pairs(capture, allow_legacy_order=allow_legacy_order)
    except (ValueError, KeyError, IndexError) as exc:
        result['errors'].append(f'Привязка записи: {exc}')
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        return result
    for filename in summary['camera_files']:
        file = capture / filename
        frame = cv2.imread(str(file))
        if frame is None:
            result['errors'].append(f'Не прочитан кадр {file.name}')
            continue
        if camera_width and frame.shape[1] > camera_width:
            frame = cv2.resize(frame, (camera_width, round(frame.shape[0] * camera_width / frame.shape[1])), interpolation=cv2.INTER_AREA)
        start = time.perf_counter()
        detection = detect_ramp(frame)
        elapsed = (time.perf_counter() - start) * 1000
        detection['observation_id'] = f'{capture.name}/{file.name}'
        # Подтверждённой RGB-калибровки и плоскости сейчас нет.
        # Пиксельный вход не должен выглядеть готовой точкой управления.
        detection['metric_target'] = camera_detection_to_metric_target(
            detection, None, image_size=(frame.shape[1], frame.shape[0]),
            observation_id=detection['observation_id'])
        (output / f'{file.stem}.json').write_text(json.dumps(detection, ensure_ascii=False, indent=2), encoding='utf-8')
        cv2.imwrite(str(output / f'{file.stem}-overlay.jpg'), render_overlay(frame, detection))
        result['camera'].append({'file': file.name, 'processing_ms_excludes_io_overlay': elapsed,
                                 'candidate_count': len(detection['candidates']),
                                 'metric_target_status': detection['metric_target']['status'],
                                 'image_size': detection['image_size']})
    for pair in pairs:
        file, record, pose = pair['file'], pair['metadata'], pair['pose']
        try:
            start = time.perf_counter()
            detection = analyze_voxels(file, record, pose)
            elapsed = (time.perf_counter() - start) * 1000
            detection['capture_sync'] = {
                'metadata_binding': pair['pairing'],
                'sensor_clock_synchronization_verified': False,
                'receive_time_difference_s': pose['elapsed_s'] - record['elapsed_s'],
                'selection': 'nearest_saved_pose_by_receive_time_only'}
            (output / f'{file.stem}.json').write_text(json.dumps(detection, ensure_ascii=False, indent=2), encoding='utf-8')
            result['voxel'].append({'file': file.name, 'processing_ms_includes_npz_load': elapsed,
                                    'candidate_count': len(detection['inclined_planes'])})
        except Exception as exc:
            result['errors'].append(f'{file.name}: {type(exc).__name__}: {exc}')
    for key, timing in [('camera', 'processing_ms_excludes_io_overlay'), ('voxel', 'processing_ms_includes_npz_load')]:
        values = [r[timing] for r in result[key]]
        result[key + '_timing_ms'] = {'mean': float(np.mean(values)), 'max': float(np.max(values)), 'n': len(values)} if values else None
    (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera-width', type=int, default=0)
    parser.add_argument('--allow-legacy-order', action='store_true', help='Разрешить проверенные полные записи старого формата без npz_file')
    args = parser.parse_args()
    if args.camera_width and args.camera_width < 320:
        parser.error('Ширина должна быть 0 (оригинал) либо не меньше 320')
    cv2.setNumThreads(1)
    report = {'platform': platform.platform(), 'python': sys.version,
              'versions': {'numpy': np.__version__, 'opencv': cv2.__version__, 'scipy': scipy.__version__},
              'opencv_threads': cv2.getNumThreads(), 'cpu_temperature_start_c': _thermal(),
              'code_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')},
              'captures': []}
    args.output.mkdir(parents=True, exist_ok=True)
    for capture in args.capture:
        report['captures'].append(process_capture(capture, args.output / capture.name, args.camera_width, args.allow_legacy_order))
    report['cpu_temperature_end_c'] = _thermal()
    (args.output / 'benchmark.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if any(c['errors'] for c in report['captures']) else 0


if __name__ == '__main__':
    raise SystemExit(main())
