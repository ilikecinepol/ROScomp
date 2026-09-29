"""Офлайн-диагностика сохранённой сессии. Нет транспорта или команд движения.

Архив не удостоверяет калибровку. Сопоставление по времени получения служит
только диагностике; профиль из файлов записи никогда не повышает доверие.
"""
from collections import Counter, defaultdict
from fractions import Fraction
import json
import math
from pathlib import Path, PureWindowsPath
import time
import zipfile

import cv2
import numpy as np

from .consistency import ConsistencyMonitor
from .models import Policy
from .perception import PerceptionPipeline, camera_candidates
from .sensors import SensorMonitor, finite, pose_from_message, stamp_seconds


def _inside(root, name):
    if not isinstance(name, str) or not name or ':' in name or '\x00' in name:
        raise ValueError('Недопустимое имя файла записи')
    portable = name.replace('\\', '/')
    if Path(portable).is_absolute() or PureWindowsPath(name).drive or '..' in Path(portable).parts:
        raise ValueError('Выход за каталог записи запрещён: '+name)
    path = (root/portable).resolve()
    if not path.is_relative_to(root):
        raise ValueError('Ссылка выходит за каталог записи: '+name)
    return path


def _json_file(path, limit=16*1024*1024):
    if path.stat().st_size > limit:
        raise ValueError('Файл превышает предел чтения: '+path.name)
    return json.loads(path.read_text(encoding='utf-8-sig'))


def _receive(record):
    for key in ('t', 'elapsed_s', 'received_s', 't_s'):
        if finite(record.get(key)):
            return float(record[key])
    return None


def _source(record, message):
    if 'source_s' in record:
        return float(record['source_s']) if finite(record['source_s']) else None
    if finite(record.get('pts')) and record.get('time_base') is not None:
        try:
            result = float(record['pts'])*float(Fraction(str(record['time_base'])))
            return result if math.isfinite(result) else None
        except (ValueError, ZeroDivisionError, OverflowError):
            return None
    data = message.get('data', message) if isinstance(message, dict) else {}
    if not isinstance(data, dict):
        return None
    header = data.get('header', {})
    return stamp_seconds(data.get('stamp', header.get('stamp') if isinstance(header, dict) else None))


def _clean(value):
    if isinstance(value, dict):
        return {str(key): _clean(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_clean(item) for item in value]
    if isinstance(value, np.generic):
        return _clean(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _timing(samples):
    received = np.asarray([t for t, _ in samples], dtype=float)
    source_pairs = [(t, source) for t, source in samples if finite(source)]
    source = np.asarray([s for _, s in source_pairs], dtype=float)
    dr, ds = np.diff(received), np.diff(source)
    span = float(received[-1]-received[0]) if len(received) > 1 else 0.
    valid_span = source_pairs[-1][0]-source_pairs[0][0] if len(source_pairs) > 1 else 0.
    scale = float((source[-1]-source[0])/valid_span) if valid_span > 0 else None
    return {'saved_samples': len(samples), 'valid_source_stamps': len(source), 'receive_span_s': span,
            'median_receive_period_s': float(np.median(dr[dr > 0])) if np.any(dr > 0) else None,
            'receive_backwards': int(np.sum(dr < 0)), 'source_unique': len(set(source.tolist())),
            'source_duplicates': int(np.sum(ds == 0)), 'source_backwards': int(np.sum(ds < 0)),
            'source_span_s': float(source[-1]-source[0]) if len(source) > 1 else None,
            'source_seconds_per_receive_second': scale,
            'source_scale_anomaly': bool(scale is not None and not .8 <= scale <= 1.2),
            'timing_calibrated': False}


def _nearest(poses, received):
    if received is None or not poses:
        return None, None
    t, pose = min(poses, key=lambda sample: abs(sample[0]-received))
    return pose, t-received


def _point_array(path):
    # Проверка размера до распаковки ограничивает случайную/враждебную zip-bomb.
    with zipfile.ZipFile(path) as archive:
        members = archive.infolist()
        if len(members) > 32 or sum(item.file_size for item in members) > 128*1024*1024:
            raise ValueError('NPZ превышает предел распаковки')
    with np.load(path, allow_pickle=False) as archive:
        keys = [key for key in archive.files if key == 'points' or key.endswith('_points')]
        if len(keys) != 1:
            raise ValueError('Нет единственного native массива points; mesh/байты не переинтерпретируются')
        points = archive[keys[0]]
        if points.ndim != 2 or points.shape[1] != 3 or points.dtype.kind not in 'fiu' or len(points) > 500000:
            raise ValueError('Нужен числовой native Nx3 не более 500000 точек')
        return keys[0], points.copy()


def _point_metadata(value):
    if not isinstance(value, dict):
        return None
    if isinstance(value.get('points'), dict):
        return value['points']
    for child in value.values():
        found = _point_metadata(child)
        if found is not None:
            return found
    return None


def _pixel_registration(first, last):
    def features(image):
        resized = cv2.resize(image, (640, round(image.shape[0]*640/image.shape[1])), interpolation=cv2.INTER_AREA)
        points, descriptors = cv2.ORB_create(nfeatures=600).detectAndCompute(cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY), None)
        return points, descriptors, cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    a, da, gray_a = features(first)
    b, db, gray_b = features(last)
    result = {'diagnostic_only': True, 'body_rotation_deg': None, 'hardware_cause': None,
              'normalized_width_px': 640, 'inliers': 0, 'p95_displacement_px': None,
              'method': 'ORB correspondences, LK subpixel refinement, affine RANSAC'}
    if da is None or db is None:
        return result
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(da, db)
    matches = sorted((m for m in matches if m.distance <= 55), key=lambda m: m.distance)[:350]
    if len(matches) < 12:
        return result
    source = np.float32([a[m.queryIdx].pt for m in matches])
    target = np.float32([b[m.trainIdx].pt for m in matches])
    if gray_a.shape != gray_b.shape:
        return result
    refined, status, _ = cv2.calcOpticalFlowPyrLK(gray_a, gray_b, source.reshape(-1, 1, 2),
        target.reshape(-1, 1, 2).copy(), winSize=(21, 21), maxLevel=3, flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    if refined is None or status is None:
        return result
    keep = status.ravel().astype(bool) & np.isfinite(refined.reshape(-1, 2)).all(axis=1)
    source, target = source[keep], refined.reshape(-1, 2)[keep]
    if len(source) < 12:
        return result
    matrix, mask = cv2.estimateAffinePartial2D(source, target, method=cv2.RANSAC, ransacReprojThreshold=2.)
    if matrix is not None and mask is not None:
        inliers = mask.ravel().astype(bool)
        result['inliers'] = int(inliers.sum())
        if inliers.sum() >= 12:
            result['p95_displacement_px'] = float(np.percentile(np.linalg.norm(target[inliers]-source[inliers], axis=1), 95))
            result['image_affine_rotation_deg'] = float(math.degrees(math.atan2(matrix[1, 0], matrix[0, 0])))
    return result


def replay_capture(capture_path) -> dict:
    """Читает только явный каталог, возвращает JSON без записей и motion API."""
    started = time.perf_counter()
    root = Path(capture_path).expanduser().resolve()
    if not root.is_dir():
        raise ValueError('Нужен существующий каталог записи')
    summary_path = _inside(root, 'summary.json')
    summary = _json_file(summary_path) if summary_path.is_file() else {}
    if not isinstance(summary, dict):
        raise ValueError('summary.json должен содержать объект')
    warnings, errors, records = [], [], []
    sample_path = _inside(root, 'samples.jsonl')
    if not sample_path.is_file():
        sample_path = _inside(root, 'sensor_capture.jsonl')
    if sample_path.is_file():
        if sample_path.stat().st_size > 128*1024*1024:
            raise ValueError('JSONL превышает предел чтения')
        with sample_path.open(encoding='utf-8-sig') as source:
            for index, line in enumerate(source, 1):
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                    if not isinstance(record, dict):
                        raise ValueError('нужен JSON объект')
                    records.append(record)
                except (ValueError, TypeError) as exc:
                    errors.append(f'{sample_path.name}:{index}: {exc}')
    else:
        errors.append('Нет samples.jsonl или sensor_capture.jsonl')

    npz_refs, camera_refs = {}, {}
    npz_files = {p.name for p in root.glob('*.npz')}
    camera_files = {p.name for pattern in ('*.jpg', '*.jpeg', '*.JPG') for p in root.glob(pattern)}
    for key, collection in (('lidar_files', npz_files), ('camera_files', camera_files)):
        declared = summary.get(key, [])
        if not isinstance(declared, list):
            raise ValueError('Некорректный manifest '+key)
        for name in declared:
            _inside(root, name)
            collection.add(name)
    for record in records:
        if record.get('npz_file') is not None:
            name = record['npz_file']
            _inside(root, name)
            npz_files.add(name)
            if name in npz_refs:
                errors.append('Повторная неоднозначная привязка NPZ: '+name)
                npz_refs[name] = None
            else:
                npz_refs[name] = record
        if record.get('image_file') is not None:
            name = record['image_file']
            _inside(root, name)
            camera_files.add(name)
            camera_refs[name] = record
    for record in summary.get('camera_records', []):
        if not isinstance(record, dict) or not isinstance(record.get('file'), str):
            errors.append('Некорректная запись camera_records')
            continue
        name = record['file']
        _inside(root, name)
        camera_files.add(name)
        if name not in camera_refs:
            camera_refs[name] = {**record, 'topic': 'CAMERA', 'image_file': name}
            records.append(camera_refs[name])

    monitor, timing, yaw, poses = SensorMonitor(), defaultdict(list), defaultdict(list), []
    count = Counter()
    for record in records:
        name = record.get('topic')
        if not isinstance(name, str):
            errors.append('В записи отсутствует topic')
            continue
        count[name] += 1
        received = _receive(record)
        if received is None:
            errors.append('Нет конечного receive time: '+name)
            continue
        message = record.get('message', {})
        source = _source(record, message)
        timing[name].append((received, source))
        try:
            monitor.ingest(name, message, received, source)
            data = message.get('data', message) if isinstance(message, dict) else {}
            if name == 'ROBOTODOM':
                pose = pose_from_message(message)
                poses.append((received, pose))
                yaw[name].append((received, pose.yaw))
            elif name in ('LF_SPORT_MOD_STATE', 'LOW_STATE'):
                value = data.get('imu_state', {}).get('rpy', [None, None, None])[2]
                if finite(value):
                    yaw[name].append((received, float(value)))
        except (ValueError, TypeError, AttributeError, IndexError, KeyError) as exc:
            errors.append(name+': '+str(exc))
    poses.sort(key=lambda row: row[0])
    now = max((t for samples in timing.values() for t, _ in samples), default=0.)
    timings = {name: _timing(samples) for name, samples in timing.items()}
    yaw_report = {}
    for name, samples in yaw.items():
        angles = np.unwrap([angle for _, angle in samples])
        yaw_report[name] = {'samples': len(samples), 'receive_span_s': samples[-1][0]-samples[0][0],
                            'yaw_delta_deg': float(np.degrees(angles[-1]-angles[0])), 'quaternion_order': 'named_xyzw' if name == 'ROBOTODOM' else 'rpy_yaw'}

    pipeline, clouds, cameras, missing = PerceptionPipeline(), [], [], []
    for name in sorted(npz_files):
        path = _inside(root, name)
        if not path.is_file():
            missing.append(name)
            continue
        record = npz_refs.get(name)
        received = _receive(record) if record else None
        item = {'file': name, 'metadata_pairing': 'explicit_npz_file' if record else 'unpaired_no_order_guess',
                'receive_t': received, 'metric_walk_blocked': True, 'geometry_contract_verified': False}
        try:
            key, points = _point_array(path)
            metadata = _point_metadata(record.get('message', {})) if record else None
            shape = metadata.get('array_shape', metadata.get('shape')) if metadata else None
            if shape is not None and list(shape) != list(points.shape):
                raise ValueError('Форма NPZ не совпадает с явно привязанными метаданными')
            valid = np.isfinite(points).all(axis=1)
            item.update(array_key=key, point_count=len(points), dtype=str(points.dtype), finite_points=int(valid.sum()),
                        bounds_reported_units=np.column_stack([points[valid].min(axis=0), points[valid].max(axis=0)]).tolist() if valid.any() else None,
                        metric_units_verified=False, semantics='native occupied/surface points; не сырые лучи')
            pose, gap = _nearest(poses, received)
            item['nearest_pose_receive_delta_s'] = gap
            observation = pipeline.build_observation(received if received is not None else 0., pose, points, geometry_contract=None)
            item.update(localized=observation.localized, geometry_candidates=len(observation.obstacles), diagnostics=observation.diagnostics)
        except (ValueError, OSError, zipfile.BadZipFile, KeyError) as exc:
            item['error'] = str(exc)
            errors.append(name+': '+str(exc))
        clouds.append(item)

    consistency = ConsistencyMonitor()
    first_image = last_image = None
    for name in sorted(camera_files, key=lambda name: (_receive(camera_refs.get(name, {})) is None, _receive(camera_refs.get(name, {})) or 0., name)):
        path = _inside(root, name)
        if not path.is_file():
            missing.append(name)
            continue
        record = camera_refs.get(name, {})
        received = _receive(record)
        item = {'file': name, 'receive_t': received, 'source_s': _source(record, {}), 'metric_target': False}
        if path.stat().st_size > 32*1024*1024:
            item['error'] = 'Кадр превышает предел чтения'
            errors.append(name+': '+item['error'])
            cameras.append(item)
            continue
        image = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[0]*image.shape[1] > 16000000:
            item['error'] = 'Не удалось прочитать ограниченный BGR кадр'
            errors.append(name+': '+item['error'])
            cameras.append(item)
            continue
        item['shape'] = list(image.shape)
        item['pixel_candidates'] = camera_candidates(image)
        pose, gap = _nearest(poses, received)
        item['nearest_pose_receive_delta_s'] = gap
        if pose is not None and gap is not None and abs(gap) <= .25:
            consistency.observe(image, pose, received)
            item['consistency'] = consistency.report
            if first_image is None:
                first_image = image.copy()
            last_image = image
        else:
            item['consistency_skipped'] = 'Нет позы в пределах 0.25 с по приёму; это не синхронизация измерений'
        cameras.append(item)
    for name, value in timings.items():
        if value['source_scale_anomaly']:
            warnings.append('Масштаб source/receive отличается от 1: '+name)
        if value['source_duplicates']:
            warnings.append('Повторяются source timestamps: '+name)
    if any(camera.get('receive_t') is None for camera in cameras):
        warnings.append('У части кадров нет receive time; синхронизация по порядку файлов не предполагается')
    report = {'schema_version': 1, 'capture_path': str(root), 'read_only': True, 'motion_authorized': False,
              'metric_walk_blocked': True, 'geometry_contract_verified': False, 'status': 'incomplete_capture' if missing or errors else 'diagnostic_only',
              'files': {'npz_count': len(npz_files), 'npz_processed': len(clouds), 'jpg_count': len(camera_files), 'images_processed': len(cameras), 'missing': missing},
              'saved_record_counts': dict(count), 'capture_reported_counts': summary.get('counts', {}),
              'timing': timings, 'yaw': yaw_report, 'sensor_monitor': monitor.report(now),
              'health_on_saved_subset': monitor.health(now, Policy()), 'pointclouds': clouds, 'camera': cameras,
              'consistency_final': consistency.report,
              'first_last_camera_registration': _pixel_registration(first_image, last_image) if first_image is not None and last_image is not None else None,
              'warnings': warnings, 'errors': errors,
              'limitations': ['Receive-nearest — диагностическое сопоставление, не временная калибровка.',
                 'Редко сохранённые кадры могут сбросить окно ConsistencyMonitor; пропущенные кадры не восстанавливаются.',
                 'Health отражает сохранённый поднабор, а не полное состояние живого потока.',
                 'Границы массива приведены в сообщённых единицах; метрическая система и разрешение движения не подтверждены.',
                 'Причина расхождения yaw/видео и неподвижных timestamps этим отчётом не устанавливается.'],
              'runtime_s': time.perf_counter()-started}
    return _clean(report)
