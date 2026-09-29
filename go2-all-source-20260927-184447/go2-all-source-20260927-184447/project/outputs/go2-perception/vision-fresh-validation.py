"""Проверка новой записи того же вида без перенастройки детектора."""
import sys
import json
import hashlib
import time
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'work/go2-perception'))
from ramp_vision import detect_ramp, render_overlay

OUT = Path(__file__).resolve().parent
source = ROOT / 'outputs/go2-remote-setup/logs/20260916T110043Z'
module_sha256 = hashlib.sha256((ROOT / 'work/go2-perception/ramp_vision.py').read_bytes()).hexdigest()
previous = json.loads((OUT / 'vision-validation.json').read_text(encoding='utf-8'))
assert module_sha256 == previous['detector_sha256'], 'Модуль изменился после первой серии проверок'
reference = np.array([[296.,336.],[425.,337.],[337.,521.],[175.,505.]], dtype=np.float32)
images = []
for path in sorted(source.glob('camera_*.jpg')):
    images.append((path, cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_COLOR)))
assert len(images) == 5
results = []
durations = []
for path, frame in images:
    started = time.perf_counter()
    result = detect_ramp(frame)
    elapsed_ms = (time.perf_counter()-started)*1000
    durations.append(elapsed_ms)
    iou = None
    error = None
    passed = len(result['candidates']) == 1
    if result['candidates']:
        quad = np.array(result['candidates'][0]['quadrilateral_px'],dtype=np.float32)
        intersection, _ = cv2.intersectConvexConvex(quad, reference)
        iou = float(intersection/(abs(cv2.contourArea(quad))+abs(cv2.contourArea(reference))-intersection))
        error = float(np.linalg.norm(quad[2:].mean(axis=0)-reference[2:].mean(axis=0)))
        passed = passed and iou > .75 and error < frame.shape[1]*.025
    results.append({'file': str(path.relative_to(ROOT)), 'file_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'passed': passed, 'manual_quad_iou': iou, 'entrance_center_error_px': error, 'detection_elapsed_ms': elapsed_ms, 'result': result})
    encoded = cv2.imencode('.jpg',render_overlay(frame,result))[1]
    encoded.tofile(str(OUT/f'vision-fresh-{path.stem}.jpg'))
    print(path.name, 'candidates', len(result['candidates']), 'passed', passed, 'IoU', iou, 'ms', elapsed_ms)
report = {'status': 'passed' if all(x['passed'] for x in results) else 'failed', 'passed': sum(x['passed'] for x in results), 'total': len(results),
    'detector_sha256': module_sha256, 'unchanged_since_initial_validation': True,
    'scope': 'Новая запись того же неподвижного вида. Это не независимая сцена и не новый ракурс. Ручная разметка приблизительна и перенесена из первой серии.',
    'local_timing': {'device': 'локальный Windows ПК, не Raspberry Pi', 'opencv_version': cv2.__version__, 'frames': len(durations),
        'mean_ms': float(np.mean(durations)), 'max_ms': float(np.max(durations)), 'min_ms': float(np.min(durations)),
        'includes': 'Только detect_ramp; чтение JPG, разметка и запись файлов исключены. Включён первый вызов в процессе; это короткая последовательная выборка.'},
    'cases': results}
(OUT/'vision-fresh-validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'status':report['status'],'local_timing':report['local_timing']},ensure_ascii=False))
if report['status'] != 'passed':
    raise SystemExit(1)
