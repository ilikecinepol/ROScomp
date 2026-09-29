"""Ограниченная локальная проверка детектора; не оценка качества на новых сценах."""
import sys
import json
import hashlib
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'work/go2-perception'))
from ramp_vision import detect_ramp, render_overlay

OUT = Path(__file__).resolve().parent
source = ROOT / 'outputs/go2-remote-setup/logs/20260916T105034Z'
cases = []
# Приблизительная ручная разметка только для оценки этих кадров; детектор
# не читает эту разметку и не содержит координат этого объекта.
reference_quad = np.array([[296., 336.], [425., 337.], [337., 521.], [175., 505.]], dtype=np.float32)
for path in sorted(source.glob('camera_*.jpg')):
    frame = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    cases.append((path.stem, frame, reference_quad, 'исходный кадр того же неподвижного вида'))
frame = cases[0][1]
cases.extend([
    ('mirror', cv2.flip(frame, 1), reference_quad * [-1, 1] + [frame.shape[1] - 1, 0], 'горизонтальное отражение'),
    ('brightness_plus', np.clip(frame.astype(float) * 1.15 + 20, 0, 255).astype(np.uint8), reference_quad, 'яркость ×1,15 +20'),
    ('brightness_minus', np.clip(frame.astype(float) * .72, 0, 255).astype(np.uint8), reference_quad, 'яркость ×0,72'),
    ('half_resolution', cv2.resize(frame, None, fx=.5, fy=.5), reference_quad * .5, 'уменьшение разрешения вдвое'),
    ('translation', cv2.warpAffine(frame, np.float32([[1,0,150],[0,1,40]]), (frame.shape[1],frame.shape[0]), borderValue=(160,160,160)), reference_quad + [150,40], 'перенос содержимого кадра вправо и вниз'),
    ('negative_wall', frame[120:610, 785:1260].copy(), None, 'фрагмент стены без рампы'),
    ('negative_mat', frame[451:543, 394:750].copy(), None, 'фрагмент правого мата без рампы'),
    ('negative_ceiling', frame[0:235, 0:625].copy(), None, 'фрагмент потолка без рампы'),
])
results = []
for name, image, reference, note in cases:
    result = detect_ramp(image)
    expected = reference is not None
    success = (len(result['candidates']) == 1) if expected else (len(result['candidates']) == 0)
    iou = None
    entrance_error = None
    if expected and result['candidates']:
        quad = np.array(result['candidates'][0]['quadrilateral_px'], dtype=np.float32)
        reference = np.array(reference, dtype=np.float32)
        intersection, _ = cv2.intersectConvexConvex(quad, reference)
        iou = float(intersection / (abs(cv2.contourArea(quad)) + abs(cv2.contourArea(reference)) - intersection))
        entrance_error = float(np.linalg.norm(quad[2:].mean(axis=0) - reference[2:].mean(axis=0)))
        success = success and iou > .75 and entrance_error < image.shape[1] * .025
    entry = {'case': name, 'expected_candidate': expected, 'found_candidates': len(result['candidates']), 'pass': success, 'manual_quad_iou': iou, 'entrance_center_error_px': entrance_error, 'note': note, 'result': result}
    results.append(entry)
    ok, encoded = cv2.imencode('.jpg', render_overlay(image, result))
    assert ok
    encoded.tofile(str(OUT / f'vision-{name}.jpg'))
    print(name, len(result['candidates']), success, 'IoU', iou, 'entrance_error_px', entrance_error)
report = {'status': 'passed' if all(x['pass'] for x in results) else 'failed', 'passed': sum(x['pass'] for x in results), 'total': len(results),
    'detector_sha256': hashlib.sha256((ROOT / 'work/go2-perception/ramp_vision.py').read_bytes()).hexdigest(),
    'scope': 'Пять почти одинаковых кадров одного вида; отражение и изменение яркости не независимые сцены. Отрицательные примеры — локальные вырезки. Нет оценки точности на других ракурсах или объектах.',
    'cases': results}
(OUT / 'vision-validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
if report['status'] != 'passed':
    raise SystemExit(1)
