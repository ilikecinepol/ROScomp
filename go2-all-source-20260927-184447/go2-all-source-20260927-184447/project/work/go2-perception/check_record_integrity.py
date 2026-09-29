"""Проверка: неполную запись нельзя объявлять успешно обработанной."""
import json
import shutil
import tempfile
from pathlib import Path

from replay_perception import process_capture

root = Path(__file__).resolve().parents[2]
source = root / 'outputs/go2-remote-setup/logs/20260916T110043Z'
with tempfile.TemporaryDirectory(prefix='go2-integrity-') as temp:
    base = Path(temp)
    capture = base / 'capture'
    shutil.copytree(source, capture)
    # Удаляем только копию: lidar_01 не должен получить метаданные lidar_00.
    (capture / 'lidar_00.npz').unlink()
    report = process_capture(capture, base / 'missing-first', 640)
    assert report['errors'] and not report['voxel']
    # Полностью пустая запись также обязана завершаться явной ошибкой.
    empty = base / 'empty'
    empty.mkdir()
    (empty / 'summary.json').write_text(json.dumps({'camera_files': [], 'lidar_files': []}))
    (empty / 'samples.jsonl').write_text('')
    report = process_capture(empty, base / 'empty-result', 640)
    assert len(report['errors']) == 2
print('PASS: missing-first-lidar; empty-recording')
