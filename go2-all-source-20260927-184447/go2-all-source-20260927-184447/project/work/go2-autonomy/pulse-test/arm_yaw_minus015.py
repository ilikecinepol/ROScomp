"""Однократный атомарный старт уже подготовленного теста 0,15 рад/с × 1 с."""
import json
import os
from pathlib import Path
from datetime import datetime, timezone
import time


def main():
    team=Path('/home/ubuntu/ai-robot/team_wolf_setup')
    state=json.loads((team/'last_camera_reverse_run.json').read_text())
    folder=Path(state['output']).resolve()
    if folder.parent!=(team/'motion_logs').resolve():raise RuntimeError('Неверная папка теста')
    deadline=time.monotonic()+35
    while not (folder/'ready.json').exists() and time.monotonic()<deadline:
        if (folder/'summary.json').exists():raise RuntimeError('Тест уже завершён')
        time.sleep(.2)
    ready=json.loads((folder/'ready.json').read_text())
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(ready['ready_at_utc'])).total_seconds()
    if not 0<=age<60 or ready['health_errors'] or ready['plan']!={'vx_m_s':0.,'vy_m_s':0,'vyaw_rad_s':-.15,'duration_s':1.0}:
        raise RuntimeError('Не соответствует свежему разрешённому тесту')
    if (folder/'summary.json').exists() or (folder/'arm.json').exists():raise RuntimeError('Повторный старт запрещён')
    value=json.dumps({'nonce':ready['nonce'],'operator_ready':True})
    with (folder/'arm.pending').open('x') as out:
        out.write(value);out.flush();os.fsync(out.fileno())
    os.replace(folder/'arm.pending',folder/'arm.json')
    print('ARMED',folder,flush=True)


if __name__=='__main__':main()
