"""Передача уже полученного разрешения оператора одной новой попытке."""
import json
import subprocess
import time
from pathlib import Path

root = Path.home() / 'ai-robot'
folder = root / 'team_wolf_setup'
started = time.time()
proc = subprocess.Popen([str(root/'venv/bin/python'), str(folder/'run_motion_probe.py')])
armed = False
try:
    deadline = time.monotonic()+75
    while proc.poll() is None and time.monotonic() < deadline:
        status = folder/'last_motion_run.json'
        if status.exists() and status.stat().st_mtime >= started:
            info = json.loads(status.read_text())
            out = Path(info['output']).resolve()
            if not out.is_relative_to((folder/'motion_logs').resolve()):
                raise RuntimeError('Неверный каталог теста')
            ready_file = out/'ready.json'
            if ready_file.exists():
                ready = json.loads(ready_file.read_text())
                expected = dict(vx_m_s=.1, vy_m_s=0, vyaw_rad_s=0, duration_s=1.)
                if ready['plan'] != expected or ready['health_errors']:
                    raise RuntimeError('План или состояние не соответствует разрешению')
                if time.time()-ready_file.stat().st_mtime > 10:
                    raise RuntimeError('Готовность устарела')
                tmp = out/'arm.pending'
                tmp.write_text(json.dumps(dict(nonce=ready['nonce'], operator_ready=True)))
                tmp.rename(out/'arm.json')
                armed = True
                print('Разрешена одна новая попытка: 0,1 м/с, 1 секунда', flush=True)
                break
        time.sleep(.2)
finally:
    # Не убиваем wrapper: он должен восстановить штатное соединение.
    proc.wait(timeout=280)
if not armed:
    raise SystemExit('Разрешение не передано; движение не запрошено')
