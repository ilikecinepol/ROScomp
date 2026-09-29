"""Один ограниченный тест; освобождает WebRTC и восстанавливает fleet-dog."""
import fcntl
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

root = Path.home()/'ai-robot'
folder = root/'team_wolf_setup'
teleop_lock = (folder/'teleop.lock').open('a')
fcntl.flock(teleop_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
lock = (folder/'motion_probe.lock').open('a')
fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
out = folder/'motion_logs'/stamp
out.mkdir(parents=True, exist_ok=False)
was_active = subprocess.run(['systemctl', 'is-active', '--quiet', 'fleet-dog']).returncode == 0
status = {'fleet_was_active': was_active, 'output': str(out), 'wrapper_pid': os.getpid()}
(folder/'last_camera_reverse_run.json').write_text(json.dumps(status), encoding='utf-8')
child = None
try:
    if was_active:
        subprocess.run(['sudo', '-n', 'systemctl', 'stop', 'fleet-dog'], check=True, timeout=25)
    time.sleep(15)
    with (out/'console.log').open('w', encoding='utf-8') as logfile:
        child = subprocess.Popen([str(root/'venv/bin/python'), '-u', str(folder/'motion_yaw_minus015.py'), str(out), '--execute'],
                                 cwd=root, stdout=logfile, stderr=subprocess.STDOUT)
        status['probe_pid'] = child.pid
        (folder/'last_camera_reverse_run.json').write_text(json.dumps(status), encoding='utf-8')
        try:
            status['probe_exit_code'] = child.wait(timeout=230)
        except subprocess.TimeoutExpired:
            child.terminate()
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
            raise RuntimeError('Превышен общий срок теста; требуется проверка остановки')
except Exception as exc:
    status['error'] = type(exc).__name__ + ': ' + str(exc)
finally:
    if child is not None and child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=5)
    if was_active:
        time.sleep(15)
        try:
            restored = subprocess.run(['sudo', '-n', 'systemctl', 'start', 'fleet-dog'], timeout=25)
            status['fleet_start_exit_code'] = restored.returncode
            status['fleet_restored'] = subprocess.run(['systemctl', 'is-active', '--quiet', 'fleet-dog']).returncode == 0
        except Exception as exc:
            status['restore_error'] = str(exc)
    (out/'run_status.json').write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(status, ensure_ascii=False, indent=2), flush=True)
