"""Штатно завершает только обёртки нашей программы на текущей Pi."""
import fcntl
import os
from pathlib import Path
import signal
import time

def reset(timeout=60):
    team = Path.home() / 'ai-robot/team_wolf_setup'
    wrapper = str(team / 'teleop/run_teleop.py')
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal():
            continue
        fd = None
        try:
            fd = os.pidfd_open(int(proc.name))
            args = (proc / 'cmdline').read_bytes().split(b'\0')
            # Точное имя скрипта и владелец; не трогаем службу и чужие программы.
            if proc.stat().st_uid == os.getuid() and wrapper.encode() in args[1:3]:
                signal.pidfd_send_signal(fd, signal.SIGTERM)
        except (ProcessLookupError, FileNotFoundError):
            pass
        finally:
            if fd is not None:
                os.close(fd)
    deadline = time.monotonic() + timeout
    with (team / 'teleop.lock').open('a') as lock:
        while True:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                print('RESET_OK', flush=True)
                return
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Сеанс не завершился за 60 секунд; принудительное убийство не выполнено')
                time.sleep(.2)

if __name__ == '__main__':
    reset()
