"""Один тренировочный сеанс: освобождение WebRTC и восстановление fleet-dog."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

ROOT = Path.home() / "ai-robot"
TEAM = ROOT / "team_wolf_setup"


def emit(message):
    """Закрытие окна клиента не должно мешать восстановлению службы."""
    data = (json.dumps(message, ensure_ascii=False) + "\n").encode("utf-8")
    def write():
        try:
            os.write(sys.stdout.fileno(), data)
        except (BrokenPipeError, OSError):
            pass
    thread = threading.Thread(target=write, daemon=True)
    thread.start()
    thread.join(timeout=.15)


def active():
    return subprocess.run(
        ["systemctl", "is-active", "--quiet", "fleet-dog"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8,
    ).returncode == 0


def service(action):
    subprocess.run(
        ["sudo", "-n", "systemctl", action, "fleet-dog"],
        check=True, stdout=sys.stderr, stderr=sys.stderr, timeout=25,
    )


def main():
    TEAM.mkdir(exist_ok=True)
    lock = (TEAM / "teleop.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        emit({"type": "error", "reason": "Другой сеанс управления уже запущен."})
        return 2

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = TEAM / "teleop_logs" / stamp
    out.mkdir(parents=True)
    state = {"session_dir": str(out), "fleet_was_active": False,
             "fleet_restored": False, "exit_code": 2, "errors": []}
    child = None
    child_log = None
    interrupted = False
    restore_needed = False

    def interrupted_handler(signum, _frame):
        nonlocal interrupted
        interrupted = True
        if child is not None and child.poll() is None:
            try:
                child.send_signal(signal.SIGTERM)
            except ProcessLookupError:
                pass

    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, interrupted_handler)

    def pause(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if interrupted:
                return False
            time.sleep(min(.1, max(0, deadline - time.monotonic())))
        return True

    try:
        state["fleet_was_active"] = active()
        if state["fleet_was_active"]:
            # Флаг ставится до вызова: даже при таймауте stop служба могла остановиться.
            restore_needed = True
            service("stop")
        emit({"type": "progress", "phase": "connecting", "session_dir": str(out),
              "reason": "Освобождение соединения с роботом: 15 секунд."})
        if not pause(15):
            return 2
        env = dict(os.environ)
        env["OPENBLAS_NUM_THREADS"] = "1"
        env["OMP_NUM_THREADS"] = "1"
        env["PYTHONUNBUFFERED"] = "1"
        child_log = (out / "library.log").open("w", encoding="utf-8")
        child = subprocess.Popen(
            [str(ROOT / "venv/bin/python"), "-u", str(Path(__file__).with_name("remote_teleop.py")),
             "--output", str(out), "--execute"],
            cwd=ROOT, env=env, start_new_session=True,
            # stdin/stdout — протокол GUI; журнал библиотеки идёт только в stderr.
            stdin=sys.stdin, stdout=sys.stdout, stderr=child_log,
        )
        deadline = time.monotonic() + 385
        while child.poll() is None:
            if interrupted or time.monotonic() > deadline:
                child.send_signal(signal.SIGTERM)
                try:
                    child.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=5)
                    state["errors"].append("Процесс пришлось завершить после таймаута остановки.")
                break
            time.sleep(.1)
        state["exit_code"] = child.returncode
    except Exception as exc:
        state["errors"].append(type(exc).__name__ + ": " + str(exc)[:300])
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=30)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
        if child_log is not None:
            child_log.close()
        if restore_needed:
            emit({"type": "progress", "phase": "restoring", "session_dir": str(out),
                  "reason": "Запись завершена. Восстановление штатного подключения…"})
            # Интервал между WebRTC-клиентами нужен даже после закрытия SSH.
            time.sleep(15)
            try:
                service("start")
                state["fleet_restored"] = active()
            except Exception as exc:
                state["errors"].append("Восстановление fleet-dog: " + str(exc)[:200])
        else:
            state["fleet_restored"] = None  # До сеанса служба уже была выключена.
        (out / "run_status.json").write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
        emit({"type": "session_end", **state})
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()
    return state["exit_code"] or (2 if state["errors"] else 0)


if __name__ == "__main__":
    raise SystemExit(main())
