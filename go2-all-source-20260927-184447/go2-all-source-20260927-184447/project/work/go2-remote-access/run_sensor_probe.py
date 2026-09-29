"""Запускает ограниченную диагностику и восстанавливает штатный агент в finally."""
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

root = Path.home() / "ai-robot"
folder = root / "team_wolf_setup"
stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
out = folder / "logs" / stamp
out.mkdir(parents=True, exist_ok=True)
was_active = subprocess.run(["systemctl", "is-active", "--quiet", "fleet-dog"]).returncode == 0
status = {"fleet_was_active": was_active, "output": str(out)}
try:
    if was_active:
        subprocess.run(["sudo", "-n", "systemctl", "stop", "fleet-dog"], check=True, timeout=25)
    print("FLEET_RELEASED; ожидание освобождения соединения", flush=True)
    time.sleep(15)
    with (out / "console.log").open("w", encoding="utf-8") as logfile:
        result = subprocess.run([str(root / "venv/bin/python"), "-u", str(folder / "sensor_probe.py"), str(out)],
            cwd=root, stdout=logfile, stderr=subprocess.STDOUT, timeout=85)
        status["probe_exit_code"] = result.returncode
except Exception as exc:
    status["error"] = type(exc).__name__ + ": " + str(exc)
finally:
    if was_active:
        time.sleep(15)
        restored = subprocess.run(["sudo", "-n", "systemctl", "start", "fleet-dog"], timeout=25)
        status["fleet_start_exit_code"] = restored.returncode
        status["fleet_restored"] = subprocess.run(["systemctl", "is-active", "--quiet", "fleet-dog"]).returncode == 0
    (out / "run_status.json").write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(status, ensure_ascii=False, indent=2), flush=True)
if (out / "summary.json").exists():
    report = json.loads((out / "summary.json").read_text())
    report.pop("first_samples", None)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
