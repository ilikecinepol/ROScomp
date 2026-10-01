"""Run the complete deterministic Webots regression suite.

The suite is intentionally independent of ROS/WSL. It exercises the model,
virtual Sport command semantics, collision/ramp behavior, A-frame course
profile, visual gait, and passive teeter physics. Results describe simulation
only and must not be presented as physical-robot validation.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
RESULTS = ROOT / "test-results"

SCENARIOS = (
    ("spawn", "go2_spawn_test.wbt", "go2_spawn_test.txt", 45),
    ("teleop", "go2_teleop_test.wbt", "go2_teleop_test.txt", 60),
    ("virtual_sport", "go2_virtual_sport_test.wbt", "go2_virtual-sport-test.txt", 45),
    ("hardware_response", "go2_hardware_response_test.wbt", "go2_hardware_response_test.txt", 45),
    ("collision", "go2_collision_test.wbt", "go2_collision-test.txt", 45),
    ("ramp", "go2_ramp_test.wbt", "go2_ramp-test.txt", 45),
    ("aframe", "go2_aframe_climb.wbt", "go2_aframe_test.txt", 60),
    ("flat_gait", "go2_flat_gait.wbt", "go2_flat_gait_test.txt", 45),
    ("teeter", "teeter_physics_test.wbt", "teeter_physics_test.txt", 45),
)


def find_webots(explicit: str | None) -> Path:
    candidates = [
        explicit,
        os.environ.get("WEBOTS_EXECUTABLE"),
        shutil.which("webots"),
        r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
        r"C:\Program Files\Webots\webots.exe",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return Path(candidate)
    raise FileNotFoundError("Webots executable was not found")


def run_scenario(webots: Path, item: tuple[str, str, str, int]) -> dict:
    name, world_name, result_name, timeout_s = item
    world = ROOT / "worlds" / world_name
    result_path = RESULTS / result_name
    previous_mtime = result_path.stat().st_mtime_ns if result_path.exists() else None
    started = time.monotonic()
    command = [
        str(webots),
        "--batch",
        "--mode=fast",
        "--no-rendering",
        "--stdout",
        "--stderr",
        str(world),
    ]
    environment = os.environ.copy()
    # Pin Qt to ordinary Windows rendering and a stable font scale. Webots does
    # not advance the simulation with the generic offscreen QPA plugin.
    environment["QT_QPA_PLATFORM"] = "windows"
    environment["QT_FONT_DPI"] = "96"
    environment["QT_SCALE_FACTOR"] = "1"
    environment["QT_AUTO_SCREEN_SCALE_FACTOR"] = "0"
    environment.pop("QT_SCREEN_SCALE_FACTORS", None)
    environment.pop("QT_DEVICE_PIXEL_RATIO", None)
    environment.pop("QT_ENABLE_HIGHDPI_SCALING", None)
    try:
        completed = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            env=environment,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_s,
            check=False,
        )
        return_code = completed.returncode
        process_error = None
    except subprocess.TimeoutExpired as exc:
        return_code = None
        process_error = f"timeout after {timeout_s}s"
        completed = exc

    fresh = result_path.exists() and result_path.stat().st_mtime_ns != previous_mtime
    evidence = result_path.read_text(encoding="utf-8").strip() if fresh else ""
    passed = return_code == 0 and fresh and "PASS" in evidence
    stderr = (completed.stderr or "")[-2000:] if hasattr(completed, "stderr") else ""
    stdout = (completed.stdout or "")[-4000:] if hasattr(completed, "stdout") else ""
    return {
        "name": name,
        "world": world_name,
        "passed": passed,
        "return_code": return_code,
        "duration_s": round(time.monotonic() - started, 3),
        "evidence": evidence,
        "error": process_error,
        "stdout_tail": stdout,
        "stderr_tail": stderr,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--webots", help="Path to webots executable")
    parser.add_argument(
        "--output",
        type=Path,
        default=RESULTS / "regression-summary.json",
    )
    parser.add_argument(
        "--scenario",
        action="append",
        choices=[item[0] for item in SCENARIOS],
        help="Run only the named scenario; may be repeated",
    )
    args = parser.parse_args()

    webots = find_webots(args.webots)
    results = []
    selected = [item for item in SCENARIOS if not args.scenario or item[0] in args.scenario]
    for scenario in selected:
        print(f"[webots-regression] running {scenario[0]}...", flush=True)
        result = run_scenario(webots, scenario)
        results.append(result)
        print(
            f"[webots-regression] {'PASS' if result['passed'] else 'FAIL'} "
            f"{result['name']}: {result['evidence'] or result['error'] or 'no fresh evidence'}",
            flush=True,
        )

    summary = {
        "scope": "webots_simulation_not_physical_robot_validation",
        "webots": str(webots),
        "passed": all(item["passed"] for item in results),
        "scenario_count": len(results),
        "results": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"[webots-regression] summary: {args.output}", flush=True)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
