"""Measure actual displacement and attitude during the automatic trot test."""

from pathlib import Path
from math import atan2, degrees

from controller import Supervisor


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
result_path = Path(__file__).resolve().parents[2] / "test-results" / "go2_teleop_test.txt"
result_path.parent.mkdir(exist_ok=True)
go2 = robot.getFromDef("GO2")
base = go2

if base is None:
    result = "[go2-teleop-test] FAIL: Go2 root not found"
    passed = False
else:
    start = base.getPosition()
    for _ in range(int(9.0 * 1000 / timestep)):
        if robot.step(timestep) == -1:
            break
    finish = base.getPosition()
    upright = base.getOrientation()[8]
    matrix = base.getOrientation()
    yaw_deg = degrees(atan2(matrix[3], matrix[0]))
    distance = finish[0] - start[0]
    passed = distance >= 0.08 and abs(yaw_deg) >= 20.0 and finish[2] >= 0.18 and upright >= 0.65
    result = (
        f"[go2-teleop-test] {'PASS' if passed else 'FAIL'}: "
        f"forward={distance:.3f} m, yaw={yaw_deg:.1f} deg, "
        f"base z={finish[2]:.3f} m, upright={upright:.3f}"
    )

result_path.write_text(result + "\n", encoding="utf-8")
print(result, flush=True)
robot.step(timestep)
robot.simulationQuit(0 if passed else 2)
