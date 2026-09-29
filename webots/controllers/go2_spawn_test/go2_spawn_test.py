"""Webots smoke test for the imported Unitree Go2 model."""

from pathlib import Path
from controller import Supervisor


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
result_path = Path(__file__).resolve().parents[2] / "test-results" / "go2_spawn_test.txt"
result_path.parent.mkdir(exist_ok=True)
go2 = robot.getFromDef("GO2")
base = go2

if base is None:
    result = "[go2-test] FAIL: imported Go2 root was not found"
    result_path.write_text(result + "\n", encoding="utf-8")
    print(result, flush=True)
    robot.step(timestep)
    robot.simulationQuit(2)
else:
    for _ in range(int(3.0 * 1000 / timestep)):
        if robot.step(timestep) == -1:
            break

    position = base.getPosition()
    orientation = base.getOrientation()
    upright = orientation[8]
    passed = 0.20 <= position[2] <= 0.55 and upright >= 0.80
    result = (
        f"[go2-test] {'PASS' if passed else 'FAIL'}: "
        f"base z={position[2]:.3f} m, upright={upright:.3f}"
    )
    result_path.write_text(result + "\n", encoding="utf-8")
    print(result, flush=True)
    robot.step(timestep)
    robot.simulationQuit(0 if passed else 2)
