"""Fail-fast Webots test proving that the teeter responds to an off-centre load."""

from math import atan2, degrees

from controller import Supervisor


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
teeter = robot.getFromDef("TEST_TEETER")
board = teeter.getFromProtoDef("TEETER_BOARD") if teeter else None

if board is None:
    print("[teeter-test] FAIL: TEETER_BOARD is not reachable through TEST_TEETER", flush=True)
    robot.step(timestep)
    robot.simulationQuit(2)
else:
    # Let the 12 kg box fall onto the positive-X half of the board.
    for _ in range(int(3.0 * 1000 / timestep)):
        if robot.step(timestep) == -1:
            break

    matrix = board.getOrientation()
    angle_deg = degrees(atan2(matrix[2], matrix[0]))
    passed = abs(angle_deg) >= 3.0
    print(
        f"[teeter-test] {'PASS' if passed else 'FAIL'}: "
        f"board angle = {angle_deg:.2f} deg (required magnitude >= 3 deg)",
        flush=True,
    )
    robot.step(timestep)
    robot.simulationQuit(0 if passed else 2)
