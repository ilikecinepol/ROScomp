"""Keyboard teleoperation with a diagonal-pair trot gait for Unitree Go2."""

from math import acos, atan2, cos, pi, sin
import sys

from controller import Keyboard, Robot


robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0
demo_mode = "--demo" in sys.argv

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

legs = ("FL", "FR", "RL", "RR")
# Diagonal pairs alternate in the same pattern as the stock Go2 trot.  The old
# four-beat crawl rocked in place in ODE because each short swing was cancelled
# by the three simultaneously sliding stance feet.
phase_offset = {"FL": 0.0, "RR": 0.0, "FR": 0.50, "RL": 0.50}
side_sign = {"FL": 1.0, "RL": 1.0, "FR": -1.0, "RR": -1.0}

UPPER_LEG = 0.213
LOWER_LEG = 0.213
STANCE_HEIGHT = 0.311
STEP_LENGTH = 0.105
TURN_STEP_LENGTH = 0.120
STEP_HEIGHT = 0.045
SWING_FRACTION = 0.42

motors = {}
current = {}
neutral = {}
for leg in legs:
    for joint, angle in (("hip", 0.0), ("thigh", 0.8), ("calf", -1.5)):
        name = f"{leg}_{joint}_joint"
        motor = robot.getDevice(name)
        motor.setVelocity(6.0)
        motor.setControlPID(25.0, 0.0, 0.6)
        motors[name] = motor
        current[name] = angle
        neutral[name] = angle
        motor.setPosition(angle)

phase = 0.0
forward_command = 0.0
turn_command = 0.0
last_key_time = -10.0
last_mode = None


def read_keyboard(now: float) -> tuple[float, float]:
    """Return normalized forward and yaw commands with a short key timeout."""
    global forward_command, turn_command, last_key_time

    received = False
    key = keyboard.getKey()
    while key != -1:
        code = key & Keyboard.KEY
        if code in (ord("W"), Keyboard.UP):
            forward_command = 1.0
            received = True
        elif code in (ord("S"), Keyboard.DOWN):
            forward_command = -0.75
            received = True
        elif code in (ord("A"), Keyboard.LEFT):
            turn_command = 1.0
            received = True
        elif code in (ord("D"), Keyboard.RIGHT):
            turn_command = -1.0
            received = True
        elif code == ord(" "):
            forward_command = 0.0
            turn_command = 0.0
            received = True
        key = keyboard.getKey()

    if received:
        last_key_time = now
    elif now - last_key_time > 0.28:
        forward_command = 0.0
        turn_command = 0.0
    return forward_command, turn_command


def leg_inverse_kinematics(foot_x: float, foot_z: float) -> tuple[float, float]:
    """Solve the sagittal two-link leg using the URDF joint convention."""
    x = -foot_x
    z = -foot_z
    cosine_knee = (
        x * x + z * z - UPPER_LEG * UPPER_LEG - LOWER_LEG * LOWER_LEG
    ) / (2.0 * UPPER_LEG * LOWER_LEG)
    cosine_knee = max(-1.0, min(1.0, cosine_knee))
    calf = -acos(cosine_knee)
    thigh = atan2(x, z) - atan2(
        LOWER_LEG * sin(calf),
        UPPER_LEG + LOWER_LEG * cos(calf),
    )
    return thigh, calf


def gait_targets(forward: float, turn: float) -> dict[str, float]:
    targets = dict(neutral)
    if abs(forward) < 0.05 and abs(turn) < 0.05:
        return targets

    for leg in legs:
        cycle = (phase + phase_offset[leg]) % 1.0
        stride = STEP_LENGTH * forward - side_sign[leg] * TURN_STEP_LENGTH * turn
        stride = max(-TURN_STEP_LENGTH, min(TURN_STEP_LENGTH, stride))
        activity = min(1.0, abs(forward) + abs(turn))

        if cycle < SWING_FRACTION:
            progress = cycle / SWING_FRACTION
            smooth = 0.5 - 0.5 * cos(pi * progress)
            foot_x = -0.5 * stride + stride * smooth
            foot_z = -STANCE_HEIGHT + STEP_HEIGHT * activity * sin(pi * progress)
        else:
            progress = (cycle - SWING_FRACTION) / (1.0 - SWING_FRACTION)
            foot_x = 0.5 * stride - stride * progress
            foot_z = -STANCE_HEIGHT

        thigh, calf = leg_inverse_kinematics(foot_x, foot_z)
        targets[f"{leg}_hip_joint"] = 0.055 * side_sign[leg]
        targets[f"{leg}_thigh_joint"] = thigh
        targets[f"{leg}_calf_joint"] = calf
    return targets


print("[go2_teleop] W/S move, A/D turn, Space stand.", flush=True)
if demo_mode:
    print("[go2_teleop] Automatic forward-and-turn gait test enabled.", flush=True)

while robot.step(timestep) != -1:
    now = robot.getTime()
    if demo_mode:
        if 1.0 <= now < 5.0:
            forward, turn = 1.0, 0.0
        elif 5.0 <= now < 8.0:
            forward, turn = 0.0, 1.0
        else:
            forward, turn = 0.0, 0.0
    else:
        forward, turn = read_keyboard(now)

    moving = abs(forward) >= 0.05 or abs(turn) >= 0.05
    if moving:
        phase = (phase + 1.35 * dt) % 1.0

    targets = gait_targets(forward, turn)
    # Smooth commands to avoid an impulse when starting or releasing a key.
    blend = min(1.0, dt * 16.0)
    for name, target in targets.items():
        current[name] += (target - current[name]) * blend
        motors[name].setPosition(current[name])

    mode = "moving" if moving else "standing"
    if mode != last_mode:
        print(f"[go2_teleop] {mode}", flush=True)
        last_mode = mode
