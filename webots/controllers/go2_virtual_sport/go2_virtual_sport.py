"""High-level velocity controller for the simulated Go2.

The controller accepts body-frame ``vx vy wz`` commands over UDP and provides
the same abstraction as Go2 Sport Mode. Keyboard input remains available for
interactive testing. Dedicated course and flat-gait tests move the robot
kinematically because the real Go2 balance controller is not part of Webots;
low-level contact and gait validation belongs in the official Unitree MuJoCo
simulator.
"""

import json
from math import acos, atan2, cos, isfinite, pi, sin, sqrt, tan
import socket
import struct
import sys
import zlib
from pathlib import Path

from controller import Keyboard, Supervisor
from gait_model import foot_target, maximum_foot_speed
from response_profile import load_response_profile


UDP_HOST = "0.0.0.0"
DEFAULT_UDP_PORT = 15000
COMMAND_TTL = 0.35
MAX_VX = 0.60
MAX_VY = 0.40
MAX_WZ = 1.50
MAX_LINEAR_ACCEL = 1.20
MAX_ANGULAR_ACCEL = 4.00
# In-place rotation needs a strong actuator gain, while multiplying small
# steering corrections by the same amount makes the path oscillate.
SIM_TURN_IN_PLACE_GAIN = 3.2
SIM_DRIVING_YAW_GAIN = 1.25
PHYSICS_LINEAR_COMPENSATION = 1.0
ARENA_X = (-4.85, 4.85)
ARENA_Y = (-3.35, 3.35)
CAMERA_MAGIC = b"GO2CAM1"
CAMERA_CHUNK_SIZE = 60000


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def approach(current: float, target: float, maximum_delta: float) -> float:
    return current + clamp(target - current, -maximum_delta, maximum_delta)


def current_yaw(node) -> float:
    orientation = node.getOrientation()
    return atan2(orientation[3], orientation[0])


def quaternion_from_orientation(matrix):
    """Convert a Webots row-major rotation matrix to w, x, y, z."""
    trace = matrix[0] + matrix[4] + matrix[8]
    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        return (0.25 * scale, (matrix[7] - matrix[5]) / scale,
                (matrix[2] - matrix[6]) / scale, (matrix[3] - matrix[1]) / scale)
    if matrix[0] > matrix[4] and matrix[0] > matrix[8]:
        scale = sqrt(1.0 + matrix[0] - matrix[4] - matrix[8]) * 2.0
        return ((matrix[7] - matrix[5]) / scale, 0.25 * scale,
                (matrix[1] + matrix[3]) / scale, (matrix[2] + matrix[6]) / scale)
    if matrix[4] > matrix[8]:
        scale = sqrt(1.0 + matrix[4] - matrix[0] - matrix[8]) * 2.0
        return ((matrix[2] - matrix[6]) / scale, (matrix[1] + matrix[3]) / scale,
                0.25 * scale, (matrix[5] + matrix[7]) / scale)
    scale = sqrt(1.0 + matrix[8] - matrix[0] - matrix[4]) * 2.0
    return ((matrix[3] - matrix[1]) / scale, (matrix[2] + matrix[6]) / scale,
            (matrix[5] + matrix[7]) / scale, 0.25 * scale)


def quaternion_multiply(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def axis_angle_from_quaternion(quaternion):
    w, x, y, z = quaternion
    norm = sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    angle = 2.0 * acos(clamp(w, -1.0, 1.0))
    axis_norm = sqrt(x * x + y * y + z * z)
    if axis_norm < 1e-8:
        return [0.0, 0.0, 1.0, 0.0]
    return [x / axis_norm, y / axis_norm, z / axis_norm, angle]


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0
demo_mode = "--demo" in sys.argv
collision_test_mode = "--collision-test" in sys.argv
ramp_test_mode = "--ramp-test" in sys.argv
aframe_test_mode = "--aframe-test" in sys.argv
flat_gait_test_mode = "--flat-gait-test" in sys.argv
flat_demo_mode = "--flat-demo" in sys.argv
kinematic_navigation_mode = "--kinematic-nav" in sys.argv
hardware_response_test_mode = "--hardware-response-test" in sys.argv
measured_response_mode = kinematic_navigation_mode or hardware_response_test_mode
kinematic_gait_mode = (
    aframe_test_mode
    or flat_gait_test_mode
    or kinematic_navigation_mode
    or hardware_response_test_mode
)
save_camera = "--save-camera" in sys.argv
udp_port = next(
    (
        int(argument.split("=", 1)[1])
        for argument in sys.argv
        if argument.startswith("--udp-port=")
    ),
    DEFAULT_UDP_PORT,
)
response_profile_path = next(
    (
        argument.split("=", 1)[1]
        for argument in sys.argv
        if argument.startswith("--response-profile=")
    ),
    str(Path(__file__).resolve().parents[2] / "config" / "go2_r6_measured_response.json"),
)
response_profile = (
    load_response_profile(response_profile_path) if measured_response_mode else None
)

joint_targets = {
    "FL_hip_joint": 0.08,
    "FL_thigh_joint": 0.8,
    "FL_calf_joint": -1.5,
    "FR_hip_joint": -0.08,
    "FR_thigh_joint": 0.8,
    "FR_calf_joint": -1.5,
    "RL_hip_joint": 0.08,
    "RL_thigh_joint": 0.8,
    "RL_calf_joint": -1.5,
    "RR_hip_joint": -0.08,
    "RR_thigh_joint": 0.8,
    "RR_calf_joint": -1.5,
}

go2 = robot.getFromDef("GO2")
if go2 is None:
    raise RuntimeError("DEF GO2 was not found")

motors = {}
position_sensors = {}
kinematic_joint_fields = {}
proto_position_fields = {
    "FL_hip_joint": "flHipPosition",
    "FL_thigh_joint": "flThighPosition",
    "FL_calf_joint": "flCalfPosition",
    "FR_hip_joint": "frHipPosition",
    "FR_thigh_joint": "frThighPosition",
    "FR_calf_joint": "frCalfPosition",
    "RL_hip_joint": "rlHipPosition",
    "RL_thigh_joint": "rlThighPosition",
    "RL_calf_joint": "rlCalfPosition",
    "RR_hip_joint": "rrHipPosition",
    "RR_thigh_joint": "rrThighPosition",
    "RR_calf_joint": "rrCalfPosition",
}
for joint_name, target in joint_targets.items():
    motor = robot.getDevice(joint_name)
    motor.setVelocity(8.0)
    motor.setPosition(target)
    motors[joint_name] = motor
    sensor = robot.getDevice(f"{joint_name}_sensor")
    sensor.enable(timestep)
    position_sensors[joint_name] = sensor
    if kinematic_gait_mode:
        # Webots keeps internal PROTO fields read-only for supervisors. Go2.proto
        # therefore exposes one writable position field for every joint.
        position_field = go2.getField(proto_position_fields[joint_name])
        if position_field is None:
            raise RuntimeError(f"Go2 PROTO position field was not found: {joint_name}")
        kinematic_joint_fields[joint_name] = position_field


def set_joint_target(joint_name: str, target: float) -> None:
    """Set a motor target and, in stable visual tests, its exact joint pose."""
    motors[joint_name].setPosition(target)
    if kinematic_gait_mode:
        kinematic_joint_fields[joint_name].setSFFloat(target)

translation_field = go2.getField("translation")
rotation_field = go2.getField("rotation")
x, y, body_z = translation_field.getSFVec3f()
yaw = current_yaw(go2)
start_x, start_y, start_yaw = x, y, yaw
start_z = body_z
max_body_z = body_z
max_abs_pitch = 0.0
aframe_x, aframe_y, aframe_yaw = x, y, yaw
follow_view_position = None
follow_view_offset = None
if flat_gait_test_mode:
    follow_view = robot.getFromDef("FOLLOW_VIEW")
    if follow_view is not None:
        follow_view_position = follow_view.getField("position")
        view_x, view_y, view_z = follow_view_position.getSFVec3f()
        follow_view_offset = (view_x - x, view_y - y, view_z - body_z)

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

sensor_period = timestep * max(1, round(100 / timestep))
camera = robot.getDevice("front_camera_sensor")
lidar = robot.getDevice("lidar")
if not aframe_test_mode:
    camera.enable(sensor_period)
    lidar.enable(sensor_period)

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.setblocking(False)
udp.bind((UDP_HOST, udp_port))
sensor_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

target_vx = target_vy = target_wz = 0.0
requested_vx = requested_vy = requested_wz = 0.0
vx = vy = wz = 0.0
last_external_command = -10.0
last_keyboard_command = -10.0
last_reported_source = None
sensor_client = None
next_sensor_time = 0.0
next_state_time = 0.0
camera_frame = 0
camera_saved = False
next_gait_report = 0.0
latest_lidar_ranges = []
next_collision_report = 0.0
terrain_height = 0.0
terrain_pitch = 0.0
terrain_roll = 0.0
response_peak_vx = 0.0
response_peak_left_wz = 0.0
response_peak_right_wz = 0.0


def measured_command(command_vx: float, command_vy: float, command_wz: float):
    """Map a requested Sport command through the recorded hardware response."""
    if response_profile is None:
        raise RuntimeError("Measured response profile is not active")
    return response_profile.realize(command_vx, command_vy, command_wz)


def course_coordinates(world_x: float, world_y: float, center_x: float, center_y: float, yaw: float):
    """Convert a world point to an obstacle's longitudinal/lateral coordinates."""
    delta_x = world_x - center_x
    delta_y = world_y - center_y
    return (
        delta_x * cos(yaw) + delta_y * sin(yaw),
        -delta_x * sin(yaw) + delta_y * cos(yaw),
    )


def is_traversable_course_point(world_x: float, world_y: float) -> bool:
    """True for ramps/decks that must be traversed instead of treated as walls."""
    course_footprints = (
        (-2.65, 2.55, 0.0, 2.32, 0.66),       # A-frame
        (-2.65, 0.70, pi / 2.0, 2.08, 0.66),  # teeter
        (-2.65, -2.30, 0.0, 2.10, 0.66),      # swing bridge
        (2.70, 1.55, pi / 2.0, 1.65, 0.67),   # two-level platform
    )
    for center_x, center_y, yaw, half_length, half_width in course_footprints:
        local_x, local_y = course_coordinates(world_x, world_y, center_x, center_y, yaw)
        if abs(local_x) <= half_length and abs(local_y) <= half_width:
            return True
    return False


def course_surface_profile(world_x: float, world_y: float):
    """Return height and world XY gradient of the traversable course surface."""
    # A-frame, longitudinal axis is world X.
    local_x, local_y = course_coordinates(world_x, world_y, -2.65, 2.55, 0.0)
    if abs(local_y) <= 0.66:
        run = 1.09
        slope = tan(0.429700)
        if -run <= local_x <= 0.0:
            return slope * (local_x + run), slope, 0.0
        if 0.0 < local_x <= run:
            return slope * (run - local_x), -slope, 0.0

    # Swing bridge: two shallow ramps and a level moving-deck approximation.
    local_x, local_y = course_coordinates(world_x, world_y, -2.65, -2.30, 0.0)
    if abs(local_y) <= 0.66:
        if -2.06 <= local_x < -0.875:
            slope = 0.27 / 1.185
            return slope * (local_x + 2.06), slope, 0.0
        if -0.875 <= local_x <= 0.875:
            return 0.27, 0.0, 0.0
        if 0.875 < local_x <= 2.06:
            slope = -0.27 / 1.185
            return 0.27 + slope * (local_x - 0.875), slope, 0.0

    # Teeter, whose longitudinal axis is world Y. The dynamic board itself is
    # approximated at its neutral height during general SLAM navigation.
    local_x, local_y = course_coordinates(world_x, world_y, -2.65, 0.70, pi / 2.0)
    if abs(local_y) <= 0.66:
        if -1.10 <= local_x < -0.88:
            slope = 0.24 / 0.22
            return slope * (local_x + 1.10), 0.0, slope
        if -0.88 <= local_x <= 0.88:
            return 0.24, 0.0, 0.0
        if 0.88 < local_x <= 1.10:
            slope = -0.24 / 0.22
            return 0.24 + slope * (local_x - 0.88), 0.0, slope

    # Two-level platform, longitudinal axis is world Y.
    local_x, local_y = course_coordinates(world_x, world_y, 2.70, 1.55, pi / 2.0)
    if abs(local_y) <= 0.67:
        if -1.50 <= local_x < -0.78:
            slope = 0.36 / 0.72
            return slope * (local_x + 1.50), 0.0, slope
        if -0.78 <= local_x < -0.19:
            return 0.36, 0.0, 0.0
        if -0.19 <= local_x < -0.11:
            slope = 0.12 / 0.08
            return 0.36 + slope * (local_x + 0.19), 0.0, slope
        if -0.11 <= local_x <= 0.49:
            return 0.48, 0.0, 0.0
        if 0.49 < local_x <= 1.52:
            slope = -0.48 / 1.03
            return 0.48 + slope * (local_x - 0.49), 0.0, slope

    return 0.0, 0.0, 0.0


def kinematic_motion_blocked(command_vx: float, command_vy: float, command_wz: float) -> bool:
    """Check the short swept body footprint against the current 360° scan."""
    if not kinematic_navigation_mode:
        return False

    half_length = 0.36
    half_width = 0.24
    margin = 0.015
    horizon = 0.10
    center_dx = command_vx * horizon
    center_dy = command_vy * horizon
    yaw = current_yaw(go2)
    proposed_x = aframe_x + center_dx * cos(yaw) - center_dy * sin(yaw)
    proposed_y = aframe_y + center_dx * sin(yaw) + center_dy * cos(yaw)

    # Room bounds already include the real rectangular body clearance.
    if not (ARENA_X[0] <= proposed_x <= ARENA_X[1] and ARENA_Y[0] <= proposed_y <= ARENA_Y[1]):
        return True

    count = len(latest_lidar_ranges)
    if count < 2:
        return False
    fov = lidar.getFov()
    if not latest_lidar_ranges:
        return False
    min_x = min(-half_length, center_dx - half_length) - margin
    max_x = max(half_length, center_dx + half_length) + margin
    min_y = min(-half_width, center_dy - half_width) - margin
    max_y = max(half_width, center_dy + half_width) + margin
    yaw_delta = command_wz * horizon

    for index, distance in enumerate(latest_lidar_ranges):
        if not isfinite(distance) or distance < lidar.getMinRange() or distance > lidar.getMaxRange():
            continue
        # Webots stores samples from +FOV/2 to -FOV/2.
        angle = 0.5 * fov - index * fov / (count - 1)
        point_x = 0.08 + distance * cos(angle)
        point_y = distance * sin(angle)
        # Ignore self-returns from legs and any stale point already inside the
        # current body. Only newly occupied swept space may stop the robot.
        if abs(point_x) <= half_length + margin and abs(point_y) <= half_width + margin:
            continue
        # The scan omits course surfaces that can be climbed. Remaining returns
        # are checked against the actual rectangular body, not a large circle.
        if abs(command_wz) > 0.03:
            for fraction in (0.5, 1.0):
                angle_delta = yaw_delta * fraction
                relative_x = point_x - center_dx * fraction
                relative_y = point_y - center_dy * fraction
                future_x = cos(angle_delta) * relative_x + sin(angle_delta) * relative_y
                future_y = -sin(angle_delta) * relative_x + cos(angle_delta) * relative_y
                if abs(future_x) <= half_length + margin and abs(future_y) <= half_width + margin:
                    return True
        if abs(command_vx) + abs(command_vy) > 0.01 and min_x <= point_x <= max_x and min_y <= point_y <= max_y:
            return True
    return False


def update_gait(now: float) -> None:
    """Move each foot through stance and swing using planar leg IK."""
    activity = clamp(max(abs(vx) / MAX_VX, abs(vy) / MAX_VY, abs(wz) / MAX_WZ), 0.0, 1.0)
    if activity < 0.02:
        for joint_name, target in joint_targets.items():
            set_joint_target(joint_name, target)
        return

    for leg in ("FL", "FR", "RL", "RR"):
        foot = foot_target(leg, now, vx, vy, wz)
        foot_x = foot.x
        foot_down = foot.down

        length = 0.213
        distance_squared = foot_x * foot_x + foot_down * foot_down
        knee_cosine = clamp(
            (distance_squared - 2.0 * length * length) / (2.0 * length * length),
            -1.0,
            1.0,
        )
        knee_angle = acos(knee_cosine)
        upper_angle = atan2(foot_x, foot_down) - atan2(
            length * sin(knee_angle), length * (1.0 + cos(knee_angle))
        )
        thigh_target = -upper_angle
        calf_target = -knee_angle

        # Hip ab/adduction tracks the lateral component of the same planted-
        # foot trajectory. A turn therefore also moves front/rear legs in
        # opposite lateral directions instead of faking a straight walk.
        hip_target = (0.08 if leg.endswith("L") else -0.08) + atan2(
            foot.y, foot_down
        )
        set_joint_target(f"{leg}_hip_joint", hip_target)
        set_joint_target(f"{leg}_thigh_joint", thigh_target)
        set_joint_target(f"{leg}_calf_joint", calf_target)


def send_state(now: float) -> None:
    global next_state_time
    if sensor_client is None or now + 1e-9 < next_state_time:
        return
    next_state_time = now + 0.02
    position = translation_field.getSFVec3f()
    w, qx, qy, qz = quaternion_from_orientation(go2.getOrientation())
    velocity = go2.getVelocity()
    heading = current_yaw(go2)
    body_vx = cos(heading) * velocity[0] + sin(heading) * velocity[1]
    body_vy = -sin(heading) * velocity[0] + cos(heading) * velocity[1]
    angular_velocity = [float(value) for value in velocity[3:]]
    if kinematic_gait_mode:
        # These modes reset Webots physics every step, so getVelocity() is zero
        # even while the supervisor changes the pose. Report the commanded body
        # twist so ROS odometry agrees with the motion observed by Nav2.
        body_vx = vx
        body_vy = vy
        angular_velocity = [0.0, 0.0, wz]
    state = json.dumps(
        {
            "type": "state",
            "stamp": now,
            "position": [round(float(value), 6) for value in position],
            "orientation": [round(qx, 7), round(qy, 7), round(qz, 7), round(w, 7)],
            "linear_velocity": [round(body_vx, 6), round(body_vy, 6), round(float(velocity[2]), 6)],
            "angular_velocity": [round(float(value), 6) for value in angular_velocity],
            "requested_twist": [
                round(requested_vx, 6),
                round(requested_vy, 6),
                round(requested_wz, 6),
            ],
            "response_profile": response_profile.profile_id if response_profile else None,
        },
        separators=(",", ":"),
    ).encode("utf-8")
    try:
        sensor_udp.sendto(state, sensor_client)
    except OSError as error:
        print(f"[virtual-sport] State UDP send failed: {error}", flush=True)


def aframe_profile(world_x: float):
    """Return ideal stock-locomotion body height offset and pitch for the test A-frame."""
    start, apex, end = 0.302, 1.390, 2.478
    angle = 0.436332
    transition = 0.18
    if start <= world_x < apex:
        height = tan(angle) * (world_x - start)
        entry = clamp((world_x - start) / transition, 0.0, 1.0)
        crest = clamp((apex - world_x) / transition, 0.0, 1.0)
        pitch = -angle * min(entry, crest)
        return height, pitch
    if apex <= world_x <= end:
        height = tan(angle) * (end - world_x)
        crest = clamp((world_x - apex) / transition, 0.0, 1.0)
        exit_blend = clamp((end - world_x) / transition, 0.0, 1.0)
        pitch = angle * min(crest, exit_blend)
        return height, pitch
    return 0.0, 0.0


def receive_udp(now: float) -> bool:
    global target_vx, target_vy, target_wz
    global requested_vx, requested_vy, requested_wz
    global last_external_command, sensor_client
    received = False
    while True:
        try:
            payload, sender = udp.recvfrom(2048)
        except BlockingIOError:
            break
        try:
            message = json.loads(payload.decode("utf-8"))
            sensor_client = sender
            if message.get("type") == "hello":
                continue
            requested_vx = float(message["vx"])
            requested_vy = float(message.get("vy", 0.0))
            requested_wz = float(message.get("wz", 0.0))
            if measured_response_mode:
                target_vx, target_vy, target_wz = measured_command(
                    requested_vx, requested_vy, requested_wz
                )
                last_external_command = now
                received = True
                continue
            target_vx = clamp(requested_vx, -MAX_VX, MAX_VX)
            target_vy = clamp(requested_vy, -MAX_VY, MAX_VY)
            # Nav2 already commands angular velocity in rad/s and closes the
            # loop from odometry. Applying the visual demo compensation here
            # made kinematic navigation turn up to 3.2 times faster than Nav2
            # expected, causing overshoot and oscillation at the goal.
            yaw_gain = (
                SIM_DRIVING_YAW_GAIN
                if abs(target_vx) > 0.04 or abs(target_vy) > 0.04
                else SIM_TURN_IN_PLACE_GAIN
            )
            target_wz = clamp(
                yaw_gain * requested_wz,
                -MAX_WZ,
                MAX_WZ,
            )
            last_external_command = now
            received = True
        except (KeyError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
            print("[virtual-sport] Ignored malformed UDP command.", flush=True)
    return received


def send_sensors(now: float) -> None:
    global camera_frame, camera_saved, latest_lidar_ranges, next_sensor_time
    if sensor_client is None or now + 1e-9 < next_sensor_time:
        return
    next_sensor_time = now + sensor_period / 1000.0

    horizontal_resolution = lidar.getHorizontalResolution()
    layer_count = lidar.getNumberOfLayers()
    horizontal_fov = lidar.getFov()
    vertical_fov = lidar.getVerticalFov()
    orientation = go2.getOrientation()
    robot_position = translation_field.getSFVec3f()
    flat_range_image = lidar.getRangeImage()
    layer_images = [
        flat_range_image[layer * horizontal_resolution : (layer + 1) * horizontal_resolution]
        for layer in range(layer_count)
    ]
    raw_ranges = [float("inf")] * horizontal_resolution
    raw_hits = [None] * horizontal_resolution

    # Collapse the 3-D lidar into a dense bird's-eye scan. Ground returns are
    # rejected in world coordinates, while raised ramps, decks, cones and walls
    # remain visible. Webots stores layers from top to bottom.
    for layer_index, layer_ranges in enumerate(layer_images):
        vertical_angle = (
            0.0
            if layer_count == 1
            else 0.5 * vertical_fov - layer_index * vertical_fov / (layer_count - 1)
        )
        vertical_cos = cos(vertical_angle)
        vertical_sin = sin(vertical_angle)
        for index, distance in enumerate(layer_ranges):
            if not isfinite(distance) or distance < lidar.getMinRange() or distance > lidar.getMaxRange():
                continue
            horizontal_distance = distance * vertical_cos
            if horizontal_distance >= raw_ranges[index]:
                continue
            horizontal_angle = 0.5 * horizontal_fov - index * horizontal_fov / (horizontal_resolution - 1)
            local_x = 0.08 + horizontal_distance * cos(horizontal_angle)
            local_y = horizontal_distance * sin(horizontal_angle)
            local_z = 0.13 + distance * vertical_sin
            # Downward layers can see Go2's own legs. External geometry cannot
            # legitimately appear inside the current torso footprint.
            if abs(local_x) <= 0.37 and abs(local_y) <= 0.25:
                continue
            hit_x = robot_position[0] + orientation[0] * local_x + orientation[1] * local_y + orientation[2] * local_z
            hit_y = robot_position[1] + orientation[3] * local_x + orientation[4] * local_y + orientation[5] * local_z
            hit_z = robot_position[2] + orientation[6] * local_x + orientation[7] * local_y + orientation[8] * local_z
            if hit_z <= 0.07:
                continue
            raw_ranges[index] = horizontal_distance
            raw_hits[index] = (hit_x, hit_y)

    navigation_ranges = list(raw_ranges)
    if kinematic_navigation_mode:
        # Keep traversable equipment in the raw operator scan, but remove it
        # from Nav2's obstacle scan so the planner may drive onto its surface.
        for index, hit in enumerate(raw_hits):
            if hit is not None and is_traversable_course_point(hit[0], hit[1]):
                navigation_ranges[index] = float("inf")
    latest_lidar_ranges = navigation_ranges

    scan_common = {
        "stamp": now,
        "frame_id": "lidar_link",
        "angle_min": -horizontal_fov / 2.0,
        "angle_max": horizontal_fov / 2.0,
        "range_min": lidar.getMinRange(),
        "range_max": lidar.getMaxRange(),
    }
    raw_scan = json.dumps(
        {
            **scan_common,
            "type": "scan_raw",
            "ranges": [round(float(value), 4) for value in raw_ranges],
        },
        separators=(",", ":"),
    ).encode("utf-8")
    scan = json.dumps(
        {
            **scan_common,
            "type": "scan",
            "ranges": [round(float(value), 4) for value in navigation_ranges],
        },
        separators=(",", ":"),
    ).encode("utf-8")

    image = zlib.compress(bytes(camera.getImage()), level=1)
    if save_camera and not camera_saved:
        snapshot = (
            Path(__file__).resolve().parents[2]
            / "test-results"
            / "go2_camera_direct.png"
        )
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        camera.saveImage(str(snapshot), 100)
        camera_saved = True
    chunk_count = (len(image) + CAMERA_CHUNK_SIZE - 1) // CAMERA_CHUNK_SIZE
    camera_frame = (camera_frame + 1) & 0xFFFFFFFF
    try:
        sensor_udp.sendto(raw_scan, sensor_client)
        sensor_udp.sendto(scan, sensor_client)
        for chunk_index in range(chunk_count):
            offset = chunk_index * CAMERA_CHUNK_SIZE
            header = struct.pack(
                "!7sIHHHH",
                CAMERA_MAGIC,
                camera_frame,
                chunk_index,
                chunk_count,
                camera.getWidth(),
                camera.getHeight(),
            )
            sensor_udp.sendto(
                header + image[offset : offset + CAMERA_CHUNK_SIZE], sensor_client
            )
    except OSError as error:
        print(f"[virtual-sport] Sensor UDP send failed: {error}", flush=True)


def receive_keyboard(now: float) -> bool:
    global target_vx, target_vy, target_wz, last_keyboard_command
    forward = lateral = turn = 0.0
    received = False
    key = keyboard.getKey()
    while key != -1:
        code = key & Keyboard.KEY
        if code in (ord("W"), Keyboard.UP):
            forward += 0.45
            received = True
        elif code in (ord("S"), Keyboard.DOWN):
            forward -= 0.35
            received = True
        elif code in (ord("A"), Keyboard.LEFT):
            turn += 0.90
            received = True
        elif code in (ord("D"), Keyboard.RIGHT):
            turn -= 0.90
            received = True
        elif code == ord("Q"):
            lateral += 0.30
            received = True
        elif code == ord("E"):
            lateral -= 0.30
            received = True
        elif code == ord(" "):
            received = True
        key = keyboard.getKey()

    if received:
        target_vx = clamp(forward, -MAX_VX, MAX_VX)
        target_vy = clamp(lateral, -MAX_VY, MAX_VY)
        target_wz = clamp(turn, -MAX_WZ, MAX_WZ)
        last_keyboard_command = now
    return received


print(
    "[virtual-sport] W/S forward, A/D turn, Q/E lateral, Space stop; "
    f"UDP {UDP_HOST}:{udp_port} exchanges velocity, camera and lidar data.",
    flush=True,
)
if response_profile is not None:
    print(
        f"[virtual-sport] measured response profile={response_profile.profile_id} "
        f"forward={response_profile.forward_gain:.3f} "
        f"left={response_profile.left_gain:.4f} "
        f"right={response_profile.right_gain:.3f}; "
        "reverse/lateral remain disabled until measured.",
        flush=True,
    )


def finish_test(test_name: str, passed: bool, result: str) -> None:
    """Persist batch-test evidence before terminating Webots."""
    result_path = (
        Path(__file__).resolve().parents[2]
        / "test-results"
        / f"{test_name}.txt"
    )
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(result + "\n", encoding="utf-8")
    print(result, flush=True)
    robot.step(timestep)
    robot.simulationQuit(0 if passed else 2)

while robot.step(timestep) != -1:
    now = robot.getTime()

    if hardware_response_test_mode:
        if 0.5 <= now < 2.0:
            requested_vx, requested_vy, requested_wz = 0.35, 0.0, 0.0
        elif 2.7 <= now < 4.2:
            requested_vx, requested_vy, requested_wz = 0.0, 0.0, 0.35
        elif 4.9 <= now < 6.4:
            requested_vx, requested_vy, requested_wz = 0.0, 0.0, -0.35
        else:
            requested_vx = requested_vy = requested_wz = 0.0
        target_vx, target_vy, target_wz = measured_command(
            requested_vx, requested_vy, requested_wz
        )
        source = "hardware-response-test"
    elif aframe_test_mode:
        # Deterministic high-level traversal. This validates the virtual Sport
        # contract and course profile, not physical Go2 stability or traction.
        target_vx = 0.0 if now < 0.5 or aframe_x >= 2.72 else 0.25
        target_vy = target_wz = 0.0
        source = "aframe-test"
    elif flat_demo_mode:
        target_vx, target_vy, target_wz = (0.0, 0.0, 0.0) if now < 2.0 else (0.25, 0.0, 0.0)
        source = "flat-demo"
    elif demo_mode:
        if 0.5 <= now < 2.5:
            target_vx, target_vy, target_wz = 0.40, 0.0, 0.0
        elif 2.5 <= now < 4.5:
            target_vx, target_vy, target_wz = 0.0, 0.0, 0.70
        else:
            target_vx = target_vy = target_wz = 0.0
        source = "demo"
    else:
        udp_received = receive_udp(now)
        keyboard_received = receive_keyboard(now)
        if keyboard_received or now - last_keyboard_command <= 0.22:
            source = "keyboard"
        elif udp_received or now - last_external_command <= COMMAND_TTL:
            source = "cmd_vel"
        else:
            target_vx = target_vy = target_wz = 0.0
            source = "idle"

    # The A-frame controller needs only pose and attitude. Navigation still needs
    # lidar/camera packets even though its stable gait visualization is kinematic.
    if not aframe_test_mode:
        # Send the pose first. UDP preserves datagram order for this local
        # socket, so the bridge can stamp the following scan with the pose at
        # which Webots captured it instead of the preceding state sample.
        send_state(now)
        send_sensors(now)
    else:
        send_state(now)

    if source != last_reported_source:
        print(f"[virtual-sport] source={source}", flush=True)
        last_reported_source = source

    vx = approach(vx, target_vx, MAX_LINEAR_ACCEL * dt)
    vy = approach(vy, target_vy, MAX_LINEAR_ACCEL * dt)
    wz = approach(wz, target_wz, MAX_ANGULAR_ACCEL * dt)
    if hardware_response_test_mode:
        response_peak_vx = max(response_peak_vx, vx)
        response_peak_left_wz = max(response_peak_left_wz, wz)
        response_peak_right_wz = min(response_peak_right_wz, wz)
    if kinematic_motion_blocked(vx, vy, wz):
        vx = vy = wz = 0.0
        if now >= next_collision_report:
            next_collision_report = now + 1.0
            print("[virtual-sport] Kinematic collision guard stopped motion.", flush=True)
    update_gait(now)
    if flat_gait_test_mode and now >= next_gait_report:
        next_gait_report = now + 0.75
        print(
            "[flat-gait] "
            f"FL=({position_sensors['FL_thigh_joint'].getValue():+.2f},"
            f"{position_sensors['FL_calf_joint'].getValue():+.2f}) "
            f"FR=({position_sensors['FR_thigh_joint'].getValue():+.2f},"
            f"{position_sensors['FR_calf_joint'].getValue():+.2f})",
            flush=True,
        )

    x, y, current_z = translation_field.getSFVec3f()
    max_body_z = max(max_body_z, current_z)
    yaw = current_yaw(go2)
    world_vx = vx * cos(yaw) - vy * sin(yaw)
    world_vy = vx * sin(yaw) + vy * cos(yaw)
    current_velocity = go2.getVelocity()
    if not aframe_test_mode:
        go2.setVelocity(
            [
                PHYSICS_LINEAR_COMPENSATION * world_vx,
                PHYSICS_LINEAR_COMPENSATION * world_vy,
                current_velocity[2],
                current_velocity[3],
                current_velocity[4],
                0.0,
            ]
        )
    half_delta = 0.5 * wz * dt
    yaw_delta = (cos(half_delta), 0.0, 0.0, sin(half_delta))
    if kinematic_gait_mode:
        aframe_world_vx = vx * cos(aframe_yaw) - vy * sin(aframe_yaw)
        aframe_world_vy = vx * sin(aframe_yaw) + vy * cos(aframe_yaw)
        aframe_x += aframe_world_vx * dt
        aframe_y += aframe_world_vy * dt
        aframe_yaw = (aframe_yaw + wz * dt + pi) % (2.0 * pi) - pi
        if aframe_test_mode:
            target_height, target_pitch = aframe_profile(aframe_x)
            target_roll = 0.0
        elif kinematic_navigation_mode:
            target_height, gradient_x, gradient_y = course_surface_profile(aframe_x, aframe_y)
            forward_gradient = gradient_x * cos(aframe_yaw) + gradient_y * sin(aframe_yaw)
            lateral_gradient = -gradient_x * sin(aframe_yaw) + gradient_y * cos(aframe_yaw)
            target_pitch = -atan2(forward_gradient, 1.0)
            target_roll = atan2(lateral_gradient, 1.0)
        else:
            target_height = 0.0
            target_pitch = 0.012 * sin(4.0 * pi * 1.6 * now)
            target_roll = 0.0
        # Blend steps and slope boundaries so the torso and feet don't jump
        # when crossing from the tiled floor onto an obstacle.
        terrain_height = approach(terrain_height, target_height, 0.75 * dt)
        terrain_pitch = approach(terrain_pitch, target_pitch, 1.8 * dt)
        terrain_roll = approach(terrain_roll, target_roll, 1.8 * dt)
        max_abs_pitch = max(max_abs_pitch, abs(terrain_pitch))
        gait_activity = clamp(
            max(abs(vx) / MAX_VX, abs(vy) / MAX_VY, abs(wz) / MAX_WZ), 0.0, 1.0
        )
        gait_frequency = 1.25 + 1.15 * clamp(
            maximum_foot_speed(vx, vy, wz) / MAX_VX, 0.0, 1.0
        )
        body_bob = (
            0.006
            * gait_activity
            * (1.0 - cos(4.0 * pi * gait_frequency * now))
            / 2.0
        )
        go2.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        translation_field.setSFVec3f([aframe_x, aframe_y, start_z + terrain_height + body_bob])
        half_yaw = 0.5 * aframe_yaw
        yaw_quaternion = (cos(half_yaw), 0.0, 0.0, sin(half_yaw))
        pitch_quaternion = (cos(terrain_pitch / 2.0), 0.0, sin(terrain_pitch / 2.0), 0.0)
        roll_quaternion = (cos(terrain_roll / 2.0), sin(terrain_roll / 2.0), 0.0, 0.0)
        rotation_field.setSFRotation(
            axis_angle_from_quaternion(
                quaternion_multiply(
                    quaternion_multiply(yaw_quaternion, pitch_quaternion),
                    roll_quaternion,
                )
            )
        )
        # Teleporting the kinematic torso must not leave old impulses in child links.
        go2.resetPhysics()
    else:
        # Apply yaw without erasing roll and pitch created by ordinary contacts.
        current_quaternion = quaternion_from_orientation(go2.getOrientation())
        rotation_field.setSFRotation(
            axis_angle_from_quaternion(quaternion_multiply(yaw_delta, current_quaternion))
        )

    if flat_gait_test_mode and follow_view_position is not None:
        follow_view_position.setSFVec3f(
            [
                x + follow_view_offset[0],
                y + follow_view_offset[1],
                start_z + follow_view_offset[2],
            ]
        )

    if aframe_test_mode and now >= 13.0:
        x, y, final_z = translation_field.getSFVec3f()
        distance = ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5
        rise = max_body_z - start_z
        settled_height_error = abs(final_z - start_z)
        passed = (
            distance >= 2.60
            and rise >= 0.40
            and max_abs_pitch >= 0.30
            and settled_height_error <= 0.08
        )
        result = (
            f"[aframe-test] {'PASS' if passed else 'FAIL'}: "
            f"distance={distance:.3f} m, rise={rise:.3f} m, "
            f"max_pitch={max_abs_pitch:.3f} rad, "
            f"settled_height_error={settled_height_error:.3f} m"
        )
        finish_test("go2_aframe_test", passed, result)
        break

    if flat_gait_test_mode and now >= 6.0:
        x, y, final_z = translation_field.getSFVec3f()
        distance = ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5
        settled_height_error = abs(final_z - start_z)
        passed = distance >= 0.75 and settled_height_error <= 0.08
        result = (
            f"[flat-gait-test] {'PASS' if passed else 'FAIL'}: "
            f"distance={distance:.3f} m, "
            f"settled_height_error={settled_height_error:.3f} m"
        )
        finish_test("go2_flat_gait_test", passed, result)
        break

    if hardware_response_test_mode and now >= 7.0:
        expected_vx, _, expected_left = measured_command(0.35, 0.0, 0.35)
        _, _, expected_right = measured_command(0.0, 0.0, -0.35)
        errors = (
            abs(response_peak_vx - expected_vx),
            abs(response_peak_left_wz - expected_left),
            abs(response_peak_right_wz - expected_right),
        )
        passed = max(errors) <= 0.003
        result = (
            f"[hardware-response-test] {'PASS' if passed else 'FAIL'}: "
            f"vx={response_peak_vx:.6f}/{expected_vx:.6f}, "
            f"left_wz={response_peak_left_wz:.6f}/{expected_left:.6f}, "
            f"right_wz={response_peak_right_wz:.6f}/{expected_right:.6f}, "
            f"profile={response_profile.profile_id}"
        )
        finish_test("go2_hardware_response_test", passed, result)
        break

    if demo_mode and not flat_demo_mode and now >= 5.0:
        x, y, _ = translation_field.getSFVec3f()
        yaw = current_yaw(go2)
        distance = ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5
        yaw_change = abs((yaw - start_yaw + pi) % (2.0 * pi) - pi)
        if collision_test_mode:
            passed = distance <= 0.55
            test_name = "collision-test"
        elif ramp_test_mode:
            passed = distance >= 0.35 and max_body_z >= start_z + 0.10
            test_name = "ramp-test"
        else:
            passed = distance >= 0.55 and yaw_change >= 1.0
            test_name = "virtual-sport-test"
        result = (
            f"[{test_name}] "
            f"{'PASS' if passed else 'FAIL'}: "
            f"distance={distance:.3f} m, yaw={yaw_change:.3f} rad, "
            f"rise={max_body_z - start_z:.3f} m"
        )
        finish_test(f"go2_{test_name}", passed, result)
        break
