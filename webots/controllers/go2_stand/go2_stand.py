"""Keep the imported Unitree Go2 in a neutral standing pose.

This deliberately small controller is a bring-up controller. It proves that
all 12 imported joints are addressable and keeps the robot upright while the
ROS 2 driver/controller is prepared.
"""

from controller import Robot


robot = Robot()
timestep = int(robot.getBasicTimeStep())

joint_targets = {
    "FL_hip_joint": 0.0,
    "FL_thigh_joint": 0.8,
    "FL_calf_joint": -1.5,
    "FR_hip_joint": 0.0,
    "FR_thigh_joint": 0.8,
    "FR_calf_joint": -1.5,
    "RL_hip_joint": 0.0,
    "RL_thigh_joint": 0.8,
    "RL_calf_joint": -1.5,
    "RR_hip_joint": 0.0,
    "RR_thigh_joint": 0.8,
    "RR_calf_joint": -1.5,
}

motors = []
for name, target in joint_targets.items():
    motor = robot.getDevice(name)
    motor.setVelocity(3.0)
    motor.setPosition(target)
    motors.append(motor)

print(f"[go2_stand] Holding {len(motors)} joints in the neutral standing pose.", flush=True)

while robot.step(timestep) != -1:
    pass
