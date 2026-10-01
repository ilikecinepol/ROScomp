# True Tech Arena — Webots scene

Real-robot command limits and unresolved measurements are tracked in
[`REAL_GO2_MEASURED_LIMITS.md`](REAL_GO2_MEASURED_LIMITS.md). The Webots launch
files explicitly arm the safety mux; its standalone defaults remain
observe-only and dry-run for hardware bring-up.

The project contains a real-scale Webots reconstruction of the current Blender layout.

## Open the world

Open `worlds/truetech_arena.wbt` in Webots R2025a or newer. From a terminal with Webots on `PATH`:

```powershell
webots .\worlds\truetech_arena.wbt
```

The world uses the ENU coordinate convention: X east, Y north, Z up. Distances are in metres, matching Blender and ROS.

## Scene structure

- `TrueTechArena.proto` assembles the room, floor, fixed obstacles and slalom.
- `Teeter.proto` contains a passive `HingeJoint`, a 6 kg board and ±12° hard stops. An off-centre load makes the loaded end descend under gravity.
- `SwingBridge.proto` contains a passive lateral hinge, an 8 kg deck and ±8° hard stops.
- Fixed collision geometry uses boxes and cylinders rather than render meshes.

The normal arena world has no controller and produces no supervisor warnings. To check the teeter physics independently, open `worlds/teeter_physics_test.wbt` or run it in batch mode. Its supervisor drops a 12 kg test load onto one end and fails unless the board rotates by at least 3 degrees.

The imported Unitree Go2 starts on the marker at `(4.25, -2.75)` and faces into the course. `Go2.proto` contains the 12 actuated joints, collision bodies, masses and visual DAE meshes converted from the project's Apache-2.0 `go2_robot_sdk` URDF. In the arena, `go2_virtual_sport` accepts high-level body velocity commands. Keyboard controls are `W/S` forward/back, `A/D` turn, `Q/E` lateral motion and `Space` stop.

After starting the simulation, click once inside the 3D viewport so Webots sends keyboard events to the controller. Hold a movement key; releasing it returns the robot to a standing pose after a short timeout.

The virtual Sport Mode applies acceleration limits and a command timeout. It is intended for perception, navigation and mission testing against the same `vx/vy/wz` contract used by the real Go2. The lower-level `go2_teleop` crawl controller and its test remain available for contact experiments.

The ROS 2 package under `ros2_ws/src/go2_webots_bridge` subscribes to `/cmd_vel` and forwards `geometry_msgs/Twist` to Webots. The same bridge publishes the simulated sensors as `sensor_msgs/LaserScan` on `/scan`, `sensor_msgs/Image` on `/camera/image_raw`, and `sensor_msgs/CameraInfo` on `/camera/camera_info`. Build it with `colcon build`, source `install/setup.bash`, and run `ros2 launch go2_webots_bridge bridge.launch.py`.

For a `twist_mux` setup whose output is `/cmd_vel_out`, run:

```bash
ros2 launch go2_webots_bridge bridge.launch.py topic:=/cmd_vel_out
```

When ROS 2 runs in WSL or another machine, pass the Windows host address:

```bash
ros2 launch go2_webots_bridge bridge.launch.py host:=192.168.1.10
```

Quick command test:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.35}, angular: {z: 0.0}}"
```

Use `worlds/go2_spawn_test.wbt` for an isolated import and standing-pose smoke test.
Use `worlds/go2_teleop_test.wbt` for the automatic forward-gait test.

Run all deterministic Webots checks headlessly and save a machine-readable
summary with:

```powershell
uv run --no-project python .\webots\run_regression.py
```

If Python is already on `PATH`, `python .\webots\run_regression.py` is
equivalent.

The regression covers model spawn, the legacy contact teleop, virtual Sport
motion, the measured r6 high-level response, collision stopping, ramp response,
the kinematic A-frame course profile, visual gait, and passive teeter physics.
`truetech_arena.wbt` loads `config/go2_r6_measured_response.json`: a requested
0.35 m/s becomes 0.29995 m/s, while requested yaw 0.35 rad/s becomes
0.143675 rad/s left and 0.08015 rad/s right. Reverse and lateral motion are
disabled because they were not confirmed by the supplied hardware evidence.

This is equivalence only for the measured high-level response. It does not
claim unmeasured traction, command latency, balance, braking, battery effects,
robot-to-robot variation, or the competition ROS driver.

## ROS 2 integration

Keep the arena world separate from the robot controller package. Launch Webots through `webots_ros2_driver`, inject the chosen Go2 URDF/PROTO and map ROS topics at the driver boundary. The arena itself does not depend on ROS packages.

The local development environment uses Ubuntu 24.04 under WSL2 with ROS 2 Jazzy. Webots remains a native Windows application. Open Ubuntu from the Start menu, then verify the environment with:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix go2_webots_bridge
```

To run the complete command path, first open `worlds/truetech_arena.wbt` in Webots and start the simulation. Then open Ubuntu and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
WINDOWS_HOST="$(ip route show default | awk '{print $3; exit}')"
ros2 launch go2_webots_bridge bridge.launch.py host:="$WINDOWS_HOST"
```

In a second Ubuntu terminal, publish a short motion command:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.25}, angular: {z: 0.35}}'
```

Stop the publisher with `Ctrl+C`; the Webots controller times out the command and returns the robot to idle. The helper `tools/test_cmd_vel_bridge.sh` performs the same bridge and publisher smoke test while the arena is running.

Inspect the simulated sensors with:

```bash
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
rqt_image_view /camera/image_raw
```

For the lidar in RViz, run `rviz2`, set **Fixed Frame** to `lidar_link`, add a **LaserScan** display and select `/scan`. The isolated `worlds/go2_sensor_test.wbt` world and `tools/test_sensor_bridge.sh` validate both sensor streams without occupying the arena command port.

The ready-made visualization starts the bridge and RViz with the correct fixed frame, LaserScan and camera displays:

```bash
WINDOWS_HOST="$(ip route show default | awk '{print $3; exit}')"
ros2 launch go2_webots_bridge visualize.launch.py host:="$WINDOWS_HOST"
```

From PowerShell, the equivalent shortcut is:

```powershell
.\webots\run_rviz.ps1
```

## SLAM navigation from an RViz goal

The complete mapping and navigation stack starts Webots, the UDP bridge, SLAM Toolbox,
Nav2 and a browser goal map from one Windows PowerShell command:

```powershell
cd "C:\Users\drmma\Documents\ChatGPT\ROS2 Competition"
powershell -ExecutionPolicy Bypass -File ".\webots\run_slam_navigation.ps1"
```

Allow the lidar map to appear, press the left mouse button on a light free cell and
drag the arrow to set the final heading. Nav2 plans around mapped obstacles and sends
the resulting velocity commands to the simulated Go2 on `/cmd_vel`.

The arena obstacles have Webots collision bodies. The virtual Sport Mode uses physics-resolved linear motion so walls, poles and ramps block or lift the robot instead of allowing it to teleport through them. Rotation remains a high-level yaw command because this model does not yet simulate a full Unitree walking gait. Regression worlds `go2_collision_test.wbt` and `go2_ramp_test.wbt` verify stopping at a solid obstacle and climbing an incline.

The Linux workspace at `~/ros2_ws` links its source package to the Windows project, so editing `webots/ros2_ws/src/go2_webots_bridge` and rebuilding with `colcon build --symlink-install` updates the same code.

## Known calibration limits

The obstacle dimensions follow the competition specification and the accepted Blender model. Room dimensions and absolute obstacle positions still depend on the photo-derived estimate. Update only the top-level `translation` fields when measured coordinates become available.
