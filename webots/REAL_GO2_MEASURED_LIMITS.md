# Real Go2 measured limits

This file records what the 2026-09-29 training session can and cannot justify.
It is not a robot profile and must not be used to bypass a fresh ROS interface
probe on the assigned Raspberry Pi.

## Accepted for conservative software limits

- Three one-second `vx=0.35 m/s` trials produced 0.223-0.240 m of settled
  forward odometry. Two longer trials reached 0.570 and 0.573 m toward a
  0.60 m odometry target.
- Positive forward commands produced positive forward odometry.
- Stop requests were acknowledged and a settled pose was observed.
- The final command publisher uses a maximum 0.20 s request age, with an
  absolute configuration ceiling of 0.25 s.
- Until a successful lateral trial exists, the safety mux default is
  `max_vy=0.0`.

## Still unresolved

- The two arc trials were asymmetric. In-place rotation was not accepted as a
  repeatable response, so `0.35 rad/s` is only a ceiling, not a calibrated rate.
- Reported post-stop forward travel was about 0.051-0.067 m in the two free
  walks. No guaranteed braking distance exists.
- The old lidar source stamp was constant or too coarse to align scans with
  odometry. Real motion requires fresh source timestamps and TF at acquisition
  time.
- Returns intersected the nominal body footprint. Do not shrink the footprint
  or remove presumed self-returns before measuring the lidar-to-body transform.
- Camera evidence showed five training poles while lidar geometry confirmed
  fewer. Perception thresholds must not fabricate the missing poles.
- No obstacle speed, footprint, camera extrinsic, lidar extrinsic, covariance,
  or judge-start interface was validated.

## Command ownership

Only `go2_safety_cmd_mux` may publish the final `/cmd_vel` command:

- Nav2 publishes `/cmd_vel_nav`.
- Obstacle controllers publish `/cmd_vel_skill`.
- Supervised tests publish `/cmd_vel_teleop_test`.
- The mux publishes `/cmd_vel` and `/cmd_vel_safe_preview`.

The mux starts with `observe_only=true` and `dry_run=true`. A real launch must
change both values explicitly only after current robot identity, topic types,
QoS, timestamps, TF, state, and start authority have been verified. Webots
launch files explicitly arm it for simulation.
