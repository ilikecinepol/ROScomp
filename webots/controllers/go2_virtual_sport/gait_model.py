"""Planar foot trajectories for a quadruped body twist.

The trajectory is kinematic: during stance each foot moves opposite to the
velocity of its attachment point on the body.  This makes a pure yaw command
look like an in-place turn instead of four legs walking straight ahead.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, pi, sin


LEG_ANCHORS = {
    "FL": (0.1934, 0.1420),
    "FR": (0.1934, -0.1420),
    "RL": (-0.1934, 0.1420),
    "RR": (-0.1934, -0.1420),
}


@dataclass(frozen=True)
class FootTarget:
    x: float
    y: float
    down: float
    frequency: float


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def maximum_foot_speed(vx: float, vy: float, wz: float) -> float:
    """Return the fastest horizontal attachment-point speed in body axes."""
    return max(
        hypot(vx - wz * anchor_y, vy + wz * anchor_x)
        for anchor_x, anchor_y in LEG_ANCHORS.values()
    )


def foot_target(
    leg: str,
    cycle_time: float,
    vx: float,
    vy: float,
    wz: float,
    *,
    duty_factor: float = 0.62,
) -> FootTarget:
    """Compute one foot target for the commanded body twist.

    ``x`` and ``y`` are offsets from the neutral foot location. ``down`` is
    positive distance below the hip plane. Diagonal pairs share a phase.
    """
    anchor_x, anchor_y = LEG_ANCHORS[leg]
    speed = maximum_foot_speed(vx, vy, wz)
    frequency = 1.25 + 1.15 * _clamp(speed / 0.60, 0.0, 1.0)
    phase_offset = 0.0 if leg in ("FL", "RR") else 0.5
    phase = (frequency * cycle_time + phase_offset) % 1.0

    # The body velocity at this foot is (vx - wz*y, vy + wz*x). During
    # stance, the foot moves by the opposite velocity and stays planted in
    # the world. Each component is independently limited to the safe IK box.
    span_x = _clamp((vx - wz * anchor_y) * duty_factor / frequency, -0.155, 0.155)
    span_y = _clamp((vy + wz * anchor_x) * duty_factor / frequency, -0.075, 0.075)
    lift = 0.055 + 0.025 * _clamp(speed / 0.60, 0.0, 1.0)

    if phase < duty_factor:
        progress = phase / duty_factor
        factor = 0.5 - progress
        down = 0.318
    else:
        progress = (phase - duty_factor) / (1.0 - duty_factor)
        smooth = progress**3 * (10.0 - 15.0 * progress + 6.0 * progress**2)
        factor = -0.5 + smooth
        down = 0.318 - lift * sin(pi * progress) ** 2
    return FootTarget(span_x * factor, span_y * factor, down, frequency)
