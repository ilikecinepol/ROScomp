"""Measured high-level Go2 command response used by the Webots twin.

Only axes backed by recorded r6 calibration evidence are enabled. Unknown axes
are deliberately mapped to zero so simulation cannot silently demonstrate a
capability that was not observed on hardware.
"""

from __future__ import annotations

import json
import math
from pathlib import Path


def _finite_number(mapping: dict, name: str) -> float:
    value = mapping.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Response profile field {name!r} must be numeric")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"Response profile field {name!r} must be finite")
    return value


class ResponseProfile:
    def __init__(self, data: dict, path: Path):
        if data.get("schema_version") != 1:
            raise ValueError("Unsupported response profile schema")
        self.path = path
        self.profile_id = str(data.get("profile_id") or "")
        if not self.profile_id:
            raise ValueError("Response profile_id is required")

        limits = data.get("command_limits") or {}
        gains = data.get("gains") or {}
        self.min_vx = _finite_number(limits, "min_vx_mps")
        self.max_vx = _finite_number(limits, "max_vx_mps")
        self.max_vy = _finite_number(limits, "max_vy_mps")
        self.max_wz = _finite_number(limits, "max_wz_rps")
        self.forward_gain = _finite_number(gains, "forward")
        self.reverse_gain = _finite_number(gains, "reverse")
        self.lateral_gain = _finite_number(gains, "lateral")
        self.left_gain = _finite_number(gains, "yaw_left")
        self.right_gain = _finite_number(gains, "yaw_right")
        values = (
            self.min_vx, self.max_vx, self.max_vy, self.max_wz,
            self.forward_gain, self.reverse_gain, self.lateral_gain,
            self.left_gain, self.right_gain,
        )
        if self.min_vx > 0.0 or self.max_vx <= 0.0:
            raise ValueError("Response profile vx limits are invalid")
        if any(value < 0.0 for value in values):
            raise ValueError("Response profile limits and gains must be nonnegative")

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def realize(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        requested_vx = self._clamp(float(vx), self.min_vx, self.max_vx)
        requested_vy = self._clamp(float(vy), -self.max_vy, self.max_vy)
        requested_wz = self._clamp(float(wz), -self.max_wz, self.max_wz)
        if not all(math.isfinite(value) for value in (requested_vx, requested_vy, requested_wz)):
            raise ValueError("Non-finite velocity command")
        vx_gain = self.forward_gain if requested_vx >= 0.0 else self.reverse_gain
        yaw_gain = self.left_gain if requested_wz >= 0.0 else self.right_gain
        return (
            requested_vx * vx_gain,
            requested_vy * self.lateral_gain,
            requested_wz * yaw_gain,
        )


def load_response_profile(path: str | Path) -> ResponseProfile:
    profile_path = Path(path).resolve()
    data = json.loads(profile_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Response profile root must be an object")
    return ResponseProfile(data, profile_path)
