"""Pure command arbitration and limiting used by the ROS safety mux."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Velocity:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass
class CommandSample:
    velocity: Velocity
    received_at: float


class SafetyPolicy:
    """Select a fresh source and apply conservative real-robot limits."""

    PRIORITY = ("teleop", "skill", "nav")

    def __init__(
        self,
        *,
        timeout_s=0.20,
        max_vx=0.35,
        max_vy=0.0,
        max_wz=0.35,
        max_linear_accel=0.25,
        max_yaw_accel=0.60,
    ):
        positive = (timeout_s, max_vx, max_wz, max_linear_accel, max_yaw_accel)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("Safety limits must be finite and positive")
        if not math.isfinite(max_vy) or max_vy < 0.0:
            raise ValueError("Lateral velocity limit must be finite and non-negative")
        if timeout_s > 0.25:
            raise ValueError("Command timeout must not exceed 0.25 s")
        self.timeout_s = timeout_s
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_wz = max_wz
        self.max_linear_accel = max_linear_accel
        self.max_yaw_accel = max_yaw_accel
        self.samples = {}
        self.previous = Velocity()
        self.previous_at = None

    def submit(self, source, velocity, now):
        if source not in self.PRIORITY:
            raise ValueError(f"Unknown command source: {source}")
        if not math.isfinite(now) or not all(
            math.isfinite(value) for value in (velocity.vx, velocity.vy, velocity.wz)
        ):
            raise ValueError("Command and receipt time must be finite")
        self.samples[source] = CommandSample(velocity, now)

    def command(self, now, *, enabled=True):
        if not math.isfinite(now):
            raise ValueError("Current time must be finite")
        source = None
        target = Velocity()
        if enabled:
            for candidate in self.PRIORITY:
                sample = self.samples.get(candidate)
                if sample is not None and 0.0 <= now - sample.received_at <= self.timeout_s:
                    source = candidate
                    target = self._clamp(sample.velocity)
                    break

        if source is None:
            self.previous = Velocity()
            self.previous_at = now
            return self.previous, None

        if self.previous_at is None or now <= self.previous_at:
            result = self.previous
        else:
            dt = min(now - self.previous_at, self.timeout_s)
            linear_step = self.max_linear_accel * dt
            yaw_step = self.max_yaw_accel * dt
            result = Velocity(
                self._approach(self.previous.vx, target.vx, linear_step),
                self._approach(self.previous.vy, target.vy, linear_step),
                self._approach(self.previous.wz, target.wz, yaw_step),
            )
        self.previous = result
        self.previous_at = now
        return result, source

    def stop(self, now):
        self.samples.clear()
        self.previous = Velocity()
        self.previous_at = now
        return self.previous

    def _clamp(self, velocity):
        return Velocity(
            max(-self.max_vx, min(self.max_vx, velocity.vx)),
            max(-self.max_vy, min(self.max_vy, velocity.vy)),
            max(-self.max_wz, min(self.max_wz, velocity.wz)),
        )

    @staticmethod
    def _approach(current, target, step):
        return max(current - step, min(current + step, target))
