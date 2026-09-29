"""Ограниченное восстановление доставки датчиков до начала движения."""
import math


class SensorRecovery:
    def __init__(self, started_at, max_attempts=2, cooldown=6.):
        self.started_at = started_at
        self.max_attempts = max_attempts
        self.cooldown = cooldown
        self.attempts = {}
        self.last_attempt = {}
        self.confirmed = {}

    def actions(self, now, received, moving_session=False):
        # После старта миссии потерю данных обрабатывает остановка исполнения.
        # Восстановление не должно самостоятельно повторно разрешать движение.
        if moving_session or not math.isfinite(now) or now < self.started_at:
            return []
        result = []
        for name, stamp in received.items():
            previous = self.last_attempt.get(name)
            if previous is not None and math.isfinite(stamp) and previous < stamp <= now:
                self.confirmed[name] = stamp
            timeout = 5. if name == 'CAMERA' else 2.
            if now-self.started_at < timeout:
                continue
            if math.isfinite(stamp) and 0 <= now-stamp < timeout:
                continue
            if self.attempts.get(name, 0) >= self.max_attempts:
                continue
            if previous is not None and now-previous < self.cooldown:
                continue
            self.attempts[name] = self.attempts.get(name, 0)+1
            self.last_attempt[name] = now
            result.append(name)
        return result

    def report(self):
        return {'attempts': dict(self.attempts),
                'transport_resumed_after_attempt': {
                    name: self.confirmed.get(name, -math.inf) > stamp
                    for name, stamp in self.last_attempt.items()},
                'measurement_freshness_verified': False,
                'motion_authorized': False}
