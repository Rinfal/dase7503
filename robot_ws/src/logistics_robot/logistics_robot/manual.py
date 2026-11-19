import math
from dataclasses import dataclass

from .safety import Velocity


@dataclass(frozen=True)
class ManualLimits:
    max_linear: float = 0.08
    max_angular: float = 0.35
    command_timeout: float = 0.25
    feedback_timeout: float = 0.5

    def __post_init__(self):
        if any(not math.isfinite(value) or value <= 0 for value in self.__dict__.values()):
            raise ValueError("遥控安全参数必须是有限正数")


class ManualGate:
    def __init__(self, limits=None):
        self.limits = limits or ManualLimits()
        self.estop = True
        self.armed = False
        self.feedback_time = None
        self.command_time = None
        self.command = Velocity()
        self.reason = "emergency_stop"

    def disarm(self):
        self.armed = False
        self.command = Velocity()
        self.command_time = None

    def emergency_stop(self, active):
        self.estop = bool(active)
        self.disarm()

    def update_feedback(self, valid, now):
        self.feedback_time = now if valid else None

    def update_command(self, command, now):
        if not command.finite():
            self.disarm()
            self.reason = "invalid_command"
            return False
        self.command = command
        self.command_time = now
        return True

    def ready(self, now):
        if self.feedback_time is None or not 0 <= now - self.feedback_time <= self.limits.feedback_timeout:
            return False, "controller_feedback_stale"
        return True, "ready"

    def arm(self, now):
        if self.estop:
            return False, "emergency_stop"
        ready, reason = self.ready(now)
        if not ready:
            return False, reason
        self.command = Velocity()
        self.command_time = now
        self.armed = True
        self.reason = "armed"
        return True, self.reason

    def output(self, now):
        if self.estop:
            self.reason = "emergency_stop"
            return Velocity()
        if not self.armed:
            self.reason = "disarmed"
            return Velocity()
        ready, reason = self.ready(now)
        if not ready:
            self.disarm()
            self.reason = reason
            return Velocity()
        if self.command_time is None or not 0 <= now - self.command_time <= self.limits.command_timeout:
            self.disarm()
            self.reason = "command_stale"
            return Velocity()
        speed = math.hypot(self.command.forward, self.command.lateral)
        factor = min(1.0, self.limits.max_linear / speed) if speed else 1.0
        rotation = max(-self.limits.max_angular, min(self.command.angular, self.limits.max_angular))
        self.reason = "running"
        return Velocity(self.command.forward * factor, self.command.lateral * factor, rotation)
