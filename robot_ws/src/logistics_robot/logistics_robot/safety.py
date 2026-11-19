import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Velocity:
    forward: float = 0.0
    lateral: float = 0.0
    angular: float = 0.0

    def finite(self):
        return all(math.isfinite(value) for value in (self.forward, self.lateral, self.angular))


@dataclass(frozen=True)
class SafetyLimits:
    max_linear: float = 0.15
    max_angular: float = 0.5
    command_timeout: float = 0.25
    scan_timeout: float = 0.3
    odom_timeout: float = 0.3
    robot_radius: float = 0.22
    clearance: float = 0.08
    reaction_time: float = 0.25
    braking_acceleration: float = 0.3
    minimum_coverage: float = 0.9

    def __post_init__(self):
        if any(not math.isfinite(value) or value <= 0 for value in self.__dict__.values()):
            raise ValueError("安全参数必须是有限正数")
        if self.minimum_coverage > 1:
            raise ValueError("激光覆盖率不能大于 1")


class SafetyGate:
    def __init__(self, limits=None):
        self.limits = limits or SafetyLimits()
        self.armed = False
        self.estop = True
        self.command = Velocity()
        self.command_time = None
        self.scan_time = None
        self.odom_time = None
        self.nearest = 0.0
        self.coverage = 0.0
        self.reason = "emergency_stop"

    def disarm(self):
        self.armed = False
        self.command = Velocity()
        self.command_time = None

    def emergency_stop(self, active):
        self.estop = bool(active)
        self.disarm()

    def update_command(self, command, now):
        self.command = command
        self.command_time = now

    def update_odom(self, valid, now):
        self.odom_time = now if valid else None

    def update_scan(self, ranges, angle_increment, range_min, range_max, now):
        self.scan_time = None
        self.coverage = 0.0
        self.nearest = 0.0
        if (not ranges or not all(math.isfinite(value) for value in
                (angle_increment, range_min, range_max)) or angle_increment == 0
                or range_min < 0 or range_max <= range_min):
            return
        usable = [distance for distance in ranges
                  if (math.isfinite(distance) and range_min <= distance <= range_max)
                  or distance == math.inf]
        full_circle_samples = 2 * math.pi / abs(angle_increment)
        self.coverage = min(len(usable) / full_circle_samples, 1.0)
        if usable:
            self.nearest = min(usable)
            self.scan_time = now

    def ready(self, now):
        for name, stamp, timeout in (
                ("scan", self.scan_time, self.limits.scan_timeout),
                ("odom", self.odom_time, self.limits.odom_timeout)):
            if stamp is None or not 0 <= now - stamp <= timeout:
                return False, name + "_stale"
        if self.coverage < self.limits.minimum_coverage:
            return False, "scan_incomplete"
        if self.nearest <= self.limits.robot_radius + self.limits.clearance:
            return False, "obstacle"
        return True, "ready"

    def arm(self, now):
        ready, reason = self.ready(now)
        if self.estop:
            return False, "emergency_stop"
        if not ready:
            return False, reason
        self.command = Velocity()
        self.command_time = now
        self.armed = True
        return True, "armed"

    def output(self, now):
        if self.estop:
            return self._stop("emergency_stop")
        if not self.armed:
            return self._stop("disarmed")
        ready, reason = self.ready(now)
        if not ready:
            self.disarm()
            return self._stop(reason)
        if self.command_time is None or not 0 <= now - self.command_time <= self.limits.command_timeout:
            self.disarm()
            return self._stop("command_stale")
        if not self.command.finite():
            self.disarm()
            return self._stop("invalid_command")
        speed = math.hypot(self.command.forward, self.command.lateral)
        factor = min(1.0, self.limits.max_linear / speed) if speed else 1.0
        speed = min(speed, self.limits.max_linear)
        rotation = max(-self.limits.max_angular, min(self.command.angular, self.limits.max_angular))
        stopping_distance = (self.limits.robot_radius + self.limits.clearance
                             + speed * self.limits.reaction_time
                             + speed * speed / (2 * self.limits.braking_acceleration))
        if self.nearest <= stopping_distance:
            self.disarm()
            return self._stop("braking_distance")
        self.reason = "running"
        return Velocity(self.command.forward * factor, self.command.lateral * factor, rotation)

    def _stop(self, reason):
        self.reason = reason
        return Velocity()
