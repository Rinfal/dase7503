import math
import secrets

from .manual import ManualGate
from .safety import Velocity


class ControlError(ValueError):
    pass


class RemoteControl:
    def __init__(self, drive_enabled=False, lift_angles=None, limits=None, lateral_enabled=False):
        self.gate = ManualGate(limits)
        self.drive_enabled = drive_enabled
        self.lateral_enabled = lateral_enabled
        self.lift_angles = dict(lift_angles or {})
        self.lease = ""
        self.sequence = -1
        self.pending_lift = None
        self.last_lift = None
        self.lift_block_until = 0.0

    def stop(self):
        self.gate.emergency_stop(True)
        self.lease = ""
        self.pending_lift = None

    def arm(self, now):
        if self.gate.armed:
            raise ControlError("已有控制会话，请先锁定再重新解锁")
        if not self.drive_enabled:
            raise ControlError("底盘轮向或板端失联停车尚未验收，当前仅可查看")
        self.gate.emergency_stop(False)
        success, reason = self.gate.arm(now)
        if not success:
            self.stop()
            raise ControlError(reason)
        self.lease = secrets.token_urlsafe(18)
        self.sequence = -1
        return self.lease

    def validate_request(self, body, now):
        if not self.lease or not secrets.compare_digest(str(body.get("lease", "")), self.lease):
            raise ControlError("控制会话已失效，请重新解锁")
        sequence = body.get("sequence")
        clock = body.get("server_clock")
        if type(sequence) is not int or sequence <= self.sequence:
            raise ControlError("拒绝重复或乱序控制指令")
        if type(clock) not in (int, float) or not math.isfinite(clock) or not 0 <= now - clock <= 0.3:
            self.stop()
            raise ControlError("网络指令已过期，底盘已锁定")
        self.sequence = sequence
        self.gate.output(now)
        if not self.gate.armed:
            self.lease = ""
            raise ControlError("运动已锁定，请重新解锁")

    def drive(self, body, now):
        self.validate_request(body, now)
        values = [body.get(key, 0) for key in ("forward", "lateral", "angular")]
        if any(type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1
               for value in values):
            self.stop()
            raise ControlError("无效方向指令，底盘已锁定")
        if values[1] and not self.lateral_enabled:
            self.stop()
            raise ControlError("横移尚未验收，底盘已锁定")
        if now < self.lift_block_until and any(values):
            raise ControlError("升降指令执行期间禁止底盘移动")
        limits = self.gate.limits
        self.gate.update_command(Velocity(values[0] * limits.max_linear,
                                         values[1] * limits.max_linear,
                                         values[2] * limits.max_angular), now)

    def lift(self, body, now):
        self.validate_request(body, now)
        direction = body.get("direction")
        if direction not in self.lift_angles:
            raise ControlError("升降安全行程尚未标定，禁止发送舵机角度")
        if self.gate.command != Velocity() or now < self.lift_block_until:
            raise ControlError("请先停车，等待升降机构稳定")
        self.pending_lift = self.lift_angles[direction]
        self.last_lift = direction
        self.lift_block_until = now + 2.0
        self.gate.update_command(Velocity(), now)

    def output(self, now):
        velocity = self.gate.output(now)
        if not self.gate.armed:
            self.lease = ""
            self.pending_lift = None
        return velocity

    def status(self, now):
        self.output(now)
        feedback_age = None if self.gate.feedback_time is None else now - self.gate.feedback_time
        return {
            "armed": self.gate.armed,
            "estop": self.gate.estop,
            "reason": self.gate.reason,
            "drive_enabled": self.drive_enabled,
            "lateral_enabled": self.lateral_enabled,
            "lift_enabled": bool(self.lift_angles),
            "last_lift_command": self.last_lift,
            "lift_busy": now < self.lift_block_until,
            "feedback_age": feedback_age,
            "server_clock": now,
            "max_linear": self.gate.limits.max_linear,
            "max_angular": self.gate.limits.max_angular,
        }
