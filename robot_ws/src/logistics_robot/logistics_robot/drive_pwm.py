"""Open-loop mecanum mixing for empirically calibrated motor ports.

Physical wheel positions are M1 right-front, M2 left-front, M3 right-rear,
M4 left-rear. PWM output is not measured wheel speed.
"""
import math


class DrivePWM:
    def __init__(self, forward_signs, duty=180, forward_limit=0.08, yaw_limit=0.35, lateral_enabled=False, lateral_duty=None, forward_duty=None):
        if len(forward_signs) != 4 or any(type(v) is not int or v not in (-1, 1) for v in forward_signs):
            raise ValueError("Each motor needs an observed physical forward sign")
        if type(duty) is not int or not 1 <= duty <= 600:
            raise ValueError("Bench duty must be 1..600 /1000")
        lateral_duty = duty if lateral_duty is None else lateral_duty
        if type(lateral_duty) is not int or not 1 <= lateral_duty <= 800:
            raise ValueError("Lateral duty must be 1..800 /1000")
        forward_duty = duty if forward_duty is None else forward_duty
        if type(forward_duty) is not int or not 1 <= forward_duty <= 800:
            raise ValueError("Forward duty must be 1..800 /1000")
        if any(not math.isfinite(v) or v <= 0 for v in (forward_limit, yaw_limit)):
            raise ValueError("Velocity scales must be positive and finite")
        self.signs = tuple(forward_signs)
        self.duty = duty
        self.forward_limit = forward_limit
        self.yaw_limit = yaw_limit
        self.lateral_enabled = lateral_enabled
        self.lateral_duty = lateral_duty
        self.forward_duty = forward_duty

    def output(self, forward, lateral, angular, *, lateral_duty=None):
        if any(not math.isfinite(v) for v in (forward, lateral, angular)):
            raise ValueError("Nonfinite drive request")
        if lateral != 0 and not self.lateral_enabled:
            raise ValueError("Lateral movement has not been calibrated")
        lateral_duty = self.lateral_duty if lateral_duty is None else lateral_duty
        if type(lateral_duty) is not int or not 1 <= lateral_duty <= 800:
            raise ValueError("Lateral duty must be 1..800 /1000")
        f = max(-1.0, min(1.0, forward/self.forward_limit))
        a = max(-1.0, min(1.0, angular/self.yaw_limit))
        left = max(-1.0, min(1.0, lateral/self.forward_limit))
        # Standard X rollers: FR, FL, RR, RL. Positive lateral is left.
        f *= self.forward_duty
        a *= self.duty
        lateral_pwm = left * lateral_duty
        physical = (f+lateral_pwm+a, f-lateral_pwm-a,
                    f-lateral_pwm+a, f+lateral_pwm-a)
        # Each axis has its own output; normalize mixtures to the active ceiling.
        ceiling = max(self.duty, abs(f), abs(lateral_pwm))
        scale = max(1.0, *(abs(v)/ceiling for v in physical))
        return [round(value/scale*sign) for value, sign in zip(physical, self.signs)]
