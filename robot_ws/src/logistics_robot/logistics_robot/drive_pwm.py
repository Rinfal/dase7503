"""Open-loop forward/yaw mixing for empirically calibrated motor ports.

Physical wheel positions are M1 right-front, M2 left-front, M3 right-rear,
M4 left-rear. PWM output is not measured wheel speed.
"""
import math


class DrivePWM:
    def __init__(self, forward_signs, duty=180, forward_limit=0.08, yaw_limit=0.35):
        if len(forward_signs) != 4 or any(type(v) is not int or v not in (-1, 1) for v in forward_signs):
            raise ValueError("Each motor needs an observed physical forward sign")
        if type(duty) is not int or not 1 <= duty <= 600:
            raise ValueError("Bench duty must be 1..600 /1000")
        if any(not math.isfinite(v) or v <= 0 for v in (forward_limit, yaw_limit)):
            raise ValueError("Velocity scales must be positive and finite")
        self.signs = tuple(forward_signs)
        self.duty = duty
        self.forward_limit = forward_limit
        self.yaw_limit = yaw_limit

    def output(self, forward, lateral, angular):
        if any(not math.isfinite(v) for v in (forward, lateral, angular)):
            raise ValueError("Nonfinite drive request")
        if lateral != 0:
            raise ValueError("Lateral movement has not been calibrated")
        f = max(-1.0, min(1.0, forward/self.forward_limit))
        a = max(-1.0, min(1.0, angular/self.yaw_limit))
        physical = (f+a, f-a, f+a, f-a)
        scale = max(1.0, *(abs(v) for v in physical))
        return [round(value/scale*self.duty*sign) for value, sign in zip(physical, self.signs)]
