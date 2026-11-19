"""Handshake the web control lease with the independent board drive guard."""
import json


def decode_status(payload, firmware):
    value = json.loads(payload)
    if not isinstance(value, dict) or value.get('firmware') != firmware:
        raise ValueError('Unexpected controller firmware')
    for key in ('armed', 'watchdog_timeout', 'servo_active'):
        if type(value.get(key)) is not bool:
            raise ValueError('Invalid controller flag')
    for key in ('pwm', 'encoders'):
        vector = value.get(key)
        if not isinstance(vector, list) or len(vector) != 4 or any(type(v) is not int for v in vector):
            raise ValueError('Invalid controller vector')
    if any(abs(v) > 600 for v in value['pwm']):
        raise ValueError('Invalid controller duty')
    if type(value.get('board_us')) is not int or value['board_us'] <= 0:
        raise ValueError('Invalid controller clock')
    return value


class PWMBridge:
    def __init__(self, mixer):
        self.mixer = mixer
        self.lease = ''
        self.phase = 'locked'
        self.started = 0.0
        self.board = None
        self.feedback_time = None
        self.stopped_since = None

    def feedback(self, status, now):
        previous = self.board
        reboot = previous is not None and status['board_us'] < previous['board_us']
        if previous is not None and status['board_us'] == previous['board_us']:
            return False, False
        self.board = status
        self.feedback_time = now
        stationary = (status['pwm'] == [0]*4 and previous is not None
                      and status['encoders'] == previous['encoders'])
        if not stationary or reboot:
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = now
        return not reboot, reboot

    def step(self, control, command, now):
        """Return (raw wheel PWM, optional arm message, optional S2 pulse)."""
        fresh = self.feedback_time is not None and 0 <= now-self.feedback_time < .3
        if not control.gate.armed or not control.lease or not fresh:
            if control.gate.armed and not fresh:
                control.stop()
            self.lease = ''
            self.phase = 'locked'
            return [0]*4, False, None
        if control.lease != self.lease:
            self.lease = control.lease
            self.phase = 'neutral'
            self.started = now
        if self.phase == 'neutral':
            if now-self.started < .12:
                return [0]*4, False, None
            self.phase = 'arming'
        if self.phase == 'arming':
            if self.board['armed']:
                self.phase = 'ready'
            elif now-self.started > .8:
                control.stop()
                self.phase = 'locked'
                return [0]*4, False, None
            else:
                return [0]*4, True, None
        if not self.board['armed'] or self.board['watchdog_timeout']:
            control.stop()
            self.phase = 'locked'
            return [0]*4, False, None
        if control.pending_lift is not None:
            if self.stopped_since is not None and now-self.stopped_since >= .2:
                pulse = control.pending_lift
                control.pending_lift = None
                # The board's two-second interlock starts on receipt, after
                # this publication. Leave transport time before driving.
                control.lift_block_until = max(control.lift_block_until, now+2.2)
                return [0]*4, None, pulse
            return [0]*4, None, None
        if now < control.lift_block_until:
            return [0]*4, None, None
        return self.mixer.output(command.forward, command.lateral, command.angular), None, None
