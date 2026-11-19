import math
from dataclasses import dataclass
from enum import Enum

from .qr import parse_qr


class State(str, Enum):
    WAIT_START = "wait_start"
    NAV_PICKUP = "nav_pickup"
    IDENTIFY = "identify"
    DOCK = "dock"
    LIFT = "lift"
    NAV_DROP = "nav_drop"
    VERIFY_END = "verify_end"
    LOWER = "lower"
    RETREAT = "retreat"
    FINISHED = "finished"
    FAULT = "fault"


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("位置必须是有限数值")


@dataclass(frozen=True)
class Job:
    rack: str
    payload: str
    pickup: Pose
    drop: Pose
    retreat: Pose

    def __post_init__(self):
        code = parse_qr(self.payload)
        if code is None or code.kind != "RACK" or code.rack != self.rack:
            raise ValueError("每个货架必须配置本组的完整二维码字符串")


@dataclass(frozen=True)
class Action:
    kind: str
    value: object = None
    token: int = 0


class Mission:
    def __init__(self, jobs, duration=180.0, stage_timeout=25.0):
        if not jobs or len({job.rack for job in jobs}) != len(jobs):
            raise ValueError("货架列表不能为空或重复")
        if not all(math.isfinite(value) and value > 0 for value in (duration, stage_timeout)):
            raise ValueError("任务时限必须是有限正数")
        self.jobs = list(jobs)
        self.duration = duration
        self.stage_timeout = stage_timeout
        self.state = State.WAIT_START
        self.started = None
        self.entered = None
        self.index = 0
        self.delivered = []
        self.carrying = False
        self.reason = ""
        self.token = 0

    @property
    def job(self):
        return self.jobs[self.index]

    def _enter(self, state, now, kind, value=None):
        self.state = state
        self.entered = now
        self.token += 1
        return [Action(kind, value, self.token)]

    def abort(self, reason, now):
        if self.state in (State.FINISHED, State.FAULT):
            return []
        self.reason = reason
        return self._enter(State.FAULT, now, "stop", reason)

    def tick(self, now):
        if self.started is None or self.state in (State.FINISHED, State.FAULT):
            return []
        if now < self.started or now - self.started >= self.duration:
            return self.abort("game_timeout", now)
        if now - self.entered >= self.stage_timeout:
            return self.abort("stage_timeout:" + self.state.value, now)
        return []

    def qr(self, payload, now):
        timeout = self.tick(now)
        if timeout:
            return timeout
        code = parse_qr(payload)
        if code is None:
            return []
        if self.state == State.WAIT_START and code.kind == "START":
            self.started = now
            return self._enter(State.NAV_PICKUP, now, "navigate", self.job.pickup)
        if self.state == State.IDENTIFY and payload == self.job.payload:
            return self._enter(State.DOCK, now, "dock", self.job.payload)
        if self.state == State.VERIFY_END and code.kind == "END":
            return self._enter(State.LOWER, now, "lift", "down")
        return []

    def complete(self, token, success, now, detail=""):
        timeout = self.tick(now)
        if timeout:
            return timeout
        if token != self.token or self.state in (State.FAULT, State.FINISHED, State.WAIT_START):
            return []
        if not success:
            return self.abort("action_failed:" + detail, now)
        if self.state == State.NAV_PICKUP:
            return self._enter(State.IDENTIFY, now, "scan", self.job.payload)
        if self.state == State.DOCK:
            return self._enter(State.LIFT, now, "lift", "up")
        if self.state == State.LIFT:
            self.carrying = True
            return self._enter(State.NAV_DROP, now, "navigate", self.job.drop)
        if self.state == State.NAV_DROP:
            return self._enter(State.VERIFY_END, now, "scan", "END")
        if self.state == State.LOWER:
            self.carrying = False
            return self._enter(State.RETREAT, now, "navigate", self.job.retreat)
        if self.state == State.RETREAT:
            self.delivered.append(self.job.rack)
            self.index += 1
            if self.index == len(self.jobs):
                return self._enter(State.FINISHED, now, "stop", "all_delivered")
            return self._enter(State.NAV_PICKUP, now, "navigate", self.job.pickup)
        return []
