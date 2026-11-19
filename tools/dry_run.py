#!/usr/bin/env python3
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))

from logistics_robot.mission import Job, Mission, Pose, State


def main():
    jobs = [Job(rack, payload, Pose(1.5, 1, 0), Pose(0.5, 0.5 + index * 0.3, 0),
                Pose(1.0, 0.5 + index * 0.3, 0))
            for index, (rack, payload) in enumerate([
                ("A", "RACKA_4X6M"), ("B", "RACKB_AM9L"),
                ("C", "RACKC_K8Y3"), ("D", "RACKD_R983")])]
    mission = Mission(jobs)
    now = 0.0
    actions = mission.qr("START", now)
    while mission.state not in (State.FAULT, State.FINISHED):
        for action in actions:
            print(json.dumps({"time": now, "state": mission.state.value,
                              "action": action.kind, "token": action.token}, ensure_ascii=False))
        now += 1
        if mission.state == State.IDENTIFY:
            actions = mission.qr(mission.job.payload, now)
        elif mission.state == State.VERIFY_END:
            actions = mission.qr("END", now)
        else:
            actions = mission.complete(mission.token, True, now)
    print(json.dumps({"SIMULATION_ONLY": True, "state": mission.state.value,
                      "delivered": mission.delivered, "seconds": now,
                      "warning": "模拟确认全部动作成功，不连接硬件；坐标及用时不代表实车。"}, ensure_ascii=False))
    return 0 if mission.state == State.FINISHED else 1


if __name__ == "__main__":
    raise SystemExit(main())
