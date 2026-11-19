import json
import math
import time
import uuid
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .mission import Job, Mission, Pose, State


def load_jobs(filename):
    with Path(filename).expanduser().open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if config.get("survey_confirmed") is not True:
        raise ValueError("场地测量及本组二维码尚未确认，禁止启动实际任务")
    jobs = []
    for item in config["jobs"]:
        poses = {key: Pose(**item[key]) for key in ("pickup", "drop", "retreat")}
        jobs.append(Job(item["rack"], item["payload"], **poses))
    return jobs


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission")
        filename = self.declare_parameter("field_file", "").value
        self.ready = self.declare_parameter("hardware_ready", False).value
        self.mission = Mission(load_jobs(filename))
        self.session = uuid.uuid4().hex
        self.navigation = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None
        self.requests = self.create_publisher(String, "mission/requests", 10)
        self.status = self.create_publisher(String, "mission/status", 10)
        self.velocity = self.create_publisher(Twist, "cmd_vel_requested", 1)
        self.disarm = self.create_client(Trigger, "safety/disarm")
        self.pending_safety_stop = False
        self.safety_stamp = None
        self.safety_armed = False
        self.create_subscription(String, "qr/detections", self.qr, 10)
        self.create_subscription(String, "mission/feedback", self.feedback, 10)
        self.create_subscription(String, "safety/status", self.safety, 10)
        self.create_timer(0.05, self.tick)
        self.previous_status = None

    def safety(self, message):
        try:
            status = json.loads(message.data)
            self.safety_armed = status.get("armed") is True and status.get("estop") is False
            self.safety_stamp = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            self.safety_armed = False

    def qr(self, message):
        now = time.monotonic()
        if (not self.ready or not self.safety_armed or self.safety_stamp is None
                or now - self.safety_stamp > 1):
            return
        try:
            payload = json.loads(message.data)["payload"]
        except (ValueError, KeyError, TypeError):
            return
        self.dispatch(self.mission.qr(payload, now))

    def feedback(self, message):
        try:
            event = json.loads(message.data)
            if event["session"] != self.session or event["token"] != self.mission.token:
                return
            expected = {State.DOCK: "dock", State.LIFT: "lift", State.LOWER: "lift"}
            if event["kind"] != expected.get(self.mission.state):
                return
            if type(event["success"]) is not bool:
                return
            actions = self.mission.complete(event["token"], event["success"],
                                            time.monotonic(), str(event.get("detail", "")))
        except (ValueError, KeyError, TypeError):
            return
        self.dispatch(actions)

    def dispatch(self, actions):
        for action in actions:
            self.get_logger().info(f"{self.mission.state.value}: {action.kind} #{action.token}")
            if action.kind == "navigate":
                self.navigate(action)
            elif action.kind == "stop":
                self.velocity.publish(Twist())
                self.pending_safety_stop = True
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
            elif action.kind in ("dock", "lift", "scan"):
                self.velocity.publish(Twist())
                request = {"session": self.session, "token": action.token,
                           "kind": action.kind, "value": action.value}
                self.requests.publish(String(data=json.dumps(request)))

    def navigate(self, action):
        if not self.navigation.server_is_ready():
            self.dispatch(self.mission.abort("nav2_unavailable", time.monotonic()))
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = action.value.x
        goal.pose.pose.position.y = action.value.y
        goal.pose.pose.orientation.z = math.sin(action.value.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(action.value.yaw / 2)
        future = self.navigation.send_goal_async(goal)
        future.add_done_callback(lambda result: self.goal_response(result, action.token))

    def goal_response(self, future, token):
        try:
            handle = future.result()
            if token != self.mission.token or self.mission.state == State.FAULT:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.dispatch(self.mission.complete(token, False, time.monotonic(), "nav_rejected"))
                return
            self.goal_handle = handle
            result = handle.get_result_async()
            result.add_done_callback(lambda response: self.goal_result(response, token))
        except Exception as error:
            self.dispatch(self.mission.abort("nav_exception:" + str(error), time.monotonic()))

    def goal_result(self, future, token):
        try:
            result = future.result()
            if token != self.mission.token:
                return
            self.goal_handle = None
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self.dispatch(self.mission.complete(token, success, time.monotonic(), "nav_result"))
        except Exception as error:
            self.dispatch(self.mission.abort("nav_exception:" + str(error), time.monotonic()))

    def tick(self):
        now = time.monotonic()
        if self.mission.started is not None and self.mission.state not in (State.FAULT, State.FINISHED):
            if self.safety_stamp is None or now - self.safety_stamp > 1 or not self.safety_armed:
                self.dispatch(self.mission.abort("safety_not_armed", now))
        self.dispatch(self.mission.tick(now))
        if self.mission.state not in (State.NAV_PICKUP, State.NAV_DROP, State.RETREAT, State.DOCK):
            self.velocity.publish(Twist())
        if self.pending_safety_stop and self.disarm.service_is_ready():
            self.disarm.call_async(Trigger.Request())
            self.pending_safety_stop = False
        status = json.dumps({"session": self.session, "state": self.mission.state.value,
                             "token": self.mission.token, "reason": self.mission.reason,
                             "delivered": self.mission.delivered, "carrying": self.mission.carrying})
        if status != self.previous_status:
            self.status.publish(String(data=status))
            self.previous_status = status


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.dispatch(node.mission.abort("operator_exit", time.monotonic()))
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
