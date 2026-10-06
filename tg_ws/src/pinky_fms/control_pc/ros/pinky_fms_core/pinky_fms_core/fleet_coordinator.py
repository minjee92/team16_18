"""조정 층: "누가, 언제 움직이는가".

Task 를 받아 놀고 있는(IDLE) 로봇을 골라 배정하고, 그 로봇의 Nav2(navigate_to_pose)에 목표만 전달한다.
로봇의 상태(배터리/위치/heartbeat)를 모아 /fleet/robot_status 로 내보낸다.
경로 계획·모터 제어는 하지 않는다 (실행 층 = 각 로봇의 Nav2).

로봇 목록은 고정하지 않는다. /<namespace>/battery/percent 또는 /<namespace>/odom 토픽이 보이면 그 namespace 를
로봇 ID 로 자동 등록한다. 그래서 GUI 에서 로봇을 추가해도 이 노드를 재시작할 필요가 없다.
"""
import math
import re
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32, String

from pinky_fms_interfaces.msg import RobotStatus, Task, TaskState
from .registry import load_registry

# /<namespace>/battery/percent 또는 /<namespace>/odom 을 내보내는 namespace 를 로봇으로 본다
ROBOT_TOPIC_RE = re.compile(r'^/([a-z][a-z0-9_]*)/(?:battery/percent|odom)$')


class Robot:
    """로봇 한 대의 관제 쪽 상태. 모든 토픽/액션 이름이 namespace 로만 결정된다."""

    def __init__(self, node, rid, ns):
        self.id = rid
        self.ns = ns
        self.battery = -1.0
        self.pose = (0.0, 0.0, 0.0)
        self.localized = False    # amcl_pose 를 받은 적이 있는가 (로봇이 OFFLINE 이 되면 다시 false)
        self.last_seen = None
        self.task = None          # 진행 중인 Task
        self.goal_handle = None
        self.client = ActionClient(node, NavigateToPose, f'/{ns}/navigate_to_pose')
        node.create_subscription(Float32, f'/{ns}/battery/percent', self._on_batt, 10)
        node.create_subscription(Odometry, f'/{ns}/odom', self._on_odom, 10)
        node.create_subscription(PoseWithCovarianceStamped, f'/{ns}/amcl_pose', self._on_amcl, 10)

    def _on_batt(self, m):
        self.battery = float(m.data)
        self.last_seen = time.monotonic()

    def _on_odom(self, m):
        self.last_seen = time.monotonic()

    def _on_amcl(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = m.pose.pose.position
        self.pose = (p.x, p.y, yaw)
        self.localized = True
        self.last_seen = time.monotonic()

    def age(self):
        return float('inf') if self.last_seen is None else time.monotonic() - self.last_seen


class FleetCoordinator(Node):
    def __init__(self):
        super().__init__('fleet_coordinator')
        self.declare_parameter('robots_file', '')
        path = self.get_parameter('robots_file').value
        cfg = load_registry(path) if path else {'robots': {}, 'heartbeat_timeout_sec': 3.0}
        self.timeout = float(cfg['heartbeat_timeout_sec'])
        self.robots = {}
        for rid in cfg['robots']:          # robots.yaml 의 고정 로봇은 처음부터 OFFLINE 으로 표시
            self.add_robot(rid)
        self.create_timer(2.0, self.discover)

        self.create_subscription(Task, '/fleet/task', self.on_task, 10)
        self.create_subscription(String, '/fleet/global_cmd', self.on_global_cmd, 10)
        self.state_pub = self.create_publisher(TaskState, '/fleet/task_state', 10)
        self.status_pub = self.create_publisher(RobotStatus, '/fleet/robot_status', 10)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f'fleet_coordinator ready, robots={list(self.robots)}')

    # ---------- 로봇 발견 ----------
    def add_robot(self, ns):
        if ns not in self.robots:
            self.robots[ns] = Robot(self, ns, ns)     # 로봇 ID = namespace
            self.get_logger().info(f'robot registered: /{ns}')

    def discover(self):
        for name, _ in self.get_topic_names_and_types():
            m = ROBOT_TOPIC_RE.match(name)
            if m:
                self.add_robot(m.group(1))

    # ---------- 상태 ----------
    def robot_state(self, r):
        if r.age() > self.timeout:
            return 'OFFLINE'
        return 'BUSY' if r.task else 'IDLE'

    def publish_status(self):
        for r in self.robots.values():
            age = r.age()
            if age > self.timeout:
                r.localized = False
            self.status_pub.publish(RobotStatus(
                robot_id=r.id, state=self.robot_state(r), battery_percent=r.battery,
                localized=r.localized, x=r.pose[0], y=r.pose[1], yaw=r.pose[2],
                current_task_id=r.task.task_id if r.task else '',
                last_seen_sec_ago=-1.0 if math.isinf(age) else float(age)))

    def emit(self, task, robot_id, state, dist=0.0, msg=''):
        self.state_pub.publish(TaskState(task_id=task.task_id, mission_id=task.mission_id,
                                         robot_id=robot_id, state=state,
                                         distance_remaining=float(dist), message=msg))

    # ---------- 배정 ----------
    def pick_robot(self, task):
        if task.robot_id:
            r = self.robots.get(task.robot_id)
            if r is None:
                return None, f'등록되지 않은 로봇: {task.robot_id}'
            st = self.robot_state(r)
            return (r, '') if st == 'IDLE' else (None, f'{r.id} 상태가 {st}')
        idle = [r for r in self.robots.values() if self.robot_state(r) == 'IDLE']
        if not idle:
            return None, '배정 가능한(IDLE) 로봇이 없음'
        gx, gy = task.goal.pose.position.x, task.goal.pose.position.y
        return min(idle, key=lambda r: math.hypot(r.pose[0] - gx, r.pose[1] - gy)), ''

    def on_task(self, task):
        if task.type == 'CANCEL':
            return self.cancel(task)
        r, why = self.pick_robot(task)
        if r is None:
            return self.emit(task, task.robot_id, 'FAILED', msg=why)
        if not r.client.wait_for_server(timeout_sec=1.0):
            return self.emit(task, r.id, 'FAILED', msg=f'/{r.ns}/navigate_to_pose 서버 없음 (Nav2 미실행?)')

        r.task = task
        self.emit(task, r.id, 'ASSIGNED')
        goal = NavigateToPose.Goal()
        goal.pose = task.goal
        fut = r.client.send_goal_async(goal, feedback_callback=lambda fb, r=r: self.on_feedback(r, fb))
        fut.add_done_callback(lambda f, r=r: self.on_goal_response(r, f))

    def on_goal_response(self, r, fut):
        gh = fut.result()
        if not gh.accepted:
            self.emit(r.task, r.id, 'FAILED', msg='로봇이 목표를 거절함')
            r.task = None
            return
        r.goal_handle = gh
        self.emit(r.task, r.id, 'RUNNING')
        gh.get_result_async().add_done_callback(lambda f, r=r: self.on_result(r, f))

    def on_feedback(self, r, fb):
        if r.task:
            self.emit(r.task, r.id, 'RUNNING', dist=fb.feedback.distance_remaining)

    def on_result(self, r, fut):
        status = fut.result().status
        task, r.task, r.goal_handle = r.task, None, None
        if task is None:
            return
        name = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(status, 'FAILED')
        self.emit(task, r.id, name)

    def on_global_cmd(self, msg):
        # E_STOP: 모든 로봇의 진행 중인 주행 목표를 취소한다 (Nav2 가 정지).
        # 주의: 이것은 소프트웨어 취소다. 안전용 비상정지는 로봇 쪽 watchdog/하드웨어가 따로 필요하다.
        if msg.data == 'E_STOP':
            self.get_logger().warn('GLOBAL E_STOP: cancel all goals')
            for r in self.robots.values():
                if r.goal_handle is not None:
                    r.goal_handle.cancel_goal_async()

    def cancel(self, task):
        targets = [self.robots[task.robot_id]] if task.robot_id in self.robots else list(self.robots.values())
        for r in targets:
            if r.goal_handle is not None:
                self.get_logger().info(f'cancel {r.id}')
                r.goal_handle.cancel_goal_async()


def main():
    rclpy.init()
    node = FleetCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
