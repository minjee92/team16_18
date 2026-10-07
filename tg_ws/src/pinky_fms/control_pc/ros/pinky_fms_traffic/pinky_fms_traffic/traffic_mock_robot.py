"""지도를 아는 시험용 가짜 로봇 (pinky_fms_core 의 mock_robot 은 벽을 무시하고 직선으로 간다).

실제 Nav2 처럼 /<ns>/navigate_to_pose (새 목표가 오면 이전 목표를 대체, cancel 지원) 와
/<ns>/compute_path_to_pose 를 제공하고, A* 경로를 따라 speed 로 이동한다.
  ros2 run pinky_fms_traffic traffic_mock_robot --ros-args -p namespace:=amr_01 -p map_yaml:=/path/map.yaml
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

from .gridmap import GridMap
from .planner import astar


def to_path(pts, yaw_end=0.0):
    p = Path()
    p.header.frame_id = 'map'
    for i, (x, y) in enumerate(pts):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        if i + 1 < len(pts):
            yaw = math.atan2(pts[i + 1][1] - y, pts[i + 1][0] - x)
        else:
            yaw = yaw_end
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        p.poses.append(ps)
    return p


class TrafficMockRobot(Node):
    def __init__(self):
        super().__init__('traffic_mock_robot')
        self.declare_parameter('namespace', 'amr_01')
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('map_yaml', '')
        ns = self.get_parameter('namespace').value
        self.speed = float(self.get_parameter('speed').value)
        self.gm = GridMap(self.get_parameter('map_yaml').value)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.localized = False
        self.cur = 0                      # 현재 실행 중인 목표 번호 (새 목표가 오면 증가 → 이전 목표는 스스로 중단)
        self.batt_pub = self.create_publisher(Float32, f'/{ns}/battery/percent', 10)
        self.odom_pub = self.create_publisher(Odometry, f'/{ns}/odom', 10)
        self.amcl_pub = self.create_publisher(PoseWithCovarianceStamped, f'/{ns}/amcl_pose', 10)
        self.create_subscription(PoseWithCovarianceStamped, f'/{ns}/initialpose', self.on_initialpose, 10)
        self.create_timer(0.1, self.tick)
        cb = ReentrantCallbackGroup()
        self.nav = ActionServer(self, NavigateToPose, f'/{ns}/navigate_to_pose', self.execute,
                                goal_callback=self.on_goal, cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.cp = ActionServer(self, ComputePathToPose, f'/{ns}/compute_path_to_pose', self.compute,
                               goal_callback=lambda g: GoalResponse.ACCEPT, cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.tp = ActionServer(self, NavigateThroughPoses, f'/{ns}/navigate_through_poses', self.execute_through,
                               goal_callback=self.on_goal, cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.bu = ActionServer(self, BackUp, f'/{ns}/backup', self.backup,
                               goal_callback=self.on_goal, cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.get_logger().info(f'traffic mock robot /{ns} ready')

    def on_initialpose(self, m):
        q, p = m.pose.pose.orientation, m.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.localized = True

    def tick(self):
        now = self.get_clock().now().to_msg()
        self.batt_pub.publish(Float32(data=90.0))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        t = time.monotonic()      # 실물처럼 속도도 채운다 (fleet_traffic 이 서 있는지 판단할 때 쓴다)
        prev = getattr(self, '_odom_prev', None)
        if prev is not None and t > prev[2]:
            o.twist.twist.linear.x = math.hypot(self.x - prev[0], self.y - prev[1]) / (t - prev[2])
        self._odom_prev = (self.x, self.y, t)
        self.odom_pub.publish(o)
        if self.localized:
            a = PoseWithCovarianceStamped()
            a.header.stamp, a.header.frame_id = now, 'map'
            a.pose.pose.position.x, a.pose.pose.position.y = self.x, self.y
            a.pose.pose.orientation.z, a.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            self.amcl_pub.publish(a)

    def on_goal(self, g):
        self.cur += 1                      # 이전 목표를 대체
        return GoalResponse.ACCEPT

    def backup(self, gh):
        """Nav2 BackUp 처럼 방향을 유지한 채 target.x 만큼 뒤로 간다"""
        mine = self.cur
        dist, sp = abs(gh.request.target.x), abs(gh.request.speed) or 0.05
        done, dt = 0.0, 0.1
        while done < dist:
            if gh.is_cancel_requested:
                gh.canceled()
                return BackUp.Result()
            if self.cur != mine:
                gh.abort()
                return BackUp.Result()
            step = min(sp * dt, dist - done)
            nx, ny = self.x - step * math.cos(self.yaw), self.y - step * math.sin(self.yaw)
            if self.gm.clearance_at(np.array([nx, ny]))[0] < 0.07:    # 뒤에 벽
                gh.abort()
                return BackUp.Result()
            self.x, self.y = float(nx), float(ny)
            done += step
            time.sleep(dt)
        gh.succeed()
        return BackUp.Result()

    def compute(self, gh):
        s, g = gh.request.start.pose.position, gh.request.goal.pose.position
        start = (s.x, s.y) if gh.request.use_start else (self.x, self.y)
        pts = astar(self.gm, start, (g.x, g.y))
        res = ComputePathToPose.Result()
        if pts is None:
            gh.abort()
            return res
        res.path = to_path(pts)
        gh.succeed()
        return res

    def execute_through(self, gh):
        """경유점을 차례로 지나 마지막 점까지 (실제 Nav2 처럼 경유점 사이는 각각 계획)"""
        mine, pts, cur = self.cur, [], (self.x, self.y)
        for ps in gh.request.poses:
            g = (ps.pose.position.x, ps.pose.position.y)
            seg = astar(self.gm, cur, g)
            if seg is None:
                gh.abort()
                return NavigateThroughPoses.Result()
            pts.append(seg if not pts else seg[1:])
            cur = g
        return self._drive(gh, mine, np.vstack(pts), NavigateThroughPoses)

    def execute(self, gh):
        mine = self.cur
        g = gh.request.pose.pose
        goal = (g.position.x, g.position.y)
        pts = astar(self.gm, (self.x, self.y), goal)
        if pts is None:
            gh.abort()
            return NavigateToPose.Result()
        return self._drive(gh, mine, pts, NavigateToPose)

    def _drive(self, gh, mine, pts, Action):
        seg = np.hypot(*np.diff(pts, axis=0).T) if len(pts) > 1 else np.array([0.0])
        total = float(seg.sum())
        i, fb, dt = 0.0, Action.Feedback(), 0.1
        while True:
            if gh.is_cancel_requested:
                gh.canceled()
                return Action.Result()
            if self.cur != mine:              # 새 목표가 왔다
                gh.abort()
                return Action.Result()
            if i >= len(pts) - 1 - 1e-9:
                break
            i = min(len(pts) - 1, i + self.speed * dt / 0.02)
            p = pts[int(i)]
            self.yaw = math.atan2(p[1] - self.y, p[0] - self.x) if np.hypot(p[0] - self.x, p[1] - self.y) > 1e-6 else self.yaw
            self.x, self.y = float(p[0]), float(p[1])
            fb.distance_remaining = float(max(total - i * 0.02, 0.0))
            gh.publish_feedback(fb)
            time.sleep(dt)
        gh.succeed()
        return Action.Result()


def main():
    rclpy.init()
    node = TrafficMockRobot()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
