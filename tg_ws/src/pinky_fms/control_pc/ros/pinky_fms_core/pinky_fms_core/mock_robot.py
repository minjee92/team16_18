"""테스트용 가짜 로봇. 로봇이 없어도 실제 로봇과 같은 토픽/액션 이름(/<namespace>/...)을 흉내 낸다.

  ros2 run pinky_fms_core mock_robot --ros-args -p namespace:=amr_01
발행: battery/percent, odom, amcl_pose / 액션 서버: navigate_to_pose (직선 이동, cancel 지원)
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32


class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        self.declare_parameter('namespace', 'amr_01')
        self.declare_parameter('speed', 0.3)
        ns = self.get_parameter('namespace').value
        self.speed = float(self.get_parameter('speed').value)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.batt = 87.0
        self.localized = False   # 실제 AMCL 처럼 /initialpose 를 받기 전에는 amcl_pose 를 발행하지 않는다

        self.batt_pub = self.create_publisher(Float32, f'/{ns}/battery/percent', 10)
        self.odom_pub = self.create_publisher(Odometry, f'/{ns}/odom', 10)
        self.amcl_pub = self.create_publisher(PoseWithCovarianceStamped, f'/{ns}/amcl_pose', 10)
        self.create_subscription(PoseWithCovarianceStamped, f'/{ns}/initialpose', self.on_initialpose, 10)
        self.create_timer(0.2, self.tick)
        self.create_timer(5.0, self.drain)

        self.server = ActionServer(
            self, NavigateToPose, f'/{ns}/navigate_to_pose', self.execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'mock robot /{ns} ready')

    def on_initialpose(self, m):
        q, p = m.pose.pose.orientation, m.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.localized = True
        self.get_logger().info(f'initialpose set: ({self.x:.2f}, {self.y:.2f})')

    def drain(self):
        self.batt = max(0.0, self.batt - 0.2)

    def tick(self):
        now = self.get_clock().now().to_msg()
        self.batt_pub.publish(Float32(data=self.batt))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        self.odom_pub.publish(o)
        if not self.localized:
            return
        a = PoseWithCovarianceStamped()
        a.header.stamp, a.header.frame_id = now, 'map'
        a.pose.pose.position.x, a.pose.pose.position.y = self.x, self.y
        a.pose.pose.orientation.z, a.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.amcl_pub.publish(a)

    def execute(self, gh):
        tx = gh.request.pose.pose.position.x
        ty = gh.request.pose.pose.position.y
        fb = NavigateToPose.Feedback()
        dt = 0.1
        while True:
            if gh.is_cancel_requested:
                gh.canceled()
                return NavigateToPose.Result()
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                break
            step = min(self.speed * dt, dist)
            self.yaw = math.atan2(dy, dx)
            self.x += step * dx / dist
            self.y += step * dy / dist
            fb.distance_remaining = float(dist)
            gh.publish_feedback(fb)
            time.sleep(dt)
        gh.succeed()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = MockRobot()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
