# 로봇을 일정 거리 이동 후 복귀시키기

import rclpy as rp
from rclpy.node import Node
import math
import time
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped


class PinkyOdometry(Node):

    def __init__(self):
        super().__init__('pinky_movement')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 로봇 위치 초기화
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # 타이머를 활용하여 오도메트리 구독하면 미션 실행
        self.timer = self.create_timer(0.5, self.check_ready_and_move)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 쿼터니언 >>> 오일러 변환
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w*q.z + q.y*q.x)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def check_ready_and_move(self):
        if self.odom_received:
            self.timer.cancel()
            # 별도의 thread에서 분리 실행
            mission_thread = threading.Thread(target=self.execute_mission)
            mission_thread.start()

    def execute_mission(self):
        twist = Twist()
        rate = self.create_rate(20)

        # 1.직진 단계
        self.get_logger().info('직진 주행을 시작합니다!')
        origin_x = self.current_x
        origin_y = self.current_y
        target_distance = 0.7

        while rp.ok():
            # 이동거리 계산
            distance = math.sqrt((self.current_x-origin_x)**2 + (self.current_y-origin_y)**2)

            if distance >= target_distance:
                break

            twist.linear.x = 0.5 # 직진 속도
            twist.angular.z = 0.0 # 회전 속도
            self.publisher.publish(twist)
            # rp.spin_once(self, timeout_sec=0.01)
            rate.sleep()

        # 직진 후 잠시 멈춤
        self.stop_robot()
        time.sleep(0.5)

        # 2.회전 단계
        self.get_logger().info('\n 1단계 직진 주행 완료! 2. 회전 주행을 시작합니다!!\n')
        origin_yaw = self.current_yaw
        target_rotation = math.pi # 180도 회전


        while rp.ok():
            # 회전 각도 계산 (음수 각 제외 및 -pi ~ pi 예외처리)
            yaw_diff = self.current_yaw - origin_yaw
            while yaw_diff > math.pi:
                yaw_diff -= 2*math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2*math.pi

            # 절댓값으로 회전량 비교
            if abs(yaw_diff) >= target_rotation-0.02: # 0.02 rad 허용 오차
                break

            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.publisher.publish(twist)
            # rp.spin_once(self, timeout_sec=0.01)
            rate.sleep()

        self.stop_robot()
        self.get_logger().info('\n 2단계 회전 주행 완료! 3. 복귀를 시작합니다!!\n')

        # 원 위치로 복귀
        while rp.ok():
            distance_to_origin = math.sqrt((self.current_x - origin_x)**2 + (self.current_y - origin_y)**2)

            if distance_to_origin <= 0.05: # 0.05m 허용 오차
                self.stop_robot()
                break

            twist.linear.x = 0.15
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            # rp.spin_once(self, timeout_sec=0.01)
            rate.sleep()


        while rp.ok():
                    # 회전 각도 계산 (음수 각 제외 및 -pi ~ pi 예외처리)
                    yaw_diff = self.current_yaw - origin_yaw
                    while yaw_diff > math.pi:
                        yaw_diff -= 2*math.pi
                    while yaw_diff < -math.pi:
                        yaw_diff += 2*math.pi
        
                    # 절댓값으로 회전량 비교
                    if abs(yaw_diff) <= 0.02: # 0.02 rad 허용 오차
                        self.stop_robot()
                        break
        
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                    self.publisher.publish(twist)
                    # rp.spin_once(self, timeout_sec=0.01)
                    rate.sleep()

                    

        self.get_logger().info('\n 미션 수행 완료!\n')

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)



def main(args=None):
    rp.init(args=args)

    node = PinkyOdometry()

    try:
        rp.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()