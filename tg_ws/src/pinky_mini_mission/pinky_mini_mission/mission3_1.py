# 주제1. 특정 위치로 주행. 실시간 위치 정보 알림. 화면에 표시. (터미널 혹은 GUI)

import rclpy as rp
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped 
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import threading
import tf2_ros
import sys

class WaypointNavigator(Node) :
    def __init__(self):
        super().__init__('navigation_node')

        # 1.TF 설정 (초기 위치 설정)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 2.통신 인터페이스 설정
        # /clicked_point 토픽 구독
        self.subscription = self.create_subscription(PointStamped, 
                                        '/clicked_point', 
                                        self.point_addition_callback, 
                                        10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 3.상태 변수
        self.waypoints = []
        self.initial_pose = None
        self.is_navigating = False
        self.current_goal_idx = 0

        # 노드 시작 후 초기 위치 가져오기 위한 타이머
        self.create_timer(1.0, self.get_initial_pose)

        self.get_logger().info('목적지 이동 자울주행 미션 시작!')


        self.get_logger().info('\n주행 미션을 시작합니다!\n')

        # 4.키보드 입력(enter) 대기 스레드 생성 및 시작
        self.input_thread = threading.Thread(target=self.wait_for_enter, 
                                             daemon=True) # 메인 프로그램 종료 시 스레드도 함께 종료
        self.input_thread.start()

        # 5.실시간 위치 정보 출력 위한 타이머 실행
        self.create_timer(2.0, self.timer_realtime_pose_callback)

    def timer_realtime_pose_callback(self):
        """일정 간격으로 실시간 위치 알림"""
        if self.is_navigating:
            try:
                # map 기준 현재 위치
                t = self.tf_buffer.lookup_transform('map', 'base_link', rp.time.Time())
                current_x = t.transform.translation.x
                current_y = t.transform.translation.y

                self.get_logger().info(f'[실시간 위치] x: {current_x:.2f}, y: {current_y:.2f}')
            except tf2_ros.TransformException:
                # TF를 읽지 못하는 에러 무시
                pass

    def wait_for_enter(self):
        while rp.ok():
            # 터미널에서 Enter키 입력 기다림
            sys.stdin.readline()

            if self.is_navigating:
                self.get_logger().warn('이미 주행 중입니다...')
                continue

            if self.initial_pose is None:
                self.get_logger().warn('초기 위치(TF)를 아직 받아오지 못했습니다. 잠시 후 다시 눌러주세요.')
                continue

            if len(self.waypoints) == 0:
                self.get_logger().warn('저장된 waypoint가 없습니다. RViz에서 목표 지점을 누르고 Enter를 눌러주세요.')
                continue

            # 조건 모두 만족 시, 미션 시작
            self.start_mission()
            break

    def get_initial_pose(self):
        if self.initial_pose is None:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rp.time.Time())
                self.initial_pose = PoseStamped()
                self.initial_pose.header.frame_id = 'map'
                self.initial_pose.pose.position.x = t.transform.translation.x
                self.initial_pose.pose.position.y = t.transform.translation.y
                self.initial_pose.pose.orientation = t.transform.rotation
                self.get_logger().info('확인: 초기 위치 저장 완료. RViz에서 목표 지점 클릭 후 Enter!')
            except tf2_ros.TransformException:
                pass
    
    def point_addition_callback(self, msg):
        if self.is_navigating:
            self.get_logger().warn('주행 중에는 waypoint를 추가할 수 없습니다')
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0

        self.waypoints.append(pose)
        self.get_logger().info(f'지점 추가됨[{len(self.waypoints)}]: x={msg.point.x:.2f}, y={msg.point.y:.2f}')

    def start_mission(self):
        """조건이 충족되어 Enter를 눌렀을 때 호출"""
        # 리스트 마지막에 복귀할 초기 위치 추가
        self.waypoints.append(self.initial_pose)
        
        self.is_navigating = True
        self.current_goal_idx = 0
        
        self.get_logger().info('주행 미션을 시작합니다! (마지막 지점은 초기 출발 위치)')
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_goal_idx < len(self.waypoints):
            target_pose = self.waypoints[self.current_goal_idx]

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = target_pose

            self.get_logger().info(f'{self.current_goal_idx + 1}번째 목적지로 이동 중...')

            self.nav_client.wait_for_server()
            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표가 거부되었습니다. 미션을 중단합니다.')
            self.is_navigating = False
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        """현재 목표에 도달하면 다음 목표를 호출"""
        status = future.result().status

        if status == 4: # SUCCEEDED
            self.get_logger().info('\n목표 도달 완료!!!\n')
            self.current_goal_idx += 1

            # 다음 목표 인덱스가 총 길이랑 같다면 (모든 목표 + 복귀 지점까지 완료)
            if self.current_goal_idx >= len(self.waypoints):
                self.get_logger().info('모든 주행(복귀 포함)이 완료되었습니다. 프로그램을 종료합니다.')
                # rclpy.shutdown() 대신 SystemExit 예외를 발생시킴
                raise SystemExit
            else:
                self.send_next_goal()

        
        else:
            self.get_logger().warn(f'목표 도달 실패 (상태 코드: {status}). 미션을 중단합니다.')
            self.is_navigating = False


def main(args=None):
    rp.init(args=args)
    node = WaypointNavigator()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        node.get_logger().info('노드가 안전하게 종료됩니다.')
    finally:
        node.destroy_node()
        if rp.ok():
            rp.shutdown()

if __name__ == '__main__':
    main()