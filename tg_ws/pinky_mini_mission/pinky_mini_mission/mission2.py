# 로봇을 특정 위치로 이동시킨 후 초기 위치로 복귀시키기

import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints

# TF (위치 변환) 데이터를 읽어오기
from tf2_ros import Buffer, TransformListener

class FollowWaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # 1. TF 버퍼 및 리스너 생성 (로봇 위치 추적 목적)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF 데이터를 충분히 수신할 수 있도록 대기 시간 증가                
        # [핵심] 노드가 켜진 후 시뮬레이션 시간을 받을 수 있도록 1초>>>2초 대기 후 전송
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        self.timer.cancel()  # 타이머를 멈추고 1번만 실행
        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Action 서버 연결을 기다립니다.')

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action 서버를 찾을 수 없습니다.')
            rp.shutdown()
            return
        
        # ==========================================
        # 2. 로봇의 현재 위치(출발 위치) 동적으로 읽어오기
        # ==========================================
        try:
            # map을 기준으로 base_footprint(로봇 중심)의 현재 좌표를 가져오기
            # (만약 에러가 난다면 'base_footprint'를 'base_link'로 변경)
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint', 
                rp.time.Time()
            )
        except Exception as e:
            self.get_logger().error(f'로봇의 현재 위치를 찾을 수 없습니다: {e}')
            rp.shutdown()
            return

        
        goal_msg = FollowWaypoints.Goal()

        # '가상 시간'을 가져오기 (시뮬레이터로 실습 시에 현실 시간 적용 시 충돌 오류로 미작동 가능성 있음)
        now = self.get_clock().now().to_msg()

        # 목표 지점 (로봇의 앞쪽 비어있는 공간 좌표여야 함)
        first_wp = PoseStamped()
        first_wp.header.frame_id = 'map'
        first_wp.header.stamp = now
        first_wp.pose.position.x = 2.0
        first_wp.pose.position.y = 1.5
        first_wp.pose.orientation.w = 1.0

        # 두 번째 목적지
        second_wp = PoseStamped()
        second_wp.header.frame_id = 'map'
        second_wp.header.stamp = now
        second_wp.pose.position.x = 1.5
        second_wp.pose.position.y = -1.1
        second_wp.pose.orientation.w = 1.0

        # 초기 위치 (위치 좌표 하드코딩)
        # initial_wp = PoseStamped()
        # initial_wp.header.frame_id = 'map'
        # initial_wp.header.stamp = now
        # initial_wp.pose.position.x = 0.0
        # initial_wp.pose.position.y = 0.0
        # initial_wp.pose.orientation.w = 1.0

        # ==========================================
        # 3. 초기 위치(복귀 지점)를 현재 위치로 설정 (로봇 자체적으로 현재 위치 인식)
        # ==========================================
        initial_wp = PoseStamped()
        initial_wp.header.frame_id = 'map'
        # 하드코딩(0.0) 대신 TF에서 읽어온 x, y 값을 넣습니다.
        initial_wp.pose.position.x = trans.transform.translation.x
        initial_wp.pose.position.y = trans.transform.translation.y
        # 바라보는 방향(회전값)도 출발할 때와 똑같은 방향(w=1.0)을 보도록 그대로 복사
        initial_wp.pose.orientation = trans.transform.rotation

        goal_msg.poses = [first_wp, second_wp, initial_wp]

        self.get_logger().info(f'현재 위치 ({initial_wp.pose.position.x:.2f}, {initial_wp.pose.position.y: .2f}을 복귀 지점으로 설정...)')
        self.get_logger().info('Action 서버로 Goal 전송 중...!')

        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # Goal에 대한 수행 유무 관련 응답 콜백
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Action 서버가 Goal을 거절하였습니다.')
            rp.shutdown()
            return

        self.get_logger().info('Action 서버가 Goal을 수락하였습니다.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Goal 수행 중 현황 피드백
    def feedback_callback(self, feedback_msg):
        current_wp = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'피드백 수신: 현재 waypoint index [{current_wp}]로 이동 중...')

    # Goal 수행 완료 현황 피드백
    def get_result_callback(self, future):
        result = future.result().result

        # 실패한 웨이포인트가 있는지 최우선 확인
        if len(result.missed_waypoints) > 0:
            self.get_logger().error(f'주행 실패! 총 {len(result.missed_waypoints)}개의 목표를 놓쳤습니다.')
            for missed in result.missed_waypoints:
                self.get_logger().error(f'실패한 waypoint index: {missed.index}')
        elif result.error_code == 0:
            self.get_logger().info('🎉 성공! 목표 지점을 거쳐 초기 위치로 복귀 완료!')
        else:
            self.get_logger().error(f'작업 실패. 에러코드: {result.error_code}')
                    
        rp.shutdown()

def main(args=None):
    rp.init(args=args)

    action_client_node = FollowWaypointNavigator()

    # [중요] action_client_node.send_goal() 이 있었는데, 이 경우 send_goal 후 바로 node가 destroy되는 현상 발생 (타이머 주는 것이 중요)
    
    try:
        rp.spin(action_client_node)
    except KeyboardInterrupt:
        print("\n사용자에 의해 강제 종료 되었습니다\n") # ctrl + c
    finally:
        action_client_node.destroy_node()
        if rp.ok():
            rp.shutdown()

if __name__ == '__main__':
    main()