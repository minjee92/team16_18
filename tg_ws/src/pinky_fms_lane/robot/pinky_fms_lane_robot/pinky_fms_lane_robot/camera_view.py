"""키보드 조종 중 카메라 녹화용: pinky_autonomous 의 차선 인식 화면(HUD)만 내보내고 로봇은 움직이지 않는 노드.

pinky_autonomous 가 카메라(Picamera2)를 독점하므로, 키보드로 몰면서 차선 인식 화면을 녹화하려면
autonomous_drive 대신 이 노드를 띄운다 (launch/robot_camera_view.launch.xml). 둘을 같이 띄울 수는 없다.

AutonomousDriveNode 를 상속하고 pinky_autonomous 는 수정하지 않는다. 다른 점:
  - cmd_vel 발행자를 만들지 않는다 → 키보드 조종(teleop)과 속도 명령이 겹치지 않는다.
    (/<ns>/cmd_vel 발행자는 teleop 하나뿐이어야 한다. ros2 topic info 로 확인)
  - cross_lane 을 봐도 '갈림길 복귀' 동작으로 넘어가지 않고 매 프레임 인식 화면을 계속 그린다.
  - LED 를 바꾸지 않는다 (HUD 의 DRIVE/STOPPED 는 차선 노드였다면 냈을 판단일 뿐, 실제 움직임은 키보드).
HUD 영상은 /<ns>/camera/compressed 를 구독하는 노드가 있을 때만 나온다 (pinky_autonomous 와 같음).
"""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions

from pinky_autonomous.autonomous_drive_node import AutonomousDriveNode


class _NoDrive:
    """cmd_vel 자리에 두는 빈 발행자: 아무것도 보내지 않는다."""

    def publish(self, msg):
        pass

    def get_subscription_count(self):
        return 0


class CameraViewNode(AutonomousDriveNode):
    def create_publisher(self, msg_type, topic, *args, **kwargs):
        if topic == 'cmd_vel':
            return _NoDrive()
        return super().create_publisher(msg_type, topic, *args, **kwargs)

    def _apply_lamp(self, lamp):
        pass

    def _drive_loop(self):
        self.junction_latched = False
        super()._drive_loop()


def main(args=None):
    # pinky_autonomous main() 과 같은 종료 처리: 카메라 점유를 풀고 끝낸다
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = None
    try:
        node = CameraViewNode()
        node.get_logger().info('🎥 녹화용 차선 인식 화면만 냅니다 (cmd_vel 을 내지 않음: 로봇은 키보드로 조종)')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown_hardware()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
