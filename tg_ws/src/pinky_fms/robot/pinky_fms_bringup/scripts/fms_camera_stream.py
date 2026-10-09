#!/usr/bin/env python3
"""Nav2 미션(Goal / Coordinated Fleet Navigation)용 주행 영상: camera/rec (CompressedImage, JPEG 320 폭, 기본 5 Hz).

차선 주행(Lane Following) 중에는 차선 노드(fms_lane_mission)가 같은 토픽을 낸다. 이 노드는 Nav2 스택(robot_nav)에서만
뜨므로 두 노드가 카메라를 동시에 잡지 않는다.
구독자(관제 GUI 카메라 버튼, REC LOG)가 있을 때만 카메라를 열고, 구독자가 없어진 뒤 idle_close 초가 지나면 닫는다.
picamera2 가 없거나(시뮬) 카메라를 못 열면 경고만 남기고 retry 초마다 다시 시도한다.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraStream(Node):
    def __init__(self):
        super().__init__('fms_camera_stream')
        self.declare_parameter('rec_fps', 5.0)
        self.declare_parameter('rec_width', 320)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('idle_close', 10.0)
        self.declare_parameter('retry', 5.0)
        p = lambda n: self.get_parameter(n).value
        self.rec_w, self.quality = int(p('rec_width')), int(p('jpeg_quality'))
        self.idle_close, self.retry = float(p('idle_close')), float(p('retry'))
        self.name = self.get_namespace().strip('/') or 'robot'
        self.pub = self.create_publisher(CompressedImage, 'camera/rec', 2)
        self.camera = None
        self.last_sub_t = 0.0
        self.next_try_t = 0.0
        self.create_timer(1.0 / max(0.5, float(p('rec_fps'))), self._tick)
        self.get_logger().info('주행 영상 대기 (camera/rec 구독자가 생기면 카메라를 엽니다)')

    def _open(self):
        try:
            from picamera2 import Picamera2
            cam = Picamera2()
            cam.configure(cam.create_video_configuration(main={'size': (640, 480), 'format': 'RGB888'}))
            cam.start()
            self.camera = cam
            self.get_logger().info('📸 카메라 열림 (주행 영상 송출 시작)')
        except Exception as e:
            self.camera = None
            self.next_try_t = time.monotonic() + self.retry
            self.get_logger().warn(f'카메라를 열 수 없습니다 ({e}) — {self.retry:.0f}s 뒤 다시 시도', throttle_duration_sec=30.0)

    def _close(self):
        try:
            self.camera.stop()
            self.camera.close()
        except Exception:
            pass
        self.camera = None
        self.get_logger().info('카메라 닫음 (구독자 없음)')

    def _tick(self):
        now = time.monotonic()
        if self.pub.get_subscription_count() > 0:
            self.last_sub_t = now
        elif self.camera is not None and now - self.last_sub_t > self.idle_close:
            self._close()
            return
        if self.pub.get_subscription_count() == 0:
            return
        if self.camera is None:
            if now >= self.next_try_t:
                self._open()
            return
        try:
            import cv2
            img = cv2.rotate(self.camera.capture_array(), cv2.ROTATE_180)   # 차선 노드(원본)와 같은 방향·색 처리
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            h, w = img.shape[:2]
            img = cv2.resize(img, (self.rec_w, int(h * self.rec_w / w)))
            cv2.putText(img, f'{self.name} nav {time.strftime("%H:%M:%S")}', (4, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                m = CompressedImage()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = self.name
                m.format = 'jpeg'
                m.data = buf.tobytes()
                self.pub.publish(m)
        except Exception as e:
            self.get_logger().warn(f'영상 만들기 실패: {e}', throttle_duration_sec=10.0)

    def shutdown(self):
        if self.camera is not None:
            self._close()


def main():
    rclpy.init()
    node = CameraStream()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
