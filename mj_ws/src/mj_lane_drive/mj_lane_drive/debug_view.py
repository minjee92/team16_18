"""
검출 표시 영상 토픽 (lane_debug). 녹화(--record) 영상과 같은 그림(drive_record.render_frame)을 ROS 로 내보낸다.

  lane_debug/image             sensor_msgs/Image (bgr8, 640x480)        ← PC·시뮬에서 보기
  lane_debug/image/compressed  sensor_msgs/CompressedImage (JPEG)       ← 실물 로봇을 Wi-Fi 로 볼 때

제어 루프를 늦추지 않게:
  - 구독자가 하나도 없으면 아무것도 하지 않는다 (wanted() 가 False → 그림 재료도 넘기지 않음)
  - 그리기·인코딩·발행은 별도 스레드. 넘겨받는 칸은 "최신 1장" 뿐이라, 스레드가 바쁘면 이전 것을 덮어쓴다 (기다리지 않음)
  - 녹화 중이면 녹화 스레드가 이미 그린 그림을 publish_rendered() 로 받아 그대로 낸다 (두 번 그리지 않음)
QoS 는 reliable·깊이 1: rqt_image_view(기본 reliable)와 best effort 구독자 모두 받을 수 있다.
"""

import threading

import cv2
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from mj_lane_drive.drive_record import render_frame

DEBUG_TOPIC = 'lane_debug/image'
JPEG_QUALITY = 80


class DebugImagePublisher:
    def __init__(self, node, frame_id='', render=render_frame):
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.node, self.frame_id, self.render = node, frame_id, render
        self.raw_pub = node.create_publisher(Image, DEBUG_TOPIC, qos)
        self.jpeg_pub = node.create_publisher(CompressedImage, DEBUG_TOPIC + '/compressed', qos)
        self.topic = self.raw_pub.topic_name
        self.cond = threading.Condition()
        self.pending = None
        self.closed = False
        self.rendered = self.published = self.overwritten = self.failed = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def wanted(self):
        """구독자가 있는지 (raw 또는 compressed). 제어 루프에서 매 프레임 부른다 (가벼움)."""
        return self.raw_pub.get_subscription_count() > 0 or self.jpeg_pub.get_subscription_count() > 0

    def submit(self, item):
        """그림 재료(render_frame 의 item)를 넘긴다. 기다리지 않는다: 앞의 것이 아직이면 덮어쓴다."""
        with self.cond:
            if self.pending is not None:
                self.overwritten += 1
            self.pending = item
            self.cond.notify()

    def publish_rendered(self, view):
        """이미 그린 그림을 낸다 (녹화 스레드에서 불림). 구독자가 없으면 바로 돌아간다."""
        if self.wanted():
            self._publish(view)

    def _run(self):
        while True:
            with self.cond:
                self.cond.wait_for(lambda: self.pending is not None or self.closed)
                if self.closed:
                    return
                item, self.pending = self.pending, None
            try:
                view = self.render(item)
                self.rendered += 1
                self._publish(view)
            except Exception as error:             # 표시 실패가 주행을 멈추지 않게
                self.failed += 1
                if self.failed == 1:
                    self.node.get_logger().warn(f'lane_debug 그리기 오류 (이후 같은 오류는 세기만 함): {error}')

    def _publish(self, view):
        try:
            stamp = self.node.get_clock().now().to_msg()
            if self.raw_pub.get_subscription_count() > 0:
                msg = Image()
                msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
                msg.height, msg.width = view.shape[:2]
                msg.encoding, msg.step = 'bgr8', view.shape[1] * 3
                msg.data = view.tobytes()
                self.raw_pub.publish(msg)
            if self.jpeg_pub.get_subscription_count() > 0:
                ok, buf = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    msg = CompressedImage()
                    msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
                    msg.format = 'jpeg'
                    msg.data = buf.tobytes()
                    self.jpeg_pub.publish(msg)
            self.published += 1
        except Exception as error:
            self.failed += 1
            if self.failed == 1:
                self.node.get_logger().warn(f'lane_debug 발행 오류 (이후 같은 오류는 세기만 함): {error}')

    def close(self):
        with self.cond:
            self.closed = True
            self.cond.notify()
        self.thread.join(timeout=5)
