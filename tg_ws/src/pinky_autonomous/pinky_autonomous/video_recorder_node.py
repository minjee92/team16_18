
import os
from datetime import datetime
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import Log

class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder')

        self.declare_parameter('save_dir', os.path.expanduser('~/pinky_recordings'))
        self.declare_parameter('fps',    10.0)
        self.declare_parameter('width',  640)
        self.declare_parameter('height', 480)
        self.declare_parameter('topic',  'camera/compressed')

        save_dir = self.get_parameter('save_dir').value
        fps      = self.get_parameter('fps').value
        self.width    = self.get_parameter('width').value
        self.height   = self.get_parameter('height').value
        topic    = self.get_parameter('topic').value

        os.makedirs(save_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = timestamp + '.mp4'
        self.filepath = os.path.join(save_dir, self.filename)
        self.log_filepath = os.path.join(save_dir, timestamp + '_log.txt')
        
        self.log_file = open(self.log_filepath, 'w', buffering=1)

        self.writer = cv2.VideoWriter(
            self.filepath,
            cv2.VideoWriter_fourcc(*'mp4v'),
            fps,
            (self.width, self.height),
        )
        if not self.writer.isOpened():
            self.get_logger().error(f'VideoWriter 열기 실패: {self.filepath}')

        self.get_logger().info(f'🎥 Recording Video → {self.filepath}')
        self.get_logger().info(f'📝 Recording Logs → {self.log_filepath}')

        self.sub = self.create_subscription(
            CompressedImage, topic, self._image_callback, 10
        )
        
        # Subscribe to rosout to catch autonomous_drive logs
        self.log_sub = self.create_subscription(
            Log, '/rosout', self._rosout_callback, 100
        )

        self._frame_count = 0

    def _rosout_callback(self, msg: Log):
        # autonomous_drive 로그 + 초음파 노드 전체 + 램프 노드의 경고 이상 (센서·LED 문제를 로그에서 바로 보기 위함)
        is_main = msg.name == 'autonomous_drive'
        is_helper = (msg.name == 'ultrasonic_sensor' or
                     (msg.name == 'pinky_lamp_control' and msg.level >= Log.WARN))
        if is_main or is_helper:
            # Format: [Time] [Level] Message
            timestamp_sec = msg.stamp.sec + msg.stamp.nanosec / 1e9
            dt = datetime.fromtimestamp(timestamp_sec)
            time_str = dt.strftime('%H:%M:%S.%f')[:-3]
            
            level_str = "INFO"
            if msg.level == Log.WARN: level_str = "WARN"
            elif msg.level == Log.ERROR: level_str = "ERROR"
            elif msg.level == Log.FATAL: level_str = "FATAL"
            
            tag = '' if is_main else f'[{msg.name}] '
            log_line = f"[{time_str}] [{level_str}] {tag}{msg.msg}\n"
            self.log_file.write(log_line)

    def _image_callback(self, msg: CompressedImage):
        buf   = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return

        frame_resized = cv2.resize(frame, (self.width, self.height))
        self.writer.write(frame_resized)
        self._frame_count += 1

        if self._frame_count % 100 == 0:
            self.get_logger().info(f'{self._frame_count} frames recorded.')

    def destroy_node(self):
        self.writer.release()
        self.log_file.close()
        self.get_logger().info(
            f'Recording stopped. Total frames: {self._frame_count}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():          # 이미 종료된 컨텍스트에 shutdown을 또 호출하면 RCLError로 비정상 종료한다
            rclpy.shutdown()

if __name__ == '__main__':
    main()
