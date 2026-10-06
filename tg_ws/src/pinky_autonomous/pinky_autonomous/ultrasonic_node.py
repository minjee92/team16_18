#!/usr/bin/env python3
"""
US-016 초음파 센서 퍼블리셔 노드 (RPi5에서 실행)

TRIG/ECHO 펄스폭 측정 방식 (HC-SR04 호환 프로토콜).
RPi5에서는 RPi.GPIO 대신 lgpio 백엔드를 사용해야 한다.

배선:
  US-016 TRIG ── GPIO 23 (BCM)  ← trig_pin 파라미터
  US-016 ECHO ── GPIO 24 (BCM)  ← echo_pin 파라미터
  US-016 VCC  ── 5V
  US-016 GND  ── GND

의존 패키지:
  pip install gpiozero lgpio

파라미터:
  trig_pin     (int,   기본 23)            : TRIG 핀 BCM 번호
  echo_pin     (int,   기본 24)            : ECHO 핀 BCM 번호
  topic        (str,   기본 ultrasonic/range) : 퍼블리시 토픽
  publish_rate (float, 기본 10.0)          : Hz
  max_range    (float, 기본 1.0)           : 측정 최대 거리 (m)
  min_range    (float, 기본 0.02)          : 측정 최소 거리 (m)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class UltrasonicNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_sensor')

        self.declare_parameter('trig_pin',     23)
        self.declare_parameter('echo_pin',     24)
        self.declare_parameter('topic',        'ultrasonic/range')
        self.declare_parameter('publish_rate',  10.0)
        self.declare_parameter('max_range',     1.0)
        self.declare_parameter('min_range',     0.02)

        p = self.get_parameter
        trig_pin       = p('trig_pin').value
        echo_pin       = p('echo_pin').value
        topic          = p('topic').value
        rate           = p('publish_rate').value
        self.max_range = p('max_range').value
        self.min_range = p('min_range').value

        self.sensor = self._init_sensor(trig_pin, echo_pin)

        self.pub = self.create_publisher(Range, topic, 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'US-016 publisher ready → {topic} @ {rate:.0f} Hz  '
            f'(TRIG=GPIO{trig_pin}, ECHO=GPIO{echo_pin})'
        )

    # ──────────────────────────────────────────────
    # 센서 초기화
    # ──────────────────────────────────────────────
    def _init_sensor(self, trig_pin: int, echo_pin: int):
        try:
            # RPi5는 lgpio 백엔드 필요 — RPi.GPIO는 RPi5 미지원
            from gpiozero.pins.lgpio import LGPIOFactory
            from gpiozero import Device, DistanceSensor
            Device.pin_factory = LGPIOFactory()

            sensor = DistanceSensor(
                echo=echo_pin,
                trigger=trig_pin,
                max_distance=self.max_range,
            )
            self.get_logger().info(
                f'GPIO 초기화 완료: TRIG=GPIO{trig_pin}, ECHO=GPIO{echo_pin}')
            return sensor

        except Exception as e:
            self.get_logger().error(
                f'GPIO 초기화 실패 (센서 없이 계속 실행): {e}\n'
                '  pip install gpiozero lgpio 확인 필요')
            return None

    # ──────────────────────────────────────────────
    # 퍼블리시 콜백
    # ──────────────────────────────────────────────
    def _publish(self):
        if self.sensor is None:
            return

        try:
            dist = self.sensor.distance   # gpiozero: 미터 단위
        except Exception as e:
            self.get_logger().warn(f'센서 읽기 실패: {e}')
            return

        if dist is None:
            return

        dist = max(self.min_range, min(self.max_range, float(dist)))

        msg = Range()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_sensor'
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = math.radians(15)
        msg.min_range       = self.min_range
        msg.max_range       = self.max_range
        msg.range           = dist
        self.pub.publish(msg)

    # ──────────────────────────────────────────────
    # 종료 처리
    # ──────────────────────────────────────────────
    def destroy_node(self):
        if self.sensor is not None:
            try:
                self.sensor.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
