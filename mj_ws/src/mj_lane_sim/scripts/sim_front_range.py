#!/usr/bin/env python3
"""
시뮬 전용: 라이다(scan)로 전방 초음파(us_sensor/range, sensor_msgs/Range)를 흉내 낸다.

시뮬 Pinky 에는 초음파 센서가 없다. 주행 노드는 실물과 같은 Range 토픽을 받아 비상 정지를 판단하므로,
라이다 점 중 초음파 빔 안(앞쪽 ±화각/2)에 든 점의 "초음파 위치에서 앞쪽 거리" 최솟값을 Range 로 낸다.

  실물 값 (pinky_sensor_adc main_node.cpp): frame ultrasonic_link, field_of_view 0.26 rad, min 0.02, max 3.0 m,
           range = adc/4096 - 0.03 이라 실제 상한은 약 0.97 m (빔 안에 아무것도 없으면 이 값).
  장착 위치 (pinky_description pinky.urdf.xacro): 라이다 base_link 기준 x -0.017 m, 뒤를 보게 yaw π.
           초음파 base_link 기준 x 0.0267 m.

실물과 다른 점: 라이다 10 Hz (실물 초음파 20 Hz). I2C 실패 무효값(-0.03) 같은 고장 모양은 없다.
시뮬 시간(use_sim_time)으로 돌고, 메시지 시각은 scan 의 시각을 그대로 쓴다.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range

FOV = 0.26               # rad, 실물 드라이버 값
MIN_RANGE = 0.02         # m, 실물 드라이버 값
MAX_RANGE = 3.0          # m, 실물 드라이버 값 (메시지에 적는 값)
SENSOR_CAP = 0.97        # m, 실물이 실제로 낼 수 있는 최대 (1.0 - 0.03)
LIDAR_X = -0.017         # m, base_link 기준 라이다 위치
LIDAR_YAW = math.pi      # rad, 라이다가 뒤를 본다
SONAR_X = 0.0267         # m, base_link 기준 초음파 위치


def front_range(ranges, angle_min, angle_increment, range_min, range_max,
                fov=FOV, lidar_x=LIDAR_X, lidar_yaw=LIDAR_YAW, sonar_x=SONAR_X, cap=SENSOR_CAP):
    """라이다 거리 목록 → 초음파 위치에서 본 전방 거리 (m). 빔 안에 점이 없으면 cap."""
    best = cap
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment + lidar_yaw          # base_link 기준 각도
        x = lidar_x + r * math.cos(angle)
        y = r * math.sin(angle)
        dx = x - sonar_x                                              # 초음파에서 앞쪽으로
        if dx <= 0 or abs(math.atan2(y, dx)) > fov / 2:
            continue
        best = min(best, math.hypot(dx, y))
    return best


class SimFrontRange(Node):
    def __init__(self):
        super().__init__('sim_front_range',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        prefix = self.get_namespace().strip('/')
        self.frame_id = f'{prefix}/ultrasonic_link' if prefix else 'ultrasonic_link'
        self.pub = self.create_publisher(Range, 'us_sensor/range', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan):
        msg = Range()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = FOV
        msg.min_range, msg.max_range = MIN_RANGE, MAX_RANGE
        msg.range = float(front_range(scan.ranges, scan.angle_min, scan.angle_increment,
                                      scan.range_min, scan.range_max))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimFrontRange()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
