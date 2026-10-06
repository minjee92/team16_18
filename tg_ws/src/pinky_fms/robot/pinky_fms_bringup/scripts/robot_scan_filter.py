#!/usr/bin/env python3
"""다른 로봇을 지운 라이다 스캔 (로봇에서 namespace 안에 실행).

  구독: scan (sensor_msgs/LaserScan), other_robots (geometry_msgs/PoseArray, map 좌표, 관제PC 의 fleet_traffic 이 보냄)
  발행: scan_robotfree — 다른 로봇의 실시간 위치 주변(radius) 에 찍힌 점을 inf 로 바꾼 스캔

전역 costmap(플래너)만 이 스캔을 쓴다. 지도에 없는 사물·사람은 그대로 남아 우회하고, 다른 로봇 때문에 통로가 막혔다고
계산하는 일은 없어진다. 지역 costmap(컨트롤러)은 원래 scan 을 써서 무엇이든 바로 앞에 있으면 멈춘다.
other_robots 가 오래됐거나(관제PC 끊김) TF 를 못 구하면 원래 스캔을 그대로 내보낸다 (= 다른 로봇도 장애물로 보는 안전한 쪽).
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class RobotScanFilter(Node):
    def __init__(self):
        super().__init__('robot_scan_filter')
        self.declare_parameter('radius', 0.20)        # 다른 로봇 중심에서 이 거리 안의 점을 지운다 (몸체 0.095 + 위치추정 오차 여유)
        self.declare_parameter('max_age', 1.0)        # other_robots 가 이보다 오래되면 지우지 않는다
        self.declare_parameter('map_frame', 'map')
        self.radius = float(self.get_parameter('radius').value)
        self.max_age = float(self.get_parameter('max_age').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.others, self.others_t = np.zeros((0, 2)), 0.0
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(LaserScan, 'scan_robotfree', 5)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseArray, 'other_robots', self.on_others, 5)
        self.n_masked, self.last_log = 0, time.monotonic()
        self.get_logger().info(f'robot_scan_filter ready (radius {self.radius} m)')

    def on_others(self, m):
        self.others = np.array([[p.position.x, p.position.y] for p in m.poses]).reshape(-1, 2)
        self.others_t = time.monotonic()

    def on_scan(self, s):
        if len(self.others) == 0 or time.monotonic() - self.others_t > self.max_age:
            self.pub.publish(s)
            return
        try:                                   # costmap 과 같은 시각(스캔 stamp)의 TF. 없으면 최신 TF, 그것도 없으면 원래 스캔
            t = self.tf.lookup_transform(self.map_frame, s.header.frame_id, Time.from_msg(s.header.stamp))
        except Exception:
            try:
                t = self.tf.lookup_transform(self.map_frame, s.header.frame_id, Time())
            except Exception:
                self.pub.publish(s)
                return
        q, tr = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        r = np.asarray(s.ranges, dtype=float)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)
        x = tr.x + r * np.cos(ang + yaw)
        y = tr.y + r * np.sin(ang + yaw)
        d = np.min(np.hypot(x[:, None] - self.others[None, :, 0], y[:, None] - self.others[None, :, 1]), axis=1)
        mask = ok & (d < self.radius)
        out = LaserScan()
        out.header, out.angle_min, out.angle_max, out.angle_increment = s.header, s.angle_min, s.angle_max, s.angle_increment
        out.time_increment, out.scan_time, out.range_min, out.range_max = s.time_increment, s.scan_time, s.range_min, s.range_max
        rr = r.copy()
        rr[mask] = float('inf')
        out.ranges = rr.astype(np.float32).tolist()
        out.intensities = s.intensities
        self.pub.publish(out)
        self.n_masked += int(mask.sum())
        if time.monotonic() - self.last_log > 10.0:
            self.get_logger().info(f'최근 10초 동안 다른 로봇으로 보고 지운 점: {self.n_masked}개 (로봇 {len(self.others)}대)')
            self.n_masked, self.last_log = 0, time.monotonic()


def main():
    rclpy.init()
    n = RobotScanFilter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
