#!/usr/bin/env python3
"""Exercise an isolated simulation: sensor receipt, short drive, then stop.

This publishes cmd_vel. Run with nav:=false and no other controller active.
"""
import argparse
import json
import math
from pathlib import Path
import time

from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image as RosImage, JointState, LaserScan, CameraInfo


class Probe(Node):
    def __init__(self, robot):
        super().__init__('pinky_sim_smoke')
        self.samples = {}
        self.counts = {}
        self.subscriptions_kept = []
        for key, msg, topic in (
            ('clock', Clock, '/clock'), ('scan', LaserScan, f'/{robot}/scan'),
            ('odom', Odometry, f'/{robot}/odom'),
            ('image', RosImage, f'/{robot}/camera/image_raw'),
            ('joints', JointState, f'/{robot}/joint_states'),
            ('camera_info', CameraInfo, f'/{robot}/camera/camera_info'),
        ):
            self.subscriptions_kept.append(self.create_subscription(
                msg, topic, lambda value, key=key: self.receive(key, value), qos_profile_sensor_data))
        self.command = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)

    def receive(self, key, value):
        self.samples[key] = value
        self.counts[key] = self.counts.get(key, 0) + 1

    def sim_time(self):
        stamp = self.samples['clock'].clock
        return stamp.sec + stamp.nanosec * 1e-9

    def position(self):
        p = self.samples['odom'].pose.pose.position
        return p.x, p.y

    def wait_for(self, predicate, timeout=120):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f'Timed out; received samples: {self.counts}')

    def advance(self, duration, speed=0.0):
        start = self.sim_time()
        msg = Twist()
        msg.linear.x = speed
        deadline = time.monotonic() + 180
        while self.sim_time() - start < duration:
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation clock did not advance in time')
            self.command.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def save_image(message, path):
    modes = {'rgb8': ('RGB', 'RGB'), 'bgr8': ('RGB', 'BGR'),
             'rgba8': ('RGBA', 'RGBA'), 'bgra8': ('RGBA', 'BGRA')}
    if message.encoding not in modes:
        raise RuntimeError(f'Unsupported camera encoding: {message.encoding}')
    mode, rawmode = modes[message.encoding]
    image = Image.frombytes(mode, (message.width, message.height), bytes(message.data),
                            'raw', rawmode, message.step)
    image.save(path)
    extrema = image.convert('L').getextrema()
    if extrema[1] - extrema[0] < 30:
        raise RuntimeError('Camera frame has insufficient contrast; inspect rendering')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='amr_01')
    parser.add_argument('--output', type=Path, required=True, help='New evidence directory')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    probe = Probe(args.robot)
    try:
        probe.wait_for(lambda: len(probe.samples) == 6 and all(n >= 2 for n in probe.counts.values())
                       and probe.command.get_subscription_count() > 0)
        scan = probe.samples['scan']
        valid = [v for v in scan.ranges if math.isfinite(v) and scan.range_min <= v <= scan.range_max]
        if len(scan.ranges) < 100 or not valid:
            raise RuntimeError('LiDAR has no usable ranges')
        if len(probe.samples['joints'].name) < 2:
            raise RuntimeError('Wheel joint states are missing')
        info = probe.samples['camera_info']
        if info.width != probe.samples['image'].width or info.k[0] <= 0:
            raise RuntimeError('CameraInfo does not match the camera image')
        save_image(probe.samples['image'], args.output / 'camera_before.png')
        start = probe.position()
        probe.advance(1.5, 0.06)
        probe.advance(0.5, 0.0)
        stop = probe.position()
        distance = math.dist(start, stop)
        if not 0.03 < distance < 0.20:
            raise RuntimeError(f'Unexpected odometry displacement: {distance:.4f}m')
        probe.advance(0.5, 0.0)
        drift = math.dist(stop, probe.position())
        if drift > 0.01:
            raise RuntimeError(f'Robot did not stop: drift={drift:.4f}m')
        save_image(probe.samples['image'], args.output / 'camera_after.png')
        result = {'status': 'passed', 'samples': probe.counts,
                  'scan_samples': len(scan.ranges), 'scan_finite': len(valid),
                  'scan_min_m': min(valid), 'drive_distance_m': distance, 'stop_drift_m': drift,
                  'image_width': probe.samples['image'].width,
                  'image_height': probe.samples['image'].height,
                  'camera_frame': probe.samples['image'].header.frame_id,
                  'odom_frame': probe.samples['odom'].header.frame_id,
                  'odom_child_frame': probe.samples['odom'].child_frame_id}
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        for _ in range(5):
            probe.command.publish(Twist())
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
