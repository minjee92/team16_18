#!/usr/bin/env python3
"""ROS camera adapter for the existing LaneTracker and steering calculation.

Only basic lane tracking is exercised, not the real robot's mission state
machine, GPIO/Picamera2, LEDs, sonar or return-home behavior. Driving is opt-in.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan
import torch
from ultralytics import YOLO

# Import the actual robot implementation; do not copy or rewrite its algorithm.
from pinky_autonomous.autonomous_drive_node import (
    AutonomousDriveNode, LaneTracker, build_masks, pick_main, STATE_DRIVING,
)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def bgr_frame(message):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(message.encoding)
    if channels is None:
        raise ValueError(f'Unsupported image encoding: {message.encoding}')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    frame = rows[:, :message.width * channels].reshape(message.height, message.width, channels)
    conversion = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                  'bgra8': cv2.COLOR_BGRA2BGR}.get(message.encoding)
    return cv2.cvtColor(frame, conversion) if conversion is not None else frame.copy()


class SimulationLane(Node):
    def __init__(self, args):
        super().__init__('simulation_lane_follow')
        self.args = args
        self.latest = {}
        self.received = {}
        self.subs = []
        for key, kind, topic in (
            ('image', Image, f'/{args.robot}/camera/image_raw'),
            ('scan', LaserScan, f'/{args.robot}/scan'),
            ('odom', Odometry, f'/{args.robot}/odom'), ('clock', Clock, '/clock'),
        ):
            self.subs.append(self.create_subscription(
                kind, topic, lambda value, key=key: self.receive(key, value), qos_profile_sensor_data))
        self.pub = self.create_publisher(Twist, f'/{args.robot}/cmd_vel', 10) if args.drive else None
        self.model = YOLO(str(args.model), task='segment')
        # Initialize inference before starting a timed trial or publishing motion.
        self.model.predict(np.zeros((480, 640, 3), dtype=np.uint8),
                           imgsz=320, conf=0.5, device='cpu', verbose=False)
        self.tracker = None
        # Inputs expected by the original _compute_command method in DRIVING state.
        self.state = STATE_DRIVING
        self.drive_speed = 0.08
        self.lane_bias = None
        self.caution_factor = 0.5
        self.history = []

    def receive(self, key, value):
        self.latest[key] = value
        self.received[key] = time.monotonic()

    def stop(self):
        if self.pub is not None:
            self.pub.publish(Twist())

    def position(self):
        p = self.latest['odom'].pose.pose.position
        return [p.x, p.y]

    def run_trial(self):
        deadline = time.monotonic() + 120
        while len(self.latest) < 4:
            time.sleep(0.01)
            if time.monotonic() > deadline:
                raise RuntimeError(f'Missing simulation inputs: {set(self.latest)}')
        start_time = seconds(self.latest['clock'].clock)
        start_position = self.position()
        previous = -math.inf
        deadline = time.monotonic() + self.args.wall_timeout
        while seconds(self.latest['clock'].clock) - start_time < self.args.duration:
            time.sleep(0.01)
            if time.monotonic() > deadline:
                raise RuntimeError('Trial wall-clock timeout')
            sim_now = seconds(self.latest['clock'].clock)
            if time.monotonic() - self.received['clock'] > 5 or any(
                    sim_now - seconds(self.latest[k].header.stamp) > 0.5
                    for k in ('image', 'scan')):
                self.stop()
                ages = {k: round(time.monotonic() - self.received[k], 3) for k in self.received}
                raise RuntimeError(f'Simulation sensor stream stalled: wall ages={ages}, frames={len(self.history)}')
            msg = self.latest['image']
            stamp = seconds(msg.header.stamp)
            if stamp - previous < 0.099:
                continue
            previous = stamp
            frame = bgr_frame(msg)  # Gazebo is upright; no Picamera2 180-degree rotation.
            if self.tracker is None:
                self.tracker = LaneTracker(msg.width, msg.height, heading_gain=0, far_fallback=False)
            result = self.model.predict(frame, imgsz=320, conf=0.5, device='cpu', verbose=False)[0]
            masks = build_masks(result, msg.width, msg.height, {'left_lane': 0.5, 'right_lane': 0.5})
            _, steer, valid, mode = self.tracker.update(
                pick_main(masks.get('left_lane')), pick_main(masks.get('right_lane')))
            speed, angular, _ = AutonomousDriveNode._compute_command(self, steer, valid)
            scan = self.latest['scan']
            forward = [value for i, value in enumerate(scan.ranges)
                       if abs(scan.angle_min + i * scan.angle_increment) < math.radians(20)
                       and math.isfinite(value) and scan.range_min <= value <= scan.range_max]
            clearance = min(forward) if forward else None
            # Explicit simulation guards, not a substitute for the robot sonar.
            guard = not valid or clearance is None or clearance < 0.12
            if guard:
                speed, angular = 0.0, 0.0
            command = Twist()
            command.linear.x, command.angular.z = float(speed), float(angular)
            if self.pub is not None:
                self.pub.publish(command)
            row = {'stamp': stamp, 'valid': bool(valid), 'mode': mode, 'steer': float(steer),
                   'linear': float(speed), 'angular': float(angular), 'guard_stop': guard,
                   'forward_clearance_m': clearance, 'odom_xy': self.position(),
                   'detections': [{'class': result.names[int(b.cls.item())],
                                   'confidence': float(b.conf.item())} for b in result.boxes]}
            self.history.append(row)
            if len(self.history) == 1:
                cv2.imwrite(str(self.args.output / 'first_detection.png'), result.plot())
            cv2.imwrite(str(self.args.output / 'last_detection.png'), result.plot())
        self.stop()
        summary = {'mode': 'basic_lane_tracking', 'drive_enabled': self.args.drive,
                   'frames': len(self.history),
                   'valid_frames': sum(r['valid'] for r in self.history),
                   'tracker_modes': dict(Counter(r['mode'] for r in self.history)),
                   'guard_stops': sum(r['guard_stop'] for r in self.history),
                   'displacement_m': math.dist(start_position, self.position()),
                   'model': self.args.model.name,
                   'algorithm': 'pinky_autonomous.LaneTracker + AutonomousDriveNode._compute_command',
                   'history': self.history}
        (self.args.output / 'result.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps({k: v for k, v in summary.items() if k != 'history'}, indent=2))
        if summary['valid_frames'] < 3:
            raise RuntimeError('Too few valid lane frames to validate tracking')
        if self.args.drive and summary['displacement_m'] < 0.03:
            raise RuntimeError('Lane tracking did not move the robot')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--robot', default='amr_01')
    parser.add_argument('--drive', action='store_true', help='Publish cmd_vel in the isolated simulation')
    parser.add_argument('--duration', type=float, default=4.0, help='Simulation seconds')
    parser.add_argument('--wall-timeout', type=float, default=180.0)
    parser.add_argument('--output', type=Path, required=True, help='Fresh evidence directory')
    args = parser.parse_args()
    if not args.model.exists() or args.duration <= 0:
        parser.error('Provide an existing model and positive duration')
    args.output.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(2)
    rclpy.init()
    node = SimulationLane(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    receiver = threading.Thread(target=executor.spin, daemon=True)
    receiver.start()
    try:
        node.run_trial()
    finally:
        for _ in range(5):
            node.stop()
            time.sleep(0.05)
        executor.shutdown()
        receiver.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
