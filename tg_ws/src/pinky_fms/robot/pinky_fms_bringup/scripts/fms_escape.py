#!/usr/bin/env python3
"""벽에서 빠져나오기 (FMS 복구 동작): 가장 가까운 장애물의 반대 방향으로 정해진 거리만큼 천천히 이동한다.

왜 따로 만드나: Nav2 의 BackUp / DriveOnHeading 은 출발 전에 costmap 충돌 검사를 해서, 이미 벽에 붙어 있으면
바로 실패한다(COLLISION_AHEAD). 그래서 costmap·AMCL 이 아니라 라이다 원시 값(scan)으로 방향을 정하고,
진행 방향에 실제로 장애물이 가까워지면 멈춘다. 관제(fleet_traffic)가 Nav2 목표를 취소한 뒤에만 요청한다.

  구독  escape_cmd    (std_msgs/String JSON {"id": n, "dist": 0.20} / {"id": n, "cancel": true})   ← 관제
  발행  escape_status (std_msgs/String JSON {"id", "state": running|done|failed, "moved", "msg"})
        cmd_vel       (geometry_msgs/Twist, 요청을 처리하는 동안만)
  방향: 몸체에서 REPULSE_R 안의 라이다 점들이 미는 방향(가까울수록 강하게)의 합.
        앞·뒤 중 그 방향에 가까운 쪽으로 움직인다(필요하면 먼저 제자리 회전, 최대 90°).
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

REPULSE_R = 0.35        # 이 거리(로봇 중심 기준) 안의 점들로 빠져나갈 방향을 정한다
NEAR_NONE = 0.20        # 이 안에 아무것도 없으면 움직일 필요 없음 (done, moved 0)
SPEED = 0.05            # 직진 속도 (m/s)
TURN_W = 0.6            # 회전 속도 (rad/s)
STOP_CLEAR = 0.11       # 진행 방향 ±40° 안의 점이 로봇 중심에서 이보다 가까우면 멈춤 (몸체 반폭 0.06 + 여유)
TIMEOUT = 15.0


class FmsEscape(Node):
    def __init__(self):
        super().__init__('fms_escape')
        self.scan_pts = None            # base_footprint 기준 (N, 2)
        self.odom = None
        self.job = None
        self.lidar_yaw, self.lidar_x = math.pi, -0.017     # URDF 값 (TF 를 읽으면 그 값으로 바꾼다)
        self.tf_ok = False
        try:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except ImportError:
            self.tf_buffer = None
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'escape_status', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 5)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(String, 'escape_cmd', self.on_cmd, 10)
        self.create_timer(0.05, self.step)

    # ---------- 입력 ----------
    def _lidar_tf(self, frame):
        if self.tf_ok or self.tf_buffer is None:
            return
        try:
            from rclpy.time import Time
            t = self.tf_buffer.lookup_transform('base_footprint', frame, Time())
            q = t.transform.rotation
            self.lidar_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.lidar_x = t.transform.translation.x
            self.tf_ok = True
        except Exception:
            pass

    def on_scan(self, m):
        self._lidar_tf(m.header.frame_id)
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment + self.lidar_yaw
        ok = np.isfinite(r) & (r > max(m.range_min, 0.02)) & (r < 1.0)
        self.scan_pts = np.stack([self.lidar_x + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
        self.scan_t = time.monotonic()

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_cmd(self, m):
        try:
            d = json.loads(m.data)
            jid, dist = int(d['id']), float(d.get('dist', 0.20))
        except (ValueError, KeyError, TypeError):
            return
        if d.get('cancel'):             # 관제의 E-STOP·취소: 즉시 정지
            if self.job is not None:
                self._finish('failed', '취소됨')
            return
        if self.job is not None and self.job['id'] == jid:
            return                      # 같은 요청 재전송
        dist = min(max(dist, 0.05), 0.40)
        self.job = {'id': jid, 'dist': dist, 't0': time.monotonic(), 'phase': 'plan'}
        self.get_logger().info(f'빠져나오기 요청 #{jid}: {dist:.2f} m')

    # ---------- 동작 ----------
    def _finish(self, state, msg=''):
        j = self.job
        self.cmd_pub.publish(Twist())
        moved = 0.0
        if j.get('start') and self.odom:
            moved = math.hypot(self.odom[0] - j['start'][0], self.odom[1] - j['start'][1])
        self.status_pub.publish(String(data=json.dumps({'id': j['id'], 'state': state, 'moved': round(moved, 3), 'msg': msg})))
        self.get_logger().info(f'빠져나오기 #{j["id"]} {state} (이동 {moved:.2f} m) {msg}')
        self.job = None

    def _escape_dir(self, pts):
        """REPULSE_R 안의 점들이 미는 방향 (로봇 기준 각도) 과 가장 가까운 점 거리"""
        d = np.hypot(pts[:, 0], pts[:, 1])
        near = d < REPULSE_R
        if not near.any():
            return None, float(d.min()) if d.size else 9.0
        w = (REPULSE_R - d[near]) / np.maximum(d[near], 1e-3)
        v = -(pts[near] * w[:, None]).sum(axis=0)
        return math.atan2(v[1], v[0]), float(d[near].min())

    def _blocked(self, pts, heading):
        """heading(로봇 기준, 0=앞 π=뒤) ±40° 안에 STOP_CLEAR 보다 가까운 점이 있나"""
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        diff = np.abs((ang - heading + math.pi) % (2 * math.pi) - math.pi)
        d = np.hypot(pts[:, 0], pts[:, 1])
        return bool(np.any((diff < math.radians(40)) & (d < STOP_CLEAR)))

    def step(self):
        j = self.job
        if j is None:
            return
        now = time.monotonic()
        if now - j['t0'] > TIMEOUT:
            return self._finish('failed', '시간 초과')
        if self.scan_pts is None or self.odom is None or now - getattr(self, 'scan_t', 0) > 1.0:
            if now - j['t0'] > 3.0:
                return self._finish('failed', 'scan/odom 없음')
            return
        pts = self.scan_pts
        if j['phase'] == 'plan':
            ang, dmin = self._escape_dir(pts)
            if ang is None or dmin > NEAR_NONE:
                return self._finish('done', f'가까운 장애물 없음 ({dmin:.2f} m)')
            # 앞으로 갈지 뒤로 갈지: 빠져나갈 방향과 가까운 쪽. 남은 각도만큼 먼저 회전
            back = abs(ang) > math.pi / 2
            move_dir = math.pi if back else 0.0
            turn = ((ang - move_dir + math.pi) % (2 * math.pi)) - math.pi
            j.update(phase='turn', back=back, yaw0=self.odom[2], turn=turn)
            self.get_logger().info(f'가장 가까운 장애물 {dmin:.2f} m → {math.degrees(ang):+.0f}° 쪽으로 {"후진" if back else "전진"} '
                                   f'(먼저 {math.degrees(turn):+.0f}° 회전)')
            self.status_pub.publish(String(data=json.dumps({'id': j['id'], 'state': 'running', 'moved': 0.0, 'msg': ''})))
        if j['phase'] == 'turn':
            done = abs(((self.odom[2] - j['yaw0']) - j['turn'] + math.pi) % (2 * math.pi) - math.pi) < 0.08 or abs(j['turn']) < 0.08
            if done:
                j.update(phase='move', start=(self.odom[0], self.odom[1]))
            else:
                t = Twist()
                t.angular.z = math.copysign(TURN_W, j['turn'])
                self.cmd_pub.publish(t)
                return
        if j['phase'] == 'move':
            moved = math.hypot(self.odom[0] - j['start'][0], self.odom[1] - j['start'][1])
            if moved >= j['dist']:
                return self._finish('done')
            if self._blocked(pts, math.pi if j['back'] else 0.0):
                return self._finish('done' if moved > 0.05 else 'failed', '진행 방향에 장애물')
            t = Twist()
            t.linear.x = -SPEED if j['back'] else SPEED
            self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = FmsEscape()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
