#!/usr/bin/env python3
"""FMS 미션: Vision-based Lane Following (목적지 왕복).

pinky_autonomous 의 AutonomousDriveNode 를 수정하지 않고 상속해서 동작만 바꾼다.
(원본은 'cross_lane 을 만나면 출발점으로 복귀' — SLAM 방식 등 다른 구현은 원본/별도 노드로 남겨 둔다)

  0) 스택은 GUI 에서 미션을 고르면 미리 뜬다 (목적지 없이 IDLE). Init Pose 로 위치를 잡고,
     Go 를 누르면 관제가 lane_cmd 로 목적지를 보낸다. 미션이 끝나도 노드는 그대로 두고 다음 목적지를 기다린다.
       lane_cmd (std_msgs/String JSON): {"id": n, "cmd": "go", "x": .., "y": ..} | {"id": n, "cmd": "cancel"}
                                        | {"id": n, "cmd": "home", "x": .., "y": ..}  (편도: 그 지점까지 차선을 따라 가서 정지, 복귀 없음)
     (launch 의 goal_x/goal_y 를 주면 위치가 잡히는 대로 바로 출발 — 터미널 실행용)
  1) 출발 위치: 목적지를 받은 순간의 AMCL(map) 위치. 위치가 없으면 Init Pose 를 받을 때까지 정지한다.
     (원본처럼 일정 시간 뒤 odom 으로 대체하거나 복귀 없이 출발하지 않는다)
  2) 출발: 먼저 지도로 목적지 쪽 방향을 판단해 필요하면 제자리 U턴한다 (왕복 뒤에는 처음과 반대를 보고 서 있다).
     가는 길(outbound): 차선을 따라 목적지로. 갈림길에서는 목적지 쪽 출구를 고른다.
  3) 목적지 도착(goal_arrive_dist 안): 정지 → 지도로 출발점 쪽 방향을 골라 회전 → 차선을 따라 복귀(inbound).
     복귀 중 갈림길에서는 출발점 쪽 출구를 고른다.
  4) 출발점 도착: 정지 (ARRIVED).
  도착 판정: 거리(goal_arrive_dist)뿐 아니라 로봇과 목표 사이에 지도상 벽이 없어야 한다 (벽 너머 목표를 도착으로 착각하지 않게).
  앞 사물 정지: 원본은 초음파만 본다. 여기서는 라이다 앞쪽 통로(폭 ±obstacle_half_width)의 점 중 지도 벽이 아닌 것까지 합쳐
     같은 규칙(obstacle_dist 감속, ultrasonic_stop_dist 정지, 횡단보도 확인 중 crosswalk_stop_dist 정지)을 적용한다.
     초음파 0.02 m 미만 값(센서 오류로 자주 나옴)은 버린다.
  코너 모드: |steer| > corner_enter 이거나 차선이 화면 위쪽에만 보이면 속도 상한 corner_speed(고정),
     |steer| < corner_exit 가 corner_exit_time 동안 이어져야(직진 확인) 정속 복귀. 원본은 조향이 잠깐 작아지면 바로 가속한다.
  몸체 점 제외: 라이다에 잡히는 자기 몸체(뒤쪽 약 0.10 m 등)는 사물로 보지 않는다 (2026-10-08 amr_01: 뒤 0.10 m 점 때문에 69초 대기).
  차선 재탐색: 차선을 놓치면(LOST) 차선 지도에서 진행 방향 앞쪽의 차선 띠 안쪽 점을 골라 천천히 그쪽으로 간다. 카메라가 다시 차선을 보면 원래 추종.
  T자 갈림길 좌/우: 차선 지도로 건너는 차선 띠 가운데까지 조금 더 들어간 뒤 제자리 90° 회전 (한쪽 차선만 따라가다 반대쪽으로 돌던 문제).
  처리 속도 제한: 실제 제어 주기(fps)로 한 프레임에 max_step_per_frame 이상 가지 않게 속도 상한 (amr_01 3 Hz 사례).
  코너 예측: 차선 지도에서 진행 방향의 차선 띠가 corner_lookahead 안에서 끝나면(코너 직전) 미리 코너 속도로 감속.
  갈림길·출발: 정지 → 지도로 출구 결정 → 그 출구 쪽 통로(라이다, 지도 벽 제외)와 관제가 보낸 다른 로봇 위치(other_robots)가
     junction_clear_time 동안 비어 있어야 출발. 앞이 벽에 가까운 갈림길(0.30 m)에서 좌/우는 제자리 90° 회전 뒤 그쪽 차선 추종.
  벽 오인 방지: 초음파 값이 그 방향 지도 벽 거리와 같으면(±6 cm) 사물로 보지 않는다. 라이다는 지도 벽 wall_ignore_dist 안의 점 제외.
  차선 위 마주침 양보 (관제 lane_traffic 노드가 판단, 이 노드는 실행만): lane_traffic_cmd (std_msgs/String JSON)
     {"id","cmd":"hold"}                      즉시 정지 (미션 상태 유지)
     {"id","cmd":"yield","x","y"}             지금 위치·방향을 복귀 자리로 기억 → 차선 밖 (x,y) 로 비켜섬 → YIELDED 로 정지
     {"id","cmd":"resume","role":"priority"|"yielder"}  HOLD 면 그대로 계속, 비켜선 상태면 복귀 자리·방향으로 돌아가 차선 추종 재개
        (선택 "x","y","yaw": 원래 자리 대신 그 자리·방향으로 복귀. 복귀 중·실패 상태에서도 새 id 로 오면 목표를 바꿔 다시 시작)
     같은 id 는 무시(관제가 1초마다 keepalive 로 재전송). HOLD·YIELDED 에서 10초 동안 명령이 없으면 resume 으로 간주.
     비켜서기·복귀 중에는 차선 추종을 멈추고 지도 위치로 직접 움직인다. 라이다 원시값 앞 ±0.08 m 에 앞면 0.06 m 보다
     가까운 점이 있으면 정지, 3초 넘게 막히거나 20초가 지나면 YIELD_FAILED 로 그 자리에 정지.
  상태는 lane_status (std_msgs/String JSON) 로 5 Hz 발행 → 로봇 LCD(fms_lcd_status)와 관제가 표시한다.
  LCD 는 이 노드가 직접 쓰지 않는다 (launch 에서 enable_lcd:=false).
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from pinky_autonomous.autonomous_drive_node import (
    AutonomousDriveNode, LAMP_DRIVE, LAMP_OFF, LAMP_STOP, STATE_CAUTION, STATE_COLLISION, STATE_DRIVING, STATE_CW_BLOCKED, STATE_OBSTACLE_STOP, STATE_RETURN_FAILED)
from pinky_autonomous.route_planner import choose_exit


class FmsLaneMission(AutonomousDriveNode):
    def __init__(self):
        super().__init__()          # 모델·카메라 로드 (주행 타이머는 spin 이 시작돼야 돈다)
        self.declare_parameter('goal_x', float('nan'))
        self.declare_parameter('goal_y', float('nan'))
        self.declare_parameter('goal_arrive_dist', 0.30)   # 목적지 반경 (m)
        self.declare_parameter('robot_name', '')
        gx, gy = float(self.get_parameter('goal_x').value), float(self.get_parameter('goal_y').value)
        self.goal = None if math.isnan(gx) or math.isnan(gy) else (gx, gy)
        self.goal_dist = float(self.get_parameter('goal_arrive_dist').value)
        self.robot_name = self.get_parameter('robot_name').value or self.get_namespace().strip('/')
        self.leg = 'outbound'           # outbound(목적지로) | inbound(출발점으로)
        self.leg_t0 = 0.0
        self.cmd_id = None              # 마지막으로 처리한 lane_cmd id (관제가 수신 확인에 쓴다)
        self.result = None              # 직전 미션 결과: None | 'ARRIVED' | 'CANCELED'
        self.departing = False          # 출발 직후 목적지 쪽 방향 판단·회전 중
        self.one_way = False            # True: 목적지에서 멈추고 끝 (lane_cmd home)
        self.status_pub = self.create_publisher(String, 'lane_status', 10)
        self.create_subscription(String, 'lane_cmd', self._on_cmd, 10)
        self.create_subscription(String, '/fleet/global_cmd', self._on_global_cmd, 10)   # GLOBAL E-STOP (관제 전체)
        # 지도(벽 확인용, 원본의 갈림길용 구독과 별개로 계속 유지)와 라이다(앞 사물)
        self.declare_parameter('obstacle_half_width', 0.10)    # 라이다 앞 통로 반폭 (몸체 반폭 0.06 + 여유)
        self.declare_parameter('lidar_front_offset', 0.07)     # 로봇 중심 → 앞면 거리 (초음파와 같은 기준으로 맞춤)
        self.declare_parameter('wall_ignore_dist', 0.12)       # 지도 벽에서 이 거리 안의 라이다 점은 사물로 보지 않음 (AMCL 오차 여유 포함)
        self.declare_parameter('corner_speed', 0.06)           # 코너 속도 상한 (m/s, drive_speed 와 무관)
        self.declare_parameter('corner_enter', 0.30)           # |steer| 이보다 크면 코너
        self.declare_parameter('corner_exit', 0.15)            # |steer| 이보다 작은 상태가
        self.declare_parameter('corner_exit_time', 1.0)        #   이 시간 이어지면 직진 확인 → 정속
        self.declare_parameter('max_step_per_frame', 0.025)    # 한 제어 주기에 갈 수 있는 최대 거리 (m). 3 Hz → 0.075 m/s, 10 Hz → 제한 없음
        self.declare_parameter('corner_lookahead', 0.30)       # 차선 지도에서 앞 차선 띠가 이 거리 안에서 끝나면 코너 직전으로 보고 감속
        self.declare_parameter('junction_check_dist', 0.50)    # 갈림길 출구 통로 길이 (m)
        self.declare_parameter('junction_check_half_width', 0.12)
        self.declare_parameter('junction_robot_radius', 0.60)  # 다른 로봇이 출구 쪽 이 거리 안(폭 ±0.20 m)에 있으면 갈림길 대기
        self.declare_parameter('junction_clear_time', 1.0)     # 이 시간 연속으로 비어 있어야 출발
        self.obs_half_w = float(self.get_parameter('obstacle_half_width').value)
        self.front_off = float(self.get_parameter('lidar_front_offset').value)
        self.wall_ignore = float(self.get_parameter('wall_ignore_dist').value)
        gp = lambda n: float(self.get_parameter(n).value)
        self.corner_speed, self.corner_enter = gp('corner_speed'), gp('corner_enter')
        self.corner_exit, self.corner_exit_time = gp('corner_exit'), gp('corner_exit_time')
        self.jn_dist, self.jn_half_w = gp('junction_check_dist'), gp('junction_check_half_width')
        self.jn_robot_r, self.jn_clear_time = gp('junction_robot_radius'), gp('junction_clear_time')
        self.max_step, self.corner_look = gp('max_step_per_frame'), gp('corner_lookahead')
        self.loop_dt, self.loop_t = 0.1, None      # 실제 제어 주기 (지수 평균)
        self.lane_free = None                      # 차선 지도의 빈 칸 (= 차선 띠)
        self.in_corner, self.corner_calm_t = False, None
        self.jn_choice, self.jn_clear_t, self.jn_busy, self.jn_note_t = None, None, '', 0.0
        self.post_turn_bias = None
        self.scan_free, self.others, self.others_t = np.zeros((0, 2)), np.zeros((0, 2)), 0.0
        self.create_subscription(PoseArray, 'other_robots', self._on_others, 5)   # 관제(fleet_traffic)가 보내는 다른 로봇 위치 (map)
        self.wall_grid = self.near_wall = None
        self.lidar_dist, self.lidar_t = float('inf'), 0.0
        self.raw_front = float('inf')          # 라이다 원시값(지도 벽 필터 없음) 앞 ±0.08 m, 앞면 기준 (비켜서기·복귀 중 안전)
        # 차선 위 마주침 양보 (lane_traffic_cmd)
        self.traffic = None                    # None | HOLD | YIELDING | YIELDED | REJOINING | YIELD_FAILED
        self.traffic_id = None
        self.traffic_role = None
        self.tr_rx = self.tr_t0 = 0.0          # 마지막 lane_traffic_cmd 수신 / 이번 이동 시작 시각
        self.tr_target = self.tr_yaw = None    # 이동 목표 (x, y) / 마지막에 맞출 방향
        self.tr_rejoin = None                  # 비켜서기 전 위치·방향 (x, y, yaw)
        self.tr_blocked_t = None
        self.tr_driving = False                # 회전 후 직진 중 (방향 오차 허용을 넓힌다)
        self.tr_at = False                     # 목표 위치 도착 (이후 방향만 맞춤)
        self.tr_role_t = 0.0
        self.create_subscription(String, 'lane_traffic_cmd', self._on_traffic_cmd, 10)
        self.create_timer(0.1, self._traffic_step)
        qos_map = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_wall_map, qos_map)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.2, self._publish_status)          # 관제 lane_traffic 이 마주침을 판단하므로 5 Hz
        if self.goal is None:
            self.get_logger().info('목적지 대기 중 (GUI: Init Pose → Set Goal → Go)')
        else:
            self.get_logger().info(f'🎯 Lane Following 미션: 목적지 ({self.goal[0]:.2f}, {self.goal[1]:.2f}) 왕복')

    # ---------- 관제 명령: 목적지 지정 / 취소 ----------
    def _on_cmd(self, m):
        try:
            d = json.loads(m.data)
            cid, cmd = int(d['id']), d['cmd']
            goal = (float(d['x']), float(d['y'])) if cmd in ('go', 'home') else None
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn(f'잘못된 lane_cmd: {m.data}')
            return
        if cid == self.cmd_id:
            return                      # 같은 명령 재전송 (관제는 수신 확인될 때까지 몇 번 보낸다)
        if goal is not None and not all(map(math.isfinite, goal)):
            return
        self.cmd_id = cid
        self._reset_mission()
        if goal is not None:
            self.goal, self.result, self.one_way = goal, None, cmd == 'home'
            self.get_logger().info(f'🎯 Lane Following 미션: 목적지 ({goal[0]:.2f}, {goal[1]:.2f}) {"편도" if self.one_way else "왕복"}')
        else:
            self.goal, self.result = None, 'CANCELED'
            self.cmd_pub.publish(Twist())
            self.get_logger().info('⏹ Lane Following 취소: 정지 후 다음 목적지 대기')

    # ---------- 지도 벽: 도착 판정(벽 너머 제외)과 라이다 점 거르기 ----------
    def _on_wall_map(self, m):
        g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        occ = g >= 65
        r = max(1, int(round(self.wall_ignore / m.info.resolution)))
        near = occ.copy()
        for dy in range(-r, r + 1):              # 원형으로 넓힌 벽 (한 번만 계산)
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r and (dx or dy):
                    near |= np.roll(np.roll(occ, dy, axis=0), dx, axis=1)
        self.wall_grid = (occ, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)
        self.lane_free = (g >= 0) & (g <= 20)                     # 차선 지도: 빈 칸이 차선 띠
        self.near_wall = near

    def _cells(self, xy):
        occ, res, ox, oy = self.wall_grid
        c = np.floor((xy[:, 0] - ox) / res).astype(int)
        r = np.floor((xy[:, 1] - oy) / res).astype(int)
        ok = (r >= 0) & (r < occ.shape[0]) & (c >= 0) & (c < occ.shape[1])
        return r, c, ok

    def _wall_between(self, a, b):
        """a→b 직선이 지도 벽을 지나는가 (지도가 없으면 False: 예전처럼 거리만 본다)"""
        if self.wall_grid is None:
            return False
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.wall_grid[1] * 0.5)) + 1)
        xy = np.stack([np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)], axis=1)
        r, c, ok = self._cells(xy)
        return bool(self.wall_grid[0][r[ok], c[ok]].any())

    def _wall_separates(self, a, b, half=0.08):
        """두 점이 벽으로 갈라져 있는가: 가운데 직선과 좌우로 half 만큼 옮긴 평행선이 모두 벽을 지나야 True.
        직선 하나로만 보면 칸막이 끝을 스치는 경우(실제로는 이어진 곳)를 벽 너머로 오판한다 (pinky-8e 지적)"""
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy) or 1.0
        px, py = -dy / n * half, dx / n * half
        return all(self._wall_between((a[0] + k * px, a[1] + k * py), (b[0] + k * px, b[1] + k * py)) for k in (0, -1, 1))

    def _reached(self, key, pose, target, radius):
        """도착: 반경 안이거나, 반경의 2배 안까지 다가왔다가 멀어지기 시작함(가장 가까운 점을 지남).
        차선은 목표점을 정확히 지나지 않을 수 있어(띠 폭·위치 오차) 지나친 뒤 계속 가 버리는 것을 막는다"""
        if self._wall_between(pose, target):
            return False
        d = math.hypot(pose[0] - target[0], pose[1] - target[1])
        best = self.__dict__.setdefault('_closest', {}).get(key)
        if best is None or d < best:
            self._closest[key] = best = d
        return d <= radius or (best <= 2 * radius and d > best + 0.03)

    def _arrived_at(self, pose, target, radius):
        return math.hypot(pose[0] - target[0], pose[1] - target[1]) <= radius and not self._wall_between(pose, target)   # 도착은 보수적으로 (직선 하나라도 벽이면 아님)

    def _wall_ahead(self, radians=(0.0,), max_d=1.0, pose=None):
        """로봇 중심에서 앞(+각도들) 방향으로 지도 벽까지의 최소 거리 (지도·위치가 없으면 None)"""
        pose = pose or (self._current_pose() if self.pose_frame == self.map_frame else None)
        if self.wall_grid is None or pose is None:
            return None
        res = self.wall_grid[1]
        d = np.arange(0.0, max_d, res)
        best = max_d
        for a in radians:
            h = pose[2] + a
            xy = np.stack([pose[0] + d * math.cos(h), pose[1] + d * math.sin(h)], axis=1)
            r, c, ok = self._cells(xy)
            hit = ~ok
            hit[ok] = self.wall_grid[0][r[ok], c[ok]]
            if hit.any():
                best = min(best, float(d[np.argmax(hit)]))
        return best

    def _lane_ahead(self, pose=None, max_d=0.6):
        """진행 방향으로 차선 띠(지도 빈 칸)가 끝나는 거리. 차선 밖이거나 지도 없으면 None"""
        pose = pose or (self._current_pose() if self.pose_frame == self.map_frame else None)
        if self.lane_free is None or pose is None:
            return None
        d = np.arange(0.0, max_d, self.wall_grid[1])
        xy = np.stack([pose[0] + d * math.cos(pose[2]), pose[1] + d * math.sin(pose[2])], axis=1)
        r, c, ok = self._cells(xy)
        free = np.zeros(len(d), bool)
        free[ok] = self.lane_free[r[ok], c[ok]]
        if not free[0]:
            return None                           # 지금 차선 위가 아님 (위치 오차 등): 판단하지 않음
        return float(d[np.argmin(free)]) if not free.all() else max_d

    def _band_inside(self, x, y, margin=0.05):
        pts = [(x, y)] + [(x + margin * math.cos(k * math.pi / 4), y + margin * math.sin(k * math.pi / 4)) for k in range(8)]
        r, c, ok = self._cells(np.array(pts))
        return bool(ok.all() and self.lane_free[r, c].all())

    def _reacquire(self, now):
        """차선을 놓쳤을 때(LOST): 진행 방향 ±70° 안, 0.12~0.30 m 앞의 차선 띠 안쪽 점으로 천천히 간다. (v, w, stopped) 또는 None"""
        if self.tracker.lost_frames < 5 or self.lane_free is None or self.pose_frame != self.map_frame:
            return None
        if self.junction_latched and self.return_phase not in (None, 'following'):
            return None
        pose = self._current_pose()
        if pose is None:
            return None
        best = None
        for r_ in (0.12, 0.18, 0.24, 0.30):
            for deg in range(-70, 71, 10):
                h = pose[2] + math.radians(deg)
                x, y = pose[0] + r_ * math.cos(h), pose[1] + r_ * math.sin(h)
                if self._band_inside(x, y) and not self._wall_between(pose[:2], (x, y)):
                    cost = abs(deg) / 70.0 + r_
                    if best is None or cost < best[0]:
                        best = (cost, x, y, math.radians(deg))
            if best is not None:
                break
        if best is None:
            return None
        _, x, y, err = best
        self.rq_t = now                                          # 재탐색 중 (상태 표시용)
        if now - self.__dict__.get('_rq_note_t', 0.0) > 3.0:
            self._rq_note_t = now
            self.get_logger().info(f'🧭 차선 재탐색: 지도상 차선 띠 ({x:.2f}, {y:.2f}) 쪽으로 ({math.degrees(err):+.0f}°)')
        if now - self.lidar_t > 0.5 or self.raw_front < 0.06:
            return 0.0, 0.0, True
        if abs(err) > 0.35:
            return 0.0, math.copysign(0.5, err), False           # 먼저 방향을 맞춘다
        return 0.04, max(-0.6, min(0.6, 1.5 * err)), False

    def _on_others(self, m):
        self.others = np.array([[p.position.x, p.position.y] for p in m.poses]).reshape(-1, 2)
        self.others_t = time.monotonic()

    # ---------- 코너 모드: 코너에서는 고정 저속, 직진이 확인돼야 정속 ----------
    def _compute_command(self, steer, valid):
        speed, angular, stopped = super()._compute_command(steer, valid)
        now = time.monotonic()
        if self.loop_t is not None and now - self.loop_t < 0.6:     # 정지·갈림길 대기 뒤의 긴 간격은 제외
            self.loop_dt = 0.8 * self.loop_dt + 0.2 * (now - self.loop_t)
        self.loop_t = now
        if stopped and valid:
            return speed, angular, stopped
        if stopped and self.state not in (STATE_DRIVING, STATE_CAUTION):
            return speed, angular, stopped                       # 사물·횡단보도·갈림길·도착 정지는 그대로
        if stopped:
            rq = self._reacquire(now)                            # 차선을 오래 놓쳐 원본이 멈춘 경우: 지도로 차선 띠를 찾아간다
            return rq if rq is not None else (speed, angular, stopped)
        # 처리 속도 제한: 느린 로봇(추론 지연)은 한 프레임 사이에 너무 멀리 가서 코너를 늦게 본다
        cap = self.max_step / max(self.loop_dt, 0.05)
        if speed > cap:
            k = cap / speed
            speed, angular = cap, angular * k
            if now - getattr(self, '_cap_note_t', 0.0) > 10.0:
                self._cap_note_t = now
                self.get_logger().info(f'🐢 제어 주기 {1.0 / self.loop_dt:.1f} Hz: 속도 상한 {cap:.3f} m/s (프레임당 {self.max_step * 100:.1f} cm)')
        if not valid:
            rq = self._reacquire(now)
            return rq if rq is not None else (speed, angular, stopped)
        ahead = self._lane_ahead()
        map_corner = ahead is not None and ahead < self.corner_look
        if abs(steer) > self.corner_enter or self.tracker.far or map_corner:
            if not self.in_corner:
                why = f'steer {steer:+.2f}' + (', 먼 차선만 보임' if self.tracker.far else '') + (f', 지도상 {ahead:.2f} m 앞 코너' if map_corner else '')
                self.get_logger().info(f'↪️ 코너 감지 ({why}): {self.corner_speed:.2f} m/s 로 감속')
            self.in_corner, self.corner_calm_t = True, None
        elif self.in_corner:
            if abs(steer) < self.corner_exit and not map_corner:
                self.corner_calm_t = self.corner_calm_t or now
                if now - self.corner_calm_t >= self.corner_exit_time:
                    self.in_corner = False
                    self.get_logger().info('⬆️ 직진 확인: 정속 주행 복귀')
            else:
                self.corner_calm_t = None
        if self.in_corner and speed > self.corner_speed:
            k = self.corner_speed / speed                        # 회전 반경(각속도/속도)을 유지하도록 같은 비율로
            speed, angular = self.corner_speed, angular * k
        return speed, angular, stopped

    # ---------- 갈림길·출발: 출구 쪽이 비어 있어야 출발 ----------
    def _exit_busy(self, choice, pose):
        """고른 출구 쪽 통로에 지도에 없는 물체나 다른 로봇이 있으면 그 이유 문자열, 비어 있으면 ''"""
        now = time.monotonic()
        d = {'straight': 0.0, 'left': math.pi / 2, 'right': -math.pi / 2, 'back': math.pi}.get(choice, 0.0)
        cd, sd = math.cos(-d), math.sin(-d)
        if now - self.lidar_t > 0.5:
            return 'lidar 없음'
        p = self.scan_free
        if len(p):
            fx, fy = cd * p[:, 0] - sd * p[:, 1], sd * p[:, 0] + cd * p[:, 1]     # 출구 방향 기준 좌표
            inside = (fx > 0.05) & (fx < self.jn_dist) & (np.abs(fy) < self.jn_half_w)
            if inside.any():
                return f'출구 쪽 물체 {float(fx[inside].min()):.2f} m'
        if pose is not None and len(self.others) and now - self.others_t < 2.0:
            # 다른 로봇은 출구 통로(길이 junction_robot_radius, 폭 ±0.20 m) 안에 있을 때만 막힌 것으로 본다.
            # 반경으로만 보면 갈림길 근처의 두 로봇이 서로를 기다리는 교착이 생긴다 (누가 먼저 갈지는 lane_traffic 이 정함)
            dx, dy = self.others[:, 0] - pose[0], self.others[:, 1] - pose[1]
            h = pose[2] + d
            fx, fy = math.cos(h) * dx + math.sin(h) * dy, -math.sin(h) * dx + math.cos(h) * dy
            inside = (fx > 0.0) & (fx < self.jn_robot_r) & (np.abs(fy) < 0.20)
            for i in np.nonzero(inside)[0]:                    # 벽 너머 로봇은 출구를 막지 않는다 (2026-10-08: 벽 건너 0.35 m 로봇 때문에 계속 대기)
                if self._wall_separates(pose[:2], self.others[i]):
                    inside[i] = False
            if inside.any():
                return f'출구 쪽 다른 로봇 {float(np.hypot(dx, dy)[inside].min()):.2f} m'
        return ''

    def _junction_wait(self, now):
        pose = self._current_pose()
        busy = self._exit_busy(self.jn_choice, pose)
        if busy:
            kind = busy.split(' ')[0:3]
            if kind != self.__dict__.get('_jn_kind') or now - self.jn_note_t > 30.0:   # 같은 이유면 30초마다 한 번만 기록
                self.get_logger().info(f'⏳ {self._jn_word()} 대기 ({self.jn_choice}): {busy}')
                self.jn_note_t, self._jn_kind = now, kind
            self.jn_busy, self.jn_clear_t, self.jn_busy_t = busy, None, now
            return
        if now - self.__dict__.get('jn_busy_t', 0.0) > 1.0:
            self.jn_busy = ''                    # 상태 표시는 1초 동안 비어야 지운다 (깜빡임 방지)
        self.jn_clear_t = self.jn_clear_t or now
        if now - self.jn_clear_t >= self.jn_clear_time:
            choice, self.jn_choice, self.jn_clear_t = self.jn_choice, None, None
            self.get_logger().info(f'✅ 출구({choice}) 쪽 {self.jn_clear_time:.0f}초 동안 비어 있음: 출발')
            self._begin_turn(choice, pose, now)

    def _jn_word(self):
        return '출발 방향' if (self.departing or self.on_lane_decision) else '갈림길'

    def _band_center_ahead(self, pose, max_d=0.40):
        """진행 방향으로 지금 있는 차선 띠가 끝나는 지점에서 반 띠(0.08 m) 앞 = 건너는 차선 띠 가운데까지 남은 거리"""
        if self.lane_free is None or pose is None:
            return 0.0
        d = np.arange(0.0, max_d, self.wall_grid[1])
        xy = np.stack([pose[0] + d * math.cos(pose[2]), pose[1] + d * math.sin(pose[2])], axis=1)
        r, c, ok = self._cells(xy)
        free = np.zeros(len(d), bool)
        free[ok] = self.lane_free[r[ok], c[ok]]
        if not free[0] or free.all():
            return 0.0
        return float(min(max(d[np.argmin(free)] - 0.08, 0.0), 0.20))

    def _begin_turn(self, choice, pose, now):
        if choice in ('left', 'right') and not self.departing and not self.on_lane_decision and pose is not None:
            # T자 갈림길 좌/우: 한쪽 차선만 따라 전진하면 반대쪽으로 도는 경우가 있었다(2026-10-08 amr_02, O 루프를 한 바퀴 돎).
            # 건너는 차선 띠 가운데까지 조금 더 들어간 뒤 제자리 90° 회전하고 일반 차선 추종으로 나간다
            adv = self._band_center_ahead(pose)
            if adv > 0.02 and not self.__dict__.get('jn_adv_done'):
                self.jn_adv = {'choice': choice, 'start': pose, 'dist': adv, 't0': now}
                self.return_phase, self.return_t0 = 'jn_adv', now
                self.get_logger().info(f'➡️ 갈림길 {choice}: 건너는 차선 가운데까지 {adv:.2f} m 전진 후 회전')
                return
            self.jn_adv_done = False
            self.exit_choice = choice
            self.turn_delta = math.pi / 2 if choice == 'left' else -math.pi / 2
            if self.map_sub is not None:
                self.destroy_subscription(self.map_sub)
                self.map_sub, self.latest_map = None, None
            self.turn_target = self._wrap(pose[2] + self.turn_delta)
            self.post_turn_bias = None
            self.return_phase, self.return_t0 = 'turning', now
            self.get_logger().info(f'↩️ 갈림길 {choice}: 제자리 90° 회전 후 차선 추종')
            return
        super()._begin_turn(choice, pose, now)

    def _junction_advance(self, now):
        j, pose = self.jn_adv, self._current_pose()
        moved = math.hypot(pose[0] - j['start'][0], pose[1] - j['start'][1]) if pose is not None else 0.0
        blocked = now - self.lidar_t > 0.5 or self.raw_front < 0.06
        if moved >= j['dist'] or now - j['t0'] > 6.0 or (blocked and now - j['t0'] > 0.5):
            self.cmd_pub.publish(Twist())
            self.jn_adv_done = True
            self._begin_turn(j['choice'], pose or j['start'], now)
            return
        t = Twist()
        t.linear.x = 0.0 if blocked else 0.04
        self.cmd_pub.publish(t)

    # ---------- 라이다 앞 사물 ----------
    def _on_scan(self, m):
        if self.tf_buffer is None:
            return
        try:
            from rclpy.time import Time
            t = self.tf_buffer.lookup_transform(self.base_frame, m.header.frame_id, Time())
        except Exception:
            return
        q = t.transform.rotation
        lyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        rg = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(rg)) * m.angle_increment + lyaw
        ok = np.isfinite(rg) & (rg > max(m.range_min, 0.03)) & (rg < 1.0)
        x = t.transform.translation.x + rg[ok] * np.cos(a[ok])
        y = t.transform.translation.y + rg[ok] * np.sin(a[ok])
        body = (x > -0.12) & (x < 0.085) & (np.abs(y) < 0.08)      # 자기 몸체·케이블에 맞은 점 (뒤 0.10 m 점이 실물에서 확인됨)
        x, y = x[~body], y[~body]
        raw = (x > 0.0) & (np.abs(y) <= 0.08)
        self.raw_front = float(x[raw].min() - self.front_off) if raw.any() else float('inf')
        pose = self._current_pose()
        if x.size and self.near_wall is not None and pose is not None and self.pose_frame == self.map_frame:
            cy, sy = math.cos(pose[2]), math.sin(pose[2])
            xy = np.stack([pose[0] + cy * x - sy * y, pose[1] + sy * x + cy * y], axis=1)
            r, c, okc = self._cells(xy)
            wall = np.ones(len(x), bool)                          # 지도 밖 = 벽으로 취급
            wall[okc] = self.near_wall[r[okc], c[okc]]
            x, y = x[~wall], y[~wall]
        self.scan_free = np.stack([x, y], axis=1)                 # 지도 벽이 아닌 점 (로봇 기준) — 갈림길 출구 확인용
        sel = (x > 0.0) & (np.abs(y) <= self.obs_half_w)            # 앞쪽 통로 안
        self.lidar_dist = float(x[sel].min() - self.front_off) if sel.any() else float('inf')
        self.lidar_t = time.monotonic()

    def _sonar_distance(self, now):
        d = super()._sonar_distance(now)
        if d < 0.02:
            d = float('inf')                     # 초음파 0.00~0.01 m 는 센서 오류 (실물 로그에서 정상 주행 중 반복)
        elif math.isfinite(d):
            exp = self._wall_ahead(radians=(-0.26, -0.13, 0.0, 0.13, 0.26))    # 초음파 빔(약 ±15°) 안의 지도 벽
            if exp is not None and d >= exp - self.front_off - 0.06:
                d = float('inf')                 # 지도 벽까지의 거리와 같다: 벽이지 사물이 아니다 (코너·갈림길에서 오인 정지하던 것)
        if now - self.lidar_t < 0.5:
            d = min(d, max(self.lidar_dist, 0.0))
        return d

    def _sonar_text(self, now):
        t = super()._sonar_text(now)
        if now - self.lidar_t < 0.5:
            t += f' lidar {self.lidar_dist:.2f}m' if math.isfinite(self.lidar_dist) else ' lidar clear'
        return t

    def _on_global_cmd(self, m):
        if m.data.strip().upper() != 'E_STOP':
            return
        was_running = self.goal is not None and self.result is None
        self._reset_mission()
        self.goal, self.result = None, 'E_STOP'
        for _ in range(3):
            self.cmd_pub.publish(Twist())
        if was_running:
            self.get_logger().warn('🛑 GLOBAL E-STOP: Lane Following 정지 후 다음 목적지 대기')

    def _reset_mission(self):
        """한 번의 왕복 미션 상태를 처음으로 되돌린다. swap_lanes 는 로봇이 실제로 향한 방향이라 유지한다."""
        self.start_pose, self.pose_frame = None, self.map_frame
        self.leg, self.leg_t0 = 'outbound', 0.0
        self.state = STATE_DRIVING
        self.junction_frames, self.junction_latched = 0, False
        self.return_phase, self.return_t0 = None, 0.0
        self.turn_target, self.exit_choice = None, None
        self.lane_bias, self.bias_pose = None, None
        self.advance_pose = None
        self.cw_armed, self.cw_check_active, self.cw_clear_since, self.cw_block_t0 = True, False, None, None
        self.ramp_t0, self.ramp_n = None, 0
        self.prev_lane = {'left': None, 'right': None}
        self.departing = False
        self.jn_choice, self.post_turn_bias, self.in_corner = None, None, False
        self.jn_adv_done = False
        self._closest = {}
        self._traffic_clear()

    # ---------- 출발 위치: 목적지를 받은 뒤의 AMCL(map) 위치 ----------
    def _try_record_start_pose(self):
        if self.start_pose is not None or self.goal is None or self.tf_buffer is None:
            return
        pose = self._lookup_pose(self.map_frame)
        if pose is None:
            return                      # Init Pose 를 받아 AMCL 이 map→odom 을 낼 때까지 기다린다
        self.start_pose = pose
        self.pose_frame = self.map_frame
        self.leg_t0 = time.monotonic()
        self.get_logger().info(f'📍 출발 위치 저장 (map): x={pose[0]:.2f} y={pose[1]:.2f} yaw={math.degrees(pose[2]):.0f}°')
        # 출발 전 방향 판단: 갈림길과 같은 절차(정지 → 지도로 목적지 쪽 판단 → 필요하면 회전 → 차선 추종)
        self.departing = True
        self.junction_latched, self.junction_frames = True, 0
        self.return_phase, self.return_t0 = 'pause', time.monotonic()
        self._ensure_map_sub()

    def _waiting_for_start_pose(self, now):
        return self.start_pose is None or self.goal is None    # 위치·목적지 없이는 움직이지 않는다

    # ---------- 갈림길·회전 방향: 지금 구간의 목표(목적지/출발점) 쪽 ----------
    def _target(self):
        return self.goal if self.leg == 'outbound' else self.start_pose[:2]

    def _plan_exit(self, now):
        pose = self._current_pose()
        choice, reason = None, ''
        if self.latest_map is not None and pose is not None:
            m = self.latest_map
            grid = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
            t0 = time.monotonic()
            choice, info = choose_exit(grid, m.info.resolution,
                                       (m.info.origin.position.x, m.info.origin.position.y),
                                       pose, self._target(), self.unknown_cost, self.inflate_m)
            if (self.departing or self.on_lane_decision) and choice is not None and 'rel_deg' in info:
                # 출발은 갈림길이 아니라 차선 위: 갈 수 있는 건 앞/뒤뿐이다 (left/right 면 막다른 쪽으로 가는 경우가 있다)
                choice = 'back' if abs(info['rel_deg']) > 90 else 'straight'
            self.get_logger().info(f'🧭 {"출발" if self.departing else self.leg} 방향 판단: {choice} ({time.monotonic() - t0:.2f}s) {info}')
            if choice is None:
                reason = '지도에서 목표까지 경로를 찾지 못함'
        elif now - self.return_t0 > self.map_wait:
            reason = f'{self.map_wait:.0f}초 안에 지도({self.map_topic})를 받지 못함'
        else:
            return
        if choice is None and self.departing:
            choice = 'straight'         # 출발 때 판단을 못 하면 보고 있는 방향 그대로 출발
            self.get_logger().warn(f'⚠️ {reason}: 지금 방향 그대로 출발합니다.')
        elif choice is None:
            choice = 'back'
            self.get_logger().warn(f'⚠️ {reason}: 왔던 길로 되돌아갑니다(U턴).')
        # 바로 출발하지 않고, 고른 출구 쪽이 비어 있는지 확인한 뒤 출발 (_junction_wait)
        self.jn_choice, self.jn_clear_t, self.jn_busy, self.jn_note_t = choice, None, '', 0.0
        self.return_phase, self.return_t0 = 'jn_wait', now

    def _start_following(self, now, bias=None):
        self.departing = self.on_lane_decision = False
        if bias is None and self.post_turn_bias is not None:
            bias = self.post_turn_bias               # 제자리 90° 회전 뒤: 고른 쪽 차선만 따라 갈림길을 빠져나간다
        self.post_turn_bias = None
        self.in_corner, self.corner_calm_t = False, None
        prev = self.swap_lanes
        super()._start_following(now, bias)
        # U턴하면 좌우가 바뀐다고 '추정'만 해 둔다. 두 차선이 함께 보이면 swap_lanes(아래)가 화면 위치로 바로잡는다
        self.swap_lanes = (not prev) if self.exit_choice == 'back' else prev

    # ---------- 좌우 차선: 화면 위치로 판단 ----------
    # 원본은 U턴 횟수로 left_lane/right_lane 클래스를 뒤바꾼다(swap_lanes). 미션을 여러 번 하거나 로봇을 손으로 옮기면
    # 이 추정이 어긋나 반대 차선을 따라가다 차선을 벗어난다(2026-10-07 실물: U턴 직후 117° 꺾여 이탈).
    # 원본 _detect 는 클래스로 고른 이번 프레임 마스크를 prev_lane 에 넣은 직후 swap_lanes 를 읽으므로,
    # 그 시점에 두 마스크의 가로 위치를 비교해 '화면 왼쪽에 있는 것이 왼쪽 차선'이 되도록 정한다.
    # 한쪽만 보이면 마지막 판단(또는 U턴 추정)을 유지한다.
    @property
    def swap_lanes(self):
        pl = self.__dict__.get('prev_lane') or {}
        lm, rm = pl.get('left'), pl.get('right')
        if lm is not None and rm is not None and lm.any() and rm.any():
            lx, rx = np.nonzero(lm)[1].mean(), np.nonzero(rm)[1].mean()
            if abs(lx - rx) > 3:                    # 1/4 해상도 픽셀: 거의 겹치면 판단하지 않는다
                sw = bool(lx > rx)
                if sw != self.__dict__.get('_swap', False):
                    self.get_logger().info(f'↔️ 좌우 차선 판단: 화면 위치 기준 교환={sw} (left_lane x={lx * 4:.0f}, right_lane x={rx * 4:.0f})')
                self.__dict__['_swap'] = sw
        return self.__dict__.get('_swap', False)

    @swap_lanes.setter
    def swap_lanes(self, v):
        self.__dict__['_swap'] = bool(v)

    def _check_arrival(self, now):
        return                          # 도착 판정은 _drive_loop 에서 구간별로 한다

    # ---------- 주행 루프 ----------
    def _drive_loop(self):
        now = time.monotonic()
        if self.traffic is not None:
            return                      # 양보 동작 중: 차선 추종·앞 사물 정지·LOST 처리를 돌리지 않는다 (_traffic_step 이 움직임)
        if self.return_phase == 'jn_wait' and self.jn_choice is not None:
            self._junction_wait(now)    # 대기 중에는 아래 원본 루프가 정지 명령을 낸다 (모르는 단계 = 정지)
        if self.return_phase == 'jn_adv':
            self._junction_advance(now)
            return                      # 원본 루프가 모르는 단계라 정지 명령을 내므로 건너뛴다
        if not self._waiting_for_start_pose(now) and self.return_phase not in ('arrived', 'failed'):
            pose = self._current_pose()
            in_junction = self.junction_latched and self.return_phase not in (None, 'following')
            if pose is not None and not in_junction:
                if self.leg == 'outbound' and self._reached('goal', pose, self.goal, self.goal_dist):
                    if self.one_way:
                        self.junction_latched, self.return_phase, self.result = True, 'arrived', 'ARRIVED'
                        self.get_logger().info('🏁 목적지 도착 (편도): 정지')
                    else:
                        self._reach_goal(now)
                elif (self.leg == 'inbound' and now - self.leg_t0 > 5.0
                      and self._reached('start', pose, self.start_pose, self.arrive_dist)):
                    self.junction_latched, self.return_phase = True, 'arrived'
                    self.result = 'ARRIVED'
                    self.get_logger().info('🏠 출발점 도착: Lane Following 미션 완료')
            # 갈림길을 빠져나와 차선 추종으로 돌아왔으면 다음 갈림길을 다시 볼 수 있게 한다
            if (self.return_phase == 'following' and self.lane_bias is None and now - self.return_t0 > 3.0):
                self.junction_latched, self.junction_frames, self.return_phase = False, 0, None
        super()._drive_loop()

    on_lane_decision = False      # 목적지 도착 직후의 방향 판단: 갈림길이 아니라 차선 위라 앞/뒤만 고른다

    def _reach_goal(self, now):
        self.on_lane_decision = True
        self.leg, self.leg_t0 = 'inbound', now
        self.junction_latched, self.junction_frames = True, 0
        self.lane_bias = None
        self.return_phase, self.return_t0 = 'pause', now       # 바로 정지 → 지도로 출발점 쪽 방향 판단
        self._ensure_map_sub()
        self.get_logger().info(f'🎯 목적지 도착: 출발점으로 복귀합니다')

    # ---------- 차선 위 마주침 양보 (lane_traffic_cmd) ----------
    def _traffic_clear(self):
        self.jn_clear_t = None                 # 갈림길 대기 중 hold/yield 가 끝나면 출구 확인(1 s)을 처음부터 다시 한다
        self.traffic, self.tr_target, self.tr_yaw, self.tr_rejoin, self.tr_blocked_t = None, None, None, None, None
        self.tr_driving = self.tr_at = False

    def _on_traffic_cmd(self, m):
        try:
            d = json.loads(m.data)
            tid, cmd = int(d['id']), d['cmd']
            xy = (float(d['x']), float(d['y'])) if cmd == 'yield' else None
            rj = (float(d['x']), float(d['y']), float(d['yaw'])) if cmd == 'resume' and 'x' in d else None
            if rj is not None and not all(map(math.isfinite, rj)):
                rj = None
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn(f'잘못된 lane_traffic_cmd: {m.data}')
            return
        now = time.monotonic()
        self.tr_rx = now                               # keepalive (같은 id 재전송 포함)
        if tid == self.traffic_id:
            return
        self.traffic_id = tid
        idle = self.goal is None or self.result is not None
        if idle and cmd == 'hold':
            return                                     # 미션 중이 아니면 이미 서 있다 (id 는 처리한 것으로 기록)
        if idle and self.traffic is None and cmd == 'resume':
            return
        # 미션이 없어도 yield/resume 은 따른다: 차선 위에 서 있는 로봇이 다른 로봇의 길을 막을 때 비켜섰다가 제자리로 돌아온다
        # (2026-10-08: 왕복을 마치고 출발점에 서 있던 amr_01 때문에 amr_02 의 복귀가 막힘)
        self.jn_clear_t = None                 # 관제 명령을 받으면 갈림길 출구 확인 타이머 초기화
        if cmd == 'hold':
            if self.traffic in (None, 'HOLD'):
                self.traffic = 'HOLD'
                self._traffic_stop()
                self.get_logger().info('✋ 마주침: 정지 (관제 판단 대기)')
        elif cmd == 'yield' and xy is not None and all(map(math.isfinite, xy)):
            pose = self._current_pose()
            if pose is None:
                return
            if self.traffic not in ('YIELDING', 'YIELDED', 'REJOINING', 'YIELD_FAILED'):
                self.tr_rejoin = pose                  # 처음 비켜설 때만: 돌아올 차선 위 자리·방향
            self.traffic, self.tr_target, self.tr_yaw = 'YIELDING', xy, None
            self.tr_t0, self.tr_blocked_t, self.tr_driving, self.tr_at = now, None, False, False
            self.get_logger().info(f'↪️ 양보: 차선 밖 ({xy[0]:.2f}, {xy[1]:.2f}) 로 비켜섭니다 (복귀 자리 {pose[0]:.2f}, {pose[1]:.2f})')
        elif cmd == 'resume':
            self.traffic_role, self.tr_role_t = d.get('role'), now
            self._traffic_resume(now, rj)

    def _traffic_resume(self, now, rejoin=None):
        """rejoin=(x, y, yaw): 관제가 정한 복귀 자리 (원래 자리를 상대가 막고 있을 때). 없으면 비켜서기 전 자리"""
        if self.traffic == 'HOLD':
            self._traffic_clear()
            self.get_logger().info(f'▶️ 주행 재개 ({self.traffic_role or "timeout"})')
            return
        moving_back = self.traffic == 'REJOINING' and rejoin is not None        # 복귀 중 목표 변경
        if self.traffic in ('YIELDING', 'YIELDED', 'YIELD_FAILED') or moving_back:
            rj = rejoin or self.tr_rejoin
            if rj is None:
                return
            self.traffic, self.tr_target, self.tr_yaw = 'REJOINING', rj[:2], rj[2]
            self.tr_t0, self.tr_blocked_t, self.tr_driving, self.tr_at = now, None, False, False
            self.get_logger().info(f'↩️ 차선 복귀: ({rj[0]:.2f}, {rj[1]:.2f}) 로 돌아갑니다'
                                   + (' (관제 지정 자리)' if rejoin is not None else ''))

    def _traffic_stop(self):
        self.cmd_pub.publish(Twist())
        self._apply_lamp(LAMP_STOP)

    def _traffic_fail(self, why):
        self.traffic = 'YIELD_FAILED'
        self._traffic_stop()
        self.get_logger().warn(f'⚠️ 양보 동작 실패: {why} — 그 자리에서 정지 (resume 이 오면 복귀 시도)')

    def _traffic_step(self):
        tr = self.traffic
        if tr is None:
            return
        now = time.monotonic()
        if tr in ('HOLD', 'YIELDED') and now - self.tr_rx > 10.0:
            self.get_logger().warn('⏱ 관제 명령이 10초 동안 없음: 주행 재개로 간주합니다')
            self.traffic_role = None
            self._traffic_resume(now)
            return
        if tr in ('HOLD', 'YIELDED', 'YIELD_FAILED'):
            self._traffic_stop()
            return
        # YIELDING / REJOINING: 지도 위치로 직접 이동
        if now - self.tr_t0 > 20.0:
            return self._traffic_fail('20초 안에 도착하지 못함')
        pose = self._current_pose()
        if pose is None:
            self._traffic_stop()
            return
        v, w, done = self._goto_step(pose)
        if v > 0.0 and (now - self.lidar_t > 0.5 or self.raw_front < 0.06):
            v = 0.0                                   # 앞이 막힘 (또는 라이다 끊김): 직진만 멈추고 기다린다
            self.tr_blocked_t = self.tr_blocked_t or now
            if now - self.tr_blocked_t > 3.0:
                return self._traffic_fail(f'앞이 3초 넘게 막힘 ({self.raw_front:.2f} m)')
            w = 0.0
        else:
            self.tr_blocked_t = None
        if done:
            self.cmd_pub.publish(Twist())
            if tr == 'YIELDING':
                self.traffic, self.tr_rx = 'YIELDED', now
                self._apply_lamp(LAMP_STOP)
                self.get_logger().info('⏸ 비켜섬 완료: 상대가 지나가길 기다립니다')
            else:
                # 갈림길 회전 뒤처럼 차선 인식 기억을 비우고 미션(갈림길·구간 상태 그대로)을 이어 간다
                self.prev_lane = {'left': None, 'right': None}
                self.tracker.reset()
                self.ramp_t0, self.ramp_n = None, 0
                self._traffic_clear()
                self.get_logger().info('✅ 차선 복귀 완료: 차선 추종을 재개합니다')
            return
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)
        self._apply_lamp(LAMP_DRIVE)

    def _goto_step(self, pose):
        """(v, w, 끝났나): 목표점으로 제자리 회전 → 직진(방향 보정), 0.03 m 안이면 마지막 방향(tr_yaw, ±8°) 맞춤"""
        wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
        dx, dy = self.tr_target[0] - pose[0], self.tr_target[1] - pose[1]
        dist = math.hypot(dx, dy)
        tol = 0.02 if self.traffic == 'YIELDING' else 0.03     # 비켜선 자리는 상대 통로 경계라 더 정확히
        err = wrap(math.atan2(dy, dx) - pose[2])
        passed = self.tr_driving and dist < 0.06 and abs(err) > math.pi / 2     # 직진하다 목표를 지나침: 되돌아가지 않고 도착 처리
        if not self.tr_at and dist > tol and not passed:
            if abs(err) > (0.5 if self.tr_driving else 0.25):
                self.tr_driving = False
                return 0.0, math.copysign(min(0.8, max(0.3, 1.5 * abs(err))), err), False
            self.tr_driving = True
            return min(0.06, 0.02 + 0.8 * dist), max(-0.6, min(0.6, 1.5 * err)), False
        self.tr_driving, self.tr_at = False, True       # 위치 도착 (방향 맞추는 동안 다시 이동하지 않음)
        if self.tr_yaw is not None:
            err = wrap(self.tr_yaw - pose[2])
            if abs(err) > math.radians(8):
                return 0.0, math.copysign(min(0.8, max(0.3, 1.5 * abs(err))), err), False
        return 0.0, 0.0, True

    # ---------- LED 규칙 (사용자 지정 10/9): 주행 초록 깜빡 / 감속 주황 깜빡 / 정지·일시정지 빨강 고정 / 충돌 빨강 빠른 깜빡 / 미션 종료 끔 ----------
    LAMP_COLLISION = (1.0, 0.0, 0.0, 2, 150)

    def _apply_lamp(self, lamp):
        g = self.__dict__.get                        # 원본 __init__ 이 이 노드의 값들이 생기기 전에 LED 를 한 번 켠다
        if 'goal' not in self.__dict__:
            return super()._apply_lamp(LAMP_OFF)     # 시작 직후: 목적지를 받기 전이라 끔
        if g('traffic') is None and (g('goal') is None or g('result') is not None or g('return_phase') == 'arrived'):
            lamp = LAMP_OFF                          # 미션 종료(도착·취소·E-STOP·대기): 끔. 원본은 도착 뒤 빨강을 유지한다
        elif self.state == STATE_COLLISION:
            lamp = self.LAMP_COLLISION               # 충돌 거리: 일반 정지(빨강 고정)와 구분
        super()._apply_lamp(lamp)

    # ---------- 상태 발행 (LCD·관제) ----------
    def _publish_status(self):
        pose = self._current_pose() if self.tf_buffer is not None else None
        target = None if self.start_pose is None or self.goal is None else self._target()
        dist = None if pose is None or target is None else math.hypot(pose[0] - target[0], pose[1] - target[1])
        if self.traffic is not None:
            state, detail = {'HOLD': ('WAITING', 'encounter'), 'YIELDING': ('WAITING', 'yielding'),
                             'YIELDED': ('WAITING', 'yielded'), 'REJOINING': ('RESUME DRIVING', 'back to lane'),
                             'YIELD_FAILED': ('WAITING', 'yield failed')}[self.traffic]
        elif self.goal is None:
            state = 'IDLE'
            detail = ('set Init Pose' if pose is None else 'canceled · set goal' if self.result == 'CANCELED'
                      else 'e-stop · set goal' if self.result == 'E_STOP' else 'set goal')
        elif self.start_pose is None:
            state, detail = 'WAITING', 'set Init Pose'
        elif self.return_phase == 'failed' or self.state == STATE_RETURN_FAILED:
            state, detail = 'FAILED', 'stopped'
        elif self.return_phase == 'arrived':
            state, detail = 'ARRIVED', 'at target' if self.one_way else 'back at start'
        elif self.state in (STATE_COLLISION, STATE_OBSTACLE_STOP):
            state, detail = 'WAITING', 'obstacle ahead'
        elif self.state == STATE_CW_BLOCKED:
            state, detail = 'WAITING', 'crosswalk'
        elif self.return_phase == 'jn_wait':
            w = 'path' if (self.departing or self.on_lane_decision) else 'junction'   # lane_traffic 은 'junction' 으로 시작하는 것만 갈림길로 본다
            state, detail = 'WAITING', f'{w} busy · {self.jn_busy}' if self.jn_busy else f'{w} check ({self.jn_choice})'
        elif self.traffic_role == 'priority' and time.monotonic() - self.tr_role_t < 5.0:
            state, detail = 'OWN DRIVING PRIORITY', 'lane'
        elif self.tracker.lost_frames >= 20 and not (self.junction_latched and self.return_phase not in (None, 'following')):
            # 차선을 놓침: lane_traffic 이 detail 의 'lost' 로 경보를 낸다
            if time.monotonic() - self.__dict__.get('rq_t', 0.0) < 1.0:
                state, detail = 'DRIVING', 'lane lost · reacquiring'
            else:
                state, detail = 'WAITING', 'lane lost'
        elif self.departing:
            state, detail = 'DRIVING', 'heading to goal'
        elif self.leg == 'outbound':
            state, detail = 'DRIVING', 'junction' if self.junction_latched and self.return_phase != 'following' else 'lane → goal'
        else:
            state, detail = 'RETURNING', 'turning' if self.return_phase in ('pause', 'planning', 'turning') else 'lane → start'
        msg = {'robot': self.robot_name, 'mission': 'lane', 'state': state, 'detail': detail,
               'dist': None if dist is None else round(dist, 1), 'leg': self.leg,
               'goal': list(self.goal) if self.goal else None,
               'start': [round(v, 2) for v in self.start_pose[:2]] if self.start_pose else None,
               'localized': pose is not None, 'cmd_id': self.cmd_id,
               'pose': None if pose is None else [round(pose[0], 3), round(pose[1], 3), round(pose[2], 3)],
               'traffic': self.traffic, 'traffic_id': self.traffic_id, 'role': self.traffic_role,
               # 갈림길에서 고른 출구 (판단~출구 확인~회전~빠져나갈 때까지 유지, 일반 주행으로 돌아가면 null) — lane_traffic 출구 예측용
               'exit': None if (self.departing or self.on_lane_decision) else (   # 출발·도착 직후 판단은 갈림길이 아니므로 내보내지 않는다
                   self.jn_choice or (self.exit_choice if self.junction_latched or self.return_phase in ('jn_adv', 'turning') or self.lane_bias else None))}
        self.status_pub.publish(String(data=json.dumps(msg)))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    import signal
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = None
    try:
        node = FmsLaneMission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 카메라 close 등에서 멈추면 프로세스가 남아 옛 상태를 계속 발행한다 (2026-10-08 실물: 스택을 꺼도 차선 노드만 종료 기록 없음).
        # 정리를 시작하고 3초가 지나면 강제로 끝낸다.
        import os, threading
        threading.Timer(3.0, lambda: os._exit(0)).start()
        if node is not None:
            node.shutdown_hardware()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
