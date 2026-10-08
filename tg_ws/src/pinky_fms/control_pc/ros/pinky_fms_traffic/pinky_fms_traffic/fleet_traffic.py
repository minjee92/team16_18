"""충돌 방지 조정 층: fleet_coordinator 를 상속해 '언제, 어디서 기다리고 비켜서는가'를 더한다.

기존 fleet_coordinator / fleet_mission / GUI 는 수정하지 않는다. fleet_coordinator 대신 이 노드를 띄우면 된다
(map_yaml 파라미터가 비어 있으면 fleet_coordinator 와 똑같이 동작한다).

동작:
  - 로봇마다 Nav2 의 계획 경로(compute_path_to_pose, 없으면 내부 A*)를 받아 TrafficManager 에 넘긴다.
  - 로봇에는 항상 NavigateToPose 목표 하나만 보낸다: 최종 목표(GO) / 충돌 구간 앞 대기 지점(HOLD) / 비켜설 자리(YIELD).
    대기·비켜서기 중에도 GUI 에는 원래 미션 ID 의 RUNNING(메시지에 사유)으로 보이고, 최종 목표 도착 때만 SUCCEEDED 가 나간다.
  - 서 있는 로봇(미션 없음)이 남의 길을 막으면 미션 없이 비켜선다.
"""
import json
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import BackUp, ComputePathToPose, DriveOnHeading, NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes

from pinky_fms_core.fleet_coordinator import FleetCoordinator
from pinky_fms_interfaces.msg import Task

from .gridmap import GridMap
from .planner import astar, resample
from .encounter import EncounterManager
from .traffic import Agent, TrafficManager

DS = 0.02
LCD_ENCOUNTER = 1.0       # 교착 확인: 충돌 예정인 두 로봇이 이 거리 안(사이에 벽 없음)에서 한쪽이라도 멈춰 서면 LCD 에 PRIORITY / WAITING 을 띄운다
LCD_RELEASE = 1.5         # 교착 해제: 우선권이 풀리거나 두 로봇이 이 거리 이상 멀어지면 (진 쪽은 RESUME DRIVING)
LCD_DONE = {'SUCCEEDED': 'ARRIVED', 'FAILED': 'FAILED', 'CANCELED': 'STOPPED'}
GOAL_MIN_CLEAR = 0.06     # 목표가 벽에서 이보다 가까우면(로봇 반폭) 갈 수 없으므로 배정하지 않는다
ESCAPE_MAX = 3            # 미션 하나에서 벽 빠져나오기를 시도하는 최대 횟수
UNSTICK_CLEAR = 0.08      # 중심~벽이 이보다 가까운데 Nav2 가 경로를 못 만들면 앞/뒤로 조금 빠져나온다 (Nav2 전역 costmap 0.025 m·padding 0 기준)


class TState:
    """로봇 한 대의 충돌 방지 상태"""

    def __init__(self):
        self.mode = 'IDLE'          # IDLE | GO | HOLD | YIELD | WAIT | YIELD_P (미션 없이 비켜서는 중) | BACKUP (교착: 뒤로 빠져 길 비키기)
        self.seq = 0                # 보낸 Nav2 목표 번호 (이전 목표의 결과는 무시)
        self.plan = None            # 최종 목표까지의 계획 경로 (N,2)
        self.plan_t = 0.0
        self.plan_pending = False
        self.sub_xy = None          # 지금 Nav2 에 보낸 목표 좌표
        self.final = None           # 미션의 최종 목표 PoseStamped
        self.note = ''
        self.cp = None              # compute_path_to_pose 클라이언트
        self.sent_t = 0.0           # 마지막으로 목표를 보낸 시각
        self.block_until = 0.0      # 비켜서기에 실패하면 이 시각까지 다시 시도하지 않는다
        self.backup_target = None   # BACKUP: 후진 뒤 이동할 자리 (x, y, yaw)
        self.assigned_t = 0.0       # 미션 배정 시각
        self.nav_fail = 0           # 이 미션에서 Nav2 가 실패(abort)한 횟수. 바로 FAILED 로 끝내지 않고 대기·회피 후 재시도한다
        self.doh = None             # drive_on_heading(앞으로 조금) 액션 클라이언트
        self.bu = None              # backup(후진) 액션 클라이언트
        self.aux = None             # 진행 중인 BackUp / DriveOnHeading 목표 (E-STOP·취소 때 같이 멈춘다)
        self.others_pub = None      # /<ns>/other_robots 발행자
        self.nav_ready = True       # bt_navigator 가 active 인가 (아니면 목표를 보내지 않는다)
        self.gs = None              # bt_navigator/get_state 클라이언트
        self.mn = None              # lifecycle_manager_navigation/manage_nodes 클라이언트
        self.lc_pending = False
        self.lc_sent_t = 0.0        # 마지막 Nav2 상태 질의 시각
        self.lc_last_fix = 0.0
        self.lc_bad_since = None    # bt_navigator 가 active 가 아니게 된 시각 (켜지는 중일 수 있어 30초 지켜본 뒤 복구)
        self.route_id = 0           # 마주침: 지금 따라가는 고정 경로 번호 (0 = Nav2 가 자유롭게 계획)
        self.tp = None              # navigate_through_poses 클라이언트 (고정 경로를 경유점으로 보낸다)
        self.note_last = (None, 0.0)
        self.lane = None            # Lane Following 미션 노드의 마지막 상태 (dict) 와 받은 시각
        self.esc_pub = None         # /<ns>/escape_cmd: 벽에서 빠져나오기 요청 (로봇의 fms_escape)
        self.esc_id = 0
        self.esc_tries = 0          # 이번 미션에서 빠져나오기를 요청한 횟수 (최대 ESCAPE_MAX)
        self.esc_block = 0.0        # 실패 뒤 이 시각까지 다시 요청하지 않는다
        self.nav_label = None       # 준비 상태 표시용: bt_navigator / amcl 의 lifecycle 상태 (None = 노드 없음)
        self.amcl_label = None
        self.amcl_client = None
        self.amcl_active_t = 0.0    # amcl 이 active 가 된 시각
        self.amcl_t = 0.0           # amcl_pose 를 마지막으로 받은 시각 (active 이후에 받았으면 localized)
        self.ready_sent_t = 0.0     # lifecycle 상태를 마지막으로 물어본 시각
        self.ready_pending = 0
        self.lane_t = 0.0
        self.unstick_flip = False   # 직전 빠져나오기가 실패했으면 다음엔 반대 방향으로 시도
        self.lcd_pub = None         # /<ns>/fms_status: 로봇 LCD 에 띄울 주행 상태 (JSON 문자열)
        self.lcd = None             # 마지막으로 보낸 (state, detail)
        self.lcd_t = 0.0
        self.motion_t = 0.0         # 바퀴 odom 으로 마지막으로 움직인 시각 (LCD: 서서 기다리는 중인지 판단)
        self.lcd_rest = 'IDLE'      # 미션이 없을 때 띄울 상태 (직전 미션 결과: ARRIVED / FAILED / STOPPED)
        self.waited = False         # 이번 미션에서 상대에게 양보(WAITING)한 적이 있으면 이후 주행은 RESUME DRIVING


def yaw_pose(x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class FleetTraffic(FleetCoordinator):
    def __init__(self):
        super().__init__()
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('traffic_mode', 'encounter')   # encounter: 실제로 마주쳤을 때만 처리(2026-10-07) / predict: 경로·시간 예측으로 미리 피함(이전 방식)
        self.declare_parameter('tick_sec', 0.1)            # 판단 주기 (predict 방식은 이 중 0.5 s 마다만 판단한다)
        self.declare_parameter('meet_dist', 0.6)          # encounter: 마주침 판단 거리 (중심 간, 사이에 벽 없음)
        self.declare_parameter('meet_ahead', 0.7)         # encounter: 침범을 볼 남은 경로 앞쪽 길이
        self.declare_parameter('tube', 0.17)              # encounter: 경로 점유 띠 반폭
        self.declare_parameter('meet_pause', 1.5)         # encounter: 마주치면 둘 다 멈춰 있는 시간
        self.declare_parameter('escape_dist', 0.20)       # Nav2 가 벽 때문에 실패하면 로봇이 벽 반대로 이만큼 빠져나온 뒤 재개
        self.declare_parameter('conf_radius', 0.34)       # 두 로봇 중심이 이보다 가까워질 것 같으면 충돌 예정 (위치추정 오차 여유 포함)
        self.declare_parameter('conf_radius_parked', 0.30)  # 서 있는 로봇과의 충돌 판정 거리 (움직이는 로봇끼리보다 작게)
        self.declare_parameter('yield_sep', 0.45)         # 비켜설 자리와 상대 경로 사이 최소 거리 (conf_radius 이상. 실물 Nav2 는 코너를 깎으므로 여유를 둔다)
        self.declare_parameter('arrive_tol', 0.25)        # Nav2 가 SUCCEEDED 라 해도 실제 위치가 목표에서 이보다 멀면 FAILED 로 보고
        self.declare_parameter('mission_timeout', 180.0)   # 재시도를 포함해 이 시간(s) 안에 못 끝내면 FAILED
        self.declare_parameter('min_resend_sec', 3.0)     # 대기·비켜서기 목표를 바꾸는 최소 간격 (자주 바꾸면 Nav2 가 매번 방향을 다시 잡는다)
        mp = self.get_parameter('map_yaml').value
        self.tm = None
        self.em = None
        self.mode = 'off'
        self.traffic_off = True
        self.ts = {}
        self.lcd_standoff = {}     # 교착이 확인된 로봇 쌍 frozenset -> 우선권 로봇 (LCD 표시용, 주행 판단에는 쓰지 않는다)
        # 교통 관제 방식은 GUI 에서 미션 설정으로 바꾼다 (진행 중인 미션이 없을 때만). 현재 방식은 /fleet/traffic_state 로 알린다
        self.mode_pub = self.create_publisher(String, '/fleet/traffic_state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/fleet/traffic_mode', self.on_traffic_mode, 10)
        self.estop_ack_pub = self.create_publisher(String, '/fleet/global_cmd_ack', 10)   # E-STOP 처리 결과 (GUI 가 확인)
        self.create_timer(5.0, lambda: self._publish_mode(''))     # GUI 가 늦게 접속해도 현재 방식을 알 수 있게
        if mp:
            self.gm = GridMap(mp)
            g = lambda n: float(self.get_parameter(n).value)
            self._em = EncounterManager(self.gm, d_meet=g('meet_dist'), l_ahead=g('meet_ahead'), r_tube=g('tube'), pause=g('meet_pause'),
                                        escape_sep=g('yield_sep'), r_conf=g('conf_radius'))
            self._pm = TrafficManager(self.gm, r_conf=g('conf_radius'), r_conf_parked=g('conf_radius_parked'), yield_sep=g('yield_sep'))
            self._predict_t = 0.0
            self._apply_mode(str(self.get_parameter('traffic_mode').value))
            self.arrive_tol = float(self.get_parameter('arrive_tol').value)
            self.mission_timeout = float(self.get_parameter('mission_timeout').value)
            self.min_resend = float(self.get_parameter('min_resend_sec').value)
            tick = float(self.get_parameter('tick_sec').value) or 0.1
            self.create_timer(tick, self.traffic_tick)
            self.get_logger().info(f'충돌 방지 준비: 방식={self.mode}, 판단 주기 {tick:.1f}s, map={mp}')
        else:
            self.get_logger().warn('map_yaml 이 없어 충돌 방지 꺼짐 (fleet_coordinator 와 동일하게 동작)')
            self._publish_mode('map_yaml 없음: 교통 관제를 쓸 수 없습니다')
        self.create_timer(0.2, self.publish_others)        # 각 로봇에 다른 로봇들의 실시간 위치를 보낸다 (로봇의 robot_scan_filter 가 사용)
        self.create_timer(5.0, self.check_nav2_lifecycle)
        self.ready_pub = self.create_publisher(String, '/fleet/robot_ready', 10)
        self.create_timer(1.0, self.publish_ready)           # 로봇별 기술 스택 준비 상태 (GUI 카드의 Ready 표시)  # Nav2 가 활성화되지 못한 로봇(초기 위치가 60초 안에 안 와서 bringup 중단)을 살려 낸다

    # ---------- 교통 관제 방식 (encounter / predict / off) ----------
    MODES = ('encounter', 'predict', 'off')

    def _apply_mode(self, mode):
        mode = mode if mode in self.MODES else 'encounter'
        self.mode, self.traffic_off = mode, mode == 'off'
        self.em = self._em if mode == 'encounter' else None
        self.tm = self._em.tm if mode == 'encounter' else self._pm      # off 에서도 경로 거리 계산 등 보조 기능은 쓴다
        self._em.reset()
        self._pm.reset()
        self.lcd_standoff.clear()
        for st in self.ts.values():
            st.waited = False
        self._publish_mode('')

    def _publish_mode(self, msg):
        avail = list(self.MODES) if self.tm is not None else ['off']
        self.mode_pub.publish(String(data=json.dumps({'mode': self.mode, 'available': avail, 'msg': msg})))

    def _busy_robots(self):
        return [r.id for r in self.robots.values()
                if r.task is not None or (r.id in self.ts and self.ts[r.id].mode in ('YIELD_P', 'BACKUP'))]

    def on_traffic_mode(self, msg):
        want = msg.data.strip().lower()
        if self.tm is None:
            return self._publish_mode('map_yaml 없음: 교통 관제를 쓸 수 없습니다')
        if want not in self.MODES:
            return self._publish_mode(f'알 수 없는 방식: {want}')
        if want == self.mode:
            return self._publish_mode('')
        busy = self._busy_robots()
        if busy:      # 진행 중인 미션의 판단 상태(우선권·비켜설 자리)가 섞이지 않게, 모두 끝난 뒤에만 바꾼다
            self.get_logger().warn(f'교통 관제 방식 변경 거부 ({self.mode} → {want}): 진행 중 {busy}')
            return self._publish_mode(f'진행 중인 미션이 있어 바꿀 수 없습니다: {", ".join(busy)}')
        self.get_logger().info(f'교통 관제 방식 변경: {self.mode} → {want}')
        self._apply_mode(want)

    # ---------- 준비 상태 (bringup · Nav2 · AMCL · 위치추정 · 빠져나오기 · LCD · 차선) ----------
    def publish_ready(self):
        now = time.monotonic()
        out = []
        for r in self.robots.values():
            st = self.st(r)
            online = self.robot_state(r) != 'OFFLINE'
            if st.ready_pending and now - st.ready_sent_t > 3.0:
                st.ready_pending = 0                        # 응답이 유실된 요청: 다시 묻는다 (안 그러면 상태가 영영 갱신되지 않음)
            if online and st.ready_pending == 0:          # lifecycle 상태를 비동기로 물어본다 (응답은 다음 주기에 반영)
                st.ready_sent_t = now
                for attr, cli in (('nav_label', st.gs), ('amcl_label', st.amcl_client)):
                    if cli.service_is_ready():
                        st.ready_pending += 1
                        cli.call_async(GetState.Request()).add_done_callback(lambda f, st=st, attr=attr: self._on_ready_state(st, attr, f))
                    else:
                        setattr(st, attr, None)
                        if attr == 'amcl_label':
                            st.amcl_active_t = 0.0
            lane = st.lane if st.lane is not None and now - st.lane_t < 3.0 else None
            out.append({
                'robot': r.id,
                'bringup': online,
                'nav': st.nav_label if online else None,
                'amcl': st.amcl_label if online else None,
                # AMCL lifecycle 상태를 못 읽는 경우(서비스 탐색 실패 등)는 위치 수신 여부(r.localized)로 판단
                'localized': bool(online and (st.amcl_t > st.amcl_active_t > 0 if st.amcl_label == 'active'
                                              else st.amcl_label is None and r.localized)),
                'escape': st.esc_pub.get_subscription_count() > 0,
                'lcd': st.lcd_pub.get_subscription_count() > 0,
                'lane': lane.get('state') if lane else None,
                'lane_detail': lane.get('detail') if lane else None,
            })
        self.ready_pub.publish(String(data=json.dumps(out)))

    def _on_ready_state(self, st, attr, fut):
        st.ready_pending = max(0, st.ready_pending - 1)
        try:
            label = fut.result().current_state.label
        except Exception:
            label = None
        if label is None:
            return          # 응답 실패(로봇의 서비스 응답이 가끔 유실됨): 이전 상태 유지. 덮어쓰면 아래에서 '새로 켜짐'으로 오인해 localized 가 풀린다
        if attr == 'amcl_label' and label == 'active' and st.amcl_label != 'active' and (st.amcl_label is not None or st.amcl_active_t == 0.0):
            st.amcl_active_t = time.monotonic()            # 새로 켜진 AMCL(꺼져 있다가 / 처음 봄): 이 뒤에 위치를 받아야 localized
        setattr(st, attr, label)

    # ---------- 보조 ----------
    def st(self, r):
        if r.id not in self.ts:
            t = TState()
            t.cp = ActionClient(self, ComputePathToPose, f'/{r.ns}/compute_path_to_pose')
            t.bu = ActionClient(self, BackUp, f'/{r.ns}/backup')
            t.doh = ActionClient(self, DriveOnHeading, f'/{r.ns}/drive_on_heading')
            t.tp = ActionClient(self, NavigateThroughPoses, f'/{r.ns}/navigate_through_poses')
            t.others_pub = self.create_publisher(PoseArray, f'/{r.ns}/other_robots', 5)
            t.gs = self.create_client(GetState, f'/{r.ns}/bt_navigator/get_state')
            t.mn = self.create_client(ManageLifecycleNodes, f'/{r.ns}/lifecycle_manager_navigation/manage_nodes')
            self.create_subscription(Odometry, f'/{r.ns}/odom', lambda m, t=t: self._on_odom_motion(t, m), 10)
            self.create_subscription(String, f'/{r.ns}/lane_status', lambda m, t=t: self._on_lane_status(t, m), 10)
            t.esc_pub = self.create_publisher(String, f'/{r.ns}/escape_cmd', 10)
            t.amcl_client = self.create_client(GetState, f'/{r.ns}/amcl/get_state')
            self.create_subscription(PoseWithCovarianceStamped, f'/{r.ns}/amcl_pose',
                                     lambda m, t=t: setattr(t, 'amcl_t', time.monotonic()), 10)
            self.create_subscription(String, f'/{r.ns}/escape_status', lambda m, r=r: self._on_escape_status(r, m), 10)
            t.lcd_pub = self.create_publisher(String, f'/{r.ns}/fms_status',
                                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))   # LCD 노드가 늦게 떠도 마지막 상태를 받는다
            self.ts[r.id] = t
        return self.ts[r.id]

    # ---------- Nav2 생존 확인 ----------
    # Nav2 lifecycle manager 는 시작 후 약 60초 안에 로봇 위치(TF map->base)가 없으면 planner 활성화에 실패하고 bringup 을 포기한다.
    # (set_initial_pose 를 끈 뒤로는 GUI 에서 초기 위치를 줄 때까지 TF 가 없다.) 초기 위치가 들어온 뒤 bt_navigator 가 active 가 아니면
    # RESET -> STARTUP 으로 다시 올린다. 모든 호출은 비동기(콜백 체인)라 노드가 멈추지 않는다.
    def check_nav2_lifecycle(self):
        for r in self.robots.values():
            if self.robot_state(r) == 'OFFLINE' or not r.localized:
                continue
            st = self.st(r)
            if st.lc_pending and time.monotonic() - st.lc_sent_t > 10.0:
                st.lc_pending = False       # 응답이 유실된 질의 (amr_02 에서 서비스 응답 유실 확인): 막히지 않게 다시 묻는다
            if st.lc_pending or not st.gs.service_is_ready():
                continue
            st.lc_pending, st.lc_sent_t = True, time.monotonic()
            st.gs.call_async(GetState.Request()).add_done_callback(lambda f, r=r: self._on_nav_state(r, f))

    def _on_nav_state(self, r, fut):
        st = self.st(r)
        try:
            label = fut.result().current_state.label
        except Exception:
            st.lc_pending = False
            return
        ready = (label == 'active')
        if ready != st.nav_ready:
            self.get_logger().warn(f'{r.id}: Nav2(bt_navigator) 상태 = {label}' + ('' if ready else ' → 목표를 보내지 않고 다시 올립니다'))
        st.nav_ready = ready
        now = time.monotonic()
        st.lc_bad_since = None if ready else (st.lc_bad_since or now)
        if ready or not st.mn.service_is_ready() or now - st.lc_bad_since < 30.0 or now - st.lc_last_fix < 40.0:
            st.lc_pending = False
            return
        st.lc_last_fix = time.monotonic()
        self.get_logger().warn(f'{r.id}: Nav2 다시 올리기 (RESET → STARTUP)')
        st.mn.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.RESET)).add_done_callback(lambda f, r=r: self._on_nav_reset(r, f))

    def _on_nav_reset(self, r, fut):
        st = self.st(r)
        st.mn.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)).add_done_callback(lambda f, r=r: self._on_nav_startup(r, f))

    def _on_nav_startup(self, r, fut):
        st = self.st(r)
        st.lc_pending = False
        try:
            ok = fut.result().success
        except Exception:
            ok = False
        self.get_logger().info(f'{r.id}: Nav2 STARTUP 응답 {ok} (실제 상태는 5초 뒤 다시 확인)')

    def publish_others(self):
        live = [r for r in self.robots.values() if self.robot_state(r) != 'OFFLINE' and r.localized]
        now = self.get_clock().now().to_msg()
        for r in self.robots.values():
            st = self.st(r)
            m = PoseArray()
            m.header.frame_id, m.header.stamp = 'map', now
            for o in live:
                if o.id != r.id:
                    p = Pose()
                    p.position.x, p.position.y = float(o.pose[0]), float(o.pose[1])
                    p.orientation.z, p.orientation.w = math.sin(o.pose[2] / 2), math.cos(o.pose[2] / 2)
                    m.poses.append(p)
            st.others_pub.publish(m)

    @staticmethod
    def _on_lane_status(st, m):
        try:
            st.lane, st.lane_t = json.loads(m.data), time.monotonic()
        except ValueError:
            pass

    def lane_active(self, r):
        """Lane Following 미션 중인가 (Nav2 가 아니라 차선 추종 노드가 움직이는 로봇)"""
        st = self.ts.get(r.id)
        return (st is not None and st.lane is not None and time.monotonic() - st.lane_t < 3.0
                and st.lane.get('state') not in ('ARRIVED', 'FAILED', 'IDLE'))   # IDLE: 스택만 떠서 목적지 대기

    def robot_state(self, r):
        s = super().robot_state(r)
        if s == 'IDLE' and self.lane_active(r):
            return 'BUSY'               # Lane Following 미션 중: Nav2 미션을 배정하지 않는다
        if s == 'IDLE' and r.id in self.ts and self.ts[r.id].mode in ('YIELD_P', 'BACKUP'):
            return 'BUSY'               # 비켜서는 중에는 새 미션을 받지 않는다
        return s

    # ---------- 로봇 LCD 상태 ----------
    def emit(self, task, robot_id, state, dist=0.0, msg=''):
        super().emit(task, robot_id, state, dist, msg)
        r = self.robots.get(robot_id)
        if r is None:
            return
        st = self.st(r)
        if state == 'ASSIGNED':
            st.waited = False
            self._set_lcd(r, 'DRIVING')
        elif state in LCD_DONE and r.task is None:      # 미션이 실제로 끝났을 때만 (다른 미션이 진행 중인 로봇에 대한 거절은 제외)
            st.waited, st.lcd_rest = False, LCD_DONE[state]
            self._set_lcd(r, st.lcd_rest)

    def _set_lcd(self, r, state, detail='', dist=None, force=False):
        st = self.st(r)
        now = time.monotonic()
        dist = None if dist is None else round(dist, 1)     # 0.1 m 단위: 값이 바뀔 때만 LCD 를 다시 그린다
        if not force and st.lcd == (state, detail, dist) and now - st.lcd_t < 2.0:
            return
        st.lcd, st.lcd_t = (state, detail, dist), now
        st.lcd_pub.publish(String(data=json.dumps({'robot': r.id, 'state': state, 'detail': detail, 'dist': dist})))

    def _update_lcd(self, cmds, byid):
        """각 로봇의 LCD 상태를 정한다. 주행 판단(우선권·대기·비켜서기)은 충돌이 예상되면 미리 시작되지만,
        LCD 는 두 로봇이 실제로 마주쳐 멈춘 것(교착)이 확인된 뒤부터만 OWN DRIVING PRIORITY / WAITING 을 띄운다.
          교착 확인: 우선권이 정해진 쌍 + LCD_ENCOUNTER 안·사이에 벽 없음 + 한쪽이라도 멈춰 있음
          교착 해제: 우선권이 풀리거나 LCD_RELEASE 이상 멀어짐 → 우선권 로봇 DRIVING, 양보한 로봇 RESUME DRIVING"""
        if self.em is not None:
            return self._update_lcd_encounter(byid)
        now = time.monotonic()
        moving = {rid: now - st.motion_t < 1.0 for rid, (r, st, _) in byid.items()}     # 바퀴가 1초 안에 움직였으면 주행 중
        for key in list(self.lcd_standoff):
            a, b = tuple(key)
            w = self.tm.winner.get((a, b), self.tm.winner.get((b, a)))
            if w is None or a not in byid or b not in byid or self._dist(a, b) > LCD_RELEASE:
                del self.lcd_standoff[key]
            else:
                self.lcd_standoff[key] = w            # 진행 중에 역할이 바뀌면 따라간다
        for (a, b), w in self.tm.winner.items():
            key = frozenset((a, b))
            if key not in self.lcd_standoff and a in byid and b in byid and not (moving[a] and moving[b]) \
                    and self._encounter(self.robots[a], self.robots[b]):
                self.lcd_standoff[key] = w
        role = {}                                     # rid -> ('win' | 'lose', 상대)
        for key, w in self.lcd_standoff.items():
            a, b = tuple(key)
            role[a] = ('win' if w == a else 'lose', b)
            role[b] = ('win' if w == b else 'lose', a)
        for rid, (r, st, _) in byid.items():
            rl = role.get(rid)
            if r.task is None:                        # 미션 없이 서 있다가 길을 비켜 주는 로봇
                if rl is not None and rl[0] == 'lose':
                    self._set_lcd(r, 'WAITING', f'clear way for {rl[1]}')
                else:
                    self._set_lcd(r, st.lcd_rest)
                continue
            rem = float(np.sum(np.hypot(*np.diff(self._remaining(r, st), axis=0).T))) if st.plan is not None else None   # 최종 목표까지 남은 경로 (m)
            if rl is not None and rl[0] == 'lose':
                st.waited = True
                self._set_lcd(r, 'WAITING', f'yield to {rl[1]}', rem)
            elif rl is not None:
                self._set_lcd(r, 'OWN DRIVING PRIORITY', f'{rl[1]} yielding', rem)
            else:
                self._set_lcd(r, 'RESUME DRIVING' if st.waited else 'DRIVING', '', rem)

    def _update_lcd_encounter(self, byid):
        """encounter 방식: 마주침 단계를 그대로 보여 준다.
        정지(둘 다) WAITING → 우선권 OWN DRIVING PRIORITY / 양보 WAITING → 양보 로봇이 새 길로 출발하거나 해제되면 RESUME DRIVING"""
        peer = {}
        for e in self.em.enc.values():
            peer[e.a], peer[e.b] = e.b, e.a
        for rid, (r, st, _) in byid.items():
            role, o = self.em.role.get(rid), peer.get(rid, '')
            if r.task is None:
                self._set_lcd(r, 'WAITING', f'clear way for {o}') if role in ('pause', 'lose') else self._set_lcd(r, st.lcd_rest)
                continue
            rem = float(np.sum(np.hypot(*np.diff(self._remaining(r, st), axis=0).T))) if st.plan is not None else None
            if role == 'pause':
                self._set_lcd(r, 'WAITING', f'met {o}: stop', rem)
            elif role == 'win':
                self._set_lcd(r, 'OWN DRIVING PRIORITY', f'{o} yielding', rem)
            elif role == 'lose':
                st.waited = True
                self._set_lcd(r, 'WAITING', f'yield to {o}', rem)
            elif role == 'lose_go':
                st.waited = True
                self._set_lcd(r, 'RESUME DRIVING', f'detour around {o}', rem)
            else:
                self._set_lcd(r, 'RESUME DRIVING' if st.waited else 'DRIVING', '', rem)

    def _dist(self, a, b):
        pa, pb = self.robots[a].pose, self.robots[b].pose
        return math.hypot(pa[0] - pb[0], pa[1] - pb[1])

    @staticmethod
    def _on_odom_motion(st, m):
        if abs(m.twist.twist.linear.x) > 0.02 or abs(m.twist.twist.angular.z) > 0.15:
            st.motion_t = time.monotonic()

    def _encounter(self, r, o):
        """두 로봇이 LCD_ENCOUNTER 안에 있고 사이에 벽이 없는가 (벽 너머로 가까운 것은 마주친 것이 아니다)"""
        a, b = np.array(r.pose[:2]), np.array(o.pose[:2])
        d = float(np.hypot(*(b - a)))
        if d > LCD_ENCOUNTER:
            return False
        n = max(2, int(d / 0.02))
        pts = a + (b - a) * np.linspace(0.0, 1.0, n)[:, None]
        return bool(np.all(self.gm.clearance_at(pts) > 0.0))

    def _send(self, r, pose, mode, note=''):
        st = self.st(r)
        st.seq += 1
        seq = st.seq
        st.mode, st.sub_xy, st.note = mode, (pose.pose.position.x, pose.pose.position.y), note
        st.sent_t, st.route_id = time.monotonic(), 0
        goal = NavigateToPose.Goal()
        goal.pose = pose
        fut = r.client.send_goal_async(goal, feedback_callback=lambda fb, r=r, seq=seq: self._on_fb(r, seq, fb))
        fut.add_done_callback(lambda f, r=r, seq=seq: self._on_resp(r, seq, f))

    def _send_route(self, r, st, path, route_id):
        """마주침에서 정한 경로를 고정해 따라가게 한다: 경로 위 0.5 m 간격 경유점 + 최종 목표를 navigate_through_poses 로.
        (상자 양쪽 길처럼 비용이 비슷하면 Nav2 가 재계획 때마다 길을 바꾸므로, 경유점으로 길을 묶는다)"""
        if not st.tp.server_is_ready():
            self.get_logger().warn(f'{r.id}: navigate_through_poses 서버 없음 → 최종 목표만 보냅니다 (경로 고정 안 됨)')
            self._send(r, st.final, 'GO')
            return
        st.seq += 1
        seq = st.seq
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
        poses = []
        for d in np.arange(0.4, s[-1] - 0.3, 0.5):
            i = int(np.searchsorted(s, d))
            j = min(i + 1, len(path) - 1)
            yaw = math.atan2(path[j][1] - path[i][1], path[j][0] - path[i][0]) if j != i else 0.0
            poses.append(yaw_pose(float(path[i][0]), float(path[i][1]), yaw))
        poses.append(st.final)
        st.mode, st.sub_xy, st.note = 'GO', (st.final.pose.position.x, st.final.pose.position.y), ''
        st.sent_t, st.route_id = time.monotonic(), route_id
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        fut = st.tp.send_goal_async(goal, feedback_callback=lambda fb, r=r, seq=seq: self._on_fb(r, seq, fb))
        fut.add_done_callback(lambda f, r=r, seq=seq: self._on_resp(r, seq, f))

    def _stop(self, r, mode):
        """진행 중인 Nav2 목표를 취소하고 제자리에 선다 (미션은 유지)"""
        st = self.st(r)
        if st.mode == 'ESCAPE' and st.esc_pub is not None:          # 빠져나오기 이동 중이면 그것도 멈춘다 (E-STOP·취소)
            st.esc_pub.publish(String(data=json.dumps({'id': st.esc_id, 'cancel': True})))
        st.seq += 1                     # 취소 결과는 무시
        st.mode, st.sub_xy = mode, None
        if r.goal_handle is not None:
            r.goal_handle.cancel_goal_async()
            r.goal_handle = None
        self._cancel_aux(st)

    def _cancel_aux(self, st):
        """BackUp / DriveOnHeading 이 진행 중이면 취소 (계획 없이 움직이는 동작이라 남겨 두면 E-STOP 뒤에도 계속 간다)"""
        if st.aux is not None:
            st.aux.cancel_goal_async()
            st.aux = None

    # ---------- 미션 ----------
    def on_task(self, task):
        if self.tm is None or self.traffic_off:          # 교통 관제 끔: fleet_coordinator 와 똑같이 (목표만 전달)
            return super().on_task(task)
        if task.type == 'CANCEL':
            return self.cancel(task)
        r, why = self.pick_robot(task)
        if r is None:
            return self.emit(task, task.robot_id, 'FAILED', msg=why)
        if not r.localized:
            return self.emit(task, r.id, 'FAILED', msg='위치(AMCL)를 아직 모름: 초기 위치를 먼저 지정하세요')
        nav_up = r.client.wait_for_server(timeout_sec=1.0)     # GUI 가 미션과 함께 Nav2 를 막 켠 경우: 실패시키지 않고 활성화를 기다린다
        gx, gy = task.goal.pose.position.x, task.goal.pose.position.y
        gcl = float(self.gm.clearance_at(np.array([gx, gy]))[0])
        if gcl < GOAL_MIN_CLEAR:
            return self.emit(task, r.id, 'FAILED', msg=f'목표가 벽·장애물에서 {max(gcl, 0) * 100:.0f} cm (벽 속이거나 너무 가까움): 로봇이 갈 수 없습니다')
        st = self.st(r)
        r.task = task
        st.final, st.plan, st.mode = task.goal, None, 'WAIT'
        st.assigned_t, st.nav_fail = time.monotonic(), 0     # 첫 주기에 경로를 받아 판단한다
        st.esc_tries, st.esc_block = 0, 0.0
        if not nav_up:
            st.nav_ready = False                               # check_nav2_lifecycle 이 bt_navigator active 를 확인하면 출발 (mission_timeout 안에)
            self.get_logger().warn(f'{r.id}: Nav2 가 아직 준비되지 않음 → 활성화될 때까지 대기 (최대 {self.mission_timeout:.0f}초)')
        self.emit(task, r.id, 'ASSIGNED')
        self._request_plan(r)

    def cancel(self, task):
        targets = [self.robots[task.robot_id]] if task.robot_id in self.robots else list(self.robots.values())
        for r in targets:
            if r.task is not None:
                t, r.task = r.task, None
                self._stop(r, 'IDLE')
                self.emit(t, r.id, 'CANCELED')
                self.get_logger().info(f'cancel {r.id}')

    def on_global_cmd(self, msg):
        if msg.data != 'E_STOP':
            return
        self.get_logger().warn('GLOBAL E_STOP: cancel all goals')
        stopped, lane = [], []
        for r in self.robots.values():
            st = self.ts.get(r.id)
            if r.task is not None:
                t, r.task = r.task, None
                self._stop(r, 'IDLE')
                self.emit(t, r.id, 'CANCELED', msg='E_STOP')
                stopped.append(r.id)
            elif r.goal_handle is not None or (st is not None and (st.mode != 'IDLE' or st.aux is not None)):
                self._stop(r, 'IDLE')       # 미션 없이 움직이는 것(비켜서기·후진·빠져나오기)도 멈춘다
                stopped.append(r.id)
            if self.lane_active(r):
                lane.append(r.id)           # 차선 로봇은 관제가 움직이지 않는다: 차선 노드가 global_cmd 를 직접 받고, GUI 도 lane_cmd cancel 을 보낸다
        self.estop_ack_pub.publish(String(data=json.dumps({'cmd': 'E_STOP', 'stopped': stopped, 'lane': lane})))

    # ---------- Nav2 콜백 ----------
    def _on_resp(self, r, seq, fut):
        st = self.st(r)
        gh = fut.result()
        if seq != st.seq:
            if gh.accepted:
                gh.cancel_goal_async()
            return
        if not gh.accepted:
            if st.mode in ('BACKUP', 'HOLD', 'YIELD', 'YIELD_P'):
                self.get_logger().warn(f'{r.id}: {st.mode} 목표를 Nav2 가 거절함 → 5초 뒤 다시 판단')
                st.mode = 'WAIT' if r.task is not None else 'IDLE'
                st.block_until = time.monotonic() + 5.0
                self.tm.last_bay.pop(r.id, None)
                return
            if r.task is not None and st.mode == 'GO':
                if time.monotonic() - st.assigned_t < self.mission_timeout:
                    # Nav2(bt_navigator)가 아직 활성화 중이거나 잠깐 바쁜 경우: 미션을 끝내지 않고 잠시 뒤 다시 보낸다
                    st.nav_fail += 1
                    st.mode = 'WAIT'
                    st.block_until = time.monotonic() + 3.0
                    self.get_logger().warn(f'{r.id}: Nav2 가 목표를 거절함 ({st.nav_fail}회) → 3초 뒤 재시도')
                    self.emit(r.task, r.id, 'RUNNING', msg=f'충돌 방지: Nav2 거절 {st.nav_fail}회, 재시도 대기')
                    return
                t, r.task = r.task, None
                st.mode = 'IDLE'
                self.emit(t, r.id, 'FAILED', msg='로봇이 목표를 거절함')
            return
        r.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f, r=r, seq=seq: self._on_res(r, seq, f))

    def _on_fb(self, r, seq, fb):
        st = self.st(r)
        if seq == st.seq and r.task is not None and st.mode == 'GO':
            self.emit(r.task, r.id, 'RUNNING', dist=fb.feedback.distance_remaining)

    def _on_res(self, r, seq, fut):
        st = self.st(r)
        if seq != st.seq:
            return                       # 우리가 대체/취소한 목표
        status = fut.result().status
        r.goal_handle = None
        if st.mode == 'GO':
            if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED) and r.task is not None \
                    and time.monotonic() - st.assigned_t < self.mission_timeout:
                # Nav2 가 실패해도 미션은 끝내지 않는다: 잠시 물러나 있다가(상대가 지나가거나 코스트맵이 갱신되면) 다시 시도
                st.nav_fail += 1
                st.mode, st.plan = 'WAIT', None
                st.block_until = time.monotonic() + min(3.0 * st.nav_fail, 10.0)
                self.get_logger().warn(f'{r.id}: Nav2 주행 실패 {st.nav_fail}회 → {min(3 * st.nav_fail, 10)}초 뒤 재시도 (미션 유지)')
                self.emit(r.task, r.id, 'RUNNING', msg=f'충돌 방지: Nav2 실패 {st.nav_fail}회, 재시도 대기')
                self._unstick_if_needed(r, st, nav_failed=True)
                return
            t, r.task = r.task, None
            st.mode = 'IDLE'
            if t is not None:
                name = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(status, 'FAILED')
                msg = '' if name != 'FAILED' else f'Nav2 실패 {st.nav_fail + 1}회, {self.mission_timeout:.0f}초 안에 끝내지 못함'
                if name == 'SUCCEEDED' and st.final is not None:
                    # Nav2 는 갈 수 없는 목표면 근처(플래너 tolerance 안)까지만 가고도 SUCCEEDED 를 낸다. 실제 위치로 한 번 더 확인한다.
                    g = st.final.pose.position
                    d = math.hypot(r.pose[0] - g.x, r.pose[1] - g.y)
                    dg = self._geodesic(r.pose[:2], (g.x, g.y))
                    if d > self.arrive_tol:
                        name, msg = 'FAILED', f'목표에서 {d * 100:.0f} cm 떨어진 곳에서 종료 (갈 수 없는 목표이거나 위치추정 오차)'
                    elif dg is not None and dg - d > 0.30:
                        name, msg = 'FAILED', f'목표와 직선으로 {d * 100:.0f} cm 지만 벽 건너편 (실제 경로로 {dg:.2f} m): 도착하지 못했습니다'
                    if name == 'FAILED':
                        self.get_logger().warn(f'{r.id}: Nav2 는 도착이라 했지만 {msg}')
                        if time.monotonic() - st.assigned_t < self.mission_timeout:
                            # 대기 지점 목표가 끝나는 순간 최종 목표로 바꾸면 Nav2 가 새 목표를 '도착'으로 보고하는 경우가 있다 → 미션 유지, 다시 보낸다
                            st.nav_fail += 1
                            r.task, st.mode, st.plan = t, 'WAIT', None
                            st.block_until = time.monotonic() + 2.0
                            self.emit(t, r.id, 'RUNNING', msg=f'충돌 방지: 아직 목표에 못 미침({d * 100:.0f} cm), 다시 주행')
                            return
                self.emit(t, r.id, name, msg=msg)
        elif st.mode == 'BACKUP':
            st.mode = 'WAIT' if r.task is not None else 'IDLE'      # 길을 비켜 준 자리에 도착 (실패해도 그 자리에서 다시 판단)
            st.block_until = time.monotonic() + (0.0 if status == GoalStatus.STATUS_SUCCEEDED else 5.0)
        elif st.mode in ('HOLD', 'YIELD', 'YIELD_P'):
            if st.mode != 'HOLD' and status != GoalStatus.STATUS_SUCCEEDED:
                st.block_until = time.monotonic() + 8.0     # 비켜서기 실패: 같은 자리를 반복해서 시도하지 않는다
                self.tm.last_bay.pop(r.id, None)
                self.get_logger().warn(f'{r.id}: 비켜서기 실패 (Nav2 상태 {status}), 8초 뒤 다른 자리로 다시 시도')
            st.mode = 'WAIT' if st.mode != 'YIELD_P' else 'IDLE'

    # ---------- 경로 ----------
    def _request_plan(self, r):
        st = self.st(r)
        if st.final is None or st.plan_pending:
            return
        start = yaw_pose(r.pose[0], r.pose[1], r.pose[2])
        if st.cp.wait_for_server(timeout_sec=0.0):
            goal = ComputePathToPose.Goal()
            goal.goal, goal.start, goal.use_start = st.final, start, True
            st.plan_pending = True
            st.cp.send_goal_async(goal).add_done_callback(lambda f, r=r: self._on_plan_resp(r, f))
        else:                            # Nav2 플래너가 없으면 같은 지도에서 내부 A* 로
            pts = astar(self.gm, (r.pose[0], r.pose[1]), (st.final.pose.position.x, st.final.pose.position.y))
            if pts is not None:
                st.plan, st.plan_t = pts, time.monotonic()

    def _on_plan_resp(self, r, fut):
        gh = fut.result()
        if not gh.accepted:
            self.st(r).plan_pending = False
            return
        gh.get_result_async().add_done_callback(lambda f, r=r: self._on_plan(r, f))

    def _on_plan(self, r, fut):
        st = self.st(r)
        st.plan_pending = False
        res = fut.result()
        if res.status == GoalStatus.STATUS_SUCCEEDED and len(res.result.path.poses) > 1:
            pts = np.array([[p.pose.position.x, p.pose.position.y] for p in res.result.path.poses])
            st.plan, st.plan_t = resample(pts, DS), time.monotonic()
        elif st.final is not None:        # Nav2 플래너 실패(벽에 붙었거나 일시적으로 막힘): 지도만으로 만든 경로로 충돌 판단은 계속한다
            pts = astar(self.gm, (r.pose[0], r.pose[1]), (st.final.pose.position.x, st.final.pose.position.y))
            if pts is not None:
                st.plan, st.plan_t = pts, time.monotonic()
            self._unstick_if_needed(r, st)

    def _remaining(self, r, st):
        pos = np.array(r.pose[:2])
        if st.plan is None or len(st.plan) < 2:
            return pos[None, :]
        i = int(np.argmin(np.hypot(*(st.plan - pos).T)))
        return np.vstack([pos, st.plan[i + 1:]]) if i + 1 < len(st.plan) else pos[None, :]

    # ---------- 주기 판단 ----------
    def traffic_tick(self):
        if self.traffic_off:
            return
        if self.mode == 'predict':                       # 예측 방식은 계산이 무거워 0.5 s 마다만
            now = time.monotonic()
            if now - self._predict_t < 0.5:
                return
            self._predict_t = now
        for r in self.robots.values():                   # 후진·이동이 너무 오래 걸리면 포기하고 다시 판단
            st = self.ts.get(r.id)
            if st is not None and st.mode in ('HOLD', 'YIELD', 'YIELD_P') and r.goal_handle is None and time.monotonic() - st.sent_t > 8.0:
                self.get_logger().warn(f'{r.id}: {st.mode} 목표에 Nav2 응답이 8초 동안 없음 → 다시 판단')
                st.mode = 'WAIT' if r.task is not None else 'IDLE'
                st.seq += 1
                st.block_until = time.monotonic() + 3.0
            if st is not None and st.mode == 'ESCAPE' and time.monotonic() - st.sent_t > 20.0:
                self.get_logger().warn(f'{r.id}: 벽 빠져나오기 응답이 20초 동안 없음 → 다시 판단')
                st.mode, st.esc_block = ('WAIT' if r.task is not None else 'IDLE'), time.monotonic() + 10.0
            if st is not None and st.mode == 'BACKUP' and time.monotonic() - st.sent_t > 40.0:
                self.get_logger().warn(f'{r.id}: 길 비키기가 40초 안에 끝나지 않아 중단')
                self._stop(r, 'WAIT' if r.task is not None else 'IDLE')
                st.block_until = time.monotonic() + 5.0
        agents, byid = [], {}
        for r in self.robots.values():
            if self.robot_state(r) == 'OFFLINE' or not r.localized or self.lane_active(r):
                continue                # 차선 추종 중인 로봇은 관제가 움직일 수 없다 (상대 로봇의 Nav2 가 장애물로 피한다)
            st = self.st(r)
            active = r.task is not None
            if active and (st.plan is None or time.monotonic() - st.plan_t > 1.0):
                self._request_plan(r)
            if active and st.plan is None:
                if time.monotonic() - st.assigned_t > self.mission_timeout:
                    t, r.task = r.task, None
                    st.mode = 'IDLE'
                    self.emit(t, r.id, 'FAILED', msg=f'{self.mission_timeout:.0f}초 동안 경로를 만들지 못했습니다 (목표가 갈 수 없는 곳이거나 길이 계속 막힘)')
                continue                  # 아직 경로를 못 받았다
            plan = self._remaining(r, st) if active else np.array(r.pose[:2])[None, :]
            a = Agent(r.id, np.array(r.pose[:2]), plan, DS, 0.2, parked=not active, yielding=st.mode in ('YIELD', 'YIELD_P', 'BACKUP', 'ESCAPE'),
                      backing=st.mode in ('BACKUP', 'ESCAPE'), yaw=float(r.pose[2]))
            agents.append(a)
            byid[r.id] = (r, st, a)
        if len(agents) < 1:
            return
        if self.em is not None:
            cmds = self.em.decide(agents, time.monotonic())
            for rid, msg in self.em.events:
                self.get_logger().info(f'{rid}: [마주침] {msg}')
            self.em.events.clear()
        else:
            cmds = self.tm.decide(agents)
        for rid, c in cmds.items():
            r, st, a = byid[rid]
            if not st.nav_ready:
                if r.task is not None:
                    self.emit(r.task, r.id, 'RUNNING', msg='Nav2 활성화 대기 중')
                continue
            self._apply(r, st, a, c)
        self._update_lcd(cmds, byid)

    def _apply(self, r, st, a, c):
        if st.mode in ('BACKUP', 'ESCAPE'):              # 후진·빠져나오기가 끝날 때까지 다른 명령은 받지 않는다
            return
        if c.kind == 'BACKUP_STRAIGHT' and time.monotonic() >= st.block_until:
            self._straight_backup(r, st, c.hold_index / 100.0, c.why)
            if r.task is not None:
                self._note(r, st, c.why)
            return
        if c.kind == 'BACKUP' and c.path is not None and len(c.path) > 1 and time.monotonic() >= st.block_until:
            self._start_backup(r, st, c)
            if r.task is not None:
                self._note(r, st, c.why)
            return
        if r.task is None:                               # 미션 없이 서 있는 로봇
            if c.kind == 'YIELD' and c.path is not None and len(c.path) > 1 and time.monotonic() >= st.block_until:
                tgt = c.path[-1]
                if st.mode != 'YIELD_P' or (st.sub_xy is None or math.hypot(st.sub_xy[0] - tgt[0], st.sub_xy[1] - tgt[1]) > 0.25) and self._can_resend(st):
                    self.get_logger().info(f'{r.id}: 길을 막고 있어 ({tgt[0]:.2f}, {tgt[1]:.2f}) 로 비켜섭니다')
                    yaw = math.atan2(tgt[1] - c.path[-2][1], tgt[0] - c.path[-2][0])
                    self._send(r, yaw_pose(tgt[0], tgt[1], yaw), 'YIELD_P', c.why)
            return
        if c.kind == 'GO':
            route = c.path is not None and len(c.path) > 1 and c.hold_index > 0      # 마주침: 이 경로로 고정해서 간다
            if time.monotonic() >= st.block_until and (st.mode != 'GO' or (route and st.route_id != c.hold_index)):
                if route:
                    self.get_logger().info(f'{r.id}: 고정 경로로 주행 ({c.why})')
                    self._send_route(r, st, c.path, c.hold_index)
                else:
                    self.get_logger().info(f'{r.id}: 주행 {"재개" if st.mode != "IDLE" else "시작"}')
                    self._send(r, st.final, 'GO')
            self._note(r, st, '' if not route else c.why)
        elif c.kind == 'HOLD':
            if c.hold_index <= 0:
                if st.mode in ('GO', 'HOLD', 'YIELD'):
                    self.get_logger().info(f'{r.id}: 대기 ({c.why})')
                    self._stop(r, 'WAIT')
            else:
                idx = min(c.hold_index, len(a.plan) - 1)
                tgt = a.plan[idx]
                if st.mode == 'WAIT' and math.hypot(r.pose[0] - tgt[0], r.pose[1] - tgt[1]) < 0.15:
                    self._note(r, st, c.why)                  # 이미 대기 지점에 와 있다: 같은 목표를 다시 보내지 않는다 (재전송마다 Nav2 가 다시 출발·회전한다)
                    return
                if (st.mode != 'HOLD' or st.sub_xy is None or math.hypot(st.sub_xy[0] - tgt[0], st.sub_xy[1] - tgt[1]) > 0.15) and self._can_resend(st):   # 재전송 간격은 항상 지킨다 (실패하면 0.1 s 마다 다시 보내던 것)
                    prev = a.plan[max(idx - 1, 0)]
                    yaw = math.atan2(tgt[1] - prev[1], tgt[0] - prev[0])
                    self.get_logger().info(f'{r.id}: ({tgt[0]:.2f}, {tgt[1]:.2f}) 에서 대기 ({c.why})')
                    self._send(r, yaw_pose(tgt[0], tgt[1], yaw), 'HOLD', c.why)
            if c.why.startswith('둘 다 비킬 곳 없음'):
                self.get_logger().error(f'{r.id}: 교착 — 둘 다 비킬 곳이 없습니다. 사람이 로봇을 옮겨야 합니다')
            self._note(r, st, c.why)
        elif c.kind == 'YIELD' and c.path is not None and len(c.path) > 1 and time.monotonic() >= st.block_until:
            tgt = c.path[-1]
            if (st.mode != 'YIELD' or st.sub_xy is None or math.hypot(st.sub_xy[0] - tgt[0], st.sub_xy[1] - tgt[1]) > 0.25) and self._can_resend(st):
                self.get_logger().info(f'{r.id}: ({tgt[0]:.2f}, {tgt[1]:.2f}) 로 비켜섭니다 ({c.why})')
                yaw = math.atan2(tgt[1] - c.path[-2][1], tgt[0] - c.path[-2][0])
                self._send(r, yaw_pose(tgt[0], tgt[1], yaw), 'YIELD', c.why)
            self._note(r, st, c.why)

    def _geodesic(self, pos, goal):
        """지도 위에서 실제로 지나갈 수 있는 경로 거리 (m). 로봇이 벽 가까이 있어 알 수 없으면 None"""
        d = self.tm._dist_to(goal)
        gm = self.gm
        r0, c0 = gm.cell(*pos)
        k = int(0.10 / gm.res)
        win = d[max(r0 - k, 0):r0 + k + 1, max(c0 - k, 0):c0 + k + 1]
        v = float(win.min()) if win.size else 1e9
        return None if v >= 1e8 else v

    def _start_backup(self, r, st, c):
        """교착 해소: 상대가 0.30 m 안이면 먼저 0.15 m 똑바로 후진(제자리 회전 공간 확보)한 뒤, 비켜설 자리로 이동"""
        tgt = c.path[-1]
        yaw = math.atan2(tgt[1] - c.path[-2][1], tgt[0] - c.path[-2][0])
        st.backup_target = (float(tgt[0]), float(tgt[1]), yaw)
        near = min((math.hypot(o.pose[0] - r.pose[0], o.pose[1] - r.pose[1]) for o in self.robots.values()
                    if o.id != r.id and o.localized and self.robot_state(o) != 'OFFLINE'), default=9.0)
        self.get_logger().warn(f'{r.id}: 교착 해소 — {c.why} (상대까지 {near:.2f} m)')
        if r.goal_handle is not None:                    # 진행 중이던 목표는 멈춘다
            r.goal_handle.cancel_goal_async()
            r.goal_handle = None
        self._cancel_aux(st)
        st.seq += 1
        seq = st.seq
        st.mode, st.sub_xy, st.sent_t = 'BACKUP', st.backup_target[:2], time.monotonic()
        if near < 0.30 and st.bu.wait_for_server(timeout_sec=0.2):
            g = BackUp.Goal()
            g.target.x, g.speed = 0.15, 0.05
            g.time_allowance.sec = 8
            fut = st.bu.send_goal_async(g)
            fut.add_done_callback(lambda f, r=r, seq=seq: self._on_backup_resp(r, seq, f))
        else:
            self._backup_go(r, seq)

    def _unstick_if_needed(self, r, st, nav_failed=False):
        """Nav2 가 벽 때문에 못 움직일 때: 로봇의 fms_escape 에 '벽 반대로 escape_dist 이동'을 요청하고, 끝나면 미션을 재개한다.
        - 마주침이 진행 중이면(우선권 로봇이 아직 안 지나감) 요청하지 않는다
        - Nav2 주행 실패(collision ahead → patience exceeded)면 바로, 경로 계획 실패면 지도상 벽 여유가 작을 때만
        - 미션당 ESCAPE_MAX 번까지. 로봇에 fms_escape 가 없으면 예전 방식(Nav2 BackUp/DriveOnHeading)"""
        now = time.monotonic()
        if r.task is None or st.mode in ('BACKUP', 'ESCAPE') or now < st.esc_block or st.esc_tries >= ESCAPE_MAX:
            return
        if self.em is not None and any(r.id in (e.a, e.b) for e in self.em.enc.values()):
            return
        if not nav_failed and float(self.gm.clearance_at(np.array(r.pose[:2]))[0]) >= UNSTICK_CLEAR:
            return
        if st.esc_pub is None or st.esc_pub.get_subscription_count() == 0:
            return self._unstick_legacy(r, st)
        self._stop(r, 'ESCAPE')
        st.esc_id += 1
        st.esc_tries += 1
        st.sent_t = now
        d = float(self.get_parameter('escape_dist').value)
        st.esc_pub.publish(String(data=json.dumps({'id': st.esc_id, 'dist': d})))
        self.get_logger().warn(f'{r.id}: 벽 때문에 Nav2 가 못 움직임 → 벽 반대로 {d:.2f} m 빠져나온 뒤 재개 ({st.esc_tries}/{ESCAPE_MAX})')
        self.emit(r.task, r.id, 'RUNNING', msg=f'충돌 방지: 벽에서 {d:.2f} m 빠져나오는 중')

    def _on_escape_status(self, r, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        st = self.st(r)
        if st.mode != 'ESCAPE' or d.get('id') != st.esc_id or d.get('state') not in ('done', 'failed'):
            return
        ok = d['state'] == 'done'
        self.get_logger().info(f'{r.id}: 벽 빠져나오기 {d["state"]} (이동 {d.get("moved", 0):.2f} m) {d.get("msg", "")}')
        st.mode = 'WAIT' if r.task is not None else 'IDLE'
        st.plan = None                                   # 새 위치에서 경로를 다시 받는다
        st.block_until = time.monotonic() + 0.5
        if not ok:
            st.esc_block = time.monotonic() + 10.0

    def _unstick_legacy(self, r, st):
        """로봇이 벽·박스에 너무 붙어(여유 < UNSTICK_CLEAR) Nav2 가 경로를 못 만들 때: 계획 없이 움직이는 BackUp / DriveOnHeading 으로 빈 쪽으로 조금 이동"""
        if st.mode == 'BACKUP' or time.monotonic() < st.block_until - 2.5:
            return
        pos = np.array(r.pose[:2])
        cl = float(self.gm.clearance_at(pos)[0])
        if cl >= UNSTICK_CLEAR:
            return
        yaw = r.pose[2]
        fwd = np.array([math.cos(yaw), math.sin(yaw)])
        def free(direction, max_d=0.25):
            d = 0.0
            while d < max_d and self.gm.clearance_at(pos + direction * (d + 0.01))[0] >= 0.08 - 0.08 * (d == 0.0):
                d += 0.01
            return d
        back, front = free(-fwd), free(fwd)
        g_dist = min(0.15, max(back, front))
        if g_dist < 0.06:
            self.get_logger().warn(f'{r.id}: 벽에서 {cl * 100:.0f} cm 인데 앞뒤로 빠질 공간이 없음')
            return
        st.seq += 1
        seq = st.seq
        st.mode, st.sub_xy, st.sent_t, st.backup_target = 'BACKUP', None, time.monotonic(), None
        use_back = back >= front
        if st.unstick_flip and min(back, front) >= 0.06:
            use_back = not use_back
        if use_back and st.bu.wait_for_server(timeout_sec=0.2):
            g = BackUp.Goal(); g.target.x, g.speed = float(g_dist), 0.05; g.time_allowance.sec = 10
            cli, name = st.bu, '후진'
        elif st.doh.wait_for_server(timeout_sec=0.2):
            g = DriveOnHeading.Goal(); g.target.x, g.speed = float(g_dist), 0.05; g.time_allowance.sec = 10
            cli, name = st.doh, '전진'
        else:
            st.mode = 'WAIT' if r.task is not None else 'IDLE'
            return
        self.get_logger().warn(f'{r.id}: 벽·장애물에서 {cl * 100:.0f} cm 로 너무 가까워 경로를 못 만듦 → {name} {g_dist:.2f} m 로 빠져나옴')
        cli.send_goal_async(g).add_done_callback(lambda f, r=r, seq=seq, name=name: self._on_straight_resp(r, seq, f, name, True))

    def _straight_backup(self, r, st, dist, why):
        """교착 마지막 수단: 방향 그대로 dist 만큼 똑바로 후진(Nav2 BackUp)하고 그 자리에서 다시 판단"""
        self.get_logger().warn(f'{r.id}: 교착 해소 — {why}')
        if r.goal_handle is not None:
            r.goal_handle.cancel_goal_async()
            r.goal_handle = None
        self._cancel_aux(st)
        st.seq += 1
        seq = st.seq
        st.mode, st.sub_xy, st.sent_t, st.backup_target = 'BACKUP', None, time.monotonic(), None
        if not st.bu.wait_for_server(timeout_sec=0.2):
            self.get_logger().error(f'{r.id}: /{r.ns}/backup 서버 없음 (Nav2 behavior_server?)')
            st.mode = 'WAIT' if r.task is not None else 'IDLE'
            st.block_until = time.monotonic() + 5.0
            return
        g = BackUp.Goal()
        g.target.x, g.speed = float(dist), 0.05
        g.time_allowance.sec = 10
        st.bu.send_goal_async(g).add_done_callback(lambda f, r=r, seq=seq: self._on_straight_resp(r, seq, f))

    def _on_straight_resp(self, r, seq, fut, name='후진', unstick=False):
        gh = fut.result()
        st = self.st(r)
        if seq != st.seq:
            if gh.accepted:             # 응답이 오기 전에 E-STOP·취소·다른 판단이 있었다: 받아들여진 동작은 멈춘다
                gh.cancel_goal_async()
            return
        if not gh.accepted:
            self.get_logger().warn(f'{r.id}: {name} 목표를 Nav2 behavior_server 가 거절함')
            return self._straight_done(r, seq, name, unstick, None)
        st.aux = gh
        gh.get_result_async().add_done_callback(lambda f, r=r, seq=seq: self._straight_done(r, seq, name, unstick, f))

    def _straight_done(self, r, seq, name='후진', unstick=False, fut=None):
        st = self.st(r)
        ok = False
        if fut is not None:
            try:
                status = fut.result().status
                ok = status == GoalStatus.STATUS_SUCCEEDED
                if not ok:
                    # ABORTED 는 대개 behavior_server 의 충돌 검사(Collision Ahead) 때문이다
                    self.get_logger().warn(f'{r.id}: {name} 실패 (status={status}, 4=SUCCEEDED 5=CANCELED 6=ABORTED)')
            except Exception as e:
                self.get_logger().warn(f'{r.id}: {name} 결과 확인 실패: {e}')
        if unstick:
            st.unstick_flip = not ok and not st.unstick_flip
        if seq != st.seq or st.mode != 'BACKUP':
            return
        st.aux = None
        st.mode = 'WAIT' if r.task is not None else 'IDLE'

    def _on_backup_resp(self, r, seq, fut):
        gh = fut.result()
        st = self.st(r)
        if seq != st.seq:
            if gh.accepted:
                gh.cancel_goal_async()
            return
        if not gh.accepted:
            return self._backup_go(r, seq)
        st.aux = gh
        gh.get_result_async().add_done_callback(lambda f, r=r, seq=seq: self._backup_go(r, seq))

    def _backup_go(self, r, seq, *_):
        st = self.st(r)
        if seq != st.seq or st.mode != 'BACKUP' or st.backup_target is None:
            return
        st.aux = None
        x, y, yaw = st.backup_target
        goal = NavigateToPose.Goal()
        goal.pose = yaw_pose(x, y, yaw)
        fut = r.client.send_goal_async(goal)
        fut.add_done_callback(lambda f, r=r, seq=seq: self._on_resp(r, seq, f))

    def _can_resend(self, st):
        return time.monotonic() - st.sent_t >= self.min_resend

    def _note(self, r, st, why):
        """대기·비켜서기 중에도 GUI 에는 RUNNING + 사유로 보이게 한다 (주행 중이면 Nav2 feedback 이 거리를 올린다)"""
        if r.task is not None and (st.mode != 'GO' or why):
            msg = f'충돌 방지: {why}' if why else ''
            now = time.monotonic()
            if st.note_last[0] == msg and now - st.note_last[1] < 1.0:      # 판단 주기가 0.1 s 라 같은 내용은 1초에 한 번만 올린다
                return
            st.note_last = (msg, now)
            rem = float(np.sum(np.hypot(*np.diff(self._remaining(r, st), axis=0).T))) if st.plan is not None else 0.0
            self.emit(r.task, r.id, 'RUNNING', dist=rem, msg=msg)


def main():
    rclpy.init()
    node = FleetTraffic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
