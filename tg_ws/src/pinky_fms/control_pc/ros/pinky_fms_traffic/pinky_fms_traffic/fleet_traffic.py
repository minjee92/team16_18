"""충돌 방지 조정 층: fleet_coordinator 를 상속해 '언제, 어디서 기다리고 비켜서는가'를 더한다.

기존 fleet_coordinator / fleet_mission / GUI 는 수정하지 않는다. fleet_coordinator 대신 이 노드를 띄우면 된다
(map_yaml 파라미터가 비어 있으면 fleet_coordinator 와 똑같이 동작한다).

동작:
  - 로봇마다 Nav2 의 계획 경로(compute_path_to_pose, 없으면 내부 A*)를 받아 TrafficManager 에 넘긴다.
  - 로봇에는 항상 NavigateToPose 목표 하나만 보낸다: 최종 목표(GO) / 충돌 구간 앞 대기 지점(HOLD) / 비켜설 자리(YIELD).
    대기·비켜서기 중에도 GUI 에는 원래 미션 ID 의 RUNNING(메시지에 사유)으로 보이고, 최종 목표 도착 때만 SUCCEEDED 가 나간다.
  - 서 있는 로봇(미션 없음)이 남의 길을 막으면 미션 없이 비켜선다.
"""
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_msgs.action import BackUp, ComputePathToPose, DriveOnHeading, NavigateToPose
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes

from pinky_fms_core.fleet_coordinator import FleetCoordinator
from pinky_fms_interfaces.msg import Task

from .gridmap import GridMap
from .planner import astar, resample
from .traffic import Agent, TrafficManager

DS = 0.02
GOAL_MIN_CLEAR = 0.06     # 목표가 벽에서 이보다 가까우면(로봇 반폭) 갈 수 없으므로 배정하지 않는다


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
        self.others_pub = None      # /<ns>/other_robots 발행자
        self.nav_ready = True       # bt_navigator 가 active 인가 (아니면 목표를 보내지 않는다)
        self.gs = None              # bt_navigator/get_state 클라이언트
        self.mn = None              # lifecycle_manager_navigation/manage_nodes 클라이언트
        self.lc_pending = False
        self.lc_last_fix = 0.0


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
        self.declare_parameter('tick_sec', 0.5)
        self.declare_parameter('conf_radius', 0.34)       # 두 로봇 중심이 이보다 가까워질 것 같으면 충돌 예정 (위치추정 오차 여유 포함)
        self.declare_parameter('conf_radius_parked', 0.30)  # 서 있는 로봇과의 충돌 판정 거리 (움직이는 로봇끼리보다 작게)
        self.declare_parameter('arrive_tol', 0.25)        # Nav2 가 SUCCEEDED 라 해도 실제 위치가 목표에서 이보다 멀면 FAILED 로 보고
        self.declare_parameter('mission_timeout', 180.0)   # 재시도를 포함해 이 시간(s) 안에 못 끝내면 FAILED
        self.declare_parameter('min_resend_sec', 3.0)     # 대기·비켜서기 목표를 바꾸는 최소 간격 (자주 바꾸면 Nav2 가 매번 방향을 다시 잡는다)
        mp = self.get_parameter('map_yaml').value
        self.tm = None
        if mp:
            self.gm = GridMap(mp)
            self.tm = TrafficManager(self.gm, r_conf=float(self.get_parameter('conf_radius').value),
                                     r_conf_parked=float(self.get_parameter('conf_radius_parked').value))
            self.arrive_tol = float(self.get_parameter('arrive_tol').value)
            self.mission_timeout = float(self.get_parameter('mission_timeout').value)
            self.min_resend = float(self.get_parameter('min_resend_sec').value)
            self.create_timer(float(self.get_parameter('tick_sec').value), self.traffic_tick)
            self.get_logger().info(f'충돌 방지 켜짐: map={mp}')
        else:
            self.get_logger().warn('map_yaml 이 없어 충돌 방지 꺼짐 (fleet_coordinator 와 동일하게 동작)')
        self.ts = {}
        self.create_timer(0.2, self.publish_others)        # 각 로봇에 다른 로봇들의 실시간 위치를 보낸다 (로봇의 robot_scan_filter 가 사용)
        self.create_timer(5.0, self.check_nav2_lifecycle)  # Nav2 가 활성화되지 못한 로봇(초기 위치가 60초 안에 안 와서 bringup 중단)을 살려 낸다

    # ---------- 보조 ----------
    def st(self, r):
        if r.id not in self.ts:
            t = TState()
            t.cp = ActionClient(self, ComputePathToPose, f'/{r.ns}/compute_path_to_pose')
            t.bu = ActionClient(self, BackUp, f'/{r.ns}/backup')
            t.doh = ActionClient(self, DriveOnHeading, f'/{r.ns}/drive_on_heading')
            t.others_pub = self.create_publisher(PoseArray, f'/{r.ns}/other_robots', 5)
            t.gs = self.create_client(GetState, f'/{r.ns}/bt_navigator/get_state')
            t.mn = self.create_client(ManageLifecycleNodes, f'/{r.ns}/lifecycle_manager_navigation/manage_nodes')
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
            if st.lc_pending or not st.gs.service_is_ready():
                continue
            st.lc_pending = True
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
        if ready or not st.mn.service_is_ready() or time.monotonic() - st.lc_last_fix < 40.0:
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

    def robot_state(self, r):
        s = super().robot_state(r)
        if s == 'IDLE' and r.id in self.ts and self.ts[r.id].mode in ('YIELD_P', 'BACKUP'):
            return 'BUSY'               # 비켜서는 중에는 새 미션을 받지 않는다
        return s

    def _send(self, r, pose, mode, note=''):
        st = self.st(r)
        st.seq += 1
        seq = st.seq
        st.mode, st.sub_xy, st.note = mode, (pose.pose.position.x, pose.pose.position.y), note
        st.sent_t = time.monotonic()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        fut = r.client.send_goal_async(goal, feedback_callback=lambda fb, r=r, seq=seq: self._on_fb(r, seq, fb))
        fut.add_done_callback(lambda f, r=r, seq=seq: self._on_resp(r, seq, f))

    def _stop(self, r, mode):
        """진행 중인 Nav2 목표를 취소하고 제자리에 선다 (미션은 유지)"""
        st = self.st(r)
        st.seq += 1                     # 취소 결과는 무시
        st.mode, st.sub_xy = mode, None
        if r.goal_handle is not None:
            r.goal_handle.cancel_goal_async()
            r.goal_handle = None

    # ---------- 미션 ----------
    def on_task(self, task):
        if self.tm is None:
            return super().on_task(task)
        if task.type == 'CANCEL':
            return self.cancel(task)
        r, why = self.pick_robot(task)
        if r is None:
            return self.emit(task, task.robot_id, 'FAILED', msg=why)
        if not r.localized:
            return self.emit(task, r.id, 'FAILED', msg='위치(AMCL)를 아직 모름: 초기 위치를 먼저 지정하세요')
        if not r.client.wait_for_server(timeout_sec=1.0):
            return self.emit(task, r.id, 'FAILED', msg=f'/{r.ns}/navigate_to_pose 서버 없음 (Nav2 미실행?)')
        gx, gy = task.goal.pose.position.x, task.goal.pose.position.y
        gcl = float(self.gm.clearance_at(np.array([gx, gy]))[0])
        if gcl < GOAL_MIN_CLEAR:
            return self.emit(task, r.id, 'FAILED', msg=f'목표가 벽·장애물에서 {max(gcl, 0) * 100:.0f} cm (벽 속이거나 너무 가까움): 로봇이 갈 수 없습니다')
        st = self.st(r)
        r.task = task
        st.final, st.plan, st.mode = task.goal, None, 'WAIT'
        st.assigned_t, st.nav_fail = time.monotonic(), 0     # 첫 주기에 경로를 받아 판단한다
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
        if msg.data == 'E_STOP':
            self.get_logger().warn('GLOBAL E_STOP: cancel all goals')
            for r in self.robots.values():
                if r.task is not None:
                    t, r.task = r.task, None
                    self._stop(r, 'IDLE')
                    self.emit(t, r.id, 'CANCELED', msg='E_STOP')
                elif r.id in self.ts and self.ts[r.id].mode in ('YIELD_P', 'BACKUP'):
                    self._stop(r, 'IDLE')

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
                self._unstick_if_needed(r, st)
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
        for r in self.robots.values():                   # 후진·이동이 너무 오래 걸리면 포기하고 다시 판단
            st = self.ts.get(r.id)
            if st is not None and st.mode in ('HOLD', 'YIELD', 'YIELD_P') and r.goal_handle is None and time.monotonic() - st.sent_t > 8.0:
                self.get_logger().warn(f'{r.id}: {st.mode} 목표에 Nav2 응답이 8초 동안 없음 → 다시 판단')
                st.mode = 'WAIT' if r.task is not None else 'IDLE'
                st.seq += 1
                st.block_until = time.monotonic() + 3.0
            if st is not None and st.mode == 'BACKUP' and time.monotonic() - st.sent_t > 40.0:
                self.get_logger().warn(f'{r.id}: 길 비키기가 40초 안에 끝나지 않아 중단')
                self._stop(r, 'WAIT' if r.task is not None else 'IDLE')
                st.block_until = time.monotonic() + 5.0
        agents, byid = [], {}
        for r in self.robots.values():
            if self.robot_state(r) == 'OFFLINE' or not r.localized:
                continue
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
            a = Agent(r.id, np.array(r.pose[:2]), plan, DS, 0.2, parked=not active, yielding=st.mode in ('YIELD', 'YIELD_P', 'BACKUP'),
                      backing=st.mode == 'BACKUP', yaw=float(r.pose[2]))
            agents.append(a)
            byid[r.id] = (r, st, a)
        if len(agents) < 1:
            return
        cmds = self.tm.decide(agents)
        for rid, c in cmds.items():
            r, st, a = byid[rid]
            if not st.nav_ready:
                if r.task is not None:
                    self.emit(r.task, r.id, 'RUNNING', msg='Nav2 활성화 대기 중')
                continue
            self._apply(r, st, a, c)

    def _apply(self, r, st, a, c):
        if st.mode == 'BACKUP':                          # 후진·이동이 끝날 때까지 다른 명령은 받지 않는다
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
            if st.mode != 'GO' and time.monotonic() >= st.block_until:
                self.get_logger().info(f'{r.id}: 주행 {"재개" if st.mode != "IDLE" else "시작"}')
                self._send(r, st.final, 'GO')
            self._note(r, st, '')
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
                if st.mode != 'HOLD' or (st.sub_xy is None or math.hypot(st.sub_xy[0] - tgt[0], st.sub_xy[1] - tgt[1]) > 0.15) and self._can_resend(st):
                    prev = a.plan[max(idx - 1, 0)]
                    yaw = math.atan2(tgt[1] - prev[1], tgt[0] - prev[0])
                    self.get_logger().info(f'{r.id}: ({tgt[0]:.2f}, {tgt[1]:.2f}) 에서 대기 ({c.why})')
                    self._send(r, yaw_pose(tgt[0], tgt[1], yaw), 'HOLD', c.why)
            if c.why.startswith('둘 다 비킬 곳 없음'):
                self.get_logger().error(f'{r.id}: 교착 — 둘 다 비킬 곳이 없습니다. 사람이 로봇을 옮겨야 합니다')
            self._note(r, st, c.why)
        elif c.kind == 'YIELD' and c.path is not None and len(c.path) > 1 and time.monotonic() >= st.block_until:
            tgt = c.path[-1]
            if st.mode != 'YIELD' or (st.sub_xy is None or math.hypot(st.sub_xy[0] - tgt[0], st.sub_xy[1] - tgt[1]) > 0.25) and self._can_resend(st):
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

    def _unstick_if_needed(self, r, st):
        """로봇이 벽·박스에 너무 붙어(여유 < 0.10 m) Nav2 가 경로를 못 만들 때: 계획 없이 움직이는 BackUp / DriveOnHeading 으로 빈 쪽으로 조금 이동"""
        if st.mode == 'BACKUP' or time.monotonic() < st.block_until - 2.5:
            return
        pos = np.array(r.pose[:2])
        cl = float(self.gm.clearance_at(pos)[0])
        if cl >= 0.10:
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
        if back >= front and st.bu.wait_for_server(timeout_sec=0.2):
            g = BackUp.Goal(); g.target.x, g.speed = float(g_dist), 0.05; g.time_allowance.sec = 10
            cli, name = st.bu, '후진'
        elif st.doh.wait_for_server(timeout_sec=0.2):
            g = DriveOnHeading.Goal(); g.target.x, g.speed = float(g_dist), 0.05; g.time_allowance.sec = 10
            cli, name = st.doh, '전진'
        else:
            st.mode = 'WAIT' if r.task is not None else 'IDLE'
            return
        self.get_logger().warn(f'{r.id}: 벽·장애물에서 {cl * 100:.0f} cm 로 너무 가까워 경로를 못 만듦 → {name} {g_dist:.2f} m 로 빠져나옴')
        cli.send_goal_async(g).add_done_callback(lambda f, r=r, seq=seq: self._on_straight_resp(r, seq, f))

    def _straight_backup(self, r, st, dist, why):
        """교착 마지막 수단: 방향 그대로 dist 만큼 똑바로 후진(Nav2 BackUp)하고 그 자리에서 다시 판단"""
        self.get_logger().warn(f'{r.id}: 교착 해소 — {why}')
        if r.goal_handle is not None:
            r.goal_handle.cancel_goal_async()
            r.goal_handle = None
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

    def _on_straight_resp(self, r, seq, fut):
        gh = fut.result()
        if seq != self.st(r).seq:
            return
        if not gh.accepted:
            return self._straight_done(r, seq)
        gh.get_result_async().add_done_callback(lambda f, r=r, seq=seq: self._straight_done(r, seq))

    def _straight_done(self, r, seq, *_):
        st = self.st(r)
        if seq != st.seq or st.mode != 'BACKUP':
            return
        st.mode = 'WAIT' if r.task is not None else 'IDLE'

    def _on_backup_resp(self, r, seq, fut):
        gh = fut.result()
        if seq != self.st(r).seq:
            return
        if not gh.accepted:
            return self._backup_go(r, seq)
        gh.get_result_async().add_done_callback(lambda f, r=r, seq=seq: self._backup_go(r, seq))

    def _backup_go(self, r, seq, *_):
        st = self.st(r)
        if seq != st.seq or st.mode != 'BACKUP' or st.backup_target is None:
            return
        x, y, yaw = st.backup_target
        goal = NavigateToPose.Goal()
        goal.pose = yaw_pose(x, y, yaw)
        fut = r.client.send_goal_async(goal)
        fut.add_done_callback(lambda f, r=r, seq=seq: self._on_resp(r, seq, f))

    def _can_resend(self, st):
        return time.monotonic() - st.sent_t >= self.min_resend

    def _note(self, r, st, why):
        """대기·비켜서기 중에도 GUI 에는 RUNNING + 사유로 보이게 한다 (주행 중이면 Nav2 feedback 이 거리를 올린다)"""
        if r.task is not None and st.mode != 'GO':
            rem = float(np.sum(np.hypot(*np.diff(self._remaining(r, st), axis=0).T))) if st.plan is not None else 0.0
            self.emit(r.task, r.id, 'RUNNING', dist=rem, msg=f'충돌 방지: {why}' if why else '')


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
