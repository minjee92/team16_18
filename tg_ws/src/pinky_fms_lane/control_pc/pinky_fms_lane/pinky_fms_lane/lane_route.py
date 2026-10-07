"""차선 미션 경로 계획 (관제 PC): GUI 목표 → 코스 모델로 길 위에 맞추고 경로·갈림길 동작을 붙여 lane_cmd 로 보낸다.

    ros2 run pinky_fms_lane lane_route [--ros-args -p course:=<코스.yaml> -p map_yaml:=<clean 지도.yaml>]

  받음  /fleet/lane_goal  (std_msgs/String JSON) {"robot": "amr_01", "id": n, "cmd": "go"|"home", "x": .., "y": ..}
        /<ns>/lane_status (팀원 fms_lane_mission 상태: pose 로 지금 위치·방향을 안다)
  보냄  /<ns>/lane_cmd    팀원 형식 {"id","cmd","x","y"} 에 선택 필드를 더한다 (필드를 모르는 로봇은 예전처럼 동작)
          "x","y"      : 길 위로 맞춘 목표 (클릭한 점이 금지 구역이면 구역 밖으로 옮긴 점)
          "route"      : 가는 길 {"points": [[x,y],...], "s_goal": 경로 길이(m), "uturn": 출발 전 제자리 U턴,
                                   "maneuvers": [{"node","x","y","s_at","radius","turn","follow","follow_dist"}]}
                           s_at = 경로 위에서 갈림길(나갈 간선 시작)까지 거리, radius = 갈림길 구역 반지름,
                           turn = STRAIGHT|LEFT|RIGHT, follow = 따라갈 테이프 쪽 LEFT|RIGHT, follow_dist = 그 테이프만 따라갈 거리
          "route_back" : 돌아오는 길 (cmd 가 go 일 때, 목표 → 지금 위치). 같은 형식
          "arrive_tol" : 경로 진행 거리로 도착 판정할 때 여유 (m)
        /fleet/lane_goal_result {"robot","id","ok","reason","goal","note","length","length_back"}
  go 의 목표는 클릭한 점이라 금지 구역이면 구역 밖으로 옮긴다. home(RETURN DOCK)은 도크 자리라 길 위로만 맞춘다.
          거부(ok=false)면 lane_cmd 를 보내지 않는다. GUI 는 reason 을 보여 준다.
  같은 (robot, id) 가 다시 오면 처음 계산한 lane_cmd 를 그대로 다시 보낸다 (GUI 가 확인될 때까지 재전송하므로).
  지도(map_yaml)를 주면 U턴할 자리의 벽 여유를 지도로 확인한다 (clean 지도. 차선 지도가 아님).
"""
import json
import math
import re
import time

from pinky_fms_lane.course import PlanError

LANE_STATUS_RE = re.compile(r'^/([a-z][a-z0-9_]{0,31})/lane_status$')
NS_RE = re.compile(r'^[a-z][a-z0-9_]{0,31}$')


class Reject(ValueError):
    """목표를 받을 수 없음. 메시지는 GUI 에 그대로 보여 줄 문장."""


def route_fields(route, course):
    """Route → lane_cmd 의 route 필드 (좌표 mm 반올림)."""
    out = []
    for m in route.maneuvers:
        j = course.junctions[m.node_id]
        out.append({'node': m.node_id, 'x': round(j.x, 3), 'y': round(j.y, 3), 's_at': round(m.s_at, 3),
                    'radius': round(j.radius, 3), 'turn': m.turn, 'follow': m.follow,
                    'follow_dist': round(m.follow_dist, 3)})
    return {'points': [[round(float(x), 3), round(float(y), 3)] for x, y in route.points],
            's_goal': round(route.length, 3), 'uturn': bool(route.uturn), 'maneuvers': out}


def _end_dir(route):
    lg = route.legs[-1]
    return lg.dir


def plan_lane_cmd(course, pose, goal_xy, cmd, cmd_id, clearance_fn=None, arrive_tol=0.05):
    """로봇 위치·방향 pose(x, y, yaw) 와 클릭한 목표 → (lane_cmd dict, 결과 dict). 받을 수 없으면 Reject."""
    if cmd not in ('go', 'home'):
        raise Reject(f'경로를 붙일 수 없는 명령: {cmd}')
    if pose is None:
        raise Reject('로봇 위치를 모름 (Init Pose 를 먼저 지정하고 차선 스택이 떠 있는지 확인)')
    x, y, yaw = (float(v) for v in pose[:3])
    start = course.locate(x, y, yaw)
    if start is None:
        raise Reject(f'로봇이 차선 위에 있지 않음 (길에서 {course.distance_to_road(x, y):.2f} m)')
    if start.dir == 0:
        raise Reject('로봇 방향이 길과 거의 수직이라 진행 방향을 알 수 없음')
    if cmd == 'go':                                      # 클릭한 목표: 길 위로, 금지 구역(갈림길·대기·횡단보도·막다른 끝)이면 구역 밖으로
        goal, note = course.snap_goal(float(goal_xy[0]), float(goal_xy[1]))
        if goal is None:
            raise Reject(f'목표를 둘 수 없음: {note}')
    else:                                                # home(도크 복귀): 도크 자리 그대로 길 위로만 맞춘다
        goal, note = course.locate(float(goal_xy[0]), float(goal_xy[1])), ''
        if goal is None:
            raise Reject('목표를 둘 수 없음: 도크가 길에서 '
                         f'{course.distance_to_road(float(goal_xy[0]), float(goal_xy[1])):.2f} m 떨어져 있음')
    ok_u, why_u = course.can_uturn(x, y, clearance_fn)
    try:
        route = course.plan(start, goal, allow_uturn=ok_u)
    except PlanError as e:
        raise Reject(f'경로 없음: {e}' + ('' if ok_u else f' (지금 자리에서 U턴 불가: {why_u})'))
    msg = {'id': int(cmd_id), 'cmd': cmd, 'x': round(goal.x, 3), 'y': round(goal.y, 3),
           'route': route_fields(route, course), 'arrive_tol': float(arrive_tol)}
    res = {'ok': True, 'goal': [msg['x'], msg['y']], 'note': note, 'length': round(route.length, 2)}
    if cmd == 'go':
        back_start = course.location_at(goal.edge, goal.s)
        back_start.dir = _end_dir(route)
        home = course.locate(x, y, None)                 # 돌아올 자리 = 지금 위치를 길 위로 옮긴 점 (금지 구역이어도 됨)
        ok_ub, why_ub = course.can_uturn(goal.x, goal.y, clearance_fn)
        try:
            back = course.plan(back_start, home, allow_uturn=ok_ub)
        except PlanError as e:
            raise Reject(f'돌아오는 경로 없음: {e}' + ('' if ok_ub else f' (목표에서 U턴 불가: {why_ub})'))
        msg['route_back'] = route_fields(back, course)
        res['length_back'] = round(back.length, 2)
    return msg, res


def main(args=None):
    import os
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    from pinky_fms_lane.course import Course
    from pinky_fms_lane.draw_course import default_course_path

    class LaneRoute(Node):
        def __init__(self):
            super().__init__('lane_route')
            self.declare_parameter('course', default_course_path())
            self.declare_parameter('map_yaml', '')          # clean 지도: U턴 자리의 벽 여유 확인용 (비우면 확인 안 함)
            self.declare_parameter('arrive_tol', 0.05)      # 경로 진행 거리 도착 판정 여유 (m)
            self.declare_parameter('status_stale', 2.0)     # 이보다 오래된 lane_status 위치는 쓰지 않는다 (s)
            gp = lambda n: self.get_parameter(n).value       # noqa: E731
            path = gp('course')
            if not path or not os.path.exists(path):
                raise RuntimeError(f'코스 파일이 없음: {path!r}')
            self.course = Course.load(path)
            self.clearance = None
            if gp('map_yaml'):
                from pinky_fms_traffic.gridmap import GridMap    # 팀원 코드 재사용
                gm = GridMap(gp('map_yaml'))
                self.clearance = lambda x, y: float(gm.clearance_at([[x, y]])[0])   # noqa: E731
            self.arrive_tol, self.stale = float(gp('arrive_tol')), float(gp('status_stale'))
            self.status = {}            # robot → (받은 시각, dict)
            self.cmd_pubs = {}
            self.sent = {}              # (robot, id) → (lane_cmd 문자열, 결과 dict)
            self.result_pub = self.create_publisher(String, '/fleet/lane_goal_result', 10)
            self.create_subscription(String, '/fleet/lane_goal', self._on_goal, 10)
            self.create_timer(2.0, self._discover)
            self._discover()
            src = {k: len(v) for k, v in self.course.source_summary().items() if v}
            self.get_logger().info(f'lane_route ready: 코스 {os.path.basename(path)} (점 출처 {src}), '
                                   f'U턴 벽 확인 {"지도 " + os.path.basename(gp("map_yaml")) if self.clearance else "안 함"}')

        def _discover(self):
            for name, _ in self.get_topic_names_and_types():
                m = LANE_STATUS_RE.match(name)
                if m and m.group(1) not in self.status:
                    rid = m.group(1)
                    self.status[rid] = (0.0, {})
                    self.create_subscription(String, name, lambda msg, r=rid: self._on_status(r, msg), 10)

        def _on_status(self, rid, msg):
            try:
                self.status[rid] = (time.monotonic(), json.loads(msg.data))
            except ValueError:
                pass

        def _pub_cmd(self, rid, text):
            if rid not in self.cmd_pubs:
                self.cmd_pubs[rid] = self.create_publisher(String, f'/{rid}/lane_cmd', 10)
            self.cmd_pubs[rid].publish(String(data=text))

        def _result(self, rid, cid, res):
            self.result_pub.publish(String(data=json.dumps({'robot': rid, 'id': cid, **res})))

        def _on_goal(self, msg):
            try:
                d = json.loads(msg.data)
                rid, cid, cmd = str(d['robot']), int(d['id']), str(d['cmd'])
                gx, gy = float(d['x']), float(d['y'])
            except (ValueError, KeyError, TypeError):
                self.get_logger().warn(f'잘못된 lane_goal: {msg.data[:200]}')
                return
            if not NS_RE.match(rid) or not (math.isfinite(gx) and math.isfinite(gy)):
                self.get_logger().warn(f'잘못된 lane_goal 값: {msg.data[:200]}')
                return
            key = (rid, cid)
            if key in self.sent:                         # GUI 재전송: 같은 결과를 다시
                text, res = self.sent[key]
                if text:
                    self._pub_cmd(rid, text)
                self._result(rid, cid, res)
                return
            t, st = self.status.get(rid, (0.0, {}))
            pose = st.get('pose') if time.monotonic() - t <= self.stale else None
            try:
                lane_cmd, res = plan_lane_cmd(self.course, pose, (gx, gy), cmd, cid, self.clearance, self.arrive_tol)
            except Reject as e:
                res = {'ok': False, 'reason': str(e)}
                self.sent[key] = ('', res)
                self.get_logger().warn(f'{rid} 목표 ({gx:.2f}, {gy:.2f}) 거부: {e}')
                self._result(rid, cid, res)
                return
            text = json.dumps(lane_cmd)
            self.sent[key] = (text, res)
            if len(self.sent) > 200:                     # 오래된 것 정리
                for k in list(self.sent)[:100]:
                    self.sent.pop(k, None)
            r = lane_cmd['route']
            moves = ', '.join(f'{m["node"]} {m["turn"]}/{m["follow"]}' for m in r['maneuvers']) or '없음'
            back = f' | 돌아오는 길 {lane_cmd["route_back"]["s_goal"]:.2f} m' if 'route_back' in lane_cmd else ''
            self.get_logger().info(
                f'{rid} 목표 ({gx:.2f}, {gy:.2f}) → 길 위 ({lane_cmd["x"]:.2f}, {lane_cmd["y"]:.2f}) {res.get("note", "")}'
                f' | 가는 길 {r["s_goal"]:.2f} m{" (출발 U턴)" if r["uturn"] else ""}, 갈림길 {moves}{back}')
            self._pub_cmd(rid, text)
            self._result(rid, cid, res)

    rclpy.init(args=args)
    node = LaneRoute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
