"""cmd_vel 게이트 (로봇): 차선 노드의 속도 명령을 받아 안전 조건을 확인하고 base 로 넘긴다. 정지만 할 수 있고 빠르게 만들지는 않는다.

    robot_lane.launch.xml use_gate:=true 로 켠다 (기본 false = 예전처럼 fms_lane_mission 이 cmd_vel 에 직접 냄).
    그러면 fms_lane_mission 의 cmd_vel 이 cmd_vel_lane 으로 바뀌고, 이 노드가 cmd_vel_lane → cmd_vel 로 넘긴다.

  rate(20 Hz)마다 항상 cmd_vel 을 낸다. 아래 중 하나면 0 을 낸다 (status 의 reasons):
    STALE_INPUT      입력(cmd_vel_in)이 input_timeout 넘게 안 옴 (차선 노드 멈춤·죽음)
    FMS_LINK         관제 살아있음 신호(/fleet/lane_heartbeat)가 link_timeout 넘게 없음 (0 이면 확인 안 함)
    NO_SONAR / SONAR_TIMEOUT / SONAR_INVALID
                     초음파 데이터 없음·끊김·무효값 연속 (FrontStop, fail-closed). 거리로 서는 것은 차선 노드가 한다
    CMD_VEL_CONFLICT 출력 cmd_vel 을 내는 노드가 이 노드 말고도 있음 (키보드·Nav2 와 겹침)
  통과할 때도 linear.x 는 0~max_linear (후진 금지, allow_reverse 로 허용), angular.z 는 ±max_angular 로 자른다.
  상태: cmd_vel_gate/status (std_msgs/String JSON {"ok", "reasons", "in_age", "link_age", "sonar"}) 를 바뀔 때와 1 s 마다.
  끝날 때 0 을 여러 번 보낸다. 게이트 프로세스가 죽으면 base 는 마지막 명령을 계속 따르므로 launch 에서 respawn 한다.
"""
import json
import math
import time

from pinky_fms_lane_robot.front_stop import FrontStop


def gate(v, w, now, st, p):
    """입력 속도(v, w)와 상태 → (v, w, 이유 목록). st: in_t, hb_t, sonar(FrontStop 또는 None), publishers. p: 파라미터 dict."""
    reasons = []
    if st['in_t'] is None or now - st['in_t'] > p['input_timeout']:
        reasons.append('STALE_INPUT')
    if p['link_timeout'] > 0 and (st['hb_t'] is None or now - st['hb_t'] > p['link_timeout']):
        reasons.append('FMS_LINK')
    if st['sonar'] is not None:
        r = st['sonar'].reason(now)
        if r and r != 'OBSTACLE':
            reasons.append(r.replace(' ', '_'))
    if st['publishers'] > 1:
        reasons.append('CMD_VEL_CONFLICT')
    if reasons or not (math.isfinite(v) and math.isfinite(w)):
        return 0.0, 0.0, reasons or ['BAD_INPUT']
    lo = -p['max_linear'] if p['allow_reverse'] else 0.0
    return min(max(v, lo), p['max_linear']), min(max(w, -p['max_angular']), p['max_angular']), []


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Range
    from std_msgs.msg import String
    import signal

    class CmdVelGate(Node):
        def __init__(self):
            super().__init__('cmd_vel_gate')
            for k, v in (('rate', 20.0), ('input_timeout', 0.5), ('link_timeout', 2.0),
                         ('heartbeat_topic', '/fleet/lane_heartbeat'), ('use_sonar', True),
                         ('sonar_topic', 'us_sensor/range'), ('sonar_timeout', 1.0), ('sonar_min_range', 0.02),
                         ('sonar_invalid_count', 3), ('max_linear', 0.25), ('max_angular', 1.5), ('allow_reverse', False)):
                self.declare_parameter(k, v)
            g = lambda n: self.get_parameter(n).value     # noqa: E731
            self.p = {k: g(k) for k in ('input_timeout', 'link_timeout', 'max_linear', 'max_angular', 'allow_reverse')}
            self.st = {'in_t': None, 'hb_t': None, 'publishers': 1,
                       'sonar': FrontStop(0.0, float(g('sonar_timeout')), float(g('sonar_min_range')),
                                          int(g('sonar_invalid_count'))) if g('use_sonar') else None}
            self.v = self.w = 0.0
            self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
            self.status_pub = self.create_publisher(String, 'cmd_vel_gate/status', 10)
            self.create_subscription(Twist, 'cmd_vel_in', self._on_in, 10)
            self.create_subscription(String, g('heartbeat_topic'), self._on_hb, 10)
            if g('use_sonar'):
                # best effort 로 받으면 발행 쪽이 reliable·best effort 어느 쪽이어도 받는다
                self.create_subscription(Range, g('sonar_topic'), self._on_sonar, qos_profile_sensor_data)
            self.last_reasons, self.status_t = None, 0.0
            self.create_timer(1.0 / float(g('rate')), self._tick)
            self.get_logger().info(f'cmd_vel_gate: cmd_vel_in → cmd_vel, 관제 신호 {self.p["link_timeout"]} s, '
                                   f'초음파 {"fail-closed" if g("use_sonar") else "안 봄"}, 최대 {self.p["max_linear"]} m/s')

        def _on_in(self, m):
            self.st['in_t'], self.v, self.w = time.monotonic(), float(m.linear.x), float(m.angular.z)

        def _on_hb(self, _m):
            self.st['hb_t'] = time.monotonic()

        def _on_sonar(self, m):
            self.st['sonar'].on_range(m.range, time.monotonic())

        def _tick(self):
            now = time.monotonic()
            self.st['publishers'] = self.count_publishers(self.pub.topic_name)
            v, w, reasons = gate(self.v, self.w, now, self.st, self.p)
            t = Twist()
            t.linear.x, t.angular.z = float(v), float(w)
            self.pub.publish(t)
            if reasons != self.last_reasons:
                if reasons:
                    self.get_logger().warn(f'정지: {", ".join(reasons)}')
                elif self.last_reasons:
                    self.get_logger().info('통과 재개')
            if reasons != self.last_reasons or now - self.status_t >= 1.0:
                self.status_t, self.last_reasons = now, reasons
                age = lambda t0: None if t0 is None else round(now - t0, 2)     # noqa: E731
                s = self.st['sonar']
                self.status_pub.publish(String(data=json.dumps({
                    'ok': not reasons, 'reasons': reasons, 'in_age': age(self.st['in_t']), 'link_age': age(self.st['hb_t']),
                    'sonar': None if s is None else s.last_range})))

        def stop(self):
            for _ in range(5):
                self.pub.publish(Twist())
                time.sleep(0.02)

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = None
    try:
        node = CmdVelGate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
