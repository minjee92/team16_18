"""lane_traffic 격리 시험 판정: 두 가짜 차선 로봇이 모두 도착하고 충돌이 없으면 PASS.
   python3 lane_scenario.py <시나리오> <제한시간 s> <조정 기대: yes|no> [goal=<로봇>,<x>,<y>,<go|home> ...] [reject=<로봇>]

 goal=...  : GUI 처럼 /fleet/lane_goal 로 목표를 보낸다 (관제 lane_route 가 경로를 붙여 lane_cmd 로 보낸다).
             로봇 lane_status 의 cmd_id 가 그 id 가 될 때까지 0.6 s 마다 다시 보낸다 (GUI 와 같음).
 reject=.. : 그 로봇의 목표는 lane_route 가 거부해야 한다 (/fleet/lane_goal_result ok=false, 로봇은 IDLE 로 남음).
 goal 이 없으면 예전처럼 가짜 로봇이 start → goal 로 바로 간다."""
import json
import sys
import time

import rclpy
from std_msgs.msg import String

sc, limit, expect = sys.argv[1], float(sys.argv[2]), sys.argv[3] == 'yes'
goals, reject = {}, set()
for a in sys.argv[4:]:
    k, _, v = a.partition('=')
    if k == 'goal':
        rid, x, y, cmd = v.split(',')
        goals[rid] = {'robot': rid, 'id': 0, 'cmd': cmd, 'x': float(x), 'y': float(y)}
    elif k == 'reject':
        reject.add(v)
rclpy.init()
n = rclpy.create_node('lane_scenario')
st, last, seen_enc, results = {}, {}, [], {}
t0 = time.monotonic()


def on_status(rid, m):
    d = json.loads(m.data)
    st[rid] = d
    key = (d['state'], d.get('traffic'))
    if last.get(rid) != key:
        last[rid] = key
        print(f'{time.monotonic() - t0:6.1f}s {rid}: {d["state"]:<20} traffic={d.get("traffic")} pose={d.get("pose")}', flush=True)


def on_state(m):
    for e in json.loads(m.data)['encounters']:
        k = (e['phase'], e['priority'], e['yielder'])
        if not seen_enc or seen_enc[-1] != k:
            seen_enc.append(k)
            print(f'{time.monotonic() - t0:6.1f}s 조정: {e["phase"]} 우선={e["priority"]} 양보={e["yielder"]} 자리={e["escape"]} {e["info"]}', flush=True)


def on_result(m):
    d = json.loads(m.data)
    if d.get('robot') in goals and d.get('id') == goals[d['robot']]['id'] and d['robot'] not in results:
        results[d['robot']] = d
        print(f'{time.monotonic() - t0:6.1f}s 경로 계획 {d["robot"]}: {"OK" if d["ok"] else "거부"} '
              f'{d.get("reason") or ""}{d.get("note") or ""} 길이 {d.get("length")} / {d.get("length_back")}', flush=True)


for rid in ('amr_01', 'amr_02'):
    n.create_subscription(String, f'/{rid}/lane_status', lambda m, r=rid: on_status(r, m), 10)
n.create_subscription(String, '/fleet/lane_traffic_state', on_state, 10)
n.create_subscription(String, '/fleet/lane_goal_result', on_result, 10)
goal_pub = n.create_publisher(String, '/fleet/lane_goal', 10)
seq, sent_t = int(time.time()) % 100000 * 10, 0.0
for g in goals.values():
    seq += 1
    g['id'] = seq


def done_now():
    if len(st) < 2:
        return False
    for rid, d in st.items():
        if rid in reject:
            if rid not in results:
                return False
        elif d['state'] != 'ARRIVED':
            return False
    return True


while time.monotonic() - t0 < limit:
    rclpy.spin_once(n, timeout_sec=0.1)
    now = time.monotonic()
    if goals and now - sent_t > 0.6 and len(st) == 2:      # 두 로봇 상태가 보이면 보낸다 (GUI 처럼 확인될 때까지 재전송)
        sent_t = now
        for rid, g in goals.items():
            acked = st.get(rid, {}).get('cmd_id') == g['id']
            if not acked and not (rid in results and not results[rid]['ok']):
                goal_pub.publish(String(data=json.dumps(g)))
    if done_now():
        break
arrived = all(d['state'] == 'ARRIVED' for r, d in st.items() if r not in reject) and len(st) == 2
rej_ok = all(r in results and not results[r]['ok'] and st.get(r, {}).get('state') == 'IDLE' for r in reject)
plan_ok = all(results.get(r, {}).get('ok') for r in goals if r not in reject)
coll = sum(d.get('collisions', 0) for d in st.values())
enc = bool(seen_enc)
ok = arrived and rej_ok and plan_ok and coll == 0 and enc == expect
print(f'[{sc}] {"PASS" if ok else "FAIL"}: 도착={arrived} 충돌={coll} 조정={enc}(기대 {expect})'
      + (f' 경로계획={plan_ok}' if goals else '') + (f' 거부={rej_ok}' if reject else '')
      + f' 시간={time.monotonic() - t0:.1f}s')
sys.exit(0 if ok else 1)
