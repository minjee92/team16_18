"""lane_traffic 격리 시험 판정: 두 가짜 차선 로봇이 모두 도착하고 충돌이 없으면 PASS.
   python3 lane_scenario.py <시나리오> <제한시간 s> <조정 기대: yes|no>"""
import json
import sys
import time

import rclpy
from std_msgs.msg import String

sc, limit, expect = sys.argv[1], float(sys.argv[2]), sys.argv[3] == 'yes'
rclpy.init()
n = rclpy.create_node('lane_scenario')
st, last, seen_enc = {}, {}, []
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


for rid in ('amr_01', 'amr_02'):
    n.create_subscription(String, f'/{rid}/lane_status', lambda m, r=rid: on_status(r, m), 10)
n.create_subscription(String, '/fleet/lane_traffic_state', on_state, 10)
while time.monotonic() - t0 < limit:
    rclpy.spin_once(n, timeout_sec=0.1)
    if len(st) == 2 and all(d['state'] == 'ARRIVED' for d in st.values()):
        break
done = len(st) == 2 and all(d['state'] == 'ARRIVED' for d in st.values())
coll = sum(d.get('collisions', 0) for d in st.values())
enc = bool(seen_enc)
ok = done and coll == 0 and enc == expect
print(f'[{sc}] {"PASS" if ok else "FAIL"}: 도착={done} 충돌={coll} 조정={enc}(기대 {expect}) 시간={time.monotonic() - t0:.1f}s')
sys.exit(0 if ok else 1)
