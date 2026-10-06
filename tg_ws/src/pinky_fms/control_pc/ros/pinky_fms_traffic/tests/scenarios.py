"""시나리오 시험:  python3 tests/scenarios.py [map.yaml] [랜덤 시도 횟수]"""
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pinky_fms_traffic.gridmap import GridMap
from pinky_fms_traffic.planner import astar, passable
from pinky_fms_traffic.sim import SimRobot, run
from pinky_fms_traffic.traffic import TrafficManager

MAP = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../../maps/mission4_3_clean_1cm.yaml')
N = int(sys.argv[2]) if len(sys.argv) > 2 else 100
R_CONF = float(sys.argv[3]) if len(sys.argv) > 3 else 0.24
NOISE = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
gm = GridMap(MAP)
import pinky_fms_traffic.traffic as _T
_T.YIELD_HORIZON = float(os.environ.get('HORIZON', _T.YIELD_HORIZON))
tm = TrafficManager(gm, r_conf=R_CONF, r_conf_parked=float(os.environ.get('PARKED', 0.26)))
E, W = (1.97, 0.13), (0.20, 0.15)


def show(name, **kw):
    msgs = []
    r_on = run(gm, tm, kw['specs'](), log=lambda t, i, m: msgs.append(f'  t={t:5.1f} {i}: {m}'), use_traffic=True)
    r_off = run(gm, tm, kw['specs'](), use_traffic=False)
    print(f'\n[{name}]\n  끄면 : {r_off}\n  켜면 : {r_on}')
    for m in msgs[:6]:
        print(m)
    return r_on


show('S1 통로 정면 마주침 (동→서 / 서→동)', specs=lambda: [SimRobot('amr_01', E, W), SimRobot('amr_02', W, E)])
show('S2 같은 방향 (뒤따라감)', specs=lambda: [SimRobot('amr_01', E, W), SimRobot('amr_02', (1.97, 0.5), (0.60, 0.28), delay=3.0)])
show('S3 서 있는 로봇이 통로를 막음', specs=lambda: [SimRobot('amr_01', (0.98, 1.0), (0.98, 1.0)), SimRobot('amr_02', E, W)])
show('S4 목표가 상대 도착지와 같음', specs=lambda: [SimRobot('amr_01', E, W), SimRobot('amr_02', (1.5, 0.9), (0.22, 0.15), delay=2.0)])

show('S5 1차선 통로 안에서 정면 마주침', specs=lambda: [SimRobot('amr_01', (1.22, 1.0), (0.2, 0.15)), SimRobot('amr_02', (0.85, 1.0), E)])
show('S6 1차선 통로 입구 근처에서 거의 동시에 진입', specs=lambda: [SimRobot('amr_01', (1.6, 0.9), (0.2, 0.15)), SimRobot('amr_02', (0.5, 0.85), (1.9, 0.5), delay=1.0)])

show('S7 박스 옆 통로에서 정면으로 붙음 (실물 교착 재현)', specs=lambda: [SimRobot('amr_01', (1.85, 0.62), (2.04, 0.10)), SimRobot('amr_02', (1.90, 0.36), (0.82, 0.15))])
show('S9 박스 옆 통로에서 15 cm 까지 붙음 (실물 18:49 재현)', specs=lambda: [SimRobot('amr_01', (1.88, 0.58), (2.04, 0.10)), SimRobot('amr_02', (1.90, 0.43), (0.82, 0.15))])
show('S8 3대: 정면 마주침 + 통로 앞에 서 있는 로봇', specs=lambda: [SimRobot('amr_01', E, W), SimRobot('amr_02', W, (1.95, 0.40)), SimRobot('amr_03', (1.40, 0.95), (1.40, 0.95))])

# 랜덤: 시작·목표를 무작위로 뽑아 충돌·교착·미완료를 센다
mask = passable(gm) & (gm.clear >= 0.10)
rr, cc = np.nonzero(mask)
rng = random.Random(1)
stat = {'OK': 0, 'DEADLOCK': 0, 'TIMEOUT': 0, 'error': 0}
stat_off = {'collide': 0}
coll_on = 0
bad = []
times_on, waits_on, dist_on = [], [], []
NR = int(os.environ.get('NR', 2))
for k in range(N):
    while True:
        pts = []
        for _ in range(2 * NR):
            i = rng.randrange(len(rr))
            pts.append(tuple(gm.world(rr[i], cc[i])))
        d = lambda p, q: np.hypot(p[0] - q[0], p[1] - q[1])
        starts, goals = pts[0::2], pts[1::2]
        ok = all(d(p, q) > 0.35 for i, p in enumerate(starts + goals) for q in (starts + goals)[i + 1:]
                 if not (p in starts and q in goals and starts.index(p) == goals.index(q)))   # 시작·목표가 서로 겹치는 경우는 어떤 알고리즘도 못 푼다
        if ok:
            break
    mk = lambda: [SimRobot(f'amr_0{j + 1}', starts[j], goals[j], speed=rng.uniform(0.15, 0.2), delay=(rng.uniform(0, 5) if j else 0.0)) for j in range(NR)]
    st = rng.getstate()
    on = run(gm, tm, mk(), T=400, pos_noise=NOISE, seed=k)
    rng.setstate(st)
    off = run(gm, tm, mk(), T=400, use_traffic=False, pos_noise=NOISE, seed=k)
    if 'error' in on:
        stat['error'] += 1
        continue
    stat[on['result']] += 1
    if on['result'] == 'OK':
        times_on.append(on['t']); waits_on.append(sum(on['waited'].values())); dist_on.append(sum(on['dist'].values()))
    coll_on += on['collision_frames'] > 0
    stat_off['collide'] += off.get('collision_frames', 0) > 0
    if on['collision_frames'] > 0 or on['result'] != 'OK':
        bad.append((k, pts, on))
print(f'\n[랜덤 {N}회] 켰을 때 {stat}, 충돌 발생 {coll_on}회   |  끄면 충돌 발생 {stat_off["collide"]}회')
import statistics as st
if times_on:
    print(f'  정상 완료 사례 평균: 전체 소요 {st.mean(times_on):.1f}s, 대기 합 {st.mean(waits_on):.1f}s, 이동거리 합 {st.mean(dist_on):.2f}m')
for b in bad[:5]:
    print('  나쁜 사례', b)
