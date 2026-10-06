"""랜덤 시험의 k 번째 사례를 그대로 다시 돌려 로그·궤적을 본다.  python3 tests/replay.py 41 69 113"""
import os, random, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pinky_fms_traffic.gridmap import GridMap
from pinky_fms_traffic.planner import passable
from pinky_fms_traffic.sim import SimRobot, run
from pinky_fms_traffic.traffic import TrafficManager
gm = GridMap(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../../maps/mission4_3_clean_1cm.yaml')); tm = TrafficManager(gm)
mask = passable(gm) & (gm.clear >= 0.10); rr, cc = np.nonzero(mask)
rng = random.Random(1)
want = set(int(a) for a in sys.argv[1:])
for k in range(max(want) + 1):
    while True:
        pts = []
        for _ in range(4):
            i = rng.randrange(len(rr)); pts.append(tuple(gm.world(rr[i], cc[i])))
        d = lambda p, q: np.hypot(p[0] - q[0], p[1] - q[1])
        if min(d(pts[0], pts[2]), d(pts[1], pts[3]), d(pts[0], pts[3]), d(pts[1], pts[2])) > 0.35: break
    sp = (rng.uniform(0.15, 0.2), rng.uniform(0.15, 0.2), rng.uniform(0, 5))
    st = rng.getstate()
    if k in want:
        msgs = []
        rs = [SimRobot('amr_01', pts[0], pts[1], speed=sp[0]), SimRobot('amr_02', pts[2], pts[3], speed=sp[1], delay=sp[2])]
        trace = []
        import pinky_fms_traffic.sim as sim
        r = run(gm, tm, rs, T=400, log=lambda t, i, m: msgs.append((t, i, m)))
        print(f'\n=== case {k}: A {tuple(round(x,2) for x in pts[0])}->{tuple(round(x,2) for x in pts[1])} v={sp[0]:.2f} | B {tuple(round(x,2) for x in pts[2])}->{tuple(round(x,2) for x in pts[3])} v={sp[1]:.2f} delay={sp[2]:.1f}')
        print(r)
        last = None
        for t, i, m in msgs:
            if (i, m) != last:
                print(f'  t={t:5.1f} {i}: {m}'); last = (i, m)
        for x in rs: print('  최종', x.id, x.mode, x.pos.round(2), 'goal', x.goal.round(2), 'hold', x.hold)
