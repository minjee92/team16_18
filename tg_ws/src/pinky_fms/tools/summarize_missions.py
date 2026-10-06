#!/usr/bin/env python3
"""record_missions.py 가 남긴 JSONL 을 미션별로 요약한다 (ROS 불필요).
  python3 summarize_missions.py ~/pinky/fms_logs/missions_*.jsonl
"""
import json
import sys
from collections import defaultdict

FINAL = ('SUCCEEDED', 'FAILED', 'CANCELED')


def load(paths):
    recs = []
    for p in paths:
        with open(p, encoding='utf-8') as f:
            recs += [json.loads(l) for l in f if l.strip()]
    return sorted(recs, key=lambda r: r['t'])


def main():
    recs = load(sys.argv[1:])
    missions = defaultdict(lambda: {'robot': '', 'states': [], 'start': None, 'end': None, 'final': None, 'min_dist': None, 'msg': ''})
    for r in recs:
        if r['kind'] != 'task':
            continue
        m = missions[r['mission']]
        m['robot'] = r['robot'] or m['robot']
        m['states'].append(r['state'])
        if m['start'] is None and r['state'] in ('ASSIGNED', 'RUNNING'):
            m['start'] = r['t']
        if r['state'] == 'RUNNING' and r['dist'] > 0:
            m['min_dist'] = r['dist'] if m['min_dist'] is None else min(m['min_dist'], r['dist'])
        if r['state'] in FINAL and m['final'] is None:
            m['final'], m['end'], m['msg'] = r['state'], r['t'], r['msg']
    rows = sorted(missions.items(), key=lambda kv: kv[1]['start'] or 0)
    print(f'{"미션":<14}{"로봇":<8}{"결과":<11}{"걸린 시간":>8}  {"가장 가까워진 거리":>14}  비고')
    ok = fail = canc = 0
    per = defaultdict(lambda: [0, 0, 0])
    for mid, m in rows:
        if m['final'] is None:
            res = '(진행중/미종료)'
        else:
            res = m['final']
            idx = {'SUCCEEDED': 0, 'FAILED': 1, 'CANCELED': 2}[res]
            per[m['robot']][idx] += 1
            ok, fail, canc = ok + (idx == 0), fail + (idx == 1), canc + (idx == 2)
        dur = f"{m['end'] - m['start']:.1f}s" if m['start'] and m['end'] else '-'
        md = f"{m['min_dist']:.2f} m" if m['min_dist'] is not None else '-'
        print(f'{mid:<14}{m["robot"]:<8}{res:<11}{dur:>8}  {md:>14}  {m["msg"]}')
    total = ok + fail
    print(f'\n합계: 성공 {ok} · 실패 {fail} · 취소 {canc}' + (f'   → 성공률 {100*ok/total:.0f}% (취소 제외)' if total else ''))
    for robot, (a, b, c) in sorted(per.items()):
        print(f'  {robot}: 성공 {a} · 실패 {b} · 취소 {c}')


if __name__ == '__main__':
    main()
