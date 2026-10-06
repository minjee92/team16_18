#!/usr/bin/env python3
"""도착(SUCCEEDED) 허용 오차 분석 (ROS 불필요). record_missions.py(v2) 의 JSONL 을 읽는다.

  python3 analyze_tolerance.py ~/pinky/fms_logs/missions_*.jsonl [--physical 실측.csv] [--xy 0.05] [--yaw 0.05]

--physical : 사용자가 줄자로 잰 실제 오차 (CSV 한 줄 = 미션ID,로봇,오차_cm  예: mmurw9h274,amr_02,6.5)
--xy/--yaw : 지금 적용 중인 Nav2 도착 허용 오차 (기본 0.05 m / 0.05 rad)

계산:
  AMCL 오차   = 도착한 순간의 AMCL 위치와 목표 좌표의 거리   (Nav2 가 '도착'이라고 본 위치의 오차)
  불확실도 σ  = 도착 순간 AMCL 공분산의 표준편차               (위치추정이 스스로 인정하는 흔들림)
  실제 오차   = 사용자가 줄자로 잰 값                          (있을 때만)
  제안 허용 오차는 이 세 값으로 정한다 (맨 아래).
"""
import argparse
import csv
import glob
import json
import math
import statistics as st
from collections import defaultdict


def pct(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    k = (len(s) - 1) * p / 100
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load(paths):
    recs = []
    for pat in paths:
        for p in glob.glob(pat):
            with open(p, encoding='utf-8') as f:
                recs += [json.loads(l) for l in f if l.strip()]
    return sorted(recs, key=lambda r: r['t'])


def build(recs):
    ms = defaultdict(lambda: {'robot': '', 'goal': None, 'start': None, 'end': None, 'final': None, 'fin': None,
                              'dist': [], 'amcl': []})
    for r in recs:
        mid = r.get('mission')
        k = r['kind']
        if k == 'goal' and mid:
            ms[mid]['goal'] = (r['x'], r['y'], r['yaw']); ms[mid]['robot'] = r['robot'] or ms[mid]['robot']
        elif k == 'task' and mid:
            m = ms[mid]; m['robot'] = r['robot'] or m['robot']
            if r['state'] in ('ASSIGNED', 'RUNNING') and m['start'] is None:
                m['start'] = r['t']
            if r['state'] == 'RUNNING':
                m['dist'].append((r['t'], r['dist']))
            if r['state'] in ('SUCCEEDED', 'FAILED', 'CANCELED') and m['final'] is None:
                m['final'], m['end'] = r['state'], r['t']
        elif k == 'final' and mid:
            ms[mid]['fin'] = r
    # 주행 중 AMCL 샘플을 미션 구간에 붙인다 (접근 시간·불확실도 추이)
    amcl_by_robot = defaultdict(list)
    for r in recs:
        if r['kind'] == 'amcl':
            amcl_by_robot[r['robot']].append(r)
    for m in ms.values():
        if m['start'] and m['end']:
            m['amcl'] = [a for a in amcl_by_robot[m['robot']] if m['start'] <= a['t'] <= m['end']]
    return ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--physical')
    ap.add_argument('--xy', type=float, default=0.05)
    ap.add_argument('--yaw', type=float, default=0.05)
    a = ap.parse_args()

    ms = build(load(a.logs))
    phys = {}
    if a.physical:
        with open(a.physical, encoding='utf-8') as f:
            for row in csv.reader(f):
                if len(row) >= 3 and not row[0].startswith('#'):
                    phys[(row[0].strip(), row[1].strip())] = float(row[2]) / 100.0

    print(f'적용 중인 Nav2 도착 허용 오차: 위치 {a.xy:.3f} m, 방향 {a.yaw:.3f} rad ({math.degrees(a.yaw):.1f}°)\n')
    hdr = f'{"미션":<12}{"로봇":<7}{"결과":<10}{"소요":>6}{"접근시간":>8}  {"AMCL오차":>8} {"방향오차":>7} {"σx":>6} {"σy":>6}  {"실측":>6}  비고'
    print(hdr)
    rows = []
    for mid, m in sorted(ms.items(), key=lambda kv: kv[1]['start'] or 0):
        if not m['final']:
            continue
        dur = (m['end'] - m['start']) if m['start'] else float('nan')
        # 접근 시간: 남은 거리가 0.30 m 이하가 된 뒤 끝날 때까지 (허용 오차가 빡빡할수록 길어진다)
        near = [t for t, d in m['dist'] if 0 < d <= 0.30]
        dwell = (m['end'] - near[0]) if near else float('nan')
        err = yerr = sx = sy = float('nan')
        fin = m['fin']
        if fin and m['goal']:
            gx, gy, gyaw = m['goal']
            err = math.hypot(fin['x'] - gx, fin['y'] - gy)
            yerr = abs(wrap(fin['yaw'] - gyaw))
            sx, sy = math.sqrt(max(fin['cov_xx'], 0)), math.sqrt(max(fin['cov_yy'], 0))
        pe = phys.get((mid, m['robot']), float('nan'))
        note = ''
        if m['final'] == 'SUCCEEDED' and not math.isnan(err):
            if err > a.xy * 1.5: note += f'AMCL 기준으로도 허용 오차 {err/a.xy:.1f}배 '
            if not math.isnan(pe) and pe > a.xy: note += '실제로는 허용 오차 초과 '
        if fin and fin.get('amcl_age', 0) > 3: note += f'(AMCL 값이 {fin["amcl_age"]:.0f}초 전 것) '
        rows.append(dict(mid=mid, robot=m['robot'], final=m['final'], dur=dur, dwell=dwell, err=err, yerr=yerr, sx=sx, sy=sy, pe=pe))
        f = lambda v, fmt: ('-' if math.isnan(v) else format(v, fmt))
        print(f'{mid:<12}{m["robot"]:<7}{m["final"]:<10}{f(dur, ".1f"):>6}{f(dwell, ".1f"):>8}  {f(err, ".3f"):>8} {f(math.degrees(yerr) if not math.isnan(yerr) else yerr, ".0f"):>6}° {f(sx, ".2f"):>6} {f(sy, ".2f"):>6}  {f(pe, ".3f"):>6}  {note}')

    ok = [r for r in rows if r['final'] == 'SUCCEEDED']
    print(f'\n성공 {len(ok)} / 실패 {sum(r["final"]=="FAILED" for r in rows)} / 취소 {sum(r["final"]=="CANCELED" for r in rows)}')
    if not ok:
        return
    errs = [r['err'] for r in ok if not math.isnan(r['err'])]
    sig = [max(r['sx'], r['sy']) for r in ok if not math.isnan(r['sx'])]
    yer = [r['yerr'] for r in ok if not math.isnan(r['yerr'])]
    pes = [r['pe'] for r in ok if not math.isnan(r['pe'])]
    dw = [r['dwell'] for r in ok if not math.isnan(r['dwell'])]

    def line(name, v, unit='m', k=1):
        if v:
            print(f'  {name:<26} 중앙값 {st.median(v)*k:.3f}{unit} · 90% {pct(v,90)*k:.3f}{unit} · 최대 {max(v)*k:.3f}{unit}  (n={len(v)})')
    print('\n분포 (성공한 미션만)')
    line('AMCL 기준 위치 오차', errs)
    line('AMCL 불확실도 σ(큰 축)', sig)
    line('실제 오차(줄자)', pes)
    line('방향 오차', yer, '°', 180 / math.pi)
    if dw:
        print(f'  접근 시간(남은 0.3 m 이후)  중앙값 {st.median(dw):.1f}s · 최대 {max(dw):.1f}s')

    # 제안: 허용 오차가 너무 작으면 위치추정 노이즈를 쫓아 맴돌고(접근 시간이 길어짐), 너무 크면 멀리서도 '도착'이라 한다.
    #  - 아래 세 값 중 가장 큰 값을 기준으로 한다 (그보다 작으면 실제로 도착한 미션도 '미도착' 처리하게 된다)
    print('\n제안 기준값')
    cand = []
    if errs:
        cand.append(('AMCL 기준 도착 오차의 90%', pct(errs, 90)))
    if pes:
        cand.append(('실제(줄자) 오차의 90%', pct(pes, 90)))
    if sig:
        cand.append(('위치 불확실도 σ 의 중앙값', st.median(sig)))
    for name, v in cand:
        print(f'  {name:<26} = {v:.3f} m')
    base = max(v for _, v in cand)
    rec = max(0.05, math.ceil(base / 0.025) * 0.025)
    print(f'\n  → 권장 xy_goal_tolerance ≈ {rec:.3f} m  (현재 {a.xy:.3f} m)')
    if sig and pct(sig, 90) > 2 * rec:
        print(f'  ! 위치 불확실도(90% {pct(sig, 90):.2f} m)가 권장값의 2배를 넘습니다. 허용 오차를 키워도 위치 신뢰도 문제는 남으니 AMCL 수렴(초기 공분산, 맵, 노이즈 파라미터)을 함께 개선해야 합니다.')
    if yer:
        recy = max(0.10, math.ceil(pct(yer, 90) / 0.05) * 0.05)
        print(f'  → 권장 yaw_goal_tolerance ≈ {recy:.2f} rad ({math.degrees(recy):.0f}°)  (현재 {a.yaw:.2f} rad)')
    # 이 허용 오차였다면 어떻게 판정됐을까 (AMCL 기준)
    if errs:
        print('\n허용 오차를 바꾸면 이번 시험의 AMCL 기준 판정이 어떻게 달라지나 (성공한 미션 중 도착 오차가 허용 오차 이내인 비율)')
        for T in (0.05, 0.075, 0.10, 0.15, 0.20):
            print(f'  {T:.3f} m: {100*sum(e <= T for e in errs)/len(errs):.0f}%')


if __name__ == '__main__':
    main()
