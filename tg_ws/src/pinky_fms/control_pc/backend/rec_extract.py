"""주행 기록(bag) → 분석용 텍스트. recorder.py 가 기록을 끝낸 뒤 ROS 환경에서 실행한다.

  python3 rec_extract.py <기록 폴더> [시작 시각(epoch s)]

만드는 파일: rosout.log, issues.log, missions.log, states.log, video_<로봇>.mp4 (/<ns>/camera/rec 가 있을 때), summary.txt
"""
import json
import re
import shutil
import subprocess
import sys
import tempfile
from collections import Counter
from datetime import datetime
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LEVEL = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
# 상태가 바뀔 때만 한 줄씩 남기는 문자열(JSON) 토픽: 접미사 → 비교에 쓸 키
STATE_TOPICS = {'/fms_status': ('state', 'lcd', 'text'), '/lane_status': ('state', 'phase', 'localized'),
                '/escape_status': ('state', 'result'), '/traffic_state': ('mode', 'msg')}


def ts(ns):
    return datetime.fromtimestamp(ns / 1e9).strftime('%H:%M:%S.%f')[:-3]


def main():
    d = Path(sys.argv[1])
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(d / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cams = {n: n.split('/')[1] for n in types if re.fullmatch(r'/[a-z][a-z0-9_]*/camera/rec', n)}   # 토픽 → 로봇 ID
    wanted = {n for n in types if n == '/rosout' or n == '/fleet/mission_state' or n in cams
              or any(n.endswith(s) for s in STATE_TOPICS)}
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))
    cls = {n: get_message(types[n]) for n in wanted}

    rosout, issues, missions, states = [], [], [], []
    levels, warn_nodes, results = Counter(), Counter(), Counter()
    last_state, counts = {}, Counter()
    tmp = Path(tempfile.mkdtemp(prefix='fms_rec_video_'))
    frames = {}                     # 로봇 → [(시각 ns, 파일)]
    first = last = None
    while reader.has_next():
        topic, raw, t = reader.read_next()
        first = t if first is None else first
        last = t
        counts[topic] += 1
        m = deserialize_message(raw, cls[topic])
        if topic in cams:
            rid = cams[topic]
            st = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec or t     # 찍은 시각 (없으면 받은 시각)
            lst = frames.setdefault(rid, [])
            f = tmp / rid / f'{len(lst):06d}.jpg'
            f.parent.mkdir(exist_ok=True)
            f.write_bytes(bytes(m.data))
            lst.append((st, f))
        elif topic == '/rosout':
            lv = LEVEL.get(m.level, str(m.level))
            line = f'{ts(t)} [{lv:<5}] {m.name}: {m.msg}'
            rosout.append(line)
            levels[lv] += 1
            if m.level >= 30:
                issues.append(line)
                warn_nodes[m.name] += 1
        elif topic == '/fleet/mission_state':
            line = f'{ts(t)} {m.robot_id or "-":<8} {m.mission_id:<16} {m.state:<10} {m.message}'
            missions.append(line)
            if m.state in ('SUCCEEDED', 'FAILED', 'CANCELED'):
                results[m.state] += 1
            if m.state == 'FAILED':
                issues.append(f'{ts(t)} [MISSION] {m.robot_id}: FAILED {m.message}')
        else:
            try:
                data = json.loads(m.data)
                key = next(k for s, ks in STATE_TOPICS.items() if topic.endswith(s) for k in [ks])
                sig = tuple(str(data.get(k)) for k in key) if isinstance(data, dict) else (m.data,)
            except (ValueError, StopIteration, AttributeError):
                data, sig = m.data, (m.data,)
            if last_state.get(topic) != sig:
                last_state[topic] = sig
                states.append(f'{ts(t)} {topic}: {m.data}')

    issues.sort()           # rosout 과 미션 실패를 시각순으로
    for name, lines in (('rosout.log', rosout), ('issues.log', issues), ('missions.log', missions),
                        ('states.log', states)):
        (d / name).write_text('\n'.join(lines) + ('\n' if lines else ''), encoding='utf-8')

    videos = [make_video(d, rid, lst) for rid, lst in sorted(frames.items())]
    shutil.rmtree(tmp, ignore_errors=True)

    dur = (last - first) / 1e9 if first else 0
    s = [f'기록 폴더: {d}',
         f'기간: {ts(first) if first else "-"} ~ {ts(last) if last else "-"} ({dur:.0f} s)',
         f'메모: {(d / "note.txt").read_text(encoding="utf-8").strip() if (d / "note.txt").exists() else ""}',
         '',
         f'ROS 로그 수준별: ' + (', '.join(f'{k} {v}' for k, v in sorted(levels.items())) or '없음'),
         f'미션 결과: ' + (', '.join(f'{k} {v}' for k, v in sorted(results.items())) or '없음'),
         '',
         '경고(WARN 이상)가 많은 노드:']
    s += [f'  {v:5d}  {k}' for k, v in warn_nodes.most_common(15)] or ['  없음']
    s += ['', '읽은 메시지 수:'] + [f'  {v:7d}  {k}' for k, v in sorted(counts.items())]
    s += ['', '주행 영상 (camera/rec, 차선 주행 스택이 떠 있을 때만):'] + ([f'  {v}' for v in videos] or ['  없음'])
    s += ['', '먼저 볼 파일: issues.log (경고·오류·실패 미션), events.log (GUI 조작), states.log (로봇 상태 변화)']
    (d / 'summary.txt').write_text('\n'.join(s) + '\n', encoding='utf-8')


def make_video(d, rid, lst):
    """JPEG 프레임 → video_<로봇>.mp4. 프레임마다 다음 프레임까지 실제 시간만큼 보여 주고 5 fps 로 맞춘다
    (영상 시간 = 실제 시간이라 로그 시각과 맞춰 보기 쉽다. 스택이 꺼진 구간은 정지 화면)"""
    lst.sort(key=lambda x: x[0])
    out = d / f'video_{rid}.mp4'
    concat = lst[0][1].parent / 'list.txt'
    with open(concat, 'w') as f:
        for (t_ns, p), nxt in zip(lst, lst[1:] + [(lst[-1][0] + 200_000_000, None)]):
            f.write(f"file '{p}'\nduration {max(0.001, (nxt[0] - t_ns) / 1e9):.3f}\n")
        f.write(f"file '{lst[-1][1]}'\n")              # concat 은 마지막 duration 을 쓰려면 마지막 파일을 한 번 더 적어야 한다
    r = subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'concat', '-safe', '0', '-i', str(concat),
                        '-vf', 'fps=5,pad=ceil(iw/2)*2:ceil(ih/2)*2', '-c:v', 'libx264', '-preset', 'veryfast',
                        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(out)], capture_output=True, text=True)
    span = (lst[-1][0] - lst[0][0]) / 1e9 + 0.2
    if r.returncode != 0 or not out.exists():
        return f'{out.name}: 만들기 실패 ({(r.stderr or "").strip()[-200:]})'
    return (f'{out.name}: {span:.0f} s, 프레임 {len(lst)} (평균 {len(lst) / span:.1f} fps), '
            f'{out.stat().st_size / 1048576:.1f} MB, {ts(lst[0][0])} ~ {ts(lst[-1][0])}')


if __name__ == '__main__':
    main()
