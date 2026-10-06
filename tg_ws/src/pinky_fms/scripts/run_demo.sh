#!/bin/bash
# 로봇 없이 관제 GUI 를 확인하는 데모 (가짜 로봇 사용). 종료: Ctrl+C
#   ./run_demo.sh [robots.yaml 경로]   ->  브라우저에서 http://localhost:8080
#   경로를 생략하면 가짜 로봇 2대 설정(robots.mock.yaml)을 쓴다.
set -e
# 저장소 위치는 이 파일 위치에서 구하고, colcon 작업공간(install/ 이 있는 곳)은 FMS_WS(기본 ~/pinky)에서 찾는다.
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WS=${FMS_WS:-$HOME/pinky}
CFG=${1:-$REPO/control_pc/ros/pinky_fms_core/config/robots.mock.yaml}
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
# robots.yaml 의 domain_id / DDS 탐색 방식(unicast 등)을 관제PC 프로세스에도 동일하게 적용
source $REPO/control_pc/fms_env.sh $CFG

PIDS=()
cleanup() {
  echo; echo "[demo] 종료 중..."
  # 가짜 로봇은 백엔드가 띄운 것이므로 OFF API 로 먼저 정리한다
  for id in amr_01 amr_02; do curl -s -m 10 -X POST localhost:8000/robots/$id/off >/dev/null 2>&1 || true; done
  for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null || true; done
  sleep 2
  for p in "${PIDS[@]}"; do kill -TERM -- -"$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

setsid ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/fms_demo_rosbridge.log 2>&1 & PIDS+=($!)
setsid ros2 launch pinky_fms_core fms_core.launch.xml robots_file:=$CFG > /tmp/fms_demo_core.log 2>&1 & PIDS+=($!)
( cd $REPO/control_pc/backend && FMS_ROBOTS_FILE=$CFG exec setsid .venv/bin/python -m uvicorn app:app --host 127.0.0.1 --port 8000 > /tmp/fms_demo_backend.log 2>&1 ) & PIDS+=($!)
( cd $REPO/control_pc/web && exec setsid python3 -m http.server 8080 --bind 127.0.0.1 > /tmp/fms_demo_web.log 2>&1 ) & PIDS+=($!)

echo "[demo] 실행 중: http://localhost:8080   (로그: /tmp/fms_demo_*.log)"
wait
