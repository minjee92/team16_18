#!/bin/bash
# 실물 로봇용 관제 스택을 한 번에 띄우는 스크립트 (run_sim.sh 의 실물 버전). 종료: Ctrl+C
#   ./run_real.sh [robots.yaml 경로]   ->  rosbridge + fleet_mission/fleet_traffic + 백엔드 + 웹 (http://localhost:8080)
# robots.yaml 의 domain_id / unicast DDS 설정은 fms_env.sh 가 적용한다.
# 시뮬 스택(run_sim.sh)이 떠 있으면 포트와 amr_01 네임스페이스가 겹치므로 먼저 끈다.
set -e
# 저장소 위치는 이 파일 위치에서 구하고, colcon 작업공간(install/ 이 있는 곳)은 FMS_WS(기본 ~/pinky)에서 찾는다.
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WS=${FMS_WS:-$HOME/pinky}
CFG=${1:-$REPO/control_pc/ros/pinky_fms_core/config/robots.yaml}
MAP=${MAP:-$REPO/maps/mission4_3_clean_1cm.yaml}     # fleet_traffic 충돌 방지용 지도. 비우면 충돌 방지 꺼짐
LANES=${LANES:-$REPO/control_pc/backend/maps/mission4_3_lanes_1cm/map.yaml}     # lane_traffic: 차선 띠 (비우면 차선 마주침 조정 꺼짐)
FLOOR=${FLOOR:-$REPO/control_pc/backend/maps/mission4_3_nolanes_1cm/map.yaml}   # lane_traffic: 바닥·벽
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
source $REPO/control_pc/fms_env.sh $CFG
for p in 8000 8080 9090; do
  if ss -ltn | grep -q ":$p "; then echo "[real] 포트 $p 가 이미 사용 중입니다. 다른 관제 스택(run_sim.sh 등)을 먼저 끄세요."; exit 1; fi
done

PIDS=()
cleanup() {
  echo; echo "[real] 종료 중... (로봇은 OFF 하지 않습니다. 끄려면 GUI 에서 OFF)"
  for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null || true; done
  sleep 2
  for p in "${PIDS[@]}"; do kill -TERM -- -"$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

setsid ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/fms_real_rosbridge.log 2>&1 & PIDS+=($!)
setsid ros2 launch pinky_fms_traffic traffic_core.launch.xml robots_file:=$CFG map_yaml:=$MAP lanes_yaml:=$LANES floor_yaml:=$FLOOR > /tmp/fms_real_core.log 2>&1 & PIDS+=($!)
( cd $REPO/control_pc/backend && FMS_ROBOTS_FILE=$CFG exec setsid .venv/bin/python -m uvicorn app:app --host 127.0.0.1 --port 8000 > /tmp/fms_real_backend.log 2>&1 ) & PIDS+=($!)
( cd $REPO/control_pc/web && exec setsid python3 -m http.server 8080 --bind 127.0.0.1 > /tmp/fms_real_web.log 2>&1 ) & PIDS+=($!)

echo "[real] 실행 중: http://localhost:8080   (설정: $CFG, 로그: /tmp/fms_real_*.log)"
wait
