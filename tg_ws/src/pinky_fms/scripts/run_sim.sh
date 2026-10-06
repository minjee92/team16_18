#!/bin/bash
# Gazebo 시뮬레이션으로 관제 전체를 돌려 보는 스크립트 (실물 로봇 없이). 종료: Ctrl+C
#   ./run_sim.sh            ->  가제보 월드(실물 지도 기반) + rosbridge + fleet_mission/fleet_traffic + 백엔드 + 웹
#   브라우저 http://localhost:8080 에서 로봇 ON → 가제보에 로봇이 생기고 Nav2 가 뜬다 → 초기 위치 지정 → 주행
# 실물용 스택(fms_core/traffic 터미널, 백엔드, 웹)이 떠 있으면 포트가 겹치므로 먼저 끈다.
set -e
# 저장소 위치는 이 파일 위치에서 구하고, colcon 작업공간(install/ 이 있는 곳)은 FMS_WS(기본 ~/pinky)에서 찾는다.
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WS=${FMS_WS:-$HOME/pinky}
CFG=$REPO/simulation/pinky_fms_sim/config/robots.sim.yaml
MAP=$REPO/maps/mission4_3_clean_1cm.yaml
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
source $REPO/control_pc/fms_env.sh $CFG
# 모두 이 PC 안: 멀티캐스트 허용 + 참가자 수 한도 상향 (Nav2 2벌이면 기본 한도 9개를 넘는다). RMW 는 실물과 같은 Cyclone 으로 통일
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
unset GZ_PARTITION
for p in 8000 8080 9090; do
  if ss -ltn | grep -q ":$p "; then echo "[sim] 포트 $p 가 이미 사용 중입니다. 실물용 스택(백엔드/웹/rosbridge)을 먼저 끄세요."; exit 1; fi
done

PIDS=()
cleanup() {
  echo; echo "[sim] 종료 중..."
  for id in amr_01 amr_02; do curl -s -m 10 -X POST localhost:8000/robots/$id/off >/dev/null 2>&1 || true; done
  for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null || true; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -TERM -- -"$p" 2>/dev/null || true; done
  pkill -f "gz sim" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

setsid ros2 launch pinky_fms_sim sim_world.launch.xml gui:=${SIM_GUI:-true} > /tmp/fms_sim_world.log 2>&1 & PIDS+=($!)
sleep 6
setsid ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/fms_sim_rosbridge.log 2>&1 & PIDS+=($!)
setsid ros2 launch pinky_fms_traffic traffic_core.launch.xml robots_file:=$CFG map_yaml:=$MAP > /tmp/fms_sim_core.log 2>&1 & PIDS+=($!)
( cd $REPO/control_pc/backend && FMS_ROBOTS_FILE=$CFG exec setsid .venv/bin/python -m uvicorn app:app --host 127.0.0.1 --port 8000 > /tmp/fms_sim_backend.log 2>&1 ) & PIDS+=($!)
( cd $REPO/control_pc/web && exec setsid python3 -m http.server 8080 --bind 127.0.0.1 > /tmp/fms_sim_web.log 2>&1 ) & PIDS+=($!)

echo "[sim] 실행 중: http://localhost:8080   (로그: /tmp/fms_sim_*.log)"
echo "[sim] GUI 에서 지도 mission4_3_clean_1cm 선택 → 로봇 ON (amr_01: 동쪽 (1.97,0.13) 서향 / amr_02: 서쪽 (0.20,0.15) 동향) → 초기 위치 지정 → 주행"
wait
