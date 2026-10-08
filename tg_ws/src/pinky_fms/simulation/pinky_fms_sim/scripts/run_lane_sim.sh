#!/bin/bash
# Gazebo 차선 미션 한 번에 띄우기 (한 대): 월드(화면) → 로봇 → 위치 추정+실물 fms_lane_mission → 관제(lane_traffic, lane_route) → 초기 위치.
# 종료: Ctrl+C (띄운 것을 모두 끈다). 로그: /tmp/lane_sim_*.log
#
#   ros2 run pinky_fms_sim run_lane_sim.sh                     # R1 에 두고 목표를 기다린다
#   GOAL=0.45,0.95,home ros2 run pinky_fms_sim run_lane_sim.sh  # 준비되면 목표까지 보낸다 (home = 편도, go = 왕복)
#
# 설정 (환경변수, 괄호 안이 기본값):
#   MODEL (~/dev_ws/yolo_mission/lane_model_ncnn/best.pt)  PYTHON (~/dev_ws/yolo_sim_venv/bin/python, ultralytics 가 있는 파이썬)
#   START (R1, 코스 출발점 이름: R1 꼬리 끝 / R2 SE)  GUI (true)  DOMAIN (93)  GOAL (없음, "x,y,home|go")
#   FMS_WS (이 스크립트가 설치된 작업공간을 source 한 상태여야 한다)
set -u
MODEL=${MODEL:-$HOME/dev_ws/yolo_mission/lane_model_ncnn/best.pt}
PYTHON=${PYTHON:-$HOME/dev_ws/yolo_sim_venv/bin/python}
START=${START:-R1}
GUI=${GUI:-true}
GOAL=${GOAL:-}
export ROS_DOMAIN_ID=${DOMAIN:-93} GZ_PARTITION=${GZ_PARTITION:-lane_sim_${DOMAIN:-93}} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY

for c in ros2 gz; do command -v $c > /dev/null || { echo "[lane_sim] $c 가 없습니다 (ROS·작업공간을 source 했는지)"; exit 1; }; done
ros2 pkg prefix pinky_fms_lane > /dev/null 2>&1 || { echo "[lane_sim] pinky_fms_lane 이 없습니다 (작업공간 install 을 source)"; exit 1; }
[ -f "$MODEL" ] || [ -d "$MODEL" ] || { echo "[lane_sim] 모델이 없습니다: $MODEL (MODEL= 로 지정)"; exit 1; }
"$PYTHON" -c "import ultralytics" 2>/dev/null || { echo "[lane_sim] $PYTHON 에 ultralytics 가 없습니다 (LANE_MISSION.md 준비 참고)"; exit 1; }

SIM=$(ros2 pkg prefix pinky_fms_sim)/share/pinky_fms_sim
CLEAN=$SIM/maps/mission4_3_clean_1cm.yaml
LANE_SHARE=$(ros2 pkg prefix pinky_fms_lane)/share/pinky_fms_lane
COURSE=$LANE_SHARE/course/mission4_3_clean_1cm.course.yaml
# lane_traffic 지도 (백엔드 지도 폴더). 소스 저장소 위치는 FMS_WS 로 찾는다
MAPS=${FMS_WS:-$HOME/dev_ws/team16_18/tg_ws}/src/pinky_fms/control_pc/backend/maps
[ -f $MAPS/mission4_3_lanes_1cm/map.yaml ] || { echo "[lane_sim] 차선 지도가 없습니다: $MAPS (FMS_WS=<tg_ws> 로 지정)"; exit 1; }
read X Y YAW < <(python3 -c "
import sys
from pinky_fms_lane.course import Course
p = Course.load(sys.argv[1]).points[sys.argv[2]]
print(p.x, p.y, p.yaw or 0.0)" "$COURSE" "$START") || { echo "[lane_sim] 코스에 출발점 $START 이 없습니다"; exit 1; }

PIDS=()
cleanup() {
  echo; echo "[lane_sim] 끄는 중..."
  for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done
  pkill -KILL -f "gz sim.*${GZ_PARTITION}" 2>/dev/null
  exit 0
}
trap cleanup INT TERM
up() { local name=$1; shift; setsid "$@" > /tmp/lane_sim_$name.log 2>&1 & PIDS+=($!); }
wait_log() {   # wait_log <로그 이름> <찾을 글자> <최대 초>
  for _ in $(seq $3); do grep -q "$2" /tmp/lane_sim_$1.log 2>/dev/null && return 0; sleep 1; done
  echo "[lane_sim] $1 이 $3 s 안에 준비되지 않음 (/tmp/lane_sim_$1.log)"; return 1
}

echo "[lane_sim] 도메인 $ROS_DOMAIN_ID, 출발점 $START ($X, $Y, yaw $YAW), 모델 $(basename $MODEL)"
up world ros2 launch pinky_fms_sim sim_world.launch.xml gui:=$GUI
sleep 10
up robot ros2 launch pinky_fms_sim sim_robot.launch.xml namespace:=amr_01 x:=$X y:=$Y yaw:=$YAW nav:=false
wait_log robot "Entity creation successful" 60 || cleanup
up control ros2 run pinky_fms_traffic lane_traffic --ros-args -p lanes_yaml:=$MAPS/mission4_3_lanes_1cm/map.yaml \
  -p floor_yaml:=$MAPS/mission4_3_nolanes_1cm/map.yaml -p "escape_block_rects:=-0.14,-0.07,-0.12,0.17"
up route ros2 launch pinky_fms_lane lane_route.launch.xml map_yaml:=$CLEAN
up mission ros2 launch pinky_fms_sim sim_lane_mission.launch.xml namespace:=amr_01 model_path:=$MODEL python:=$PYTHON
wait_log mission "목적지 대기 중" 120 || cleanup
echo "[lane_sim] 미션 노드 준비됨. 초기 위치를 보낸다"
ros2 run pinky_fms_lane send_initial_pose amr_01:$START --course $COURSE --map $CLEAN --frame-prefix amr_01/ --no-image \
  | grep -E "amcl_pose|일치율|→"

if [ -n "$GOAL" ]; then
  IFS=, read GX GY GCMD <<< "$GOAL"
  echo "[lane_sim] 목표 ($GX, $GY) ${GCMD:-home} 를 보낸다"
  ros2 topic pub -t 3 -r 1 /fleet/lane_goal std_msgs/msg/String \
    "{data: '{\"robot\":\"amr_01\",\"id\":$(date +%s),\"cmd\":\"${GCMD:-home}\",\"x\":$GX,\"y\":$GY}'}" > /dev/null
fi
cat <<EOF
[lane_sim] 준비 끝. 다른 터미널에서 (같은 source, export ROS_DOMAIN_ID=$ROS_DOMAIN_ID):
  목표: ros2 topic pub -t 3 -r 1 /fleet/lane_goal std_msgs/msg/String "{data: '{\"robot\":\"amr_01\",\"id\":<새 번호>,\"cmd\":\"home\",\"x\":0.45,\"y\":0.95}'}"
  상태: ros2 topic echo /amr_01/lane_status
  미션 로그: tail -f /tmp/lane_sim_mission.log | grep -E "관제 경로|갈림길|도착|벗어남"
[lane_sim] 끄려면 Ctrl+C
EOF
tail -f /tmp/lane_sim_mission.log | grep --line-buffered -E "🗺️|🧭|🔀|✅ 갈림길|🏁|🏠|⚠️ 관제 경로" &
PIDS+=($!)
wait
