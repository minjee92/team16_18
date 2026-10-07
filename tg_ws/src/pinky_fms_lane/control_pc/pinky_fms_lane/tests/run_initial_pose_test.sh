#!/bin/bash
# 출발점 초기 위치 도구(send_initial_pose) 격리 시험: 도메인 95, 이 PC 안에서만 (실제 스택·로봇과 무관).
# 로봇 대신 가짜 로봇(pinky_fms_lane_robot/tests/fake_base.py) + 위치 추정 launch 를 쓴다. 출발점은 코스 설정의 R1, R2.
#   FMS_WS=<tg_ws 경로> tests/run_initial_pose_test.sh
# 경우: ok          로봇이 R1 에 바르게 있음                 → OK (종료 0) 이어야 함
#       reversed    로봇이 R1 에 반대 방향으로 있음           → 확인 필요 (종료 1, 스캔 일치율 낮음) 이어야 함
#       wrong_start 로봇이 R2 에 있는데 R1 로 보냄           → 확인 필요 (종료 1) 이어야 함
#       no_amcl     위치 추정이 꺼져 있음                     → 확인 필요 (종료 1) 이어야 함
ROOT=${FMS_WS:-$HOME/pinky}
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/../../../.." && pwd)                                     # tg_ws/src
MAP=$SRC/pinky_fms/maps/mission4_3_clean_1cm.yaml
COURSE=$SRC/pinky_fms_lane/control_pc/pinky_fms_lane/course/mission4_3_clean_1cm.course.yaml
FAKE=$SRC/pinky_fms_lane/robot/pinky_fms_lane_robot/tests/fake_base.py
NS=amr_01
source /opt/ros/jazzy/setup.bash; source $ROOT/install/setup.bash
export ROS_DOMAIN_ID=95 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
pose() { python3 -c "
import math, sys
from pinky_fms_lane.course import Course
p = Course.load(sys.argv[1]).points[sys.argv[2]]
yaw = (p.yaw or 0.0) + (math.pi if sys.argv[3] == 'flip' else 0.0)
print(f'{p.x},{p.y},{yaw}')" "$COURSE" $1 ${2:-keep}; }

PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done; PIDS=(); }
trap cleanup EXIT
FAILED=()

# run_case <이름> <기대 종료 코드> <가짜 로봇 참 위치 또는 none> <위치 추정 켬 yes|no>
run_case() {
  local name=$1 want=$2 truth=$3 loc=$4
  echo "################ $name (기대 종료 코드 $want) ################"
  if [ "$truth" != none ]; then
    setsid python3 $FAKE --map $MAP --pose $truth --namespace $NS > /tmp/iptest_${name}_base.log 2>&1 & PIDS+=($!)
  fi
  if [ "$loc" = yes ]; then
    setsid ros2 launch pinky_fms_lane_robot robot_localization.launch.xml namespace:=$NS map:=$MAP > /tmp/iptest_${name}_launch.log 2>&1 & PIDS+=($!)
    sleep 5
  fi
  timeout 90 ros2 run pinky_fms_lane send_initial_pose $NS:R1 --course $COURSE --map $MAP --timeout 8
  local got=$?
  if [ $got -eq $want ]; then echo "=> $name: PASS (종료 코드 $got)"; else echo "=> $name: FAIL (종료 코드 $got, 기대 $want)"; FAILED+=($name); fi
  cleanup
}

run_case ok          0 "$(pose R1)"      yes
run_case reversed    1 "$(pose R1 flip)" yes
run_case wrong_start 1 "$(pose R2)"      yes
run_case no_amcl     1 none              no

echo "================ 결과 ================"
if [ ${#FAILED[@]} -eq 0 ]; then echo "모두 PASS"; exit 0; fi
echo "FAIL: ${FAILED[*]}"; exit 1
