#!/bin/bash
# 위치 추정 launch(robot_localization.launch.xml) 격리 시험: 도메인 92, 이 PC 안에서만 (실제 스택·로봇과 무관).
# 실물 대신 가짜 로봇(fake_base.py: 제자리 odom TF + 지도로 만든 가짜 라이다)을 쓴다. 참 위치는 코스 설정의 출발점 R1.
#   FMS_WS=<tg_ws 경로> tests/run_localization_test.sh [all|default|robotfree|prefix]
# 확인 (localization_check.py): lifecycle active, 덮어쓴 파라미터 적용, 초기 위치 후 참 위치로 수렴, map→odom TF,
#                               /<ns>/cmd_vel 발행자 0개, Nav2 주행 노드 없음
WHICH=${1:-all}
ROOT=${FMS_WS:-$HOME/pinky}
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/../../../.." && pwd)                                     # tg_ws/src
MAP=$SRC/pinky_fms/maps/mission4_3_clean_1cm.yaml
COURSE=$SRC/pinky_fms_lane/control_pc/pinky_fms_lane/course/mission4_3_clean_1cm.course.yaml
NS=amr_01
source /opt/ros/jazzy/setup.bash; source $ROOT/install/setup.bash
export ROS_DOMAIN_ID=92 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
POSE=$(python3 -c "
import sys
from pinky_fms_lane.course import Course
p = Course.load(sys.argv[1]).points[sys.argv[2]]
print(f'{p.x},{p.y},{p.yaw or 0.0}')" "$COURSE" R1) || { echo "코스 설정에서 R1 을 읽지 못함: $COURSE"; exit 1; }
echo "[loctest] 참 위치(R1) = $POSE, 지도 = $MAP"

PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done; PIDS=(); }
trap cleanup EXIT
FAILED=()

# run_case <이름> <프레임 접두어> [launch 인자...] -- [확인 인자...]
run_case() {
  local name=$1 prefix=$2; shift 2
  local launch_args=() check_args=()
  while [ $# -gt 0 ] && [ "$1" != "--" ]; do launch_args+=("$1"); shift; done
  [ $# -gt 0 ] && shift
  check_args=("$@")
  echo "################ $name ################"
  setsid python3 $HERE/fake_base.py --map $MAP --pose $POSE --namespace $NS --frame-prefix "$prefix" > /tmp/loctest_${name}_base.log 2>&1 & PIDS+=($!)
  setsid ros2 launch pinky_fms_lane_robot robot_localization.launch.xml namespace:=$NS map:=$MAP "${launch_args[@]}" > /tmp/loctest_${name}_launch.log 2>&1 & PIDS+=($!)
  if timeout 150 python3 $HERE/localization_check.py --namespace $NS --pose $POSE --frame-prefix "$prefix" "${check_args[@]}"; then
    echo "=> $name: PASS"
  else
    echo "=> $name: FAIL (로그: /tmp/loctest_${name}_*.log)"; FAILED+=($name)
  fi
  cleanup
}

case $WHICH in all|default) run_case default "" -- --scan scan;; esac
case $WHICH in all|robotfree) run_case robotfree "" scan_topic:=scan_robotfree -- --scan scan_robotfree --expect-filter;; esac
case $WHICH in all|prefix) run_case prefix "$NS/" frame_prefix:=$NS/ -- --scan scan;; esac

echo "================ 결과 ================"
if [ ${#FAILED[@]} -eq 0 ]; then echo "모두 PASS"; exit 0; fi
echo "FAIL: ${FAILED[*]}"; exit 1
