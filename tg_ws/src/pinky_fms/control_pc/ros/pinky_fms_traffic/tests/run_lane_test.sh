#!/bin/bash
# lane_traffic 격리 시험 (도메인 96, 이 PC 안에서만). 사용자 실제 스택(도메인 16)과 다른 세션 시험 도메인(91~95)은 건드리지 않는다.
#   tests/run_lane_test.sh <headon_top|headon_wall|headon_curve|junction_turn|junction_straight|junction_exit|parked|close_headon|follow|all>
SC=${1:-all}
ROOT=${FMS_WS:-$HOME/pinky}; HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../../../.." && pwd)
LANES=$REPO/control_pc/backend/maps/mission4_3_lanes_1cm/map.yaml
FLOOR=$REPO/control_pc/backend/maps/mission4_3_nolanes_1cm/map.yaml
source /opt/ros/jazzy/setup.bash; source $ROOT/install/setup.bash
export ROS_DOMAIN_ID=96 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
run() {   # 시나리오 a_start a_goal b_start b_goal b_speed 조정기대 [a_출발지연 s]
  local name=$1 PIDS=()
  setsid ros2 run pinky_fms_traffic lane_traffic --ros-args -p lanes_yaml:=$LANES -p floor_yaml:=$FLOOR > /tmp/lanetest_traffic.log 2>&1 & PIDS+=($!)
  setsid ros2 run pinky_fms_traffic lane_mock_robot --ros-args -p namespace:=amr_01 -p lanes_yaml:=$LANES -p "start:=$2" -p "goal:=$3" -p "peers:=[amr_02]" -p "junctions:=$JN" -p start_delay:=${8:-3.0} > /tmp/lanetest_amr_01.log 2>&1 & PIDS+=($!)
  setsid ros2 run pinky_fms_traffic lane_mock_robot --ros-args -p namespace:=amr_02 -p lanes_yaml:=$LANES -p "start:=$4" -p "goal:=$5" -p "peers:=[amr_01]" -p speed:=$6 -p "junctions:=$JN" > /tmp/lanetest_amr_02.log 2>&1 & PIDS+=($!)
  python3 $HERE/lane_scenario.py $name 120 $7 2>&1 | grep -v "Warn\|scipy"
  local rc=${PIPESTATUS[0]}
  for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 1.5
  for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done
   grep -E "마주침|우선권|비켜|통과|복귀|조정 끝|🆘|❌|갈림길|✋|서 있는|🔔|물러" /tmp/lanetest_traffic.log | sed 's/^/    traffic| /'
  return $rc
}
JN="[1.15,0.88]"     # 차선 지도의 T자 갈림길 (lane_traffic 이 자동 추출한 값과 같음)
fail=0
case $SC in headon_top|all) run headon_top "[0.30,0.93]" "[1.60,0.93]" "[1.60,0.93]" "[0.30,0.93]" 0.15 yes || fail=1;; esac
case $SC in headon_wall|all) run headon_wall "[0.10,0.24]" "[0.80,0.24]" "[0.80,0.24]" "[0.10,0.24]" 0.15 yes || fail=1;; esac
case $SC in headon_curve|all) run headon_curve "[0.06,0.45]" "[0.60,0.93]" "[0.60,0.93]" "[0.06,0.45]" 0.15 yes || fail=1;; esac
# T자 갈림길: amr_02 가 먼저 서서 좌회전(서쪽) — 그 출구에 amr_01 이 다가와 대기 → 갈림길 교착 → amr_01 양보 (2026-10-07 23:06 실물 재현)
case $SC in junction_turn|all) run junction_turn "[0.25,0.93]" "[1.80,0.93]" "[1.15,0.40]" "[0.45,0.93]" 0.15 yes || fail=1;; esac
# T자 갈림길: amr_01 이 먼저 서서 직진(동쪽) — amr_02 는 아래에서 대기(hold)했다가 amr_01 이 빠져나가면 출발 (비켜서기 없음 → 조정 기대 no)
case $SC in junction_straight|all) run junction_straight "[0.70,0.93]" "[1.80,0.93]" "[1.15,0.20]" "[0.45,0.93]" 0.15 no || fail=1;; esac
# 서 있는 로봇이 차선을 막음: amr_02 는 (0.62,0.93) 에 미션 없이 서 있고(start=goal → ARRIVED), amr_01 이 그 위를 지나가야 함
#   → amr_01 'obstacle ahead' 2 s → amr_02 비켜서기 → amr_01 통과 → amr_02 제자리 복귀 (pinky-96 요청 2026-10-08)
case $SC in parked|all) run parked "[0.06,0.45]" "[0.95,0.93]" "[0.62,0.93]" "[0.62,0.93]" 0.15 yes || fail=1;; esac
# T자 갈림길 출구 차선 (2026-10-08 23:44 실물 교착 재현): amr_02 가 갈림길을 먼저 차지해 좌회전(서쪽),
#   amr_01 은 늦게 출발해 그 출구 차선(위쪽 띠)으로 동쪽 접근 → 예전: 출구 위에서 hold → 우선 로봇이 그 앞에서 obstacle ahead 교착
#   지금: 출구 차선 0.75 m 안이면 hold 대신 바로 양보
case $SC in junction_exit|all) run junction_exit "[0.25,0.93]" "[1.80,0.93]" "[1.15,0.45]" "[0.45,0.93]" 0.15 yes 6.0 || fail=1;; esac
# 아주 가까이(0.19 m) 정면으로 붙은 상태에서 시작 (실물 2026-10-09 00:12 위치): 예전에는 '비켜설 자리 없음' STUCK
case $SC in close_headon|all) run close_headon "[0.605,0.92]" "[1.80,0.93]" "[0.787,0.962]" "[0.30,0.93]" 0.15 yes || fail=1;; esac
case $SC in follow|all) run follow "[0.30,0.93]" "[1.60,0.93]" "[0.70,0.93]" "[1.90,0.93]" 0.07 no || fail=1;; esac
exit $fail
