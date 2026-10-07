#!/bin/bash
# lane_traffic 격리 시험 (도메인 96, 이 PC 안에서만). 사용자 실제 스택(도메인 16)과 다른 세션 시험 도메인(91~95)은 건드리지 않는다.
#   tests/run_lane_test.sh <headon_top|headon_wall|headon_curve|follow|route_a|route_b|route_reject|route|all>
# route_* : 관제 경로 계획(pinky_fms_lane lane_route)이 붙인 경로를 가짜 로봇이 따라간다 (README "성공하는 시연 순서" A·B·C).
#           목표는 GUI 처럼 /fleet/lane_goal 로 보낸다. pinky_fms_lane 이 빌드돼 있어야 한다 (없으면 건너뜀).
SC=${1:-all}
ROOT=${FMS_WS:-$HOME/pinky}; HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../../../.." && pwd)
LANES=$REPO/control_pc/backend/maps/mission4_3_lanes_1cm/map.yaml
FLOOR=$REPO/control_pc/backend/maps/mission4_3_nolanes_1cm/map.yaml
source /opt/ros/jazzy/setup.bash; source $ROOT/install/setup.bash
export ROS_DOMAIN_ID=96 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
run() {   # 시나리오 a_start a_goal b_start b_goal b_speed 조정기대
  local name=$1 PIDS=()
  setsid ros2 run pinky_fms_traffic lane_traffic --ros-args -p lanes_yaml:=$LANES -p floor_yaml:=$FLOOR > /tmp/lanetest_traffic.log 2>&1 & PIDS+=($!)
  setsid ros2 run pinky_fms_traffic lane_mock_robot --ros-args -p namespace:=amr_01 -p lanes_yaml:=$LANES -p "start:=$2" -p "goal:=$3" -p "peers:=[amr_02]" > /tmp/lanetest_amr_01.log 2>&1 & PIDS+=($!)
  setsid ros2 run pinky_fms_traffic lane_mock_robot --ros-args -p namespace:=amr_02 -p lanes_yaml:=$LANES -p "start:=$4" -p "goal:=$5" -p "peers:=[amr_01]" -p speed:=$6 > /tmp/lanetest_amr_02.log 2>&1 & PIDS+=($!)
  python3 $HERE/lane_scenario.py $name 120 $7 2>&1 | grep -v "Warn\|scipy"
  local rc=${PIPESTATUS[0]}
  for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 1.5
  for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done
  grep -E "마주침|우선권|비켜|통과|복귀|조정 끝|🆘|❌" /tmp/lanetest_traffic.log | sed 's/^/    traffic| /'
  return $rc
}
# 경로 모드: 로봇이 출발점(코스의 R1·R2 등)에서 방향을 보고 기다리다가 lane_cmd 를 받아 간다
CLEAN=$REPO/maps/mission4_3_clean_1cm.yaml
run_route() {   # 시나리오 a_start a_yaw b_start b_yaw 조정기대 lane_scenario 추가 인자...
  local name=$1 PIDS=()
  setsid ros2 run pinky_fms_traffic lane_traffic --ros-args -p lanes_yaml:=$LANES -p floor_yaml:=$FLOOR > /tmp/lanetest_traffic.log 2>&1 & PIDS+=($!)
  setsid ros2 launch pinky_fms_lane lane_route.launch.xml map_yaml:=$CLEAN > /tmp/lanetest_route.log 2>&1 & PIDS+=($!)
  setsid ros2 run pinky_fms_traffic lane_mock_robot --ros-args -p namespace:=amr_01 -p lanes_yaml:=$LANES -p route_mode:=true -p "start:=$2" -p start_yaw:=$3 -p "peers:=[amr_02]" > /tmp/lanetest_amr_01.log 2>&1 & PIDS+=($!)
  setsid ros2 run pinky_fms_traffic lane_mock_robot --ros-args -p namespace:=amr_02 -p lanes_yaml:=$LANES -p route_mode:=true -p "start:=$4" -p start_yaw:=$5 -p "peers:=[amr_01]" > /tmp/lanetest_amr_02.log 2>&1 & PIDS+=($!)
  sleep 3
  local exp=$6; shift 6
  python3 $HERE/lane_scenario.py $name 150 $exp "$@" 2>&1 | grep -v "Warn\|scipy"
  local rc=${PIPESTATUS[0]}
  for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 1.5
  for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done
  grep -E "목표|거부|갈림길" /tmp/lanetest_route.log | sed 's/^/    route| /'
  grep -E "마주침|우선권|비켜|통과|복귀|조정 끝|🆘|❌" /tmp/lanetest_traffic.log | sed 's/^/    traffic| /'
  return $rc
}
HAVE_ROUTE=1; ros2 pkg prefix pinky_fms_lane > /dev/null 2>&1 || { HAVE_ROUTE=0; echo "pinky_fms_lane 이 없어 route_* 시험을 건너뜀"; }
R1="[0.783,0.253]"; R2="[2.0005,0.135]"; PI=3.14
fail=0
case $SC in headon_top|all) run headon_top "[0.30,0.93]" "[1.60,0.93]" "[1.60,0.93]" "[0.30,0.93]" 0.15 yes || fail=1;; esac
case $SC in headon_wall|all) run headon_wall "[0.10,0.24]" "[0.80,0.24]" "[0.80,0.24]" "[0.10,0.24]" 0.15 yes || fail=1;; esac
case $SC in headon_curve|all) run headon_curve "[0.06,0.45]" "[0.60,0.93]" "[0.60,0.93]" "[0.06,0.45]" 0.15 yes || fail=1;; esac
case $SC in follow|all) run follow "[0.30,0.93]" "[1.60,0.93]" "[0.70,0.93]" "[1.90,0.93]" 0.07 no || fail=1;; esac
if [ $HAVE_ROUTE = 1 ]; then
# A: R1(꼬리 끝) → 고리 오른변 위, R2(SE) → 꼬리 왼쪽 세로 (편도). 꼬리에서 마주쳐 한 대가 비켜선다
case $SC in route_a|route|all) run_route route_a $R1 $PI $R2 $PI yes goal=amr_01,2.0,0.80,home goal=amr_02,0.065,0.57,home || fail=1;; esac
# B: R1 → 꼬리 윗길 (0.45, 0.95) 왕복 (목적지에서 U턴해 돌아옴), R2 → 고리 윗변 (J 우회전, 편도). 마주치지 않음
case $SC in route_b|route|all) run_route route_b $R1 $PI $R2 $PI no goal=amr_01,0.45,0.95,go goal=amr_02,1.65,0.95,home || fail=1;; esac
# C: amr_01 목표가 길에서 멀다 → 관제가 이유와 함께 거부, amr_01 은 그대로 대기. amr_02 는 고리 아랫변으로 간다
case $SC in route_reject|route|all) run_route route_reject $R1 $PI $R2 $PI no goal=amr_01,0.50,0.60,go reject=amr_01 goal=amr_02,1.50,0.14,home || fail=1;; esac
fi
exit $fail
