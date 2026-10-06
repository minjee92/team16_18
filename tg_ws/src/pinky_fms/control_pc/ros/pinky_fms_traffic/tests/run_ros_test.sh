#!/bin/bash
# 격리된 ROS 환경(도메인 91, 이 PC 안에서만)에서 fleet_traffic 을 시험한다. 사용자의 실제 스택(도메인 16, 8000/8080/9090)은 건드리지 않는다.
#   tests/run_ros_test.sh <headon|follow|parked> [traffic|plain]
SC=${1:-headon}; MODE=${2:-traffic}
ROOT=${FMS_WS:-$HOME/pinky}; HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../../../.." && pwd); MAP=$REPO/maps/mission4_3_clean_1cm.yaml
source /opt/ros/jazzy/setup.bash; source $ROOT/install/setup.bash
export ROS_DOMAIN_ID=91 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done; }
trap cleanup EXIT
ARGS=(map_yaml:=$MAP); [ "$MODE" = plain ] && ARGS=()
RIDS="amr_01 amr_02"; [ "$SC" = three ] && RIDS="amr_01 amr_02 amr_03"
for rid in $RIDS; do
  setsid ros2 run pinky_fms_traffic traffic_mock_robot --ros-args -p namespace:=$rid -p map_yaml:=$MAP > /tmp/trafficmock_$rid.log 2>&1 & PIDS+=($!)
done
setsid ros2 launch pinky_fms_traffic traffic_core.launch.xml "${ARGS[@]}" > /tmp/traffic_core.log 2>&1 & PIDS+=($!)
sleep 6
python3 $HERE/ros_scenario.py $SC 2>&1 | grep -v "Warn\|np\.\|^ *\^\|scipy"
