#!/bin/bash
# 좌표 기록 도구(record_course_points) 격리 시험: 도메인 96, 이 PC 안에서만 (실제 스택·로봇과 무관).
# 가짜 로봇(pinky_fms_lane_robot/tests/fake_base.py)을 R1 에 두고, 위치 추정 launch + send_initial_pose 뒤 기록한다.
# 코스 파일은 임시 폴더에 복사해서 쓰므로 저장소의 코스 파일은 바뀌지 않는다.
#   FMS_WS=<tg_ws 경로> tests/run_record_test.sh
# 경우: save     R1 기록 → 경고 없이 저장, robot 파일의 R1 이 참 위치 2 cm 안, 방향 10° 안
#       refuse   로봇은 R1 에 있는데 J 라고 기록 → "지금 값과 … 차이" 경고, n 을 주면 저장 안 함 (파일·백업 그대로)
#       no_amcl  위치 추정이 꺼져 있음 → "준비 안 됨" (종료 1), 파일 안 만듦
ROOT=${FMS_WS:-$HOME/pinky}
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/../../../.." && pwd)                                     # tg_ws/src
MAP=$SRC/pinky_fms/maps/mission4_3_clean_1cm.yaml
COURSE_DIR=$SRC/pinky_fms_lane/control_pc/pinky_fms_lane/course
FAKE=$SRC/pinky_fms_lane/robot/pinky_fms_lane_robot/tests/fake_base.py
NS=amr_01
source /opt/ros/jazzy/setup.bash; source $ROOT/install/setup.bash
export ROS_DOMAIN_ID=96 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
TMP=$(mktemp -d /tmp/rectest.XXXXXX)
cp $COURSE_DIR/mission4_3_clean_1cm.course.yaml $COURSE_DIR/mission4_3_clean_1cm.tape.yaml $TMP/
COURSE=$TMP/mission4_3_clean_1cm.course.yaml
ROBOT=$TMP/mission4_3_clean_1cm.robot.yaml
R1=$(python3 -c "
import sys
from pinky_fms_lane.course import Course
p = Course.load(sys.argv[1]).points['R1']
print(f'{p.x},{p.y},{p.yaw}')" "$COURSE")

PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done; PIDS=(); }
trap 'cleanup; rm -rf $TMP' EXIT
FAILED=()
pass() { echo "=> $1: PASS"; }
fail() { echo "=> $1: FAIL ($2)"; FAILED+=($1); }

echo "################ no_amcl ################"
setsid python3 $FAKE --map $MAP --pose $R1 --namespace $NS > /tmp/rectest_base.log 2>&1 & PIDS+=($!)
sleep 2
printf 'q\n' | timeout 60 ros2 run pinky_fms_lane record_course_points $NS --course $COURSE --map $MAP | tee $TMP/out_no_amcl.txt
code=${PIPESTATUS[1]}
if [ $code -eq 1 ] && grep -q '준비 안 됨' $TMP/out_no_amcl.txt && [ ! -f $ROBOT ]; then pass no_amcl; else fail no_amcl "종료 $code"; fi

echo "################ save ################"
setsid ros2 launch pinky_fms_lane_robot robot_localization.launch.xml namespace:=$NS map:=$MAP > /tmp/rectest_launch.log 2>&1 & PIDS+=($!)
sleep 5
# 가짜 로봇은 움직이지 않아 AMCL 공분산이 보낸 값 근처에 머문다 (실물은 몰고 다니며 줄어듦) → 작게 보낸다
timeout 60 ros2 run pinky_fms_lane send_initial_pose $NS:R1 --course $COURSE --map $MAP --no-image --xy-std 0.02 --yaw-std-deg 3 | tail -2
printf 'R1\n\nq\n' | timeout 90 ros2 run pinky_fms_lane record_course_points $NS --course $COURSE --map $MAP | tee $TMP/out_save.txt
if python3 - "$ROBOT" "$R1" <<'PYEOF'
import math, sys, yaml
d = yaml.safe_load(open(sys.argv[1]))['points']['R1']
x, y, yaw = (float(v) for v in sys.argv[2].split(','))
err = math.hypot(d['x'] - x, d['y'] - y)
dyaw = math.degrees(abs(math.atan2(math.sin(d['yaw'] - yaw), math.cos(d['yaw'] - yaw))))
print(f'R1 기록 오차 {err * 100:.1f} cm, 방향 {dyaw:.1f}°, 일치율 {d.get("match")}, 갱신 {d["samples"]}번')
sys.exit(0 if err < 0.02 and dyaw < 10 and d['samples'] >= 2 else 1)
PYEOF
then pass save; else fail save "robot 파일의 R1 이 없거나 참 위치와 다름"; fi

echo "################ refuse ################"
before=$(md5sum < $ROBOT)
printf 'J\nn\nq\n' | timeout 90 ros2 run pinky_fms_lane record_course_points $NS --course $COURSE --map $MAP | tee $TMP/out_refuse.txt
nbak=$(ls $TMP/*.bak.* 2>/dev/null | wc -l)
if grep -q '지금 값(tape)과' $TMP/out_refuse.txt && grep -q '저장 안 함' $TMP/out_refuse.txt \
   && [ "$(md5sum < $ROBOT)" = "$before" ] && [ $nbak -eq 0 ]; then pass refuse; else fail refuse "백업 $nbak 개"; fi
cleanup

echo "================ 결과 ================"
if [ ${#FAILED[@]} -eq 0 ]; then echo "모두 PASS"; exit 0; fi
echo "FAIL: ${FAILED[*]}"; exit 1
