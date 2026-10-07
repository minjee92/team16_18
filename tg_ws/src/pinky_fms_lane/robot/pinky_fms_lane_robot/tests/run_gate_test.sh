#!/bin/bash
# cmd_vel_gate 격리 시험: 도메인 88, 이 PC 안에서만 (실제 스택·로봇과 무관).
#   FMS_WS=<tg_ws 경로> tests/run_gate_test.sh
# 1) 게이트만: 관제 신호 없음 → 0, 입력·신호·초음파가 다 있으면 통과(최대 속도로 자름), 초음파 끊김 → 0, cmd_vel 발행자 겹침 → 0
# 2) robot_lane.launch.xml use_gate:=true 이면 게이트가 /<ns>/cmd_vel_lane 을 받아 /<ns>/cmd_vel 로 낸다 (false 면 게이트 없음)
#    (차선 노드는 PC 에 카메라·모델이 없어 곧 죽는다. 게이트 연결만 본다)
ROOT=${FMS_WS:-$HOME/pinky}
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/../../../.." && pwd)
MAP=$SRC/pinky_fms/maps/mission4_3_clean_1cm.yaml
source /opt/ros/jazzy/setup.bash; source $ROOT/install/setup.bash
export ROS_DOMAIN_ID=88 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done; PIDS=(); }
trap cleanup EXIT
FAILED=()

echo "################ gate ################"
setsid ros2 run pinky_fms_lane_robot cmd_vel_gate --ros-args -r __ns:=/amr_01 -r cmd_vel_in:=cmd_vel_lane \
  -p link_timeout:=1.0 > /tmp/gatetest_gate.log 2>&1 & PIDS+=($!)
python3 - <<'PYEOF' || FAILED+=(gate)
import json, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String
rclpy.init()
n = rclpy.create_node('gate_check', namespace='/amr_01')
last = {'out': None, 'st': None}
n.create_subscription(Twist, 'cmd_vel', lambda m: last.__setitem__('out', (m.linear.x, m.angular.z)), 10)
n.create_subscription(String, 'cmd_vel_gate/status', lambda m: last.__setitem__('st', json.loads(m.data)), 10)
pin = n.create_publisher(Twist, 'cmd_vel_lane', 10)
phb = n.create_publisher(String, '/fleet/lane_heartbeat', 10)
pso = n.create_publisher(Range, 'us_sensor/range', 10)
def run(sec, hb=True, sonar=True, v=0.5, w=0.2):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        t = Twist(); t.linear.x, t.angular.z = v, w
        pin.publish(t)
        if hb: phb.publish(String(data='{"seq": 1}'))
        if sonar:
            r = Range(); r.range, r.max_range = 0.5, 2.0
            pso.publish(r)
        rclpy.spin_once(n, timeout_sec=0.05)
ok = True
def check(name, want_zero, reason=None):
    global ok
    out, st = last['out'], last['st'] or {}
    good = out is not None and ((out == (0.0, 0.0)) == want_zero) and (reason is None or reason in st.get('reasons', []))
    print(f'  {"PASS" if good else "FAIL"} {name}: cmd_vel={out} reasons={st.get("reasons")}')
    ok &= good
t0 = time.monotonic()
while n.count_publishers('cmd_vel') < 1 and time.monotonic() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
run(2.0, hb=False);              check('관제 신호 없음 → 정지', True, 'FMS_LINK')
run(2.0);                        check('모두 정상 → 통과 (0.5 → 0.25 로 자름)', False)
print(f'    통과 값 {last["out"]}'); ok &= last['out'] is not None and abs(last['out'][0] - 0.25) < 1e-6
run(2.0, sonar=False);           check('초음파 끊김 → 정지', True, 'SONAR_TIMEOUT')
run(1.0)
extra = n.create_publisher(Twist, 'cmd_vel', 10)    # 다른 노드가 cmd_vel 을 냄 (키보드 등)
run(2.0);                        check('cmd_vel 겹침 → 정지', True, 'CMD_VEL_CONFLICT')
n.destroy_publisher(extra)
run(2.0);                        check('겹침 해소 → 다시 통과', False)
sys.exit(0 if ok else 1)
PYEOF
cleanup

# PC 에는 로봇 전용 pinky_lamp_control 실행 파일이 없어 robot_lane 이 시작조차 못 한다 → 잠만 자는 가짜를 임시로 앞에 끼운다
FAKE=$(mktemp -d /tmp/gatetest_fake.XXXXXX)
mkdir -p $FAKE/share/ament_index/resource_index/packages $FAKE/share/pinky_lamp_control $FAKE/lib/pinky_lamp_control
touch $FAKE/share/ament_index/resource_index/packages/pinky_lamp_control
printf '#!/bin/bash\nexec sleep 3600\n' > $FAKE/lib/pinky_lamp_control/main_node; chmod +x $FAKE/lib/pinky_lamp_control/main_node
export AMENT_PREFIX_PATH=$FAKE:$AMENT_PREFIX_PATH
trap 'cleanup; rm -rf $FAKE' EXIT
for g in true false; do
  echo "################ robot_lane use_gate:=$g ################"
  setsid ros2 launch pinky_fms_bringup robot_lane.launch.xml namespace:=amr_01 map:=$MAP use_gate:=$g run_adc_sensor:=false > /tmp/gatetest_lane_$g.log 2>&1 & PIDS+=($!)
  sleep 8
  grep -q "Caught exception in launch" /tmp/gatetest_lane_$g.log && { echo "  FAIL launch 오류: $(grep 'Caught exception' /tmp/gatetest_lane_$g.log | head -1)"; FAILED+=(launch_$g); cleanup; continue; }
  # ros2 daemon 은 직전 실행의 노드 목록을 기억할 수 있어 --no-daemon 으로 지금 그래프만 본다
  nodes=$(timeout 15 ros2 node list --no-daemon --spin-time 3 2>&1)
  info=$(timeout 15 ros2 node info --no-daemon --spin-time 3 /amr_01/cmd_vel_gate 2>&1)
  if [ $g = true ]; then
    if echo "$info" | grep -q "/amr_01/cmd_vel_lane" && echo "$info" | grep -q "/amr_01/cmd_vel:"; then echo "  PASS 게이트가 cmd_vel_lane → cmd_vel"; else echo "  FAIL 게이트 연결"; echo "$info" | head -20; FAILED+=(launch_gate); fi
  else
    if echo "$nodes" | grep -q "/amr_01/fms_lane_mission\|/amr_01/amcl" && ! echo "$nodes" | grep -qx "/amr_01/cmd_vel_gate"; then
      echo "  PASS 게이트 없음 (예전 동작)"
    else
      echo "  FAIL use_gate:=false 확인 실패"; echo "$nodes" | head; FAILED+=(launch_nogate)
    fi
  fi
  cleanup
done

echo "================ 결과 ================"
if [ ${#FAILED[@]} -eq 0 ]; then echo "모두 PASS"; exit 0; fi
echo "FAIL: ${FAILED[*]}"; exit 1
