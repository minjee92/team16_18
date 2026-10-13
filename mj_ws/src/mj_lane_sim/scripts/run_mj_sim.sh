#!/bin/bash
# mj 시뮬 실행 (터미널마다 하나씩). 내 도메인 95 + GZ_PARTITION=mj_sim + LOCALHOST 로만 돈다. tg_ws 는 source 하지 않는다.
#
#   bash run_mj_sim.sh world             # ① Gazebo 월드 (창이 뜬다. 10초쯤 기다림)
#   bash run_mj_sim.sh robot [x y yaw]   # ② 로봇 + 카메라·라이다·전방 초음파 흉내 (기본: 위쪽 길 1.80 0.10 3.14159)
#   bash run_mj_sim.sh camera            # ③ 카메라 화면 (rqt_image_view)
#   bash run_mj_sim.sh teleop            # ④ 키보드 운전 (먼저 z 를 6~7번 눌러 0.15 m/s 쯤으로. k = 멈춤)
#   bash run_mj_sim.sh drive [옵션...]   # ④ 대신: 내 차선 주행 노드 (Space 출발, Enter 종료). 옵션 예: --dry-run --record
#   bash run_mj_sim.sh stop              # 남은 Gazebo 프로세스 정리 (mj_sim 파티션만)
#
# 바꿀 수 있는 환경 변수: MJ_NS (기본 amr_01), PY (기본 ~/venv/yolo_venv/bin/python, drive 용)
set -o pipefail

# 이 스크립트는 install 에 심볼릭 링크로 설치된다 → 원래 위치(mj_ws/src/mj_lane_sim/scripts)에서 mj_ws 를 찾는다
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
export MJ_WS=${MJ_WS:-$(cd "$HERE/../../.." && pwd)}
NS=${MJ_NS:-amr_01}
PY=${PY:-$HOME/venv/yolo_venv/bin/python}

source /opt/ros/jazzy/setup.bash
source "$HOME/pinky/install/setup.bash"
source "$MJ_WS/install/setup.bash"
export ROS_DOMAIN_ID=95 GZ_PARTITION=mj_sim ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY

cmd=${1:-}
shift || true
case "$cmd" in
  world)
    exec ros2 launch mj_lane_sim sim_world.launch.xml "$@" ;;
  robot)
    x=${1:-1.80}; y=${2:-0.10}; yaw=${3:-3.14159}
    exec ros2 launch mj_lane_sim sim_robot.launch.xml namespace:="$NS" x:="$x" y:="$y" yaw:="$yaw" ;;
  camera)
    exec ros2 run rqt_image_view rqt_image_view "/$NS/camera/image_raw" ;;
  teleop)
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:="/$NS/cmd_vel" ;;
  drive)
    cd "$MJ_WS"
    exec "$PY" src/robot/step2_lane_follow.py --camera-topic --namespace "$NS" \
      --sonar-topic us_sensor/range --use-sim-time "$@" ;;
  stop)
    # pkill 에는 환경 변수로 고르는 옵션이 없다 → GZ_PARTITION=mj_sim 인 gz sim 만 골라 끈다 (팀원 시뮬은 안 건드림)
    n=0
    for pid in $(pgrep -u "$USER" -f "gz sim"); do
      if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "GZ_PARTITION=mj_sim"; then
        kill "$pid" && n=$((n + 1))
      fi
    done
    echo "mj_sim Gazebo 프로세스 ${n}개 정리함" ;;
  *)
    sed -n '2,11p' "$0"; exit 1 ;;
esac
