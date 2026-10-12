#!/bin/bash
# 원본: tg_ws/src/pinky_fms/simulation/pinky_fms_sim/scripts/sim_urdf.sh @ fb2f983 (origin/feat/tg-gazebo-course, GPT 작성 pinky_fms_sim). mj_lane_sim 으로 가져와 수정함
# 시뮬레이션용 URDF 생성.  sim_urdf.sh <namespace> [width] [height] [tilt_deg] [hfov_deg] [lens_height_m]
# pinky_description 의 xacro 는 namespace 를 주면 조인트·가제보 토픽에는 접두어를 붙이지만 링크 이름에는 안 붙인다.
# 그래서 <gazebo reference="amr_01/rplidar_link"> 가 실제 링크(rplidar_link)와 어긋나 라이다·바퀴 마찰 설정이 통째로 빠진다.
# (원본 패키지는 고치지 않고) 여기서 링크 참조의 접두어만 벗겨 낸다. 조인트 참조와 토픽 접두어는 그대로 둔다.
# 라이다 노이즈도 0.02 -> 0.006 m 로 낮춘다 (실제 RPLidar C1 수준. 0.02 면 1 cm 지도에서 벽이 8 cm 두꺼워져 통로가 좁아진다).
# mj 수정: 카메라 화각(hfov_deg)과 렌즈 높이(lens_height_m, -1 이면 그대로)도 sim_camera.py 로 넘긴다.
set -euo pipefail
NS=${1:?Usage: sim_urdf.sh namespace}
CAMERA_WIDTH=${2:-640}
CAMERA_HEIGHT=${3:-480}
CAMERA_TILT=${4:-8}
CAMERA_HFOV=${5:-53.5}
CAMERA_LENS_HEIGHT=${6:-0.060}
if [[ ! "$NS" =~ ^[a-zA-Z_][a-zA-Z0-9_]*$ ]]; then
    echo "Invalid robot namespace: $NS" >&2
    exit 2
fi
xacro "$(ros2 pkg prefix pinky_description)/share/pinky_description/urdf/robot.urdf.xacro" namespace:=${NS}/ is_sim:=true cam_tilt_deg:="$CAMERA_TILT" \
 | sed -E "s#<stddev>0.02</stddev>#<stddev>0.006</stddev>#g; s#reference=\"${NS}/(l_wheel|r_wheel|caster_wheel|rplidar_link|front_camera_link|imu_link)\"#reference=\"\1\"#g; s#<link>${NS}/robot_lamp</link>#<link>robot_lamp</link>#g" \
 | python3 "$(dirname "$0")/sim_camera.py" "$CAMERA_WIDTH" "$CAMERA_HEIGHT" --hfov-deg "$CAMERA_HFOV" --lens-height "$CAMERA_LENS_HEIGHT"
