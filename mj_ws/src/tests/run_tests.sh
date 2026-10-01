#!/usr/bin/env bash
# 회귀 테스트 (순차 실행 — 추론을 동시에 돌리지 않는다)
#
#   bash src/tests/run_tests.sh          빠른 테스트만 (모델·ROS 불필요, 몇 초)
#   bash src/tests/run_tests.sh all      + 느린 테스트 (ncnn 모델·입력 영상·ROS 필요, 약 2~3분)
#
# 환경 변수로 바꿀 수 있는 것:
#   PY         파이썬 (기본 ~/venv/yolo_venv/bin/python: ultralytics, ncnn 설치된 venv)
#   ROS_SETUP  ROS 환경 스크립트 (기본 /opt/ros/jazzy/setup.bash)

cd "$(dirname "$0")/../.." || exit 1          # mj_ws/
PY=${PY:-$HOME/venv/yolo_venv/bin/python}
ROS_SETUP=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
export PYTHONDONTWRITEBYTECODE=1

fast=(test_drive_control.py test_lane_tracker.py test_step1_stats.py)
slow=(test_tracker_parity.py test_step2_input_size.py test_step2_replay.py test_step2_ros_e2e.py)
needs_ros=(test_step2_input_size.py test_step2_replay.py test_step2_ros_e2e.py)

tests=("${fast[@]}")
[ "${1:-}" = all ] && tests+=("${slow[@]}")

failed=()
for t in "${tests[@]}"; do
    echo
    echo "=================== $t"
    if [[ " ${needs_ros[*]} " == *" $t "* ]]; then
        # shellcheck disable=SC1090
        (source "$ROS_SETUP" && "$PY" "src/tests/$t") || failed+=("$t")
    else
        "$PY" "src/tests/$t" || failed+=("$t")
    fi
done

echo
if [ ${#failed[@]} -eq 0 ]; then
    echo "전체 통과 (${#tests[@]}개)"
else
    echo "실패: ${failed[*]}"
    exit 1
fi
