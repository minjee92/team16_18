# 로봇 SSH 터미널에서 source 해서 쓴다 (실행하는 게 아니라 현재 셸에 환경변수를 적용).
#   source <이 파일> amr_01
#
# GUI 의 ON 이 로봇에서 띄운 bringup 과 똑같은 통신 설정을 이 셸에 적용한다:
#   ROS_DOMAIN_ID, RMW_IMPLEMENTATION, CYCLONEDDS_URI (유니캐스트일 때 백엔드가 만든 /tmp/fms_cyclonedds_<ns>.xml)
# 값을 따로 적지 않고, 백엔드가 기록한 bringup 프로세스(/tmp/fms_<ns>.pid)의 환경에서 그대로 읽는다.
# 그래서 robots.yaml 의 domain_id·discovery 가 바뀌어도 이 파일은 고칠 필요가 없다.
# 비밀번호·SSH 정보는 읽지도 출력하지도 않는다.

_fms_ns="${1:-}"
_fms_pidf="/tmp/fms_${_fms_ns}.pid"
if [ -z "$_fms_ns" ]; then
  echo "[fms_robot_env] 사용법: source fms_robot_env.sh <로봇 ID, 예: amr_01>" >&2
elif [ ! -f "$_fms_pidf" ] || ! kill -0 "$(cat "$_fms_pidf")" 2>/dev/null; then
  echo "[fms_robot_env] $_fms_ns 의 bringup 이 없습니다 ($_fms_pidf). 관제 PC GUI 에서 먼저 ON 하세요" >&2
else
  _fms_environ="/proc/$(cat "$_fms_pidf")/environ"
  for _fms_k in ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI; do
    _fms_v="$(tr '\0' '\n' < "$_fms_environ" | sed -n "s/^${_fms_k}=//p" | head -1)"
    if [ -n "$_fms_v" ]; then export "${_fms_k}=${_fms_v}"; else unset "$_fms_k"; fi
  done
  unset ROS_LOCALHOST_ONLY
  ros2 daemon stop >/dev/null 2>&1 || true      # 예전 설정으로 떠 있는 ros2 CLI 데몬을 새 설정으로 다시 띄우게
  echo "[fms_robot_env] $_fms_ns: ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-(기본)} CYCLONEDDS_URI=${CYCLONEDDS_URI:-(없음: 멀티캐스트)}"
fi
unset _fms_ns _fms_pidf _fms_environ _fms_k _fms_v
