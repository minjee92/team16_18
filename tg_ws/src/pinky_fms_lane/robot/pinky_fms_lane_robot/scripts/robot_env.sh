# 로봇 SSH 터미널 환경 불러오기. 실행이 아니라 source 로 쓴다:
#   source robot_env.sh amr_01
#
# 하는 일: ROS Jazzy → ~/pinky_pro → ~/team16_18/tg_ws install 을 source 하고,
#   GUI 의 ON 이 띄운 bringup 과 같은 통신 설정(fms_robot_env.sh)을 적용한다.
#   bringup 이 아직 없으면 "GUI 에서 ON 먼저" 를 알리고 작업공간 source 만 한다 (ON 뒤 다시 source).
# 바꿀 수 있는 환경변수: PINKY_PRO_WS (~/pinky_pro), ROBOT_TG_WS (이 파일이 tg_ws 안에 있으면 그 tg_ws, 아니면 ~/team16_18/tg_ws)
# 이 파일만 따로 받아 써도 된다 (통신 설정은 설치된 pinky_fms_lane_robot 의 fms_robot_env.sh 를 쓴다).

if ! (return 0 2>/dev/null); then
  echo "[robot_env] 이 파일은 source 로 불러야 합니다:  source $0 <로봇 ID, 예: amr_01>" >&2
  exit 1
fi

_re_ns="${1:-}"
if [ -z "$_re_ns" ]; then
  echo "[robot_env] 사용법: source robot_env.sh <로봇 ID, 예: amr_01>" >&2
else
  _re_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  _re_tg="$(cd "$_re_here/../../../../.." 2>/dev/null && pwd)"          # scripts → robot pkg → robot → pinky_fms_lane → src → tg_ws
  [ -d "$_re_tg/src/pinky_fms_lane" ] || _re_tg="$HOME/team16_18/tg_ws"
  _re_tg="${ROBOT_TG_WS:-$_re_tg}"
  _re_pro="${PINKY_PRO_WS:-$HOME/pinky_pro}"
  for _re_f in /opt/ros/jazzy/setup.bash "$_re_pro/install/setup.bash" "$_re_tg/install/setup.bash"; do
    if [ -f "$_re_f" ]; then source "$_re_f"; else echo "[robot_env] 없음: $_re_f" >&2; fi
  done

  _re_link="통신 설정 안 함 (GUI 에서 $_re_ns 를 ON 먼저, 그 뒤 다시 source)"
  _re_pidf="/tmp/fms_${_re_ns}.pid"
  _re_envsh="$(ros2 pkg prefix pinky_fms_lane_robot 2>/dev/null)/share/pinky_fms_lane_robot/scripts/fms_robot_env.sh"
  if [ ! -f "$_re_envsh" ]; then
    _re_link="통신 설정 안 함 (fms_robot_env.sh 를 못 찾음: pinky_fms_lane_robot 빌드 확인)"
  elif [ -f "$_re_pidf" ] && kill -0 "$(cat "$_re_pidf")" 2>/dev/null; then
    source "$_re_envsh" "$_re_ns" > /dev/null
    _re_link="ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=${RMW_IMPLEMENTATION:-(기본)} CYCLONEDDS_URI=${CYCLONEDDS_URI:-(없음)}"
  else
    echo "[robot_env] $_re_ns 의 bringup 이 아직 없습니다: GUI 에서 ON 먼저 (작업공간만 source 함)" >&2
  fi
  _re_prefix="$(ros2 pkg prefix pinky_fms_bringup 2>/dev/null || echo '(없음)')"
  case "$_re_prefix" in
    "$_re_tg"/*) _re_which="덮어쓴 설치본" ;;
    "(없음)") _re_which="없음" ;;
    *) _re_which="팀원 설치본 (덮어쓰기 빌드 안 됨)" ;;
  esac
  echo "[robot_env] $_re_ns | pinky_fms_bringup: $_re_prefix ($_re_which) | $_re_link"
fi
unset _re_ns _re_here _re_tg _re_pro _re_f _re_link _re_pidf _re_envsh _re_prefix _re_which
