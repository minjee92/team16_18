# 관제 PC 터미널 환경 불러오기. 실행이 아니라 source 로 쓴다:
#   source <tg_ws>/src/pinky_fms_lane/scripts/pc_env.sh
#
# 하는 일 (여러 번 불러도 같다):
#   ROS Jazzy → pinky 작업공간 → tg_ws install 을 source, FMS_WS·MAP 설정,
#   fms_env.sh 로 robots.yaml 의 ROS_DOMAIN_ID·DDS(유니캐스트) 설정을 적용.
# 바꿀 수 있는 환경변수 (괄호 안이 기본값):
#   FMS_WS (이 파일 위치에서 찾은 tg_ws)  PINKY_WS (~/pinky)
#   MAP ($FMS_WS/src/pinky_fms/maps/mission4_3_clean_1cm.yaml)
#   FMS_ROBOTS_FILE ($FMS_WS/src/pinky_fms/control_pc/ros/pinky_fms_core/config/robots.yaml)
# 주의: ROS_DOMAIN_ID 가 실제 관제 값(robots.yaml, 보통 16)으로 바뀐다. 가제보(run_lane_sim.sh)·격리 시험은
#   스스로 다른 도메인을 export 하므로 이 파일을 불러 둔 터미널에서 돌려도 섞이지 않는다.

if ! (return 0 2>/dev/null); then
  echo "[pc_env] 이 파일은 source 로 불러야 합니다:  source $0" >&2
  exit 1
fi

_pe_fail=0
_pe_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FMS_WS="${FMS_WS:-$(cd "$_pe_here/../../.." && pwd)}"           # scripts → pinky_fms_lane → src → tg_ws
_pe_pinky="${PINKY_WS:-$HOME/pinky}"
export MAP="${MAP:-$FMS_WS/src/pinky_fms/maps/mission4_3_clean_1cm.yaml}"
_pe_robots="${FMS_ROBOTS_FILE:-$FMS_WS/src/pinky_fms/control_pc/ros/pinky_fms_core/config/robots.yaml}"
_pe_fms_env="$FMS_WS/src/pinky_fms/control_pc/fms_env.sh"

for _pe_f in /opt/ros/jazzy/setup.bash "$_pe_pinky/install/setup.bash" "$FMS_WS/install/setup.bash" \
             "$MAP" "$_pe_robots" "$_pe_fms_env"; do
  if [ ! -f "$_pe_f" ]; then
    echo "[pc_env] 없음: $_pe_f" >&2
    _pe_fail=1
  fi
done
[ -f "$FMS_WS/install/setup.bash" ] || echo "[pc_env]   → tg_ws 를 먼저 빌드: cd $FMS_WS && colcon build" >&2

[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "$_pe_pinky/install/setup.bash" ] && source "$_pe_pinky/install/setup.bash"
[ -f "$FMS_WS/install/setup.bash" ] && source "$FMS_WS/install/setup.bash"
if [ -f "$_pe_fms_env" ] && [ -f "$_pe_robots" ]; then
  source "$_pe_fms_env" "$_pe_robots" > /dev/null
fi

echo "[pc_env] FMS_WS=$FMS_WS  MAP=$(basename "$MAP")  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-(없음)}  RMW=${RMW_IMPLEMENTATION:-(기본)}$([ $_pe_fail = 1 ] && echo '  ⚠ 위에 없는 파일이 있음')"
unset _pe_fail _pe_here _pe_pinky _pe_robots _pe_fms_env _pe_f
