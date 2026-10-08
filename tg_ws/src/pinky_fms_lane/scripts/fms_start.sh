#!/bin/bash
# 관제 시작 (실물): pc_env.sh 로 환경을 불러온 뒤 pinky_fms/scripts/run_real.sh 를 실행한다. source 가 아니라 실행으로 쓴다:
#   bash <tg_ws>/src/pinky_fms_lane/scripts/fms_start.sh [robots.yaml 경로]
# - 관제가 이미 떠 있으면(8000/8080/9090 포트 사용 중, 또는 run_real.sh 프로세스가 있음) 다시 띄우지 않고 끝낸다.
# - 끄기: 이 터미널에서 Ctrl+C (run_real.sh 가 정리한다). 끈 뒤 남은 프로세스·포트가 있으면 알려 준다.
# - run_real.sh 의 환경변수(ESCAPE_MARGIN, ESCAPE_BLOCK, MAP, LANES, FLOOR 등)는 그대로 넘어간다.
#   예: ESCAPE_MARGIN=0.08 bash fms_start.sh

if (return 0 2>/dev/null); then
  echo "[fms_start] 이 파일은 source 가 아니라 실행합니다:  bash ${BASH_SOURCE[0]}" >&2
  return 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/pc_env.sh" || exit 1
RUN_REAL="$FMS_WS/src/pinky_fms/scripts/run_real.sh"
[ -f "$RUN_REAL" ] || { echo "[fms_start] 없음: $RUN_REAL" >&2; exit 1; }

busy_ports() { for p in 8000 8080 9090; do ss -ltn 2>/dev/null | grep -q ":$p " && printf '%s ' $p; done; }
running() { pgrep -af "[r]un_real.sh" | grep -v "fms_start.sh" ; }

ports="$(busy_ports)"
procs="$(running)"
if [ -n "$ports" ] || [ -n "$procs" ]; then
  echo "[fms_start] 관제가 이미 켜져 있습니다. 다시 띄우지 않습니다."
  [ -n "$ports" ] && echo "  사용 중인 포트: $ports"
  [ -n "$procs" ] && echo "  실행 중: $procs"
  echo "  끄려면 그 관제를 띄운 터미널에서 Ctrl+C. 화면: http://localhost:8080"
  exit 0
fi

# Ctrl+C 는 run_real.sh 가 받아 정리한다. 이 스크립트는 정리가 끝날 때까지 기다렸다가 남은 것을 확인한다
trap '' INT
echo "[fms_start] 관제 시작: $RUN_REAL ${1:-}  (끄기: Ctrl+C)"
bash "$RUN_REAL" "$@"
rc=$?
trap - INT

sleep 2
left_ports="$(busy_ports)"
left_procs="$(pgrep -af "[r]osbridge_websocket|[t]raffic_core.launch|[f]leet_traffic|[f]leet_mission|[l]ane_traffic|[l]ane_route|uvicorn app:app|http.server 8080" | cut -c1-120)"
if [ -n "$left_ports" ] || [ -n "$left_procs" ]; then
  echo "[fms_start] 끈 뒤 남은 것이 있습니다 (직접 확인 후 정리):"
  [ -n "$left_ports" ] && echo "  사용 중인 포트: $left_ports"
  [ -n "$left_procs" ] && echo "$left_procs" | sed 's/^/  /'
  echo "  예: kill <PID>   (로봇은 OFF 하지 않습니다. 로봇은 GUI 에서 OFF)"
  exit 1
fi
echo "[fms_start] 관제 종료. 남은 프로세스 없음 (run_real.sh 종료 코드 $rc)"
exit $rc
