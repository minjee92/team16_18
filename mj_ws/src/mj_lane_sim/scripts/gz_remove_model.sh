#!/bin/bash
# 원본: tg_ws/src/pinky_fms/simulation/pinky_fms_sim/scripts/gz_remove_model.sh @ fb2f983 (origin/feat/tg-gazebo-course, GPT 작성 pinky_fms_sim). mj_lane_sim 으로 가져옴
# 같은 이름의 가제보 모델이 남아 있으면 지운다 (OFF 뒤 다시 ON 할 때). 없으면 그냥 끝난다.  gz_remove_model.sh <world> <model>
gz service -s /world/$1/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req "name: \"$2\" type: MODEL" >/dev/null 2>&1 || true
