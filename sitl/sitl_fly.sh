#!/bin/bash
# PP 결과 하나를 PX4 SITL 에서 pp_waypoint_offboard 로 비행시키고 로그를 남긴다.
#   사용: ./sitl/sitl_fly.sh [PP 결과 폴더 (기본: out)] [이름 (기본: 날짜시각)]
#   전제: PX4 SITL + uXRCE-DDS 가 떠 있고 /fmu 토픽이 브리지된 ROS2 컨테이너가 있을 것
#         (인하우스 심이면 envs/ros2.env 의 ROS2_APP="" 로 띄우면 됨), 기본 컨테이너 이름 ros2-env.
#         그 컨테이너에 px4_msgs 가 빌드된 워크스페이스가 있어야 한다 (ROS2_SETUP 로 지정).
#   결과: sitl/logs/<이름>/ 에 track.csv, node.log, result.png, stats.json + 입력 사본
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"; REPO="$(dirname "$HERE")"
SRC="${1:-$REPO/out}"; NAME="${2:-$(date +%Y%m%d_%H%M%S)}"
C=${SITL_CONTAINER:-ros2-env}
ROS2_SETUP=${ROS2_SETUP:-/home/user/workspace/ros2/ros2_ws/install/setup.bash}
OUT="$HERE/logs/$NAME"; mkdir -p "$OUT"
cp "$SRC/pp_goals_ned.csv" "$SRC/pp_result.json" "$OUT/"
for f in heightmap_px.npy obstacles.json; do [ -f "$SRC/$f" ] && cp "$SRC/$f" "$OUT/"; done
[ -f "$SRC/pp_route.png" ] && cp "$SRC/pp_route.png" "$OUT/pp_plan.png"

ENV="source /opt/ros/humble/setup.bash; source $ROS2_SETUP"
docker exec $C bash -c 'pkill -f "[r]ec_pos.py"; pkill -f "[p]p_waypoint_offboard.py"; rm -rf /tmp/ofb_track.csv /tmp/ofb_run.log /tmp/ofb_mission.csv /tmp/ofb_pkg' || true
docker cp "$REPO/offboard/position_offboard_test" $C:/tmp/ofb_pkg      # 빌드 없이 노드 파일을 바로 실행
docker cp "$OUT/pp_goals_ned.csv" $C:/tmp/ofb_mission.csv
docker cp "$HERE/rec_pos.py" $C:/tmp/rec_pos.py
docker exec -d $C bash -lc "$ENV; python3 /tmp/rec_pos.py"
sleep 4
docker exec -d $C bash -lc "$ENV; cd /tmp/ofb_pkg; PP_WAYPOINT_CSV=/tmp/ofb_mission.csv python3 -u position_offboard_test/pp_waypoint_offboard.py > /tmp/ofb_run.log 2>&1"
echo "[$NAME] 비행 중... (Ctrl+C 로 끊어도 비행은 계속됨 — 노드는 컨테이너 안에서 돈다)"
for i in $(seq 1 120); do
  docker exec $C grep -qE "mission complete|ABORT|still armed|NOT arm" /tmp/ofb_run.log 2>/dev/null && break; sleep 4
done
sleep 3
docker exec $C bash -c 'pkill -f "[r]ec_pos.py"; pkill -f "[p]p_waypoint_offboard.py"' || true
docker cp $C:/tmp/ofb_track.csv "$OUT/track.csv"; docker cp $C:/tmp/ofb_run.log "$OUT/node.log"
grep -E "State|ABORT|complete|NOT" "$OUT/node.log" | sed 's/^\[INFO\] \[[0-9.]*\] \[pp_waypoint_offboard\]: /  /'
python3 "$HERE/plot_sitl.py" "$OUT"
echo "로그: $OUT"
