#!/bin/bash
# PP 경로를 PathFollowing(PF) 으로 인하우스 SITL(PX4-SITL-Runner) 에서 비행하고 로그를 남긴다.
#   사용: ./pf/run_pf_sitl.sh [PP 결과 폴더 (기본: out)] [이름 (기본: pf_날짜시각)]
#   전제: PX4-SITL-Runner 가 설치돼 있을 것 (RUNNER, 기본 ~/PX4-SITL-Runner-runtime,
#         배포 폴더 DEPLOY, 기본 ~/Documents/A4VAI-SITL). PF·path_following_test 가 빌드돼 있을 것.
#   하는 일:
#     1) PP 결과 → PF 경유점 CSV (x=북, y=동, z=고도) → DEPLOY/ROS2/pp_mission/pf_wp.csv
#     2) pf/pp_pf.sh 를 러너 앱으로 복사, ROS2_APP="pp_pf"·기체 iris 로 SITL 실행
#     3) 비행 궤적 기록 → 착륙하면 SITL 을 끄고 러너 설정을 원래대로 되돌림
#     4) sitl/logs/<이름>/ 에 track.csv, node.log, PF 로그, result.png(계획 vs 실제), stats.json
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"; REPO="$(dirname "$HERE")"
SRC="${1:-$REPO/out}"; NAME="${2:-pf_$(date +%Y%m%d_%H%M%S)}"
RUNNER=${RUNNER:-$HOME/PX4-SITL-Runner-runtime}
DEPLOY=${DEPLOY:-$HOME/Documents/A4VAI-SITL}
OUT="$REPO/sitl/logs/$NAME"; mkdir -p "$OUT"
MAXWAIT=${MAXWAIT:-600}

[ -x "$RUNNER/scripts/run.sh" ] || { echo "✗ 러너 없음: $RUNNER"; exit 1; }
docker ps --format '{{.Names}}' | grep -qE '^(px4-env|ros2-env|gz-sim)$' && { echo "✗ SITL 이 이미 떠 있습니다. 먼저 끄세요."; exit 1; }

# 1) 경유점
cp "$SRC/pp_result.json" "$SRC/pp_goals_ned.csv" "$OUT/" 2>/dev/null || cp "$SRC/pp_result.json" "$OUT/"
for f in heightmap_px.npy obstacles.json; do [ -f "$SRC/$f" ] && cp "$SRC/$f" "$OUT/"; done
python3 "$HERE/make_pf_csv.py" "$SRC/pp_result.json" "$OUT/pf_wp.csv"
mkdir -p "$DEPLOY/ROS2/pp_mission" && cp "$OUT/pf_wp.csv" "$DEPLOY/ROS2/pp_mission/pf_wp.csv"

# 2) 러너 앱 설치 + 설정 백업/변경
cp "$HERE/pp_pf.sh" "$RUNNER/scripts/ros2/apps/pp_pf.sh"; chmod +x "$RUNNER/scripts/ros2/apps/pp_pf.sh"
cp "$RUNNER/envs/ros2.env" "$OUT/.ros2.env.bak"; cp "$RUNNER/envs/gazebo-classic.env" "$OUT/.gazebo-classic.env.bak"
restore() {
  docker stop px4-env gz-sim ros2-env qgc-app airsim-binary >/dev/null 2>&1 || true
  cp "$OUT/.ros2.env.bak" "$RUNNER/envs/ros2.env"; cp "$OUT/.gazebo-classic.env.bak" "$RUNNER/envs/gazebo-classic.env"
  rm -f "$OUT/.ros2.env.bak" "$OUT/.gazebo-classic.env.bak" "$OUT/.start" "$RUNNER/scripts/ros2/apps/pp_pf.sh"
  echo "SITL 종료, 러너 설정 원복"
}
trap restore EXIT
sed -i 's/^ROS2_APP=.*/ROS2_APP="pp_pf"/' "$RUNNER/envs/ros2.env"
sed -i 's/^SITL_AIRFRAME=.*/SITL_AIRFRAME="iris"              # DRONE AIRFRAME: x8 (octocopter) | iris (quadrotor)/' "$RUNNER/envs/gazebo-classic.env"

# 3) SITL 실행
touch "$OUT/.start"          # 이 시각 이후에 새로 쓰인 로그만 본다 (이전 로그는 컨테이너 root 소유라 못 지움)
( cd "$RUNNER" && nohup ./scripts/run.sh gazebo-classic-airsim-sitl run > "$OUT/sitl_run.log" 2>&1 & )
echo "[$NAME] SITL 기동 중..."
for i in $(seq 1 90); do grep -aq "PX4 BRIDGED" "$OUT/sitl_run.log" 2>/dev/null && break; sleep 3; done
grep -aq "PX4 BRIDGED" "$OUT/sitl_run.log" || { echo "✗ PX4 브리지 실패 — $OUT/sitl_run.log 확인"; exit 1; }
ENV='source /opt/ros/humble/setup.bash; source /home/user/workspace/ros2/ros2_ws/install/setup.bash'
docker cp "$REPO/sitl/rec_pos.py" ros2-env:/tmp/rec_pos.py
docker exec ros2-env bash -c 'sed -i "s/while time.time()-t0<[0-9]*/while time.time()-t0<900/" /tmp/rec_pos.py; rm -f /tmp/ofb_track.csv'
docker exec -d ros2-env bash -lc "$ENV; python3 /tmp/rec_pos.py"
echo "[$NAME] 비행 중 (PF)..."
T0=$SECONDS; STATE=timeout
while [ $((SECONDS - T0)) -lt $MAXWAIT ]; do
  L="$DEPLOY/ROS2/logs/path_following_test.log"
  if [ "$L" -nt "$OUT/.start" ] && grep -aq "Disarmed" "$L" 2>/dev/null; then STATE=done; break; fi
  grep -aqE "TIMEOUT|died unexpectedly|FATAL" "$OUT/sitl_run.log" && { STATE=fail; break; }
  sleep 5
done
sleep 2
echo "[$NAME] 결과: $STATE ($((SECONDS - T0)) s)"

# 4) 로그 수집
docker cp ros2-env:/tmp/ofb_track.csv "$OUT/track.csv" 2>/dev/null || true
for f in path_following_test.log node_pathfollowing.log; do cp "$DEPLOY/ROS2/logs/$f" "$OUT/" 2>/dev/null || true; done
PFCSV=$(find "$DEPLOY"/ROS2/logs -maxdepth 1 -name "*.csv" -newer "$OUT/.start" 2>/dev/null | head -1); [ -n "$PFCSV" ] && cp "$PFCSV" "$OUT/pf_log.csv"
# plot_sitl.py 는 node.log 의 "start: x= y= z=" 를 원점으로 쓴다 → PF 테스트의 Home 과 첫 궤적 z 로 만든다
python3 - "$OUT" <<'PY'
import re, sys, numpy as np
d = sys.argv[1]
log = open(f"{d}/path_following_test.log", errors="ignore").read()
m = re.search(r"Home=\(([-\d.]+), ([-\d.]+)\)", log)
T = np.genfromtxt(f"{d}/track.csv", delimiter=",", names=True)
hx, hy = (float(m.group(1)), float(m.group(2))) if m else (T["x"][0], T["y"][0])
open(f"{d}/node.log", "w").write(f"start: x={hx:.3f} y={hy:.3f} z={T['z'][0]:.3f}\n" + log)
PY
grep -aE "Armed!|Waypoints sent|COMPLETE|LANDING|Disarmed" "$OUT/path_following_test.log" | sed 's/^.*\]: /  /' | head -8
python3 "$REPO/sitl/plot_sitl.py" "$OUT"
echo "로그: $OUT"
