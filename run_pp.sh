#!/bin/bash
# Hybrid PP 경로계획 (호스트에서 실행).  사용: ./run_pp.sh
#   1) PP 컨테이너(기본 realgazebo)를 켜고 이 레포의 코드·지도를 컨테이너 /tmp/pp_flight 로 복사
#   2) 컨테이너 안에서 pp_inu.py 실행 → 출발·도착·경유점 위경도를 물어보고 GPU 로 계획
#   3) 결과를 out/ 으로 가져와 그림(out/pp_route.png) 저장
#   4) 오프보드 노드가 읽는 위치(offboard/position_offboard_test/config/)에 자동 복사
#
# 옵션 (환경변수):
#   PP_Z_MIN=3 PP_Z_MAX=25   고려할 비행 고도 범위 [m, 지면 기준]
#   PP_LAYERS=23             고도 층 수 (기본: 1 m 간격)
#   PP_RADIUS=1.0            기체 수평 반경 [m] (장애물을 이만큼 피함)
#   PP_CONTAINER=realgazebo  PP 라이브러리가 설치된 컨테이너 (setup_pp_container.sh 로 설치)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
C=${PP_CONTAINER:-realgazebo}
T=/opt/pp_libs
W=/tmp/pp_flight

docker ps --format '{{.Names}}' | grep -qx "$C" || { echo "컨테이너 $C 시작"; docker start "$C" >/dev/null; sleep 2; }
docker exec "$C" test -d $T/hybrid_learning_path_planning || {
  echo "✗ $C 컨테이너에 Hybrid PP 가 없습니다. 먼저 ./setup_pp_container.sh <wheel 경로> 를 실행하세요."; exit 1; }

docker exec -u root "$C" rm -rf $W
docker exec -u root "$C" mkdir -p $W/out $W/map
for f in site_map.py pp_inu.py; do docker cp "$HERE/$f" "$C:$W/$f"; done
for f in innov_heightmap_meta.json innov_heightmap_m.npy lot_mask.npy lot_polygon.json; do
  docker cp "$HERE/map/$f" "$C:$W/map/$f"
done
for f in heightmap_px.npy obstacles.json; do
  [ -f "$HERE/out/$f" ] && docker cp "$HERE/out/$f" "$C:$W/out/$f"
done
docker exec -u root "$C" chown -R 1000:1001 $W

TTY=-i; [ -t 0 ] && TTY=-it
docker exec $TTY -u 1000 \
  -e HOME=/tmp -e PYTHONPATH=$T -e CUDA_PATH=$T/nvidia/cuda_runtime \
  -e LD_LIBRARY_PATH=$T/nvidia/cuda_nvrtc/lib:$T/nvidia/cuda_runtime/lib \
  -e CUPY_CACHE_DIR=/tmp/cupy_cache \
  -e PP_Z_MIN="${PP_Z_MIN:-3}" -e PP_Z_MAX="${PP_Z_MAX:-25}" -e PP_RADIUS="${PP_RADIUS:-1.0}" ${PP_LAYERS:+-e PP_LAYERS=$PP_LAYERS} \
  "$C" python3 $W/pp_inu.py

mkdir -p "$HERE/out"
for f in pp_path.csv pp_goals_enu.csv pp_goals_ned.csv pp_result.json; do
  docker cp "$C:$W/out/$f" "$HERE/out/$f"
done
python3 "$HERE/plot_pp.py" && echo "그림: $HERE/out/pp_route.png"

# PP 결과를 오프보드 노드(pp_waypoint_offboard)가 기본으로 읽는 위치로 복사
OFFBOARD_CFG=${OFFBOARD_CFG:-$HERE/offboard/position_offboard_test/config}
mkdir -p "$OFFBOARD_CFG"
cp "$HERE/out/pp_goals_ned.csv" "$HERE/out/pp_path.csv" "$HERE/out/pp_result.json" "$OFFBOARD_CFG/"
python3 - "$HERE/out/pp_result.json" <<'PY'
import json, sys
r = json.load(open(sys.argv[1])); la, lo = r["origin_latlon"]
print(f"오프보드 미션 갱신: {len(r['path_enu']) - 1}개 경유점, 이륙 지점 {la:.7f}, {lo:.7f} (여기에 기체를 두고 이륙)")
PY
echo "  → $OFFBOARD_CFG/pp_goals_ned.csv"
