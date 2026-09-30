# pp-flight-test

인천대 송도 이노베이션센터 옆 공터에서 **장애물 배치 → Hybrid PP 경로계획 → PX4 오프보드 비행**을
한 흐름으로 돌리는 도구 모음.

```
1. python3 make_heightmap.py     장애물(박스 3·원통 1) 위경도 입력 → 장애물 heightmap
2. ./run_pp.sh                   출발·도착·경유점 위경도 입력 → Hybrid PP 경로 (NED 경유점)
                                 → offboard/position_offboard_test/config/ 에 자동 저장
3. ros2 run position_offboard_test pp_waypoint_offboard
                                 저장된 경로로 이륙 → 경로 추종 → 착륙
```

---

## 0. 준비

| 필요한 것 | 용도 |
|---|---|
| python3, numpy, scipy, opencv, pillow | 1단계, 그림 |
| Docker 컨테이너 (NVIDIA GPU 보이는 것) + Hybrid PP 휠 | 2단계 |
| ROS 2 Humble + `px4_msgs`(PX4 v1.16) + uXRCE-DDS 에이전트 | 3단계 |

**Hybrid PP 라이브러리(`hybrid_learning_path_planning` 0.3.2)는 이 레포에 없다** (연구실 코드).
A4VAI 워크스페이스의 `pathplanning/vendor/hybrid_learning_path_planning-0.3.2-py3-none-any.whl` 을 쓴다.
컨테이너에 한 번만 설치한다 (호스트에는 아무것도 안 깐다):

```bash
./setup_pp_container.sh <휠 경로>            # 기본 컨테이너: realgazebo (PP_CONTAINER 로 변경)
```

컨테이너 안 `/opt/pp_libs` 에만 설치되고, 컨테이너 시스템 파이썬은 건드리지 않는다.

---

## 1. 장애물 넣기 — `make_heightmap.py`

```bash
python3 make_heightmap.py
```

박스 3개, 원통 1개의 **중심 위경도**를 차례로 묻는다 (`37.38451, 126.65418` 또는 띄어쓰기).

| 장애물 | 크기 | 방향 |
|---|---|---|
| 박스 | 폭 3 m × 길이 6 m × 높이 3 m | 긴 쪽이 공터 짧은 변 방향 (약 37°) |
| 원통 | 반지름 0.8 m, 높이 20 m | — |

크기·개수·방향은 파일 위쪽 상수(`N_BOX`, `BOX_SIZE`, `BOX_YAW_DEG`, `N_CYL`, `CYL_R`, `CYL_H`)에서 바꾼다.
결과는 `out/heightmap_px.npy`, `out/obstacles.json`, `out/preview.png`.
기본 지도(건물만)로 돌아가려면 `out/` 을 지운다.

## 2. 경로 계획 — `run_pp.sh`

```bash
./run_pp.sh                       # 출발 → 도착 → 경유점 개수(0 가능) → 경유점 순으로 묻는다
PP_Z_MAX=5 ./run_pp.sh            # 최대 고도 5 m 로 제한
```

| 옵션 | 기본 | 의미 |
|---|---|---|
| `PP_Z_MIN`, `PP_Z_MAX` | 3, 25 | 비행 고도 범위 [m, 지면 기준] |
| `PP_LAYERS` | 1 m 간격 | 고도 층 수. PP 는 이 층들 중에서만 고도를 고른다 |
| `PP_RADIUS` | 1.0 | 기체 수평 반경 [m]. 장애물을 이만큼 부풀려 피한다 |

출력 (`out/`):

| 파일 | 내용 |
|---|---|
| `pp_goals_ned.csv` | **오프보드용** 경유점. 출발 지점 원점, x=북, y=동, z=아래(−고도) [m] |
| `pp_goals_enu.csv` | 같은 점 ENU (x=동, y=북, z=위) |
| `pp_path.csv` | 경로점 + 위경도 |
| `pp_route.png` | 지도 위 경로 (색 = 고도) + 옆면 고도 그래프 |

`pp_goals_ned.csv`, `pp_path.csv`, `pp_result.json` 은 **`offboard/position_offboard_test/config/` 에 자동 복사**된다.
3단계 노드는 이 파일을 기본으로 읽는다.

직선 경로만 빠르게 보고 싶으면 `python3 site_map.py` (PP 없이 구간별 장애물 통과 여부만 검사).

## 3. 비행 — `offboard/position_offboard_test`

JOCIIIII/PX4-Offboard-Tests 의 `position_offboard_test` 패키지에 **`pp_waypoint_offboard` 노드를 추가**한 것.
원래 예제(`offboard_control`, 2 m 이륙 → 앞으로 1 m)도 그대로 들어 있다.

```bash
# 워크스페이스 src/ 에 넣고 (심볼릭 링크 권장: PP 결과가 바뀌어도 다시 빌드 안 해도 됨)
ln -s $(pwd)/offboard/position_offboard_test ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install --packages-select position_offboard_test
ros2 run position_offboard_test pp_waypoint_offboard
```

동작: 위치 유효·사전점검 통과 확인 → 시동·오프보드 → `takeoff_altitude` 이륙 → 경로 추종 → 마지막 점 정지 → 착륙.

- 경유점은 **시동 시점 위치 기준 상대 좌표**로 적용한다. PP 원점(출발 지점)에 기체를 두고 시작할 것.
- PP 경유점은 20 m 넘게 떨어져 있을 수 있어서, 목표점을 경로 위로 `cruise_speed` 로 끌고 간다
  (기체가 `max_lead` 보다 뒤처지면 기다린다). 한 번에 먼 점을 던지지 않는다.
- 조종사가 모드를 바꾸거나 PX4 페일세이프가 걸리면 즉시 명령을 멈춘다.
- 경유점이 `max_horizontal_m`(120 m) 밖이거나 고도가 0.5~`max_altitude_m`(30 m) 밖이면 시동을 안 건다.

| 파라미터 | 기본 |
|---|---|
| `waypoint_csv` | 비우면 `$PP_WAYPOINT_CSV` → 패키지 `config/pp_goals_ned.csv` |
| `takeoff_altitude` | 3.0 m |
| `cruise_speed` / `max_lead` | 1.5 m/s / 2.0 m |
| `hover_seconds`, `reach_tolerance` | 3 s, 0.3 m |

**실비행은 RC 오버라이드 가능한 안전 조종사, PX4 페일세이프, 지오펜스, 킬 스위치가 필수.**
이 노드는 보조 안전장치일 뿐이다.

## SITL 시험 — `sitl/sitl_fly.sh`

PX4 SITL + uXRCE-DDS 가 떠 있는 ROS 2 컨테이너(기본 `ros2-env`)에서 위 노드로 비행하고 궤적을 기록한다.

```bash
./sitl/sitl_fly.sh                 # out/ 의 최근 PP 결과로 비행 → sitl/logs/<날짜시각>/
./sitl/sitl_fly.sh out my_test     # 이름 지정
```

`sitl/logs/<이름>/result.png` 에 계획(흰 선)과 실제 비행(빨간 선), `stats.json` 에 횡오차·고도오차·장애물 여유가 남는다.
연속 비행으로 SITL 가상 배터리가 닳으면 PX4 가 시동을 막는다 → SITL 에서만 `COM_LOW_BAT_ACT 0`, `CBRK_SUPPLY_CHK 894281`.

---

## 지도·좌표 기준 (`map/`)

| 파일 | 내용 |
|---|---|
| `innov_heightmap_m.npy` | 기본 heightmap [m], 602×582 px, 0.2229 m/px. 이노베이션센터 20 m, 동쪽 L자 캐노피 10 m |
| `innov_heightmap_meta.json` | 축척, 픽셀↔위경도 변환(`georef.px_to_enu`), 건물 꼭짓점 위경도 |
| `lot_polygon.json`, `lot_mask.npy` | 공터 경계 (RTK 실측 네 꼭짓점) |
| `innov_clean.png` | 배경 위성사진 (글자 제거) |
| `site.geojson` | 공터·건물·L자 다각형 (QGIS·geojson.io 로 확인) |

- 위치 기준: 공터 네 꼭짓점 **RTK 실측** (북 37.3850895, 126.6538218 / 동 37.3845770, 126.6546408 /
  남 37.3841819, 126.6542797 / 서 37.3846990, 126.6534300).
- 축척은 건물 실측(59.1 × 24.6 m)으로 잡았고 RTK 로 독립 검증(차이 0.2%), 사진은 정북(+0.11°).
- 오차: 절대 위치 약 1 m, 건물은 위성사진 지붕 밀림으로 1~3 m 더 틀릴 수 있다. 장애물 여유를 넉넉히.
- 고도는 모두 **공터 지면 기준 [m]** (공터를 평평하다고 가정).

## 예시 (`examples/`)

| 폴더 | 내용 |
|---|---|
| `ex0_zigzag_over` | 지그재그 박스, 출발→도착, 고도 제한 없음 → 박스 위로 넘어감 (최고 9 m) |
| `ex0_zigzag_zmax5`, `ex0_zigzag_zmax7` | 같은 배치, 최대 고도 5 m(옆으로 돎) / 7 m(박스 끝을 넘음) |
| `ex1_wp4` | 같은 배치, 경유점 4개 (106.7 m, 3 m 고도) |
| `ex3_wall` | 박스 3개를 이어 18 m 벽 → 넘어가는 게 최적 |
| `sitl_*` | 위 경로를 SITL 에서 비행한 기록 (횡오차 최대 0.35~0.48 m) |

배치 재현: `python3 make_heightmap.py examples/ex1_wp4/obstacles.json` (그다음 `./run_pp.sh` 에 같은 위경도 입력,
미션 위경도는 각 폴더 `pp_result.json` 의 `mission_latlon`). `drawing.png` 는 배치를 정할 때 그린 스케치.
