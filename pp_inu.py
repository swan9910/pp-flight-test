#!/usr/bin/env python3
"""[PP 연동] 인천대 공터 heightmap 으로 Hybrid PP(v0.3.2) 경로계획.

run_pp.sh 가 PP 컨테이너(기본 realgazebo) 안에서 이 파일을 실행한다 (직접 실행할 일은 거의 없음).
계획 로직은 A4VAI HybridMissionPlanner.py 와 같다:
  지점마다 필요최소고도 위 레이어부터 시도, 실패하면 한 단계씩 올려 재시도, 구간을 이어 붙인다.
다른 점: c-track 전용 값(고정 affine, 0.6517/0.8264 m/px, 픽셀값 0.1882 m) 대신
         site_map.py 의 RTK 기준 좌표 변환과 미터 단위 heightmap 을 쓴다. ROS 없이 돈다.

입력: 출발, 도착, 경유점 (위경도) — 표준입력으로 묻는다
출력 (out/ 폴더):
    pp_path.csv        경로 점: 동[m], 북[m], 고도[m, 지면 기준], 위도, 경도
    pp_goals_enu.csv   goals ENU (x=동, y=북, z=고도 위(+), 출발 지점 기준, 첫 점 제외)
    pp_goals_ned.csv   goals NED (x=북, y=동, z=아래(-고도), PX4 로컬 좌표 관례, 같은 점)
    pp_result.json     위 내용 + 설정
"""
import csv
import json
import os
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from site_map import SiteMap, ask_int, ask_ll   # noqa: E402

OUT_DIR = os.path.join(HERE, "out")

# ── PP 설정 (필요하면 여기만 바꾸면 된다) ──
GRID_RES = 0.5            # 계획 격자 [m/칸]
HORIZONTAL_RADIUS = 1.0   # 기체 수평 반경 [m]: 장애물을 이만큼 부풀려 피한다 (GPS 오차 여유 포함)
VERTICAL_CLEARANCE = 1.0  # 장애물 위로 지나갈 때 최소 수직 여유 [m]
Z_MIN, Z_MAX = 3.0, 25.0  # 고려할 비행 고도 범위 [m, 지면 기준]
NUM_LAYERS = 23           # 고도 레이어 수 (Z_MIN~Z_MAX 를 등간격, 23 이면 1 m 간격)
Z_MARGIN = 1.0            # 지점 필요최소고도에 더하는 여유 [m] (HybridMissionPlanner 와 같음)
Z_MAX_TRIES = 12
# 환경변수로 덮어쓰기 가능: PP_Z_MIN=5 ./run_pp.sh  (PP_Z_MAX, PP_RADIUS, PP_LAYERS 도 같음)
Z_MIN = float(os.environ.get("PP_Z_MIN", Z_MIN)); Z_MAX = float(os.environ.get("PP_Z_MAX", Z_MAX))
HORIZONTAL_RADIUS = float(os.environ.get("PP_RADIUS", HORIZONTAL_RADIUS))
NUM_LAYERS = int(os.environ.get("PP_LAYERS", round(Z_MAX - Z_MIN) + 1))


def build_config():
    import hybrid_learning_path_planning as h
    cfg = json.load(open(os.path.join(os.path.dirname(h.__file__), "assets", "default.json")))
    cfg["model"]["path"] = os.path.join(os.path.dirname(h.__file__), "assets", "waypoint_unet_v031_256.onnx")
    cfg["map"]["resolution_m_per_pixel"] = GRID_RES
    cfg["grid"]["resolution_xy_m"] = [GRID_RES, GRID_RES]
    cfg["vehicle"].update(horizontal_radius_m=HORIZONTAL_RADIUS, vertical_clearance_m=VERTICAL_CLEARANCE)
    cfg["slicing"].update(z_min_m=Z_MIN, z_max_m=Z_MAX, num_layers=NUM_LAYERS)
    fd, path = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, "w") as f:
        json.dump(cfg, f)
    return cfg, path


class InuPlanner:
    def __init__(self, site: SiteMap):
        from hybrid_learning_path_planning import HybridPlanner
        from hybrid_learning_path_planning.graph_cuda import build_static_graph_cuda, dijkstra_query_dense
        self.site = site
        g = site.enu_grid(GRID_RES)                       # j↑ = 북
        self.e0, self.n0 = g["e0"], g["n0"]
        self.hm = np.flipud(g["height"]).astype(np.float32).copy()   # 행↓ = 남 (영상 좌표)
        self.nrow = self.hm.shape[0]
        cfg, cfg_path = build_config()
        t0 = time.time()
        planner = HybridPlanner.from_config(cfg_path)
        fields = planner._fields(self.hm)
        gen = planner._generate_nodes(fields)
        coll = fields.effective_heightmap.copy(); coll[~fields.valid_center_mask] = np.inf
        risk = planner._risk_field(fields)
        gc = cfg["graph_3d"]; res = tuple(cfg["grid"]["resolution_xy_m"])
        vc = float(cfg["vehicle"]["vertical_clearance_m"]); eps = float(gc["collision_epsilon_m"])
        msr = float(gc["max_soft_risk"]); ang = float(cfg["vehicle"]["max_flight_path_angle_deg"])
        graph = build_static_graph_cuda(gen.nodes, gen.layers, coll, risk, fields.z_levels, res, vc, eps, msr, ang)
        self.query = lambda s, t: dijkstra_query_dense(graph, s, t, coll, risk, fields.z_levels, res, vc, eps, msr, ang)
        self.eff = fields.effective_heightmap; self.z_levels = np.asarray(fields.z_levels, float); self.vc = vc
        print(f"PP 준비 {(time.time() - t0) * 1000:.0f} ms | 격자 {self.hm.shape[1]}x{self.hm.shape[0]} @ {GRID_RES} m | "
              f"노드 {len(gen.nodes)} | 고도 레이어 {[round(float(z), 1) for z in self.z_levels]}")

    # 위경도 ↔ 계획 격자 (col, row)
    def ll_to_px(self, lat, lon):
        e, n = self.site.latlon_to_enu(lat, lon)
        return (float(e) - self.e0) / GRID_RES, (self.nrow - 1) - (float(n) - self.n0) / GRID_RES

    def px_to_en(self, col, row):
        return self.e0 + np.asarray(col) * GRID_RES, self.n0 + ((self.nrow - 1) - np.asarray(row)) * GRID_RES

    def z_candidates(self, px):
        c = int(np.clip(round(px[0]), 0, self.eff.shape[1] - 1)); r = int(np.clip(round(px[1]), 0, self.eff.shape[0] - 1))
        need = float(self.eff[r, c]) + self.vc + Z_MARGIN
        cands = [float(z) for z in self.z_levels if z > need]
        return cands[:Z_MAX_TRIES] or [float(self.z_levels[-1])]

    def plan(self, pts_ll):
        px = [self.ll_to_px(*p) for p in pts_ll]
        zi = [self.z_candidates(p) for p in px]; z_at = [c[0] for c in zi]
        merged = None; t0 = time.time()
        for i in range(len(px) - 1):
            wps = None
            for step in range(Z_MAX_TRIES):
                za = zi[i][min(step, len(zi[i]) - 1)]; zb = zi[i + 1][min(step, len(zi[i + 1]) - 1)]
                q = self.query(np.array([*px[i], za]), np.array([*px[i + 1], zb]))
                if q.success:
                    wps = np.asarray(q.path, float).copy(); z_at[i], z_at[i + 1] = za, zb; break
            if wps is None:
                raise RuntimeError(f"구간 {i + 1} 경로 없음 (모든 고도 실패)")
            print(f"  구간 {i + 1}: 고도 {z_at[i]:.0f} → {z_at[i + 1]:.0f} m, 경로점 {len(wps)}")
            if merged is None:
                merged = wps
            else:
                merged[-1, 2] = wps[0, 2]; merged = np.vstack([merged, wps[1:]])
        print(f"계획 {(time.time() - t0) * 1000:.0f} ms")
        e, n = self.px_to_en(merged[:, 0], merged[:, 1])
        return np.c_[e, n, merged[:, 2]]


def main():
    hm = os.path.join(OUT_DIR, "heightmap_px.npy")
    hm = hm if os.path.exists(hm) else None
    print(f"heightmap: {'장애물 포함' if hm else '기본 (건물·L자만)'}")
    start = ask_ll("출발 지점 위도, 경도: ")
    goal = ask_ll("도착 지점 위도, 경도: ")
    k = ask_int("경유점 개수 (없으면 0): ")
    wps = [ask_ll(f"경유점 {i + 1} 위도, 경도: ") for i in range(k)]
    pts = [start] + wps + [goal]

    site = SiteMap(origin=start, heightmap=hm)
    P = InuPlanner(site).plan(pts)
    lat, lon = site.enu_to_latlon(P[:, 0], P[:, 1])
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    print(f"\n경로점 {len(P)}개, 길이 {seg.sum():.1f} m, 고도 {P[:, 2].min():.1f} ~ {P[:, 2].max():.1f} m")

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "pp_path.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["east_m", "north_m", "alt_m", "lat", "lon"])
        for (e, n, z), la, lo in zip(P, lat, lon):
            w.writerow([f"{e:.3f}", f"{n:.3f}", f"{z:.2f}", f"{la:.8f}", f"{lo:.8f}"])
    with open(os.path.join(OUT_DIR, "pp_goals_enu.csv"), "w") as f:
        f.write("x,y,z\n")
        for e, n, z in P[1:]:
            f.write(f"{e:.2f},{n:.2f},{z:.2f}\n")
    with open(os.path.join(OUT_DIR, "pp_goals_ned.csv"), "w") as f:     # PX4 로컬 NED: x=북, y=동, z=아래(-고도)
        f.write("x_north,y_east,z_down\n")
        for e, n, z in P[1:]:
            f.write(f"{n:.2f},{e:.2f},{-z:.2f}\n")
    json.dump({"mission_latlon": pts, "origin_latlon": list(start), "path_enu": P.round(3).tolist(),
               "path_latlon": np.c_[lat, lon].round(8).tolist(), "length_m": float(seg.sum()),
               "config": {"grid_res": GRID_RES, "horizontal_radius": HORIZONTAL_RADIUS, "vertical_clearance": VERTICAL_CLEARANCE,
                          "z_min": Z_MIN, "z_max": Z_MAX, "num_layers": NUM_LAYERS, "z_margin": Z_MARGIN}},
              open(os.path.join(OUT_DIR, "pp_result.json"), "w"), ensure_ascii=False, indent=1)
    print(f"저장: out/pp_path.csv, out/pp_goals_enu.csv, out/pp_goals_ned.csv, out/pp_result.json")


if __name__ == "__main__":
    main()
