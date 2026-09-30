#!/usr/bin/env python3
"""[수정용] 기본 heightmap(건물 20 m, L자 10 m)에 CSV 로 적은 가상 장애물을 넣어 새 heightmap 을 만든다.

실행:  python3 make_heightmap.py                       # 레포 최상위 obstacles.csv 를 읽는다
       python3 make_heightmap.py examples/ex1_wp4/obstacles.csv

CSV 형식 (첫 줄 헤더, # 로 시작하는 줄은 주석):
    type,lat,lon,yaw_deg,width,length,height,radius
    box,37.3846687,126.6541205,,,,,
    box,37.3846173,126.6539944,90,3,6,3,
    cylinder,37.3847271,126.6539560,,,,20,0.8

    type     box(직사각형) 또는 cylinder(원통)
    lat,lon  장애물 중심 위경도 (필수)
    나머지는 비우면 기본값:
      box       width 3 m × length 6 m × height 3 m,
                yaw_deg = 긴 쪽(length) 방향, 북에서 시계방향 [deg]. 비우면 공터 짧은 변 방향(약 37°)
      cylinder  radius 0.8 m, height 20 m
    개수 제한 없음.

저장 (out/ 폴더) — site_map.py, run_pp.sh 가 자동으로 이 파일을 읽는다:
    heightmap_px.npy   사진 픽셀 격자 heightmap (602, 582), 0.2229 m/px
    obstacles.json     넣은 장애물 목록 (위경도·크기·방향)
    preview.png        사진 위 확인 그림 (노랑 = 추가 장애물)

장애물은 픽셀과 조금이라도 겹치면 그 픽셀을 칠한다 (보수적, 사방 최대 0.11 m 크게 들어감).
기본 heightmap(건물만)으로 되돌리려면 out/heightmap_px.npy 와 out/obstacles.json 을 지우면 된다.
"""
import csv
import json
import os
import sys

import numpy as np

from site_map import MAP_DIR, SiteMap, obstacle_height

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "out")
DEFAULT_CSV = os.path.join(HERE, "obstacles.csv")

# CSV 에서 비운 칸의 기본값
BOX_SIZE = (3.0, 6.0, 3.0)     # width, length(긴 쪽), height [m]
CYL_R, CYL_H = 0.8, 20.0       # radius, height [m]


def lot_short_side_bearing(site, folder=MAP_DIR):
    """공터 두 짧은 변(동→남, 서→북)의 평균 방위각 [deg, 북에서 시계방향, 0~180)."""
    R = json.load(open(os.path.join(folder, "lot_polygon.json"), encoding="utf-8"))["corners_latlon_fit"]  # 북,동,남,서
    E = np.array([np.r_[site.latlon_to_enu(a, b)] for a, b in R])
    v = (E[1] - E[2]) + (E[0] - E[3])                      # 남→동 + 서→북 (둘 다 북동쪽을 향함)
    return float(np.degrees(np.arctan2(v[0], v[1])) % 180)


def read_obstacles_csv(path):
    """CSV → 장애물 dict 목록 (비운 칸은 None)."""
    def num(row, k):
        v = (row.get(k) or "").strip()
        return float(v) if v else None
    obs = []
    with open(path, newline="", encoding="utf-8") as f:
        lines = [ln for ln in f if ln.strip() and not ln.lstrip().startswith("#")]
    for i, row in enumerate(csv.DictReader(lines), start=2):
        row = {(k or "").strip().lower(): v for k, v in row.items()}
        t = (row.get("type") or "").strip().lower()
        if t in ("box", "박스", "rect", "rectangle"):
            t = "box"
        elif t in ("cylinder", "cyl", "원통", "circle"):
            t = "cylinder"
        else:
            raise ValueError(f"{path} {i}번째 줄: type 은 box 또는 cylinder (지금: '{t}')")
        lat, lon = num(row, "lat"), num(row, "lon")
        if lat is None or lon is None:
            raise ValueError(f"{path} {i}번째 줄: lat, lon 이 필요합니다")
        obs.append({"type": t, "lat": lat, "lon": lon, "yaw_deg": num(row, "yaw_deg"),
                    "width": num(row, "width"), "length": num(row, "length"),
                    "height": num(row, "height"), "radius": num(row, "radius")})
    return obs


def make(obstacles, folder=MAP_DIR, out_dir=OUT_DIR):
    """obstacles: read_obstacles_csv 결과. 파일을 저장하고 (heightmap, 장애물 목록, SiteMap) 반환."""
    site = SiteMap(folder=folder)
    lot_yaw = lot_short_side_bearing(site, folder)
    obs = []
    for o in obstacles:
        e, n = site._ref_enu(o["lat"], o["lon"])
        d = {"type": o["type"], "lat": o["lat"], "lon": o["lon"], "e_ref": float(e), "n_ref": float(n)}
        if o["type"] == "box":
            size = [o.get("width") or BOX_SIZE[0], o.get("length") or BOX_SIZE[1], o.get("height") or BOX_SIZE[2]]
            yaw = o.get("yaw_deg")
            d.update(size=[float(v) for v in size], yaw_deg=round(float(lot_yaw if yaw is None else yaw), 2))
        else:
            d.update(r=float(o.get("radius") or CYL_R), h=float(o.get("height") or CYL_H))
        obs.append(d)
    h, w = site.H_base.shape
    V, U = np.mgrid[0:h, 0:w]
    E, N = site.px_to_ref_enu(U, V)
    H = np.maximum(site.H_base, obstacle_height(obs, E, N, site.m_per_px / 2))

    os.makedirs(out_dir, exist_ok=True)
    np.save(os.path.join(out_dir, "heightmap_px.npy"), H)
    json.dump({"note": "e_ref, n_ref 는 heightmap 기준 원점(공터 중심) ENU [m]", "obstacles": obs},
              open(os.path.join(out_dir, "obstacles.json"), "w"), ensure_ascii=False, indent=1)
    prev = SiteMap(folder=folder, heightmap=os.path.join(out_dir, "heightmap_px.npy"))
    nb = nc = 0; labels = []
    for o in obs:
        if o["type"] == "box":
            nb += 1; labels.append(f"박스{nb}")
        else:
            nc += 1; labels.append(f"원통{nc}")
    prev.plot([(o["lat"], o["lon"]) for o in obs], os.path.join(out_dir, "preview.png"), labels=labels, connect=False)
    return H, obs, prev


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
    if path.endswith(".json"):                       # 예전 형식(out/obstacles.json 사본)도 받는다
        src = json.load(open(path, encoding="utf-8"))["obstacles"]
        obstacles = [{"type": o["type"], "lat": o["lat"], "lon": o["lon"],
                      "yaw_deg": o.get("yaw_deg"), "width": (o.get("size") or [None] * 3)[0],
                      "length": (o.get("size") or [None] * 3)[1], "height": (o.get("size") or [None, None, o.get("h")])[2],
                      "radius": o.get("r")} for o in src]
    else:
        if not os.path.isfile(path):
            sys.exit(f"장애물 CSV 가 없습니다: {path}\n형식은 python3 make_heightmap.py 파일 맨 위 설명 참고")
        try:
            obstacles = read_obstacles_csv(path)
        except ValueError as e:
            sys.exit(f"✗ {e}")
    H, obs, site = make(obstacles)
    print(f"{path} → 장애물 {len(obs)}개")
    for o in obs:
        inl = site.in_lot(o["lat"], o["lon"])
        kind = (f"박스 {'x'.join(f'{v:g}' for v in o['size'])} m, 방향 {o['yaw_deg']:.1f}°" if o["type"] == "box"
                else f"원통 r {o['r']:g} m, h {o['h']:g} m")
        print(f"  {kind:28s} {o['lat']:.7f}, {o['lon']:.7f}  공터 안 {inl}" + ("" if inl else "  ← 확인 필요"))
    print(f"\n저장: {OUT_DIR}/heightmap_px.npy, obstacles.json, preview.png")
    print("다음: ./run_pp.sh (경로 계획) 또는 python3 site_map.py (직선 경로 검사)")


if __name__ == "__main__":
    main()
