#!/usr/bin/env python3
"""[수정용] 기본 heightmap(건물 20 m, L자 10 m)에 가상 장애물을 넣어 새 heightmap 을 만든다.

실행:  python3 make_heightmap.py
    → 박스 3개, 원통 1개의 중심 위경도를 차례로 묻는다.
       python3 make_heightmap.py examples/ex1_wp4/obstacles.json
    → 저장해 둔 배치를 그대로 다시 만든다.

장애물 크기 (아래 상수에서 바꿀 수 있음):
    박스   폭 3 m × 길이 6 m × 높이 3 m, 긴 쪽이 공터 짧은 변 방향 (약 37°)
    원통   반지름 0.8 m, 높이 20 m

저장 (out/ 폴더) — site_map.py 가 자동으로 이 파일을 읽는다:
    heightmap_px.npy   사진 픽셀 격자 heightmap (602, 582), 0.2229 m/px
    obstacles.json     넣은 장애물 목록 (위경도·크기)
    preview.png        사진 위 확인 그림 (노랑 = 추가 장애물)

장애물은 픽셀과 조금이라도 겹치면 그 픽셀을 칠한다 (보수적, 사방 최대 0.11 m 크게 들어감).
기본 heightmap 으로 되돌리려면 out/heightmap_px.npy 와 out/obstacles.json 을 지우면 된다.
"""
import json
import os

import numpy as np

from site_map import MAP_DIR, SiteMap, ask_ll, obstacle_height

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "out")

N_BOX = 3
BOX_SIZE = (3.0, 6.0, 3.0)     # 폭, 길이(긴 쪽), 높이 [m]
BOX_YAW_DEG = "lot_short"      # 박스 긴 쪽(세로) 방향: 북에서 시계방향 [deg] 숫자,
                               # 또는 "lot_short" = 공터 짧은 변 방향 (RTK 두 짧은 변 평균, 약 37°)
N_CYL = 1
CYL_R, CYL_H = 0.8, 20.0       # 반지름, 높이 [m]


def lot_short_side_bearing(site, folder=MAP_DIR):
    """공터 두 짧은 변(동→남, 서→북)의 평균 방위각 [deg, 북에서 시계방향, 0~180)."""
    R = json.load(open(os.path.join(folder, "lot_polygon.json"), encoding="utf-8"))["corners_latlon_fit"]  # 북,동,남,서
    E = np.array([np.r_[site.latlon_to_enu(a, b)] for a, b in R])
    v = (E[1] - E[2]) + (E[0] - E[3])                      # 남→동 + 서→북 (둘 다 북동쪽을 향함)
    return float(np.degrees(np.arctan2(v[0], v[1])) % 180)


def make(boxes, cylinders, folder=MAP_DIR, out_dir=OUT_DIR):
    """boxes / cylinders: [(lat, lon), ...]. 파일을 저장하고 (heightmap, 장애물 목록) 반환."""
    site = SiteMap(folder=folder)
    yaw = lot_short_side_bearing(site, folder) if BOX_YAW_DEG == "lot_short" else float(BOX_YAW_DEG)
    obs = []
    for la, lo in boxes:
        e, n = site._ref_enu(la, lo)
        obs.append({"type": "box", "lat": la, "lon": lo, "e_ref": float(e), "n_ref": float(n),
                    "size": list(BOX_SIZE), "yaw_deg": round(yaw, 2)})
    for la, lo in cylinders:
        e, n = site._ref_enu(la, lo)
        obs.append({"type": "cylinder", "lat": la, "lon": lo, "e_ref": float(e), "n_ref": float(n),
                    "r": CYL_R, "h": CYL_H})
    h, w = site.H_base.shape
    V, U = np.mgrid[0:h, 0:w]
    E, N = site.px_to_ref_enu(U, V)
    H = np.maximum(site.H_base, obstacle_height(obs, E, N, site.m_per_px / 2))

    os.makedirs(out_dir, exist_ok=True)
    np.save(os.path.join(out_dir, "heightmap_px.npy"), H)
    json.dump({"note": "e_ref, n_ref 는 heightmap 기준 원점(공터 중심) ENU [m]", "obstacles": obs},
              open(os.path.join(out_dir, "obstacles.json"), "w"), ensure_ascii=False, indent=1)
    prev = SiteMap(folder=folder, heightmap=os.path.join(out_dir, "heightmap_px.npy"))
    prev.plot([(o["lat"], o["lon"]) for o in obs], os.path.join(out_dir, "preview.png"),
              labels=[f"박스{i + 1}" for i in range(len(boxes))] + [f"원통{i + 1}" for i in range(len(cylinders))], connect=False)
    return H, obs, prev


def main():
    import sys
    if len(sys.argv) > 1:                      # 예시 재현: python3 make_heightmap.py examples/ex1_wp4/obstacles.json
        obs = json.load(open(sys.argv[1], encoding="utf-8"))["obstacles"]
        boxes = [(o["lat"], o["lon"]) for o in obs if o["type"] == "box"]
        cyls = [(o["lat"], o["lon"]) for o in obs if o["type"] != "box"]
        make(boxes, cyls)
        print(f"{sys.argv[1]} 의 박스 {len(boxes)}개, 원통 {len(cyls)}개로 만들었습니다 → {OUT_DIR}/")
        return
    align = "공터 짧은 변 방향" if BOX_YAW_DEG == "lot_short" else f"북에서 {BOX_YAW_DEG}° 회전"
    print(f"박스 {N_BOX}개 ({BOX_SIZE[0]:g}x{BOX_SIZE[1]:g}x{BOX_SIZE[2]:g} m, 긴 쪽 = {align}), "
          f"원통 {N_CYL}개 (r {CYL_R:g} m, h {CYL_H:g} m) 의 중심 위경도를 입력하세요.")
    boxes = [ask_ll(f"박스 {i + 1} 위도, 경도: ") for i in range(N_BOX)]
    cyls = [ask_ll(f"원통 {i + 1} 위도, 경도: ") for i in range(N_CYL)]
    H, obs, site = make(boxes, cyls)
    print()
    for o in obs:
        inl = site.in_lot(o["lat"], o["lon"])
        kind = f"박스 (방향 {o['yaw_deg']:.1f}°)" if o["type"] == "box" else "원통"
        print(f"  {kind} {o['lat']:.7f}, {o['lon']:.7f}  공터 안 {inl}" + ("" if inl else "  ← 확인 필요"))
    print(f"\n저장: {OUT_DIR}/heightmap_px.npy, obstacles.json, preview.png")
    print("이제 python3 site_map.py 를 실행하면 이 heightmap 으로 경로를 확인합니다.")


if __name__ == "__main__":
    main()
