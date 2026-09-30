#!/usr/bin/env python3
"""SITL 비행 기록(track.csv, node.log)과 PP 계획(pp_result.json)을 겹쳐 그림·지표를 만든다.
usage: plot_sitl.py <run_dir>   (run_dir 안에 pp_result.json, track.csv, node.log, heightmap_px.npy)
출력: <run_dir>/result.png, <run_dir>/stats.json
"""
import json
import os
import re
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from site_map import SiteMap  # noqa: E402

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


def main(run_dir, zoom=2):
    R = json.load(open(os.path.join(run_dir, "pp_result.json"), encoding="utf-8"))
    T = np.genfromtxt(os.path.join(run_dir, "track.csv"), delimiter=",", names=True)
    log = open(os.path.join(run_dir, "node.log")).read()
    sx, sy, sz = map(float, re.search(r"start: x=(\S+) y=(\S+) z=(\S+)", log).groups())
    fly = T["armed"] > 0
    t = T["t"] - T["t"][fly][0]
    N, E, U = T["x"] - sx, T["y"] - sy, -(T["z"] - sz)
    P = np.array(R["path_enu"])                                  # e, n, alt (출발점 원점)
    hm = os.path.join(run_dir, "heightmap_px.npy")
    m = SiteMap(origin=tuple(R["origin_latlon"]), heightmap=hm if os.path.exists(hm) else None)

    # 계획 경로 상 고도(보간)와 수평 횡오차
    A, B = P[:-1, :2], P[1:, :2]; AB = B - A; L2 = np.maximum((AB ** 2).sum(1), 1e-9)
    def nearest(p):
        tt = np.clip(((p - A) * AB).sum(1) / L2, 0, 1); q = A + tt[:, None] * AB
        k = int(np.argmin(np.linalg.norm(q - p, axis=1)))
        return float(np.linalg.norm(q[k] - p)), float(P[k, 2] + tt[k] * (P[k + 1, 2] - P[k, 2]))
    zmin_plan = float(P[:, 2].min())
    cruise = fly & (U > zmin_plan - 0.5)
    res = np.array([nearest(np.array([e, n])) for e, n in zip(E[cruise], N[cruise])])
    xt, zplan = res[:, 0], res[:, 1]; zerr = np.abs(U[cruise] - zplan)
    H = m.H; ys, xs = np.nonzero(H > 0)
    la, lo = m.px_to_latlon(xs.astype(float), ys.astype(float)); oe, on = m.latlon_to_enu(la, lo)
    tree = cKDTree(np.c_[oe, on]); Hh = H[ys, xs]
    # 장애물 여유: 기체 고도보다 높은(혹은 1 m 이내) 장애물까지 수평 거리
    clear = []
    for e, n, u in zip(E[cruise], N[cruise], U[cruise]):
        idx = tree.query_ball_point([e, n], 6.0)
        idx = [i for i in idx if Hh[i] > u - 1.0]
        clear.append(min(np.hypot(oe[idx] - e, on[idx] - n)) if idx else 6.0)
    clear = np.array(clear)
    st = {"flight_s": float(t[fly][-1]), "plan_length_m": float(R["length_m"]),
          "xt_mean_m": float(xt.mean()), "xt_max_m": float(xt.max()),
          "alt_err_mean_m": float(zerr.mean()), "alt_err_max_m": float(zerr.max()),
          "min_clearance_m": float(clear.min()), "max_alt_m": float(U[fly].max()),
          "plan_alt_range_m": [float(P[:, 2].min()), float(P[:, 2].max())]}
    json.dump(st, open(os.path.join(run_dir, "stats.json"), "w"), indent=1)

    # ── 평면도 ──
    base = os.path.join(run_dir, "_b.png"); m.plot([], base, zoom=zoom, connect=False)
    img = Image.open(base).convert("RGB"); os.remove(base); d = ImageDraw.Draw(img)
    f, fs = ImageFont.truetype(FONT, 19), ImageFont.truetype(FONT, 16)
    def uv(e, n):
        la_, lo_ = m.enu_to_latlon(e, n); u, v = m.latlon_to_px(la_, lo_)
        return np.c_[np.atleast_1d(u), np.atleast_1d(v)] * zoom
    pp = uv(P[:, 0], P[:, 1])
    d.line([tuple(q) for q in pp], fill=(0, 0, 0), width=9); d.line([tuple(q) for q in pp], fill=(255, 255, 255), width=5)
    for q in pp:
        d.ellipse([q[0] - 6, q[1] - 6, q[0] + 6, q[1] + 6], fill=(255, 255, 255), outline=(0, 0, 0), width=2)
    tr = uv(E[fly], N[fly]); d.line([tuple(q) for q in tr], fill=(255, 40, 40), width=3)
    ML = np.array(R["mission_latlon"]); names = ["출발"] + [f"WP{i}" for i in range(1, len(ML) - 1)] + ["도착"]
    for nm, (a, b) in zip(names, ML):
        q = np.array(m.latlon_to_px(a, b)) * zoom
        bb = d.textbbox((q[0] + 12, q[1] - 28), nm, font=f); d.rectangle([bb[0] - 4, bb[1] - 3, bb[2] + 4, bb[3] + 3], fill=(0, 0, 0))
        d.text((q[0] + 12, q[1] - 28), nm, font=f, fill=(255, 220, 0))
    W, Hi = img.size
    lines = [(f"{os.path.basename(os.path.normpath(run_dir))}", (255, 220, 0)),
             ("흰 선: PP 계획 경로   빨간 선: SITL 실제 비행 (pp_waypoint_offboard)", (255, 255, 255)),
             (f"횡오차 평균 {st['xt_mean_m']:.2f} / 최대 {st['xt_max_m']:.2f} m   고도오차 최대 {st['alt_err_max_m']:.2f} m   "
              f"장애물 최소거리 {st['min_clearance_m']:.2f} m   비행 {st['flight_s']:.0f} s", (255, 255, 255))]
    for i, (txt, c) in enumerate(lines):
        y = Hi - 100 + i * 30; bb = d.textbbox((16, y), txt, font=f)
        d.rectangle([bb[0] - 4, bb[1] - 3, bb[2] + 4, bb[3] + 3], fill=(0, 0, 0)); d.text((16, y), txt, font=f, fill=c)

    # ── 시간-고도 ──
    ph, pad = 300, 70; side = Image.new("RGB", (W, ph), (250, 250, 250)); s = ImageDraw.Draw(side)
    tf, Uf = t[fly], U[fly]; T1 = max(tf[-1], 1.0); ztop = max(5.0, float(np.ceil(max(Uf.max(), P[:, 2].max()))) + 1)
    X = lambda tt: pad + tt / T1 * (W - 2 * pad); Y = lambda z: ph - 40 - z / ztop * (ph - 80)
    for gz in range(0, int(ztop) + 1):
        s.line([(pad, Y(gz)), (W - pad, Y(gz))], fill=(228, 228, 228))
        s.text((pad - 8, Y(gz)), f"{gz}", font=fs, fill=(90, 90, 90), anchor="rm")
    tc = tf[U[fly] > zmin_plan - 0.5] if False else t[cruise]
    s.line([(X(a), Y(b)) for a, b in zip(tc, zplan)], fill=(150, 150, 150), width=3)
    s.line([(X(a), Y(b)) for a, b in zip(tf, Uf)], fill=(220, 40, 40), width=3)
    s.text((pad + 20, 12), "비행 고도 (빨강 = 실제, 회색 = 그 위치의 PP 계획 고도)", font=f, fill=(30, 30, 30))
    s.text((W - 10, ph - 12), f"시간 {T1:.0f} s →", font=fs, fill=(90, 90, 90), anchor="rd")
    s.text((6, 22), "고도[m]", font=fs, fill=(90, 90, 90), anchor="lt")
    out = Image.new("RGB", (W, Hi + ph), (255, 255, 255)); out.paste(img, (0, 0)); out.paste(side, (0, Hi))
    out.save(os.path.join(run_dir, "result.png"))
    print(json.dumps(st, ensure_ascii=False))


if __name__ == "__main__":
    main(sys.argv[1])
