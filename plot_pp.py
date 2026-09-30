#!/usr/bin/env python3
"""out/pp_result.json 의 PP 경로를 사진 위에 그린다. run_pp.sh 가 마지막에 호출.

위: 평면도 — 경로 색이 구간 안에서도 고도를 따라 연속으로 바뀐다, 경로점마다 고도 표시
아래: 옆면 — 이동 거리 대비 고도, 경로 아래 지면·장애물 높이
"""
import json
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from site_map import SiteMap

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


def font(size):
    try:
        return ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default()


def main(zoom=2):
    R = json.load(open(os.path.join(OUT, "pp_result.json"), encoding="utf-8"))
    hm = os.path.join(OUT, "heightmap_px.npy")
    m = SiteMap(origin=tuple(R["origin_latlon"]), heightmap=hm if os.path.exists(hm) else None)
    base = os.path.join(OUT, "_base.png")
    names = ["출발"] + [f"WP{i}" for i in range(1, len(R["mission_latlon"]) - 1)] + ["도착"]
    m.plot([], base, zoom=zoom, connect=False)
    top = Image.open(base).convert("RGB"); os.remove(base); d = ImageDraw.Draw(top)
    f, fs = font(19), font(16)

    P = np.array(R["path_enu"]); LL = np.array(R["path_latlon"])
    z = P[:, 2]; zl = float(np.floor(z.min())); zh = float(max(np.ceil(z.max()), zl + 1))
    lut = cv2.applyColorMap(np.arange(256, dtype=np.uint8)[:, None], cv2.COLORMAP_TURBO)[:, 0, ::-1]
    col = lambda zz: tuple(int(c) for c in lut[int(np.clip((zz - zl) / (zh - zl), 0, 1) * 255)])

    # 구간을 0.25 m 단위로 잘라 색을 고도에 따라 연속으로
    uv = np.array([m.latlon_to_px(a, b) for a, b in LL]) * zoom
    for i in range(len(P) - 1):
        n = max(2, int(np.linalg.norm(P[i + 1, :2] - P[i, :2]) / 0.25))
        t = np.linspace(0, 1, n + 1)
        Q = uv[i] + t[:, None] * (uv[i + 1] - uv[i]); Z = z[i] + t * (z[i + 1] - z[i])
        for k in range(n):
            d.line([tuple(Q[k]), tuple(Q[k + 1])], fill=(0, 0, 0), width=8)
        for k in range(n):
            d.line([tuple(Q[k]), tuple(Q[k + 1])], fill=col((Z[k] + Z[k + 1]) / 2), width=5)
    show = [i == 0 or abs(z[i] - z[i - 1]) > 0.01 or (i + 1 < len(z) and abs(z[i + 1] - z[i]) > 0.01) for i in range(len(z))]
    for i, (q, zz) in enumerate(zip(uv, z)):
        d.ellipse([q[0] - 4, q[1] - 4, q[0] + 4, q[1] + 4], fill=col(zz), outline=(0, 0, 0), width=1)
        if show[i]:
            t = f"{zz:.0f} m"; bb = d.textbbox((q[0] - 10, q[1] + 10), t, font=fs, anchor="ra")
            d.rectangle([bb[0] - 3, bb[1] - 2, bb[2] + 3, bb[3] + 2], fill=(0, 0, 0))
            d.text((q[0] - 10, q[1] + 10), t, font=fs, fill=(255, 255, 255), anchor="ra")

    ML = np.array(R["mission_latlon"]); pal = [(255, 220, 0), (0, 255, 150), (255, 90, 255), (0, 220, 255), (255, 140, 0)]
    for i, ((la_, lo_), nm) in enumerate(zip(ML, names)):
        q = np.array(m.latlon_to_px(la_, lo_)) * zoom; c = (255, 220, 0) if nm in ("출발", "도착") else pal[1 + (i - 1) % 4]
        d.line([q[0] - 13, q[1], q[0] + 13, q[1]], fill=c, width=3); d.line([q[0], q[1] - 13, q[0], q[1] + 13], fill=c, width=3)
        d.ellipse([q[0] - 9, q[1] - 9, q[0] + 9, q[1] + 9], outline=c, width=3)
        bb = d.textbbox((q[0] + 12, q[1] - 26), nm, font=f); d.rectangle([bb[0] - 4, bb[1] - 3, bb[2] + 4, bb[3] + 3], fill=(0, 0, 0))
        d.text((q[0] + 12, q[1] - 26), nm, font=f, fill=c)

    # 색 막대 + 눈금
    W, H = top.size; x0, y0, bw = 20, H - 46, 260
    for k in range(bw):
        d.line([(x0 + k, y0), (x0 + k, y0 + 16)], fill=col(zl + (zh - zl) * k / (bw - 1)))
    d.rectangle([x0 - 1, y0 - 1, x0 + bw, y0 + 17], outline=(0, 0, 0))
    ticks = np.arange(zl, zh + 0.01, 1 if zh - zl <= 10 else 5)
    for tz in ticks:
        x = x0 + (tz - zl) / (zh - zl) * (bw - 1)
        d.line([(x, y0 + 16), (x, y0 + 22)], fill=(255, 255, 255), width=2)
        d.text((x, y0 + 24), f"{tz:.0f}", font=fs, fill=(255, 255, 255), anchor="ma", stroke_width=2, stroke_fill=(0, 0, 0))
    t = f"Hybrid PP 경로  {R['length_m']:.1f} m  |  고도 {z.min():.1f} ~ {z.max():.1f} m (색, 지면 기준)"
    bb = d.textbbox((x0, y0 - 34), t, font=f); d.rectangle([bb[0] - 4, bb[1] - 3, bb[2] + 4, bb[3] + 3], fill=(0, 0, 0))
    d.text((x0, y0 - 34), t, font=f, fill=(255, 255, 255))

    # 옆면 그래프: 이동 거리 vs 고도, 경로 아래 지면·장애물
    ph, pad = 300, 60
    side = Image.new("RGB", (W, ph), (250, 250, 250)); s = ImageDraw.Draw(side)
    seg = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1); cum = np.r_[0, np.cumsum(seg)]; L = cum[-1]
    ds = np.arange(0, L + 0.1, 0.1); e = np.interp(ds, cum, P[:, 0]); nn = np.interp(ds, cum, P[:, 1]); zp = np.interp(ds, cum, z)
    la, lo = m.enu_to_latlon(e, nn); u, v = m.latlon_to_px(la, lo)
    gx = np.clip(np.round(u).astype(int), 0, m.H.shape[1] - 1); gy = np.clip(np.round(v).astype(int), 0, m.H.shape[0] - 1)
    ground = m.H[gy, gx]
    zmax = max(zh, float(ground.max())) + 2
    pad = 70
    X = lambda dd: pad + dd / max(L, 1e-6) * (W - 2 * pad); Y = lambda zz: ph - 40 - zz / zmax * (ph - 70)
    for gz in np.arange(0, zmax + 0.1, 5 if zmax > 12 else 2):
        s.line([(pad, Y(gz)), (W - pad, Y(gz))], fill=(225, 225, 225)); s.text((pad - 8, Y(gz)), f"{gz:.0f}", font=fs, fill=(90, 90, 90), anchor="rm")
    for k in range(len(ds) - 1):
        if ground[k] > 0:
            s.rectangle([X(ds[k]), Y(ground[k]), X(ds[k + 1]), Y(0)], fill=(245, 200, 60) if ground[k] < 19.9 else (220, 90, 90))
    s.line([(pad, Y(0)), (W - pad, Y(0))], fill=(120, 120, 120), width=2)
    for k in range(0, len(ds) - 1):
        s.line([(X(ds[k]), Y(zp[k])), (X(ds[k + 1]), Y(zp[k + 1]))], fill=col(zp[k]), width=5)
    for i, (c, zz) in enumerate(zip(cum, z)):
        s.ellipse([X(c) - 4, Y(zz) - 4, X(c) + 4, Y(zz) + 4], fill=col(zz), outline=(0, 0, 0))
        if show[i]:
            s.text((X(c), Y(zz) - 10), f"{zz:.0f} m", font=fs, fill=(30, 30, 30),
                   anchor="ld" if c == 0 else ("rd" if c == cum[-1] else "md"))
    # 미션 점(출발·경유·도착) 위치: 경로점 중 각 미션 점에 가장 가까운 것
    Pm = np.array([np.r_[m.latlon_to_enu(a_, b_)] for a_, b_ in ML])
    for nm, pm in zip(names, Pm):
        k = int(np.argmin(np.linalg.norm(P[:, :2] - pm, axis=1))); x = X(cum[k])
        s.line([(x, Y(0)), (x, 30)], fill=(150, 150, 150), width=1); s.text((x, Y(0) + 4), nm, font=fs, fill=(60, 60, 60), anchor="ma")
    s.text((pad + 20, 12), "옆면: 이동 거리 대비 고도 (노랑 = 경로 아래 장애물, 빨강 = 20 m 구조물)", font=f, fill=(30, 30, 30))
    s.text((W - 10, 14), f"이동 거리 {L:.1f} m →", font=fs, fill=(90, 90, 90), anchor="ra")
    s.text((6, 22), "고도[m]", font=fs, fill=(90, 90, 90), anchor="lt")

    out = Image.new("RGB", (W, H + ph), (255, 255, 255)); out.paste(top, (0, 0)); out.paste(side, (0, H))
    out.save(os.path.join(OUT, "pp_route.png"))


if __name__ == "__main__":
    main()
