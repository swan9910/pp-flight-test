#!/usr/bin/env python3
"""[사용용] 공터 heightmap 읽기 · 위경도↔ENU 변환 · 높이 조회 · 그림.

실행:  python3 site_map.py
    → 출발 지점, 도착 지점, 경유점 개수(0 가능), 경유점 위경도를 차례로 묻고
      각 점의 ENU(출발 지점 원점) 좌표·높이와 구간별 장애물 통과 여부를 출력하고 파일로 저장한다.
      경로 그림은 직선 연결이며, 장애물·건물 위를 지나는 부분은 빨강으로 표시한다.
    heightmap 은 out/heightmap_px.npy (make_heightmap.py 로 장애물 넣은 것) 가 있으면 그것을,
    없으면 기본 heightmap(건물 20 m, L자 10 m) 을 쓴다.

저장 (out/ 폴더):
    route_enu.csv    이름, 위도, 경도, 동[m], 북[m], 지면 높이[m]
    route.png        사진 위에 heightmap·점 표시
    grid_enu.npz     이륙 지점 원점 정북 정렬 높이 격자 (height[j,i] = 셀 (e0+i*res, n0+j*res), j↑ = 북)

다른 코드에서:
    from site_map import SiteMap
    m = SiteMap(origin=(37.3845030, 126.6544115), heightmap="out/heightmap_px.npy")
    e, n = m.latlon_to_enu(37.3848651, 126.6538410)
    lat, lon = m.enu_to_latlon(10.0, -5.0)
    m.height_at(37.3842, 126.6536)
    g = m.enu_grid(res=0.5)

필요 파일 (map/ 폴더): innov_heightmap_meta.json, innov_heightmap_m.npy, lot_mask.npy, innov_clean.png
"""
import csv
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(HERE, "map")      # 기본 heightmap·좌표 기준 데이터
OUT_DIR = os.path.join(HERE, "out")      # 장애물 heightmap·경로 결과
GRID_RES = 1.0          # 저장할 ENU 격자 해상도 [m]


def obstacle_height(obstacles, E, N, pad=0.0):
    """기준 ENU 좌표 배열 (E, N) 위의 가상 장애물 높이. pad 만큼 모양을 부풀려 셀 겹침 판정.
    obstacles: make_heightmap.py 가 만든 obstacles.json 의 목록 (e_ref, n_ref 는 기준 ENU)."""
    out = np.zeros(np.shape(E), np.float32)
    for o in obstacles:
        dx, dy = E - o["e_ref"], N - o["n_ref"]
        if o["type"] == "box":
            sx, sy, h = o["size"]; a = np.radians(o.get("yaw_deg", 0.0))   # 북에서 시계방향
            lx = dx * np.cos(a) - dy * np.sin(a); ly = dx * np.sin(a) + dy * np.cos(a)
            m = (np.abs(lx) <= sx / 2 + pad) & (np.abs(ly) <= sy / 2 + pad)
        else:
            h = o["h"]; m = np.hypot(dx, dy) <= o["r"] + pad
        out[m] = np.maximum(out[m], h)
    return out


class SiteMap:
    def __init__(self, folder=MAP_DIR, origin=None, heightmap=None):
        """heightmap: 사진 픽셀 격자 .npy 경로. None 이면 기본 heightmap."""
        meta = json.load(open(os.path.join(folder, "innov_heightmap_meta.json"), encoding="utf-8"))
        g = meta["georef"]
        self.folder = folder
        self.m_per_px = meta["m_per_px"]
        self.H_base = np.load(os.path.join(folder, "innov_heightmap_m.npy"))
        self.H = np.load(heightmap) if heightmap else self.H_base.copy()
        assert self.H.shape == self.H_base.shape, f"heightmap 크기 {self.H.shape} ≠ {self.H_base.shape}"
        self.obstacles = []
        if heightmap:
            oj = os.path.join(os.path.dirname(os.path.abspath(heightmap)), "obstacles.json")
            if os.path.exists(oj):
                self.obstacles = json.load(open(oj, encoding="utf-8"))["obstacles"]
        lot = os.path.join(folder, "lot_mask.npy")
        self.lot = np.load(lot) if os.path.exists(lot) else None
        self.R = g.get("earth_radius_m", 6378137.0)
        self._lat_ref, self._lon_ref = g["enu_origin_latlon"]
        self._coslat = np.cos(np.radians(self._lat_ref))
        self._M = np.array(g["px_to_enu"])                                   # 2x3: 픽셀 → 기준 ENU
        self._Minv = np.linalg.inv(np.vstack([self._M, [0, 0, 1]]))[:2]    # 기준 ENU → 픽셀
        self.set_origin(origin)

    # ── 원점 ──
    def set_origin(self, origin=None):
        """ENU 원점을 위경도로 지정. None 이면 공터 중심."""
        if origin is None:
            self.origin = (self._lat_ref, self._lon_ref); self._off = np.zeros(2)
        else:
            self.origin = tuple(origin); self._off = self._ref_enu(*origin)

    # ── 내부: 기준 원점 ENU (등장방형 근사, 이 범위(≈130 m)에서 오차 mm 수준) ──
    def _ref_enu(self, lat, lon):
        lat, lon = np.asarray(lat, float), np.asarray(lon, float)
        return np.stack([np.radians(lon - self._lon_ref) * self.R * self._coslat,
                         np.radians(lat - self._lat_ref) * self.R], -1)

    def _ref_latlon(self, e, n):
        e, n = np.asarray(e, float), np.asarray(n, float)
        return (self._lat_ref + np.degrees(n / self.R),
                self._lon_ref + np.degrees(e / (self.R * self._coslat)))

    def px_to_ref_enu(self, U, V):
        """픽셀 배열 → 기준 ENU (e, n) 배열 (같은 모양)."""
        U, V = np.asarray(U, float), np.asarray(V, float)
        en = self._M @ np.vstack([U.ravel(), V.ravel(), np.ones(U.size)])
        return en[0].reshape(U.shape), en[1].reshape(U.shape)

    # ── 공개 변환 ──
    def latlon_to_enu(self, lat, lon):
        """위경도 → (e, n) [m], 현재 원점 기준."""
        en = self._ref_enu(lat, lon) - self._off
        return en[..., 0], en[..., 1]

    def enu_to_latlon(self, e, n):
        """(e, n) [m], 현재 원점 기준 → (lat, lon)."""
        return self._ref_latlon(np.asarray(e) + self._off[0], np.asarray(n) + self._off[1])

    def latlon_to_px(self, lat, lon):
        """위경도 → 사진 픽셀 (u, v)."""
        en = np.atleast_2d(self._ref_enu(lat, lon)).reshape(-1, 2)
        uv = np.c_[en, np.ones(len(en))] @ self._Minv.T
        return (uv[:, 0], uv[:, 1]) if np.ndim(lat) else (float(uv[0, 0]), float(uv[0, 1]))

    def px_to_latlon(self, u, v):
        e, n = self.px_to_ref_enu(np.atleast_1d(u), np.atleast_1d(v))
        lat, lon = self._ref_latlon(e, n)
        return (lat, lon) if np.ndim(u) else (float(lat[0]), float(lon[0]))

    # ── 조회 ──
    def height_at(self, lat, lon):
        """해당 위경도의 높이 [m]. 사진 범위 밖이면 nan."""
        u, v = self.latlon_to_px(lat, lon); x, y = int(round(u)), int(round(v))
        if 0 <= x < self.H.shape[1] and 0 <= y < self.H.shape[0]:
            return float(self.H[y, x])
        return float("nan")

    def in_lot(self, lat, lon):
        """공터 안이면 True, 밖이면 False, 범위 밖·마스크 없음이면 None."""
        if self.lot is None:
            return None
        u, v = self.latlon_to_px(lat, lon); x, y = int(round(u)), int(round(v))
        if 0 <= x < self.lot.shape[1] and 0 <= y < self.lot.shape[0]:
            return bool(self.lot[y, x])
        return None

    def check_segment(self, a, b, step=0.2):
        """위경도 a→b 직선을 step[m] 간격으로 따라가며 heightmap 높이를 본다.
        반환: length, over_len(높이>0 위를 지나는 길이), max_h, pieces[(uv0, uv1, hit)]."""
        ea, na = self.latlon_to_enu(*a); eb, nb = self.latlon_to_enu(*b)
        L = float(np.hypot(eb - ea, nb - na)); k = max(2, int(L / step) + 1)
        t = np.linspace(0, 1, k); la, lo = self.enu_to_latlon(ea + t * (eb - ea), na + t * (nb - na))
        u, v = self.latlon_to_px(la, lo); x = np.clip(np.round(u).astype(int), 0, self.H.shape[1] - 1)
        y = np.clip(np.round(v).astype(int), 0, self.H.shape[0] - 1); h = self.H[y, x]
        hit = h > 0; pieces = []; i0 = 0
        for i in range(1, k + 1):
            if i == k or hit[i] != hit[i0]:
                pieces.append(((u[i0], v[i0]), (u[min(i, k - 1)], v[min(i, k - 1)]), bool(hit[i0]))); i0 = i
        return {"length": L, "over_len": float(hit.sum() * L / (k - 1)) if k > 1 else 0.0,
                "max_h": float(h.max()), "pieces": pieces}

    # ── 격자 ──
    def enu_axes(self, res=1.0, margin=0.0):
        """사진 범위를 덮는 정북 정렬 ENU 격자 축 (현재 원점 기준 셀 중심)."""
        h, w = self.H.shape
        lat, lon = self.px_to_latlon(np.array([0., w, w, 0.]), np.array([0., 0., h, h]))
        e, n = self.latlon_to_enu(lat, lon)
        es = np.arange(e.min() - margin + res / 2, e.max() + margin, res)
        ns = np.arange(n.min() - margin + res / 2, n.max() + margin, res)
        return es, ns

    def enu_grid(self, res=1.0, margin=0.0):
        """현재 heightmap 을 정북 정렬 ENU 격자로 재샘플 (최근접).
        반환 dict: height[j,i] = 셀 (e0 + i*res, n0 + j*res) 높이. j 가 커지면 북쪽. 범위 밖 0."""
        es, ns = self.enu_axes(res, margin)
        E, N = np.meshgrid(es, ns)
        la, lo = self.enu_to_latlon(E.ravel(), N.ravel())
        u, v = self.latlon_to_px(la, lo)
        x = np.round(u).astype(int); y = np.round(v).astype(int); h, w = self.H.shape
        ok = (x >= 0) & (x < w) & (y >= 0) & (y < h)
        out = np.zeros(E.size, np.float32); out[ok] = self.H[y[ok], x[ok]]
        out = out.reshape(E.shape)
        if self.obstacles:     # 얇은 원통이 재샘플에서 빠지지 않게 격자에서 직접 칠함 (셀과 겹치면 포함)
            out = np.maximum(out, obstacle_height(self.obstacles, E + self._off[0], N + self._off[1], res / 2))
        return {"height": out, "e0": float(es[0]), "n0": float(ns[0]), "res": res,
                "origin_latlon": np.array(self.origin)}

    # ── 그림 ──
    def plot(self, points=(), out="overlay.png", labels=None, zoom=2, connect=True):
        """heightmap 을 사진 위에 칠하고 points [(lat, lon), ...] 를 표시.
        빨강 = 건물(20 m), 파랑 = L자(10 m), 노랑 = 추가 장애물."""
        import cv2
        from PIL import Image, ImageDraw, ImageFont
        ov = cv2.imread(os.path.join(self.folder, "innov_clean.png"))
        b = self.H_base
        for m, col in ((b >= 19.99, (0, 0, 110)), ((b > 0) & (b < 19.99), (110, 50, 0)),
                       (self.H > b + 1e-3, (0, 200, 230))):
            ov[m] = np.clip(0.45 * ov[m] + np.array(col), 0, 255).astype(np.uint8)
        img = Image.fromarray(cv2.cvtColor(cv2.resize(ov, None, fx=zoom, fy=zoom, interpolation=cv2.INTER_CUBIC),
                                           cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(img)
        try:
            f = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 18)
        except OSError:
            f = ImageFont.load_default()
        if self.lot is not None:
            for c in cv2.findContours(self.lot.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                d.polygon([tuple(p[0] * zoom) for p in c], outline=(40, 120, 255), width=2)
        pal = [(255, 220, 0), (0, 255, 150), (255, 90, 255), (0, 220, 255), (255, 140, 0)]
        pts = [np.array(self.latlon_to_px(la, lo)) * zoom for la, lo in points]
        if connect and len(pts) > 1:                                           # 경로: 흰 선, 장애물·건물 위는 빨강
            for a, b_ in zip(points, points[1:]):
                seg = self.check_segment(a, b_)
                for (u0, v0), (u1, v1), hit in seg["pieces"]:
                    d.line([(u0 * zoom, v0 * zoom), (u1 * zoom, v1 * zoom)],
                           fill=(255, 40, 40) if hit else (255, 255, 255), width=4 if hit else 2)
        for i, q in enumerate(pts):
            c = pal[i % len(pal)]; la, lo = points[i]
            d.line([q[0] - 14, q[1], q[0] + 14, q[1]], fill=c, width=3)
            d.line([q[0], q[1] - 14, q[0], q[1] + 14], fill=c, width=3)
            d.ellipse([q[0] - 8, q[1] - 8, q[0] + 8, q[1] + 8], outline=c, width=3)
            t = f"{labels[i] if labels else f'P{i + 1}'}  {la:.7f}, {lo:.7f}"
            bb = d.textbbox((q[0] + 14, q[1] - 10), t, font=f)
            d.rectangle([bb[0] - 4, bb[1] - 3, bb[2] + 4, bb[3] + 3], fill=(0, 0, 0))
            d.text((q[0] + 14, q[1] - 10), t, font=f, fill=c)
        img.save(out)
        return out


# ── 대화형 입력 ──
def parse_ll(s):
    """'37.38, 126.65' 또는 '37.38 126.65' → (lat, lon)."""
    v = s.replace(",", " ").split()
    return float(v[0]), float(v[1])


def ask_ll(prompt):
    while True:
        try:
            la, lo = parse_ll(input(prompt))
            if 30 < la < 45 and 120 < lo < 135:
                return la, lo
            print("  위도 30~45, 경도 120~135 범위가 아닙니다. 순서(위도, 경도)를 확인하세요.")
        except (ValueError, IndexError):
            print("  '위도, 경도' 형식으로 입력하세요. 예: 37.3845030, 126.6544115")


def ask_int(prompt, minimum=0):
    while True:
        try:
            n = int(input(prompt))
            if n >= minimum:
                return n
        except ValueError:
            pass
        print(f"  {minimum} 이상의 정수를 입력하세요.")


def main():
    hm = os.path.join(OUT_DIR, "heightmap_px.npy")
    hm = hm if os.path.exists(hm) else None
    print(f"heightmap: {'장애물 포함 (' + hm + ')' if hm else '기본 (건물·L자만)'}")

    takeoff = ask_ll("출발 지점 위도, 경도: ")
    goal = ask_ll("도착 지점 위도, 경도: ")
    n = ask_int("경유점 개수 (없으면 0): ")
    wps = [ask_ll(f"경유점 {i + 1} 위도, 경도: ") for i in range(n)]

    m = SiteMap(origin=takeoff, heightmap=hm)
    pts = [takeoff] + wps + [goal]
    names = ["출발"] + [f"WP{i + 1}" for i in range(n)] + ["도착"]
    os.makedirs(OUT_DIR, exist_ok=True)

    print(f"\nENU 원점 = 출발 지점 {takeoff[0]:.7f}, {takeoff[1]:.7f}")
    print(f"{'':5s} {'동[m]':>9s} {'북[m]':>9s} {'높이[m]':>8s}  공터 안")
    rows = []
    for name, (la, lo) in zip(names, pts):
        e, n_ = m.latlon_to_enu(la, lo); h = m.height_at(la, lo); inl = m.in_lot(la, lo)
        warn = "" if (h == 0 and inl) else "  ← 확인 필요" + (" (장애물/건물 위)" if h > 0 else "") + (" (공터 밖)" if inl is False else "")
        print(f"{name:5s} {float(e):+9.2f} {float(n_):+9.2f} {h:8.1f}  {inl}{warn}")
        rows.append([name, f"{la:.8f}", f"{lo:.8f}", f"{float(e):.3f}", f"{float(n_):.3f}", f"{h:.2f}"])
    total = 0.0
    print("\n구간 (직선 연결 기준)")
    for (na_, a), (nb_, b) in zip(zip(names, pts), zip(names[1:], pts[1:])):
        c = m.check_segment(a, b); total += c["length"]
        msg = "장애물 없음" if c["over_len"] == 0 else f"장애물·건물 위 {c['over_len']:.1f} m 통과 (최고 {c['max_h']:.0f} m) ← 우회 필요"
        print(f"  {na_} → {nb_}: {c['length']:.1f} m, {msg}")
    print(f"경로 길이 (직선 연결) {total:.1f} m")

    with open(os.path.join(OUT_DIR, "route_enu.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f); w.writerow(["name", "lat", "lon", "east_m", "north_m", "ground_h_m"]); w.writerows(rows)
    g = m.enu_grid(GRID_RES); np.savez(os.path.join(OUT_DIR, "grid_enu.npz"), **g)
    m.plot(pts, os.path.join(OUT_DIR, "route.png"), labels=names)
    print(f"\n저장: {OUT_DIR}/route_enu.csv, route.png, grid_enu.npz ({g['height'].shape}, {GRID_RES} m)")


if __name__ == "__main__":
    main()
