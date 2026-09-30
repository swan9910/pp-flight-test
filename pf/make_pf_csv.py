#!/usr/bin/env python3
"""PP 결과(out/pp_result.json) → PF 경유점 CSV (out/pf_wp.csv).

PF(A4VAI PathFollowing, wp_type 0) 외부 경유점 규약: x=북, y=동, z=고도(위가 +) [m].
PF 가 z 를 뒤집어 NED 로 쓴다. 첫 줄 = 출발(이륙) 지점, 그 z 가 이륙 고도.
PP 결과는 출발 지점 원점이라, SITL 에서는 스폰 위치(로컬 원점 ≈ 0,0) 에서 이륙하면 그대로 맞는다.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "out", "pp_result.json")
dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "out", "pf_wp.csv")
R = json.load(open(src, encoding="utf-8"))
P = R["path_enu"]                                  # [east, north, alt]
with open(dst, "w") as f:
    f.write("x,y,z\n")
    for e, n, z in P:
        f.write(f"{n:.3f},{e:.3f},{z:.3f}\n")
print(f"PF 경유점 {len(P)}개 (출발 포함) → {dst}")
