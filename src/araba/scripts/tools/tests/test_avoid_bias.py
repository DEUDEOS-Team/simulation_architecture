"""avoidance_node direksiyon düzeltmesi kuralları."""
import math, sys
sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")
from gap_avoid_logic import GapObstacle

WHEEL_BASE_M, STEER_LIMIT_RAD = 1.675, 0.5236
MIN_LOOKAHEAD_M, GAIN, MAXB = 3.0, 1.6, 0.70
B_DEADBAND_M, NT_D, NT_LAT = 0.35, 8.0, 2.2

def bias(b, obstacles):
    rel = [o for o in obstacles if o.distance_m >= 0.3]
    nd = min((o.distance_m for o in rel), default=float("inf"))
    s = 0.0
    if abs(b) >= B_DEADBAND_M and math.isfinite(nd):
        d = max(nd, MIN_LOOKAHEAD_M)
        s = -math.atan2(2*WHEEL_BASE_M*b, d*d)/STEER_LIMIT_RAD*GAIN
        s = max(-MAXB, min(MAXB, s))
    close = [o for o in rel if o.distance_m <= NT_D]
    if close:
        n = min(close, key=lambda o: o.distance_m)
        c = 0.5*(n.lateral_min_m + n.lateral_max_m)
        if abs(c) < NT_LAT:
            s = min(s, 0.0) if c < 0 else max(s, 0.0)
    return s

ok = fail = 0
def check(n, c):
    global ok, fail
    if c: ok += 1; print(f"  PASS {n}")
    else: fail += 1; print(f"  FAIL {n}")

# Koni sağda (lat -1.6), gereken kaçış küçük (-0.1) -> araç sağa kırmamalı
cone_r = [GapObstacle(3.4, -1.75, -1.45)]
check("küçük kaçış -> düzeltme YOK (ölü bant)", abs(bias(-0.10, cone_r)) < 1e-9)
check("engel SAĞDA iken SAĞA kırılamaz", bias(-1.0, cone_r) <= 0.0)
cone_l = [GapObstacle(3.4, +1.45, +1.75)]
check("engel SOLDA iken SOLA kırılamaz", bias(+1.0, cone_l) >= 0.0)
# doğru yön hâlâ çalışıyor
check("engel SAĞDA, kaçış SOLA -> SOLA kırar", bias(+1.2, cone_r) < -0.05)
check("engel SOLDA, kaçış SAĞA -> SAĞA kırar", bias(-1.2, cone_l) > 0.05)
check("engel yoksa düzeltme yok", abs(bias(0.0, [])) < 1e-9)
print(f"\nSONUÇ: {ok} PASS, {fail} FAIL")
sys.exit(1 if fail else 0)
