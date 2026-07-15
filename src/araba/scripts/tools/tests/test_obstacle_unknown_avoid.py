#!/usr/bin/env python3
"""Koşu-6 düzeltmesi testleri: 'unknown' küme statik kaçınmaya girer, acil fren kilitlenmez."""
import sys

sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")

from obstacle_logic import ObstacleLogic, ObstacleDetection, ObstacleBehavior

ok = fail = 0


def check(name, cond):
    global ok, fail
    if cond:
        ok += 1
        print(f"  PASS {name}")
    else:
        fail += 1
        print(f"  FAIL {name}")


def det(kind, dist, lateral=0.0):
    return ObstacleDetection(
        kind=kind, confidence=0.7, bbox_px=(600.0, 300.0, 680.0, 420.0),
        estimated_distance_m=dist, estimated_lateral_m=lateral)


def run(kind, dist, lateral=0.0, frames=6):
    logic = ObstacleLogic()
    st = None
    t = 100.0
    for i in range(frames):
        st = logic.update([det(kind, dist, lateral)], now=t + i * 0.1)
    return st


# 1) 'unknown' küme 6 m'de: kaçınma önerisi var, acil yok
st = run("unknown", 6.0, lateral=0.1)
check("unknown@6m -> static_avoid", st.behavior_mode == ObstacleBehavior.STATIC_AVOID)
check("unknown@6m -> suggest_lane_change", st.suggest_lane_change is True)
check("unknown@6m -> acil YOK", st.emergency_stop is False)
check("unknown@6m -> yön önerisi var", st.avoidance_direction in ("left", "right"))

# 2) 'unknown' 2.2 m'de: hâlâ kaçınma, acil fren kilitli DEĞİL (>1.5 m)
st = run("unknown", 2.2, lateral=0.1)
check("unknown@2.2m -> acil YOK (kaçınma sürüyor)", st.emergency_stop is False)
check("unknown@2.2m -> static_avoid", st.behavior_mode == ObstacleBehavior.STATIC_AVOID)
check("unknown@2.2m -> hız tavanı kaçınma seviyesinde", 0.2 <= st.speed_cap_ratio <= 0.6)

# 3) 'unknown' 1.2 m'de: gerçek acil bandı korunur
st = run("unknown", 1.2, lateral=0.1)
check("unknown@1.2m -> ACİL", st.emergency_stop is True)

# 4) koni 6 m'de: yeni tetik (7 m) ile artık erken kaçınma
st = run("cone", 6.0, lateral=-0.2)
check("cone@6m -> static_avoid (tetik 7 m)", st.behavior_mode == ObstacleBehavior.STATIC_AVOID)
check("cone@6m -> acil YOK", st.emergency_stop is False)

# 5) koni 9.5 m'de: tetik (9 m) dışı — sadece yavaşlama bandı, kaçınma önerisi yok
st = run("cone", 9.5, lateral=-0.2)
check("cone@9.5m -> kaçınma önerisi yok", st.suggest_lane_change is False)
check("cone@9.5m -> acil YOK", st.emergency_stop is False)
# 8 m'de artık tetik İÇİ (7→9 m genişledi) -> kaçınma önerisi var
st = run("cone", 8.0, lateral=-0.2)
check("cone@8m -> static_avoid (tetik 9 m)", st.behavior_mode == ObstacleBehavior.STATIC_AVOID)

# 5b) Yoldaki koni lateral 2.5 m'de: koridor ±3.0'a genişletildi — lidar yol-onaylı
#     yayınlıyor, bu engel artık sayılmalı
#     (düzeltmeden önce koridor dışı sayılıp 'clear' kalıyordu, çarpma).
st = run("cone", 6.0, lateral=2.5)
check("cone@6m lat=2.5 -> static_avoid (koridor genişledi)",
      st.behavior_mode == ObstacleBehavior.STATIC_AVOID)
check("cone@6m lat=2.5 -> cone_in_corridor True", st.cone_in_corridor is True)
st = run("cone", 6.0, lateral=3.5)
check("cone@6m lat=3.5 -> hâlâ koridor dışı (>3.0)", st.behavior_mode != ObstacleBehavior.STATIC_AVOID)

# 6) yaya davranışı bozulmadı: 4 m'de acil (ihtiyat çarpanıyla eff<3 — eski kodla birebir),
#    6 m'de dinamik yavaşlama
st = run("pedestrian", 4.0, lateral=0.0)
check("yaya@4m -> ACİL (eski davranış korunur)", st.emergency_stop is True)
check("yaya@4m -> hız 0", st.speed_cap_ratio == 0.0)
st = run("pedestrian", 6.0, lateral=0.0)
check("yaya@6m -> DYNAMIC_SLOW", st.behavior_mode == ObstacleBehavior.DYNAMIC_SLOW)

# ── Acil histerezisi: tek karelik acil yok sayılır ──
logic = ObstacleLogic()
t = 200.0
# 5 kare normal kaçınma mesafesi
for i in range(5):
    st = logic.update([det("unknown", 6.0, 0.1)], now=t + i * 0.1)
# TEK karelik sahte "çok yakın" tespiti (kamera titremesi benzeri, 2.0 m: acil bandı ama immediate değil)
st = logic.update([det("unknown", 6.0, 0.1), det("pedestrian", 2.0, 0.0)], now=t + 0.5)
check("tek karelik acil blip'i yutuldu", st.emergency_stop is False)
# Ardışık kareler -> gerçek acil geçer (tracker onayı 2 kare + histerezis 2 kare;
# ani beliren tehdit <1.2 m ise aşağıdaki anında-geçiş istisnası devrede)
for j in range(3):
    st = logic.update([det("unknown", 6.0, 0.1), det("pedestrian", 2.0, 0.0)], now=t + 0.6 + j * 0.1)
check("süren tehditte acil geçer (≤4 kare)", st.emergency_stop is True)
# Burnun dibinde (1.1 m) tek karede bile anında acil
logic2 = ObstacleLogic()
for i in range(5):
    logic2.update([det("unknown", 6.0, 0.1)], now=t + i * 0.1)
st = logic2.update([det("unknown", 1.1, 0.1)], now=t + 0.5)
check("<1.2 m tek karede anında acil", st.emergency_stop is True)

print(f"\n{ok} PASS / {fail} FAIL")
sys.exit(1 if fail else 0)
