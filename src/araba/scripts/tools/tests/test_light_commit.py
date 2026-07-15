#!/usr/bin/env python3
"""Kırmızı ışık commit bandı + kilit testleri.

Kamera dikey FOV nedeniyle ışık 4.2-5.5 m KALA görüş alanından çıkıyor; must_stop
yalnız ≤3 m'deydi ve HİÇ tetiklenemiyordu (araç kırmızıda geçti). commit bandı
(6.5 m) ışık hâlâ görünürken dur kararı verip kilitler.
"""
import sys

sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")

from traffic_light_logic import TrafficLightLogic  # noqa: E402
from perception_fusion import fuse, StereoBbox     # noqa: E402

ok = fail = 0


def check(name, cond):
    global ok, fail
    print(f"  {'PASS' if cond else 'FAIL'} {name}")
    if cond:
        ok += 1
    else:
        fail += 1


def red_at(lg, dist, t, color='kirmizi isik', v=1.0):
    b = StereoBbox(class_name=color, confidence=0.6, bbox_px=(600, 300, 660, 400),
                   distance_m=dist, lateral_m=None)
    return lg.update(fuse(stereo=[b], lidar=[], imu=None).light_dets,
                     now=t, vehicle_speed_mps=v)


print("── 1) Mesafe bantları ──")
def steady(dist):
    lg = TrafficLightLogic()
    st = None
    for i in range(4):
        st = red_at(lg, dist, 100 + i * 0.2)
    return st


check("12 m -> soft slow, dur YOK", not steady(12.0).must_stop and steady(12.0).speed_cap_ratio == 0.8)
check("7 m -> hard slow, dur YOK", not steady(7.0).must_stop and steady(7.0).speed_cap_ratio == 0.5)
check("6 m (>FOV çıkışı, <commit) -> DUR", steady(6.0).must_stop and steady(6.0).speed_cap_ratio == 0.0)
check("4 m -> DUR", steady(4.0).must_stop)

print("\n── 2) Kilit: ışık FOV'dan çıkınca dur SÜRER ──")
lg = TrafficLightLogic()
for i in range(4):
    st = red_at(lg, 6.0, 100 + i * 0.2)
check("6 m'de dur kuruldu", st.must_stop)
for i in range(4):                      # ışık görünmüyor (boş kare)
    st = lg.update([], now=101 + i * 0.2, vehicle_speed_mps=0.0)
check("ışık kaybolunca kilit dur'u SÜRDÜRÜR", st.must_stop)

print("\n── 3) Kilit YEŞİL onayıyla açılır ──")
for i in range(4):
    st = red_at(lg, 6.0, 105 + i * 0.2, color='yesil isik', v=0.0)
check("onaylı YEŞİL -> kilit açılır, dur biter", not st.must_stop)

print(f"\n{'='*46}\nSONUÇ: {ok} PASS, {fail} FAIL\n{'='*46}")
sys.exit(1 if fail else 0)
