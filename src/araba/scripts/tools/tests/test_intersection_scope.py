#!/usr/bin/env python3
"""Yapışkan dönüş kısıtı düzeltmesi testleri:
1) traffic_sign_logic: sola-dönülmez kısıtı yapışkan; notify_intersection_passed temizler
2) mission_planning: _track_restriction_junction kavşağa bağlar, geçince yayınlar
3) _turn_perm_cb: geçiş sonrası bekleme penceresinde bayat kısıtı yok sayar
"""
import sys
import time
import types

sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")

ok = 0
fail = 0


def check(name, cond):
    global ok, fail
    if cond:
        ok += 1
        print(f"  PASS {name}")
    else:
        fail += 1
        print(f"  FAIL {name}")


# ── 1) traffic_sign_logic ──
from traffic_sign_logic import TrafficSignLogic, SignDetection, SignClass, CONFIRM_FRAMES

logic = TrafficSignLogic()
t = 100.0
for i in range(CONFIRM_FRAMES + 1):
    st = logic.update([SignDetection(class_name=SignClass.NO_LEFT_TURN, confidence=0.9,
                                     bbox_px=(0, 0, 40, 40), estimated_distance_m=6.0)], now=t + i * 0.1)
check("sola-dönülmez onaylandı -> left=False", st.turn_permissions.left is False)
# Tabela görüşten çıktı, kısıt yapışkan kalmalı (SIGN_VALIDITY sonrası bile)
st = logic.update([], now=t + 30.0)
check("tabela kaybolunca kısıt hâlâ aktif (yapışkan)", st.turn_permissions.left is False)
# Kavşak geçildi bildirimi kısıtı temizler
logic.notify_intersection_passed()
st = logic.update([], now=t + 31.0)
check("notify_intersection_passed -> kısıt temiz", st.turn_permissions.left is True)

# ── 2-3) mission_planning tarafı (Node başlatmadan, örnek üstünde) ──
from mission_planning_node import MissionPlanningNode, TURN_RULE_APPLY_DISTANCE_M
from std_msgs.msg import String


class PubStub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class LogStub:
    def info(self, *a, **k): pass
    def warn(self, *a, **k): pass
    def error(self, *a, **k): pass


n = object.__new__(MissionPlanningNode)
n._turn_perm = {"left": False, "straight": True, "right": True, "forced_direction": None}
n._restr_junction = None
n._restr_junction_min_d = float("inf")
n._turn_perm_ignore_until = 0.0
n._route_graph = None  # _is_junction -> True (graph yoksa ayrım yok)
n._pub_intersection_passed = PubStub()
n.get_logger = lambda: LogStub()

J_LAT, J_LON = 40.78993, 29.50847  # sahte kavşak

def wpstate(dist):
    return types.SimpleNamespace(
        current_wp=types.SimpleNamespace(lat=J_LAT, lon=J_LON),
        distance_to_wp_m=dist)


def pos_at(d_north_m):
    return types.SimpleNamespace(lat=J_LAT + d_north_m / 111320.0, lon=J_LON)


check("kısıt restrictive algılanıyor", MissionPlanningNode._perm_is_restrictive(n._turn_perm))
check("pass_right restrictive DEĞİL",
      not MissionPlanningNode._perm_is_restrictive({"left": True, "straight": True, "right": True,
                                                    "forced_direction": "pass_right"}))

# uzaktayken bağlanmaz
n._track_restriction_junction(pos_at(-20.0), wpstate(20.0))
check("uzakta (20m) bağlanmadı", n._restr_junction is None)
# 8 m içinde bağlanır
n._track_restriction_junction(pos_at(-7.0), wpstate(7.0))
check("7 m'de kavşağa bağlandı", n._restr_junction is not None)
# yaklaşırken yayın yok
n._track_restriction_junction(pos_at(-2.0), wpstate(2.0))
check("yaklaşırken yayın yok", len(n._pub_intersection_passed.msgs) == 0)
# 3 m uzaklaştı (min 2 + histerezis 4 = 6 gerekli) — henüz değil
n._track_restriction_junction(pos_at(3.0), wpstate(15.0))
check("3 m geçince henüz yayın yok", len(n._pub_intersection_passed.msgs) == 0)
# 6.5 m geçti -> intersection_passed
n._track_restriction_junction(pos_at(6.5), wpstate(18.0))
check("6.5 m geçince intersection_passed yayınlandı", len(n._pub_intersection_passed.msgs) == 1)
check("yerel kısıt temizlendi", n._turn_perm is None)
check("izleme sıfırlandı", n._restr_junction is None)

# bekleme penceresi: bayat kısıtlı mesaj yok sayılır
stale = String()
stale.data = '{"left": false, "straight": true, "right": true, "forced_direction": null}'
n._turn_perm_cb(stale)
check("grace içinde bayat kısıt yok sayıldı", n._turn_perm is None)
# pencere bitince kısıt yeniden kabul edilir (yeni tabela senaryosu)
n._turn_perm_ignore_until = time.monotonic() - 0.1
n._turn_perm_cb(stale)
check("grace sonrası yeni kısıt kabul", n._turn_perm is not None and n._turn_perm["left"] is False)
# izin verici mesaj grace içinde bile geçer
n._turn_perm_ignore_until = time.monotonic() + 5.0
allow = String()
allow.data = '{"left": true, "straight": true, "right": true, "forced_direction": null}'
n._turn_perm_cb(allow)
check("grace içinde izin verici mesaj kabul", n._turn_perm is not None and n._turn_perm["left"] is True)

print(f"\n{ok} PASS / {fail} FAIL")
sys.exit(1 if fail else 0)
