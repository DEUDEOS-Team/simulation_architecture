#!/usr/bin/env python3
"""Geç-tetikleme steer kazancı.

Kavşak dönüşü ~6 m kala başlıyor (köşe ancak önceki wp geçilince current olur);
tetikleme mesafesi küçükse steer güçlendirilir. Viraj ve halka muaf.
"""
import sys, types
sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")
import mission_planning_node as mpn
from mission_planning_node import MissionPlanningNode

ok = fail = 0
def check(n, c):
    global ok, fail
    print(f"  {'PASS' if c else 'FAIL'} {n}"); ok += c; fail += (not c)

n = object.__new__(MissionPlanningNode)
n._turn_is_roundabout = False

print("── 1) Kazanç mesafeyle ölçekleniyor ──")
n._turn_engage_dist = 10.0; check("10 m -> 1.00x", abs(n._turn_late_gain() - 1.00) < 0.01)
n._turn_engage_dist = 6.0;  check("6 m -> 1.67x", abs(n._turn_late_gain() - 1.667) < 0.02)
n._turn_engage_dist = 4.0;  check("4 m -> tavan 1.80x", abs(n._turn_late_gain() - 1.80) < 0.01)
n._turn_engage_dist = 12.0; check("12 m (erken) -> 1.00x taban", abs(n._turn_late_gain() - 1.00) < 0.01)

print("\n── 2) Halka muaf ──")
n._turn_is_roundabout = True; n._turn_engage_dist = 6.0
check("halka -> 1.00x", abs(n._turn_late_gain() - 1.00) < 0.01)

print("\n── 3) engage_dist yoksa 1.00x (güvenli varsayılan) ──")
n._turn_is_roundabout = False; n._turn_engage_dist = None
check("None -> 1.00x", abs(n._turn_late_gain() - 1.00) < 0.01)

print(f"\n{'='*40}\nSONUÇ: {ok} PASS, {fail} FAIL\n{'='*40}")
sys.exit(1 if fail else 0)
