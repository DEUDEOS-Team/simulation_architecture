#!/usr/bin/env python3
"""Rota yeniden hesaplanırken dönüş bastırma testi.

"Sola dönülemez" kısıtı yüzünden rota bulunamayınca (no_route) eski plan korunuyor ve
araç yasak sol dönüşü yapmaya çalışırken rota da değişince köşede savruluyor. Kural:
rota oturmamışken (_route_unstable) kavşak dönüşü tetiklenmez, aktif dönüş bırakılır,
kontrol şerit takibine geçer. Halka muaf.
"""
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mission_planning_node import MissionPlanningNode  # noqa: E402


class LogStub:
    def info(self, *a, **k): pass
    def warn(self, *a, **k): pass
    def error(self, *a, **k): pass


def make_node():
    n = object.__new__(MissionPlanningNode)
    n.get_logger = lambda: LogStub()
    n._turn_active = False
    n._turn_is_roundabout = False
    n._turn_rb_exit_idx = None
    n._turn_wp_index = None
    n._turn_path = [(0.0, 0.0), (1.0, 0.0)]
    n._turn_path_origin = (0.0, 0.0)
    n._turn_arc_s_exit = 0.0
    n._turn_arc_s_here = 0.0
    n._curve_skip_logged_idx = -1
    n._plan_points = []
    n._route_unstable_until = 0.0
    n._no_route_until = 0.0
    return n


def wp(idx=3, b_err=-80.0, dist=5.0):
    return SimpleNamespace(
        wp_index=idx, bearing_error_deg=b_err, distance_to_wp_m=dist,
        current_wp=None, next_wp=None, prev_wp=None,
    )


fail = 0


def check(name, cond):
    global fail
    print(("  PASS " if cond else "  FAIL ") + name)
    if not cond:
        fail += 1


# 1) Rota oturmuşken aktif dönüş bırakılmaz (mevcut davranış korunuyor)
n = make_node()
n._turn_active = True
n._turn_wp_index = 2
n._update_turn_state(wp(idx=2))
check("rota oturmuş: dönüş sürüyor", n._turn_active is True)

# 2) Rota yeniden hesaplanırken AKTİF dönüş bırakılır -> şerit takibi
n = make_node()
n._turn_active = True
n._turn_wp_index = 2
n._route_unstable_until = time.monotonic() + 2.0
n._update_turn_state(wp(idx=2))
check("replan sırasında dönüş bırakıldı", n._turn_active is False)
check("dönüş yayı temizlendi", n._turn_path is None)

# 3) Rota oturmamışken YENİ dönüş tetiklenmez
n = make_node()
n._route_unstable_until = time.monotonic() + 2.0
n._update_turn_state(wp(idx=2))
check("replan sırasında yeni dönüş tetiklenmez", n._turn_active is False)

# 4) Halka MUAF: replan sırasında bile halka izi bırakılmaz
n = make_node()
n._turn_active = True
n._turn_is_roundabout = True
n._turn_rb_exit_idx = 21
n._turn_arc_s_exit = 60.0
n._turn_arc_s_here = 10.0
n._route_unstable_until = time.monotonic() + 2.0
n._update_turn_state(wp(idx=10))
check("halka replan'den etkilenmez", n._turn_active is True)

# 5) Soğuma bitince dönüşler tekrar açılır
n = make_node()
n._route_unstable_until = time.monotonic() - 0.1
check("soğuma bitti: rota oturmuş sayılır", n._route_unstable() is False)

print(f"\n{5 + 1 - fail} PASS / {fail} FAIL")
sys.exit(1 if fail else 0)
