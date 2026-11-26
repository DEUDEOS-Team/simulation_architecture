#!/usr/bin/env python3
"""Durak cebi manevrası.

Durak (STOP) noktasına varınca araç sağa yanaşır (S manevrası), kısa süre bekler,
şeride döner. Manevra steer/hızı normal referansı ezer (vehicle_controller MANEUVER).
Burada iki parça test edilir:
  1) mission_planning _durak_step yön-tabanlı durum makinesi (faz geçişleri, done)
  2) mission_manager: STOP artık düz geçilmiyor; durak_active bayrağı, tamamlanınca advance
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mission_planning_node import (  # noqa: E402
    MissionPlanningNode,
    DURAK_TURN_DEG,
    DURAK_EXIT_TURN_DEG,
    DURAK_ALIGN_TOL_DEG,
    DURAK_STEER,
    DURAK_CRAWL_RATIO,
)

ok = fail = 0


def check(n, c):
    global ok, fail
    print(f"  {'PASS' if c else 'FAIL'} {n}")
    ok += c
    fail += (not c)


class _Log:
    def warn(self, *a, **k):
        pass

    def info(self, *a, **k):
        pass


# ── 1) _durak_step durum makinesi ──
print("── 1) _durak_step S manevrası ──")
n = object.__new__(MissionPlanningNode)
n._durak_phase = "idle"
n._durak_entry_heading = 0.0
n._durak_hold_until = 0.0
n._heading_deg = 100.0
n.get_logger = lambda: _Log()

s, v, d = n._durak_step()
check("giris: steer +SAĞ, hız sürünme", s == DURAK_STEER and v == DURAK_CRAWL_RATIO and not d)
check("giriş yönü yakalandı (100°)", abs(n._durak_entry_heading - 100.0) < 1e-9)

n._heading_deg = 100.0 + DURAK_TURN_DEG * 0.5
s, _, _ = n._durak_step()
check("yarı yolda hâlâ giris (+SAĞ)", n._durak_phase == "giris" and s > 0)

n._heading_deg = 100.0 + DURAK_TURN_DEG + 1
s, _, _ = n._durak_step()
check("yeterince döndü -> duzel_in (-SOL)", n._durak_phase == "duzel_in" and s < 0)

n._heading_deg = 100.0 + DURAK_ALIGN_TOL_DEG - 1
s, v, _ = n._durak_step()
check("düzeldi -> bekle (hız 0, steer 0)", n._durak_phase == "bekle" and v == 0.0 and s == 0.0)
check("bekleme sayacı kuruldu", n._durak_hold_until > 0)

n._durak_hold_until = time.monotonic() + 10
_, v, _ = n._durak_step()
check("bekleme dolmadan duruyor", n._durak_phase == "bekle" and v == 0.0)

n._durak_hold_until = time.monotonic() - 1
s, _, d = n._durak_step()
check("bekleme doldu -> cikis (-SOL)", n._durak_phase == "cikis" and s < 0 and not d)

# çıkışta sola DURAK_EXIT_TURN_DEG dönünce manevra biter (sağa savuran duzel_out YOK)
n._heading_deg = 100.0 - DURAK_EXIT_TURN_DEG - 1
_, _, d = n._durak_step()
check("burun şeride kırıldı -> tamam (done, idle)", d and n._durak_phase == "idle")

print("\n── 2) 360° sarma (entry 355°) ──")
n._durak_phase = "idle"
n._heading_deg = 355.0
n._durak_step()  # entry=355, faz=giris
n._heading_deg = (355.0 + DURAK_TURN_DEG + 2) % 360.0
n._durak_step()
check("355°+32° sarması giris->duzel_in", n._durak_phase == "duzel_in")

# ── 3) mission_manager durak akışı ──
print("\n── 3) mission_manager STOP -> durak manevrası ──")
from mission_manager import MissionManager  # noqa: E402
from waypoint_manager import GpsPosition, MissionPlan, MissionPoint, TaskType  # noqa: E402

durak = MissionPoint(index=0, point_id=0, name="durak_1", lat=40.7901161, lon=29.5089082,
                     task=TaskType.STOP, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=3.0)
nextp = MissionPoint(index=1, point_id=1, name="cp", lat=40.7902000, lon=29.5089082,
                     task=TaskType.CHECKPOINT, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=3.0)
mgr = MissionManager(MissionPlan(points=[durak, nextp]))

# Durak noktasının tam üstünde: arrived olmalı
pos = GpsPosition(lat=durak.lat, lon=durak.lon, heading_deg=0.0)
state, dec = mgr.update(pos, now_s=0.0)
check("durakta durak_active=True", dec.durak_active is True)
check("waypoint ilerlemedi (hâlâ durak)", state.current_task == TaskType.STOP)

# Bir tık daha: hâlâ tutuyor (manevra sürüyor)
state, dec = mgr.update(pos, now_s=1.0)
check("manevra sürerken hâlâ durak_active", dec.durak_active is True)

# Manevra bitti sinyali -> bir sonraki update advance etmeli
mgr.notify_durak_completed()
state, dec = mgr.update(pos, now_s=2.0)
check("tamamlanınca durak_active temizlendi", dec.durak_active is False)
check("waypoint ilerledi (checkpoint)", state.current_task == TaskType.CHECKPOINT)

print(f"\n{'='*40}\nSONUÇ: {ok} PASS, {fail} FAIL\n{'='*40}")
sys.exit(1 if fail else 0)
