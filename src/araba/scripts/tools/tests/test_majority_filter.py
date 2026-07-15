#!/usr/bin/env python3
"""Çoğunluk filtresi (avoidance_node._confirm_obstacles) — tabela salınımı.

Titreyen yol-kenarı tabela (~%25 görülür) kaçınmaya beslenmez; sürekli koni
(~%90, nadir kaybı dahil) beslenir. Kareler arası eşleme yanal+mesafe yakınlığıyla.
"""
import sys, types
sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")
from avoidance_node import AvoidanceNode
from gap_avoid_logic import GapObstacle

ok = fail = 0
def check(n, c):
    global ok, fail
    print(f"  {'PASS' if c else 'FAIL'} {n}"); ok += c; fail += (not c)

def fresh_node():
    n = object.__new__(AvoidanceNode)
    n._tracks = []
    return n

def cone(dist, lat, half=0.15):
    return GapObstacle(distance_m=dist, lateral_min_m=lat-half, lateral_max_m=lat+half)

print("── 1) Sürekli koni: birkaç kare sonra ONAYLANIR ──")
n = fresh_node()
out = []
for i in range(6):
    out = n._confirm_obstacles([cone(8.0-i*0.1, -0.5)])
check("6 kare sürekli koni -> onaylı (>=1 engel)", len(out) == 1)

print("\n── 2) Titreyen tabela (4 karede 1): ASLA onaylanmaz ──")
n = fresh_node()
seen_confirmed = False
for i in range(20):
    raw = [cone(7.0, -2.0)] if (i % 4 == 0) else []   # ~%25
    out = n._confirm_obstacles(raw)
    if out: seen_confirmed = True
check("titreyen tabela hiç onaylanmadı", not seen_confirmed)

print("\n── 3) Koni nadir kaybı (6'da 1 boş): onaylı KALIR ──")
n = fresh_node()
# önce onayla
for i in range(6): n._confirm_obstacles([cone(6.0, -0.5)])
# sonra bir kare kayıp, ertesi geri
n._confirm_obstacles([])                 # 1 kayıp
out = n._confirm_obstacles([cone(5.5, -0.5)])
check("tek kare kayıp sonrası koni hâlâ onaylı", len(out) == 1)

print("\n── 4) Onaylı koni gerçekten geçince düşer ──")
n = fresh_node()
for i in range(6): n._confirm_obstacles([cone(6.0, -0.5)])
# koni geçti: uzun süre boş
last = None
for i in range(6): last = n._confirm_obstacles([])
check("engel kalıcı gidince onay düşer", last == [])

print("\n── 5) İki ayrı koni ayrı izlenir (eşleme) ──")
n = fresh_node()
out = []
for i in range(6):
    out = n._confirm_obstacles([cone(8.0, -1.5), cone(8.0, +1.5)])
check("iki koni de onaylı", len(out) == 2)

print(f"\n{'='*44}\nSONUÇ: {ok} PASS, {fail} FAIL\n{'='*44}")
sys.exit(1 if fail else 0)
