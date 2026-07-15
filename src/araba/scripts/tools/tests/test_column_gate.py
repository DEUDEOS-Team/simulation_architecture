#!/usr/bin/env python3
"""Kolon testi + iki koruma (koni çarpma regresyonu).

Koşu-47: koni son yaklaşmada (d<5m) kolon/kamera vetosuyla elenip çarpıldı.
KORUMA 1: şerit-içi (|yanal|<1.2) küme ASLA vetolanmaz.
KORUMA 2: son karelerde ENGEL yayınlanan konuma yakın küme vetolanmaz.
Tabela (şerit dışı + onaysız + üstünde yüksek nokta) vetolanır.
"""
import sys, types
import numpy as np
sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")
from lidar_obstacle_node import LidarObstacleNode  # noqa: E402

ok = fail = 0
def check(n, c):
    global ok, fail
    print(f"  {'PASS' if c else 'FAIL'} {n}"); ok += c; fail += (not c)

n = object.__new__(LidarObstacleNode)
n._column_enable = True
n._column_radius_m = 0.3
n._column_min_pts = 2
n._column_in_lane_lat = 1.2
n._column_confirm_hold = 3.0
n._column_confirm_r = 1.0
n._lidar_off = np.array([-1.1873, -0.675])
n._pose_xy = None            # poz yok -> koruma 2 pasif, koruma 1 aktif
n._pose_yaw = 0.0
n._pose_stamp = 0.0
n._recent_obstacles = []
_t = [100.0]
n._now = types.MethodType(lambda self: _t[0], n)

# Tabela: yanal -2.0 (şerit dışı), üstünde 3 yüksek nokta AYNI xy'de
sign = np.array([[8.0, -2.0], [8.03, -1.98]])
tall_over_sign = np.array([[8.0, -2.0], [8.02, -2.01], [7.99, -1.99]])
# Koni: yanal -0.8 (şerit içi), üstünde de yüksek nokta OLSA BİLE korunmalı
cone_in = np.array([[8.0, -0.8], [8.03, -0.78]])

print("── 1) Tabela (şerit dışı, üstünde yüksek nokta) -> VETO ──")
n._tall_xy = tall_over_sign
check("şerit-dışı tabela vetolanır", n._column_vote(sign) is True)

print("\n── 2) KORUMA 1: şerit-içi koni, üstünde yüksek nokta olsa da -> KORUNUR ──")
n._tall_xy = np.array([[8.0, -0.8], [8.02, -0.81], [7.99, -0.79]])  # koninin üstünde de var
check("şerit-içi koni ASLA vetolanmaz", n._column_vote(cone_in) is False)

print("\n── 3) KORUMA 2: şerit-DIŞI ama ONAYLANMIŞ engel -> KORUNUR ──")
# poz ver, koni şerit dışına kaymış (viraj) ama daha önce engel yayınlanmış
n._pose_xy = np.array([0.0, 0.0]); n._pose_stamp = 100.0
far = np.array([[8.0, -2.0], [8.03, -1.98]])
n._tall_xy = tall_over_sign
# önce engel olarak hatırlat (bu kümenin dünya konumu)
n._remember_obstacle(far)
check("onaylanmış engel (şerit dışı olsa da) korunur", n._column_vote(far) is False)

print("\n── 4) Onaysız + şerit dışı + yüksek nokta yoksa -> tutulur ──")
n._recent_obstacles = []
n._tall_xy = None
check("yüksek nokta yok -> veto yok (koni gibi)", n._column_vote(sign) is False)

print("\n── 5) Onay BAYATLARSA koruma düşer (tabela geri vetolanır) ──")
n._pose_xy = np.array([0.0, 0.0]); n._pose_stamp = 100.0
n._tall_xy = tall_over_sign
n._remember_obstacle(far)
_t[0] = 100.0 + n._column_confirm_hold + 0.5   # onay bayatladı
check("bayat onay -> tabela tekrar vetolanır", n._column_vote(far) is True)

print(f"\n{'='*40}\nSONUÇ: {ok} PASS, {fail} FAIL\n{'='*40}")
sys.exit(1 if fail else 0)
