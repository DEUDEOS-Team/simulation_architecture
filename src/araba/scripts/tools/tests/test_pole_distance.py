#!/usr/bin/env python3
"""LiDAR direk mesafesi testleri: aday çıkarımı + kamera bbox eşleştirmesi."""
import sys
import time
import types
import json

import numpy as np

sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")

ok = fail = 0


def check(name, cond):
    global ok, fail
    if cond:
        ok += 1
        print(f"  PASS {name}")
    else:
        fail += 1
        print(f"  FAIL {name}")


# ── 1) lidar_obstacle_node._publish_pole_candidates ──
from lidar_obstacle_node import LidarObstacleNode


class PubStub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data)


n = object.__new__(LidarObstacleNode)
n._eps = 0.5
n._min_pts = 5
n._max_dist = 20.0
n._bumper_offset = 1.1873
n._pole_min_z = 1.2
n._pole_max_lat = 8.0
n._pole_min_pts = 3
n._pole_max_diam = 1.0
n._pub_poles = PubStub()

cloud = []
# Trafik ışığı direği: x=11.2 (lidar), y=-2.5, z 1.3..2.6 (7 nokta üst üste)
for z in np.linspace(1.3, 2.6, 7):
    cloud.append([11.2, -2.5, z])
# Öndeki araç: x=8, y=0, z 0.2..1.1 (direk eşiğinin ALTINDA)
for z in np.linspace(0.2, 1.1, 10):
    for y in (-0.6, 0.0, 0.6):
        cloud.append([8.0, y, z])
# Zemin halkası
for x in np.linspace(2, 15, 30):
    cloud.append([x, 0.5, -0.5])
pts = np.array(cloud, dtype=np.float32)

n._publish_pole_candidates(pts)
poles = json.loads(n._pub_poles.msgs[-1])
check("tek direk adayı bulundu (araç/zemin elenmiş)", len(poles) == 1)
if poles:
    p = poles[0]
    check("direk mesafesi ~10.0 m (tampon ofsetli)", abs(p["distance_m"] - (11.2 - 1.1873)) < 0.01)
    check("direk lateral ~-2.5", abs(p["lateral_m"] + 2.5) < 0.01)
    check("z_max ~2.6", abs(p["z_max_m"] - 2.6) < 0.01)

# Ana engel kümeleme hâlâ (N,2) ile çalışıyor (geri uyumluluk)
cl2 = n._euclidean_cluster(pts[:, :2], min_pts=5)
check("(N,2) kümeleme geriye uyumlu", len(cl2) >= 1 and cl2[0].shape[1] == 2)

# ── 2) perception_pipeline._assign_light_distances_from_poles ──
from perception_pipeline_node import (
    PerceptionPipelineNode, POLE_CAM_CX_PX, POLE_CAM_FX_PX, POLE_CAM_X_OFF_M,
)

pp = object.__new__(PerceptionPipelineNode)
pp.LIDAR_TIMEOUT_S = 1.0
pp._poles = poles + [{"distance_m": 6.0, "lateral_m": 3.0, "z_max_m": 2.0}]  # solda başka direk
pp._poles_stamp = time.monotonic()

d_pole = poles[0]["distance_m"]
u_light = POLE_CAM_CX_PX - POLE_CAM_FX_PX * (-2.5) / (d_pole + POLE_CAM_X_OFF_M)
det = types.SimpleNamespace(estimated_distance_m=None,
                            bbox_px=(u_light - 15, 200, u_light + 15, 260))
pp._assign_light_distances_from_poles([det])
check("ışık bbox'ına direk mesafesi atandı", det.estimated_distance_m is not None
      and abs(det.estimated_distance_m - d_pole) < 0.01)

# Stereo mesafesi olan tespit ellenmnez
det2 = types.SimpleNamespace(estimated_distance_m=4.2, bbox_px=(600, 200, 680, 260))
pp._assign_light_distances_from_poles([det2])
check("stereo mesafeli tespit korunur", det2.estimated_distance_m == 4.2)

# Bakış yönü uyuşmayan bbox eşleşmez (bbox solda, direk sağda)
det3 = types.SimpleNamespace(estimated_distance_m=None, bbox_px=(100, 200, 160, 260))
pp._assign_light_distances_from_poles([det3])
check("yön uyuşmazsa eşleşme yok (bbox yedeğine kalır)", det3.estimated_distance_m is None)

# Bayat direk verisi kullanılmaz
pp._poles_stamp = time.monotonic() - 5.0
det4 = types.SimpleNamespace(estimated_distance_m=None,
                             bbox_px=(u_light - 15, 200, u_light + 15, 260))
pp._assign_light_distances_from_poles([det4])
check("bayat (5 s) direk verisi yok sayılır", det4.estimated_distance_m is None)

print(f"\n{ok} PASS / {fail} FAIL")
sys.exit(1 if fail else 0)
