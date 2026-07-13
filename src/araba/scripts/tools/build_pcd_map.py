#!/usr/bin/env python3
"""
benim_dunyam parkurunun PCD haritasını üretir (pcl_localization/NDT için).

Yöntem: aracı teknofest_centerlines.geojson yol ağı boyunca ~3 m aralıklarla
Gazebo'da ışınla (set_pose), her durakta bir LiDAR taraması al, /pose/info'dan
gelen gerçek dünya pozuyla (tam oryantasyon) ENU'ya çevir, voxel seyreltmeyle
biriktir.

Tamamen gz-transport üzerinden çalışır (ROS köprüsü gerekmez):
- /odom Ackermann eklentisinden geldiği için teleportu görmez -> poz /pose/info'dan
- set_pose servisinin Python cevabı güvenilmez (uygulanır ama False döner) ->
  başarı, pozun hedefe oturmasından doğrulanır.

Önkoşul: gz sim -s -r benim_dunyam.sdf + ros_gz create ile araç spawn edilmiş.
"""
import json
import math
import threading
import time

import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

import os
# workspace kökü dosya konumundan türetilir: src/araba/scripts/tools/ -> 4 üst dizin
WS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "..", "..", "..", ".."))
CENTERLINES = f"{WS}/src/araba/missions/teknofest_centerlines.geojson"
OUT_PCD = f"{WS}/src/araba/maps/benim_dunyam.pcd"
WORLD = "benim_dunyam"
MODEL = "araba"

DATUM_LAT, DATUM_LON = 40.7899, 29.5089
EARTH_R = 6371000.0
SAMPLE_STEP_M = 3.0          # örnekleme aralığı (yol boyunca)
TELEPORT_Z = 0.264           # ölçülen dinlenme z'si — sıfıra yakın düşüş, zıplama yok
POSE_TOL_M = 0.5             # poz hedefe bu kadar yaklaşınca ışınlama başarılı
STABLE_TIMEOUT_S = 2.5       # sallanma sönümünü bekleme üst sınırı
STABLE_POS_M = 0.02          # ardışık pozlar arası konum farkı eşiği
STABLE_QUAT_DOT = 0.99998    # ardışık quaternion hizası eşiği (~0.36°)
RANGE_MIN, RANGE_MAX = 1.0, 20.0
Z_MIN_W, Z_MAX_W = -0.3, 4.0  # dünya çerçevesinde tutulan z bandı
VOXEL_STRUCT = 0.15          # yapı noktaları (z > GROUND_Z) voxel kenarı
VOXEL_GROUND = 0.40          # zemin noktaları voxel kenarı (dosya şişmesin)
# 0.30: kalıntı eğim (~0.006 rad x 20 m ~= 0.12 m) zemini yapı sınıfına sızdırmasın
GROUND_Z = 0.30
# URDF lidar_sensor eklemi: base_link -> lidar_sensor_link (rpy=0)
LIDAR_OFF = np.array([-1.1873, -0.67509, 1.1889])


def ll_to_enu(lat, lon):
    dn = math.radians(lat - DATUM_LAT) * EARTH_R
    de = math.radians(lon - DATUM_LON) * EARTH_R * math.cos(math.radians(DATUM_LAT))
    return de, dn


def sample_poses():
    """Centerline'ları ENU'ya çevirip SAMPLE_STEP_M aralıkla (x, y, yaw) üret."""
    data = json.load(open(CENTERLINES))
    poses = []
    for f in data["features"]:
        g = f["geometry"]
        if g["type"] != "LineString":
            continue
        pts = [ll_to_enu(lat, lon) for lon, lat in g["coordinates"]]
        acc = 0.0
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-6:
                continue
            yaw = math.atan2(y1 - y0, x1 - x0)
            d = acc
            while d < seg:
                t = d / seg
                poses.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw))
                d += SAMPLE_STEP_M
            acc = d - seg
    return poses


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class GzLink:
    def __init__(self):
        self.node = GzNode()
        self._lock = threading.Lock()
        self._pose = None       # (pos(3), quat_xyzw(4))
        self._cloud = None      # (seq, width*height, point_step, offsets, bytes)
        self._cloud_seq = 0
        assert self.node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self._pose_cb)
        assert self.node.subscribe(PointCloudPacked, "/lidar/scan/points", self._pc_cb)

    def _pose_cb(self, msg):
        for p in msg.pose:
            if p.name == MODEL:
                q = p.orientation
                with self._lock:
                    self._pose = (
                        np.array([p.position.x, p.position.y, p.position.z]),
                        np.array([q.x, q.y, q.z, q.w]))
                return

    def _pc_cb(self, msg):
        off = {f.name: f.offset for f in msg.field}
        with self._lock:
            self._cloud_seq += 1
            self._cloud = (self._cloud_seq, msg.width * msg.height,
                           msg.point_step, off, bytes(msg.data))

    def pose(self):
        with self._lock:
            return None if self._pose is None else (
                self._pose[0].copy(), self._pose[1].copy())

    def set_pose_verified(self, x, y, yaw, timeout=3.0):
        """set_pose gönder; başarıyı pozun hedefe oturmasından doğrula."""
        req = Pose()
        req.name = MODEL
        req.position.x, req.position.y, req.position.z = x, y, TELEPORT_Z
        req.orientation.z, req.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)

        def _fire():
            try:
                self.node.request(f"/world/{WORLD}/set_pose", req, Pose, Boolean, 300)
            except Exception:
                pass

        threading.Thread(target=_fire, daemon=True).start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            p = self.pose()
            if p is not None and math.hypot(p[0][0] - x, p[0][1] - y) < POSE_TOL_M:
                return True
            time.sleep(0.05)
        return False

    def wait_fresh_cloud(self, timeout=5.0):
        """Şu andan SONRA başlayan taramayı döndür (bir tam tarama atla)."""
        with self._lock:
            start = self._cloud_seq
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self._lock:
                if self._cloud_seq >= start + 2:
                    return self._cloud
            time.sleep(0.02)
        return None

    def wait_stable(self, timeout=STABLE_TIMEOUT_S):
        """Süspansiyon sallanması sönene kadar bekle; kararlı pozu döndür."""
        prev = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.12)
            cur = self.pose()
            if cur is None:
                continue
            if prev is not None:
                dp = np.linalg.norm(cur[0] - prev[0])
                qd = abs(float(np.dot(cur[1], prev[1])))
                if dp < STABLE_POS_M and qd > STABLE_QUAT_DOT:
                    return cur
            prev = cur
        return None


def cloud_to_xyz(cloud):
    _, n, step, off, data = cloud
    buf = np.frombuffer(data, dtype=np.uint8).reshape(n, step)
    xyz = np.column_stack([
        buf[:, off[k]:off[k] + 4].copy().view(np.float32).ravel()
        for k in ("x", "y", "z")])
    return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    poses = sample_poses()
    print(f"{len(poses)} örnekleme pozu; çıktı: {OUT_PCD}", flush=True)

    gz = GzLink()
    time.sleep(1.0)  # abonelikler otursun
    vox = {}  # (leaf, ix, iy, iz) -> [sum_xyz, count]

    def add(points, leaf):
        keys = np.floor(points / leaf).astype(np.int64)
        for k, p in zip(map(tuple, keys), points):
            e = vox.get((leaf, k))
            if e is None:
                vox[(leaf, k)] = [p.copy(), 1]
            elif e[1] < 8:
                e[0] += p
                e[1] += 1

    ok = fail = 0
    t_start = time.monotonic()
    for i, (x, y, yaw) in enumerate(poses):
        if not gz.set_pose_verified(x, y, yaw):
            fail += 1  # araç fırladı/oturmadı (ör. bariyer üstü) -> atla
            continue
        mp = gz.wait_stable()
        if mp is None or math.hypot(mp[0][0] - x, mp[0][1] - y) > POSE_TOL_M:
            fail += 1
            continue
        cloud = gz.wait_fresh_cloud()
        mp2 = gz.pose()  # tarama sırasında poz oynadıysa (hâlâ sallanıyor) atla
        if (cloud is None or mp2 is None
                or np.linalg.norm(mp2[0] - mp[0]) > STABLE_POS_M
                or abs(float(np.dot(mp2[1], mp[1]))) < STABLE_QUAT_DOT):
            fail += 1
            continue
        mp = mp2
        pts = cloud_to_xyz(cloud)
        rng = np.linalg.norm(pts[:, :2], axis=1)
        pts = pts[(rng > RANGE_MIN) & (rng < RANGE_MAX)]
        pos, quat = mp
        R = quat_to_mat(quat)
        # lidar dünyada: p_w = R_base @ (LIDAR_OFF + p_lidar) + pos
        world = (pts + LIDAR_OFF) @ R.T + pos
        world = world[(world[:, 2] > Z_MIN_W) & (world[:, 2] < Z_MAX_W)]
        add(world[world[:, 2] > GROUND_Z], VOXEL_STRUCT)
        add(world[world[:, 2] <= GROUND_Z], VOXEL_GROUND)
        ok += 1
        if (i + 1) % 20 == 0:
            el = time.monotonic() - t_start
            print(f"  {i+1}/{len(poses)} poz | ok={ok} atlanan={fail} | "
                  f"voxel={len(vox)} | {el:.0f}s", flush=True)

    pts = np.array([e[0] / e[1] for e in vox.values()], dtype=np.float32)
    with open(OUT_PCD, "w") as f:
        f.write("# .PCD v0.7 - benim_dunyam parkuru (ground-truth teleport haritasi)\n"
                "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {len(pts)}\nDATA ascii\n")
        for p in pts:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    print(f"BITTI: {len(pts)} nokta, {ok} tarama, {fail} atlanan -> {OUT_PCD}", flush=True)


if __name__ == "__main__":
    main()
