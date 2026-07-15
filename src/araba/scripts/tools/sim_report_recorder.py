#!/usr/bin/env python3
"""
Simülasyon rapor kaydedicisi — koşu sırasında sayısal çıktıları kaynak bazında ayırıp
CSV/JSONL/olay-günlüğü olarak yazar. Rapor için kullanım:

    python3 sim_report_recorder.py /yol/rapor_dizini

Üretilen dosyalar:
  lokalizasyon.csv  t_sim, ground-truth (Gazebo) ve EKF/final_odom pozu + hata metrikleri
  kontrol.csv       t_sim, kontrol zinciri: şerit komutu -> arbiter -> güvenlik sonrası /cmd_vel
  planlama.csv      t_sim, aktif görev, steering_ref, speed_limit, turn/park bayrakları
  kararlar.jsonl    /perception/decision_debug ham karar çıktısı (mimari şeması)
  olaylar.log       kaynak etiketli durum DEĞİŞİMLERİ (görev geçişi, ışık, engel, park...)

Ground truth doğrudan Gazebo /world/<w>/pose/info'dan alınır (ROS'a köprülenmez).
"""
import json
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String

try:
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    HAVE_GZ = True
except ImportError:
    HAVE_GZ = False

WORLD = "benim_dunyam"
MODEL = "araba"
SNAPSHOT_PERIOD_S = 0.1  # 10 Hz CSV örneklemesi


def yaw_from_quat(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def ang_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


class Recorder(Node):
    def __init__(self, out_dir):
        super().__init__("sim_report_recorder")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.out = out_dir
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.state = {}      # son değerler
        self.prev_event = {}  # olay-değişim takibi
        self._gt = None      # (x, y, yaw_deg) Gazebo ground truth
        self._lock = threading.Lock()

        self.f_loc = open(f"{out_dir}/lokalizasyon.csv", "w")
        self.f_loc.write("t_sim;gt_x;gt_y;gt_yaw_deg;loc_x;loc_y;loc_yaw_deg;"
                         "hata_xy_m;hata_yaw_deg;hiz_mps\n")
        self.f_ctl = open(f"{out_dir}/kontrol.csv", "w")
        self.f_ctl.write("t_sim;serit_v;serit_wz;arbiter_v;arbiter_wz;final_v;final_wz;"
                         "steering_ref;speed_limit\n")
        self.f_plan = open(f"{out_dir}/planlama.csv", "w")
        self.f_plan.write("t_sim;gorev;steering_ref;speed_limit;turn_active;arrived;"
                          "park_mode;park_kalan_s\n")
        self.f_dec = open(f"{out_dir}/kararlar.jsonl", "w")
        self.f_ev = open(f"{out_dir}/olaylar.log", "w")

        def sub(topic, mtype, key, event_src=None):
            def cb(msg):
                val = self._extract(msg)
                self.state[key] = val
                if event_src is not None:
                    self._event_on_change(event_src, key, val)
            self.create_subscription(mtype, topic, cb, qos)

        # lokalizasyon
        sub("/localization/odom/final", Odometry, "loc")
        sub("/kiss/odometry", Odometry, "kiss")
        # planlama kararları
        sub("/planning/current_task", String, "gorev", "PLANLAMA")
        sub("/planning/steering_ref", Float32, "steering_ref")
        sub("/planning/speed_limit", Float32, "speed_limit")
        sub("/planning/turn_active", Bool, "turn_active", "PLANLAMA")
        sub("/planning/arrived", Bool, "arrived", "PLANLAMA")
        sub("/planning/park_mode", Bool, "park_mode", "PLANLAMA")
        sub("/planning/park_remaining_s", Float32, "park_kalan_s")
        # algı kararları (KV metinleri)
        sub("/perception/traffic_light_state", String, "isik", "ALGI-IŞIK")
        sub("/perception/traffic_sign_state", String, "tabela", "ALGI-TABELA")
        sub("/perception/obstacle_state", String, "engel", "ALGI-ENGEL")
        sub("/perception/park_complete", Bool, "park_complete", "ALGI-PARK")
        # kontrol zinciri
        sub("/cmd_vel_lane", Twist, "serit_cmd")
        sub("/cmd_vel_raw", Twist, "arbiter_cmd")
        sub("/cmd_vel", Twist, "final_cmd")

        self.create_subscription(String, "/perception/decision_debug",
                                 self._decision_cb, qos)
        self.create_timer(SNAPSHOT_PERIOD_S, self._snapshot)

        if HAVE_GZ:
            self.gz = GzNode()
            self.gz.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self._gz_cb)

    # ---- yardımcılar ----
    @staticmethod
    def _extract(msg):
        if isinstance(msg, Odometry):
            p = msg.pose.pose
            return (p.position.x, p.position.y, yaw_from_quat(p.orientation),
                    math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y))
        if isinstance(msg, Twist):
            return (msg.linear.x, msg.angular.z)
        return msg.data

    def _gz_cb(self, msg):
        for p in msg.pose:
            if p.name == MODEL:
                q = p.orientation
                yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                              1 - 2 * (q.y * q.y + q.z * q.z)))
                with self._lock:
                    self._gt = (p.position.x, p.position.y, yaw)
                return

    def t_sim(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _event_on_change(self, src, key, val):
        sval = f"{val}"
        if isinstance(val, str) and len(sval) > 300:
            sval = sval[:300] + "…"
        if self.prev_event.get(key) != sval:
            self.prev_event[key] = sval
            self.f_ev.write(f"[{self.t_sim():9.2f}] {src:12s} | {key} = {sval}\n")
            self.f_ev.flush()

    def _decision_cb(self, msg):
        try:
            d = json.loads(msg.data)
            d["t_sim"] = round(self.t_sim(), 2)
            self.f_dec.write(json.dumps(d, ensure_ascii=False) + "\n")
            # önemli karar değişimlerini olay günlüğüne de ver
            fin = d.get("final", {})
            key_summary = (f"estop={fin.get('emergency_stop')} "
                           f"cap={fin.get('speed_cap_ratio')} "
                           f"steer_ovr={fin.get('has_steering_override')}")
            self._event_on_change("KARAR", "final", key_summary)
        except (ValueError, KeyError):
            pass

    def _snapshot(self):
        t = self.t_sim()
        with self._lock:
            gt = self._gt
        loc = self.state.get("loc")
        if gt is not None and loc is not None:
            err_xy = math.hypot(loc[0] - gt[0], loc[1] - gt[1])
            err_yaw = ang_diff(loc[2], gt[2])
            self.f_loc.write(f"{t:.2f};{gt[0]:.3f};{gt[1]:.3f};{gt[2]:.1f};"
                             f"{loc[0]:.3f};{loc[1]:.3f};{loc[2]:.1f};"
                             f"{err_xy:.3f};{err_yaw:.1f};{loc[3]:.2f}\n")
        lane = self.state.get("serit_cmd", ("", ""))
        raw = self.state.get("arbiter_cmd", ("", ""))
        fin = self.state.get("final_cmd", ("", ""))
        sref = self.state.get("steering_ref", "")
        slim = self.state.get("speed_limit", "")
        self.f_ctl.write(f"{t:.2f};{lane[0]};{lane[1]};{raw[0]};{raw[1]};"
                         f"{fin[0]};{fin[1]};{sref};{slim}\n")
        self.f_plan.write(f"{t:.2f};{self.state.get('gorev','')};{sref};{slim};"
                          f"{self.state.get('turn_active','')};"
                          f"{self.state.get('arrived','')};"
                          f"{self.state.get('park_mode','')};"
                          f"{self.state.get('park_kalan_s','')}\n")
        if int(t * 10) % 100 == 0:  # ~10 sn'de bir diske bas
            for f in (self.f_loc, self.f_ctl, self.f_plan):
                f.flush()


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "rapor_cikti"
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Recorder(out_dir)
    node.f_ev.write(f"# kayıt başladı (gz ground-truth: {'VAR' if HAVE_GZ else 'YOK'})\n")
    node.f_ev.flush()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for f in (node.f_loc, node.f_ctl, node.f_plan, node.f_dec, node.f_ev):
            f.flush()
            f.close()


if __name__ == "__main__":
    main()
