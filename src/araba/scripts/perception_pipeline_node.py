#!/usr/bin/env python3
"""
perception_pipeline_node.py
---------------------------
Sensör çıktılarını alıp algoritma modüllerine besleyen MERKEZİ pipeline düğümü.

İş akışı:
  1. /perception/detections (Float32MultiArray) → parse → StereoBbox listesi
  2. perception_fusion.fuse() ile sınıflandır (ışık / tabela / engel)
  3. TrafficLightLogic, TrafficSignLogic, ObstacleLogic modüllerini güncelle
  4. Durum çıktılarını topic'lere bas + tkinter GUI dashboard'da canlı göster

Dashboard: tkinter tabanlı, koyu tema, 4 bölümlü canlı panel.
Bu dosya SADECE altyapıdır — ROS iletişimi, veri parse, modül orkestrasyonu.
Algoritma mantığı logic modüllerindedir, burada değiştirilmez.
"""

import os
import sys

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, Float32MultiArray, String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import math
import time
import tkinter as tk
from tkinter import ttk, font as tkfont
from PIL import Image as PILImage, ImageTk
import numpy as np

import json

# ─── Algoritma modülleri (sadece çağrılır, mantık değiştirilmez) ───
from perception_fusion import PerceptionFrame, fuse, parking_detections_from_signs
from sensors.types import StereoBbox, ImuSample, LidarObstacle

from obstacle_logic import (
    ObstacleLogic, ObstacleState,
    ObstacleBehavior, ObstacleKind,
)
from traffic_light_logic import (
    TrafficLightLogic, TrafficLightState, LightColor,
)
from traffic_sign_logic import (
    TrafficSignLogic, TrafficSignState,
)
from parking_logic import ParkingLogic, ParkState
from safety_logic import CORRIDOR_HALF_WIDTH_M
from decision_arbiter import Candidate, DecisionArbiter, ReasonCode

# ─── AYARLAR ─────────────────────────────────────────────────
GUI_REFRESH_MS   = 100     # GUI yenileme periyodu (ms) = 10Hz
STATE_PUBLISH_HZ = 5.0     # durum topic'lerini yayınlama frekansı
CAMERA_WIDTH     = 420     # kamera önizleme genişliği
CAMERA_HEIGHT    = 320     # kamera önizleme yüksekliği
# Slalom/kaçınma direksiyonu ancak engel bu mesafeye girince devreye girer
# (daha uzaktaki engel için erken weave başlatma — hız tavanı zaten yavaşlatır)
SLALOM_ENGAGE_DISTANCE_M = 8.0

# Trafik ışığı mesafesi: LiDAR "direk adayı" kümeleri (lidar_obstacle_node
# /perception/pole_candidates) kamera ışık bbox'ının bakış yönüne projeksiyon ile
# eşleştirilir; eşleşen direğin LiDAR mesafesi kullanılır. Stereo mesafe varsa
# dokunulmaz; eşleşme yoksa traffic_light_logic'in bbox-yükseklik yedeği devrededir.
POLE_CAM_FX_PX = 917.4        # kamera odak (1280x720, parking_logic ile aynı)
POLE_CAM_CX_PX = 640.0
POLE_CAM_X_OFF_M = 1.2724     # ön tampon -> kamera geri ofseti (URDF)
POLE_MATCH_MAX_PX = 40.0      # bbox merkezi ile direk projeksiyonu arası tolerans
POLE_MATCH_MAX_DIST_M = 25.0  # bu mesafeden uzak direk eşleşmesi kullanılmaz


# Kamera node'undaki sınıf ID → isim eşlemesi (parse için gerekli)
CLASS_NAMES = {
    0:  'ada etrafinda donunuz',  1:  'dur tabelasi',
    2:  'durak',                   3:  'girilmez',
    4:  'iki yonlu yol',           5:  'ileri mecburi',
    6:  'ileri ve saga mecburi yon', 7:  'ileri ve sola mecburi yon',
    8:  'ileriden saga mecburi yon', 9:  'ileriden sola mecburi yon',
    10: 'isikli isaret cihazi',    11: 'kirmizi isik',
    12: 'park',                    13: 'park yapilmaz',
    14: 'sag seridin sonu',        15: 'saga donulmez',
    16: 'saga mecburi',            17: 'sagdan gidin',
    18: 'sari isik',               19: 'sol seridin sonu',
    20: 'sola donulmez',           21: 'sola mecburi',
    22: 'soldan gidin',            23: 'tunel',
    24: 'yaya gecidi',             25: 'yesil isik',
    26: 'trafik konisi',           27: 'bariyer',
    28: 'yaya',
}


# ═══════════════════════════════════════════════════════════════════
# TKINTER GUI SINIFI
# ═══════════════════════════════════════════════════════════════════

class DashboardGUI:
    """4 bölümlü koyu tema tkinter dashboard."""

    # ── Tema renkleri ──
    BG        = "#1a1a2e"   # ana arka plan
    HEADER_BG = "#16213e"   # bölüm başlığı
    FRAME_BG  = "#0f3460"   # bölüm çerçevesi
    TEXT      = "#e0e0e0"   # normal metin
    TITLE_C   = "#ffffff"   # başlık
    ACCENT    = "#e94560"   # vurgu
    GREEN     = "#00ff88"
    YELLOW    = "#ffcc00"
    RED       = "#ff3333"
    ORANGE    = "#ff8c00"
    BLUE      = "#4da6ff"
    DIM       = "#666680"
    LIGHT_BG  = "#2a2a4a"

    def __init__(self, node: 'PerceptionPipelineNode'):
        self._node = node
        self._running = True

        # ── Ana pencere ──
        self._root = tk.Tk()
        self._root.title("Perception Pipeline Dashboard")
        self._root.geometry("1350x680")
        self._root.configure(bg=self.BG)
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        # ── Fontlar ──
        self._font_title = tkfont.Font(family="DejaVu Sans", size=16, weight="bold")
        self._font_header = tkfont.Font(family="DejaVu Sans", size=12, weight="bold")
        self._font_normal = tkfont.Font(family="DejaVu Sans", size=10)
        self._font_small = tkfont.Font(family="DejaVu Sans", size=9)
        self._font_big = tkfont.Font(family="DejaVu Sans", size=22, weight="bold")

        # ── Üst çerçeve (başlık + bağlantı durumu) ──
        self._build_header()

        # ── Ana yatay container: sol 2x2 grid + sağ kamera ──
        self._body = tk.Frame(self._root, bg=self.BG)
        self._body.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # Sol: 2x2 grid
        self._main_frame = tk.Frame(self._body, bg=self.BG)
        self._main_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Sağ: Kamera paneli
        self._build_camera_panel()

        # Sol üst: Trafik Işığı
        self._light_frame, self._light_widgets = self._create_section(
            self._main_frame, "TRAFİK IŞIĞI", 0, 0)

        # Sağ üst: Trafik Tabelası
        self._sign_frame, self._sign_widgets = self._create_section(
            self._main_frame, "TRAFİK TABELASI", 0, 1)

        # Sol alt: Engel
        self._obs_frame, self._obs_widgets = self._create_section(
            self._main_frame, "ENGEL / GÜZERGAH", 1, 0)

        # Sağ alt: Sensör
        self._sensor_frame, self._sensor_widgets = self._create_section(
            self._main_frame, "SENSÖR DURUMU", 1, 1)

        # Grid ağırlıkları
        self._main_frame.grid_rowconfigure(0, weight=1)
        self._main_frame.grid_rowconfigure(1, weight=1)
        self._main_frame.grid_columnconfigure(0, weight=1)
        self._main_frame.grid_columnconfigure(1, weight=1)

        # ── Trafik ışığı özel widget'ları ──
        self._build_light_section()

        # ── Engel özel widget'ları ──
        self._build_obstacle_section()

        # ── Sensör özel widget'ları ──
        self._build_sensor_section()

        # ── Periyodik güncellemeyi başlat ──
        self._root.after(GUI_REFRESH_MS, self._tick)

        self._node.get_logger().info('tkinter dashboard olusturuldu')

    def _build_header(self):
        """Başlık çubuğu ve bağlantı durumu."""
        header = tk.Frame(self._root, bg=self.HEADER_BG, height=70)
        header.pack(fill=tk.X, padx=0, pady=0)
        header.pack_propagate(False)

        inner = tk.Frame(header, bg=self.HEADER_BG)
        inner.pack(fill=tk.BOTH, expand=True, padx=15, pady=5)

        self._title_label = tk.Label(
            inner, text="PERCEPTION PIPELINE DASHBOARD",
            font=self._font_title, fg=self.TITLE_C, bg=self.HEADER_BG)
        self._title_label.pack(side=tk.LEFT)

        self._conn_label = tk.Label(
            inner, text="BASLATILIYOR...",
            font=self._font_small, fg=self.ORANGE, bg=self.HEADER_BG)
        self._conn_label.pack(side=tk.RIGHT, padx=10)

    def _build_camera_panel(self):
        """Sağ panel: canlı kamera önizleme."""
        cam_frame = tk.Frame(self._body, bg=self.FRAME_BG, bd=2, relief=tk.RIDGE, width=430)
        cam_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(4, 0))
        cam_frame.pack_propagate(False)

        # Başlık
        cam_title = tk.Frame(cam_frame, bg=self.HEADER_BG)
        cam_title.pack(fill=tk.X)
        tk.Label(cam_title, text="KAMERA GORUNTUSU", font=self._font_header,
                 fg=self.TITLE_C, bg=self.HEADER_BG, anchor=tk.W,
                 padx=10, pady=4).pack(fill=tk.X)

        # Görüntü alanı
        cam_content = tk.Frame(cam_frame, bg="#000000")
        cam_content.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)

        self._camera_label = tk.Label(cam_content, bg="#000000")
        self._camera_label.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        # Alt bilgi
        cam_info = tk.Frame(cam_frame, bg=self.LIGHT_BG)
        cam_info.pack(fill=tk.X)
        self._camera_info_var = tk.StringVar(value="Bekleniyor...")
        tk.Label(cam_info, textvariable=self._camera_info_var,
                 font=self._font_small, fg=self.DIM, bg=self.LIGHT_BG,
                 anchor=tk.W, padx=8, pady=2).pack(fill=tk.X)

        # Placeholder görüntü
        self._camera_tk_image: ImageTk.PhotoImage | None = None

    def _create_section(self, parent, title, row, col):
        """Bölüm çerçevesi oluştur, içerik frame'ini döndür."""
        outer = tk.Frame(parent, bg=self.FRAME_BG, bd=2, relief=tk.RIDGE)
        outer.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)

        # Başlık
        title_bar = tk.Frame(outer, bg=self.HEADER_BG)
        title_bar.pack(fill=tk.X)
        tk.Label(title_bar, text=title, font=self._font_header,
                 fg=self.TITLE_C, bg=self.HEADER_BG, anchor=tk.W,
                 padx=10, pady=4).pack(fill=tk.X)

        # İçerik alanı
        content = tk.Frame(outer, bg=self.LIGHT_BG)
        content.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)

        widgets = {}
        return outer, widgets

    def _build_light_section(self):
        """Trafik ışığı bölümü — canvas üzerinde ışık göstergesi."""
        content = self._light_frame.winfo_children()[1]  # iç Frame

        # Işık canvas'ı
        self._light_canvas = tk.Canvas(
            content, width=100, height=140, bg=self.LIGHT_BG,
            highlightthickness=0)
        self._light_canvas.pack(side=tk.LEFT, padx=15, pady=10)

        # Üç ışık dairesi (kapalı halde)
        self._light_red_circle = self._light_canvas.create_oval(
            20, 10, 80, 55, fill="#440000", outline="#660000", width=2)
        self._light_yellow_circle = self._light_canvas.create_oval(
            20, 55, 80, 100, fill="#444400", outline="#666600", width=2)
        self._light_green_circle = self._light_canvas.create_oval(
            20, 100, 80, 145, fill="#004400", outline="#006600", width=2)

        # Işık etiketleri
        self._light_canvas.create_text(50, 32, text="KIRMIZI",
                                       fill=self.DIM, font=self._font_small)
        self._light_canvas.create_text(50, 77, text="SARI",
                                       fill=self.DIM, font=self._font_small)
        self._light_canvas.create_text(50, 122, text="YESIL",
                                       fill=self.DIM, font=self._font_small)

        # Durum metni
        self._light_texts = tk.Frame(content, bg=self.LIGHT_BG)
        self._light_texts.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self._light_state_var = tk.StringVar(value="Isik tespiti YOK")
        tk.Label(self._light_texts, textvariable=self._light_state_var,
                 font=self._font_normal, fg=self.DIM, bg=self.LIGHT_BG,
                 anchor=tk.W, wraplength=280, justify=tk.LEFT).pack(
            fill=tk.X, pady=2)

        self._light_detail_var = tk.StringVar(value="")
        tk.Label(self._light_texts, textvariable=self._light_detail_var,
                 font=self._font_small, fg=self.DIM, bg=self.LIGHT_BG,
                 anchor=tk.W, wraplength=280, justify=tk.LEFT).pack(
            fill=tk.X, pady=1)

        self._light_speed_var = tk.StringVar(value="")
        tk.Label(self._light_texts, textvariable=self._light_speed_var,
                 font=self._font_small, fg=self.BLUE, bg=self.LIGHT_BG,
                 anchor=tk.W).pack(fill=tk.X, pady=1)

    def _build_obstacle_section(self):
        """Engel bölümü — büyük davranış göstergesi."""
        content = self._obs_frame.winfo_children()[1]

        self._obs_behavior_var = tk.StringVar(value="CLEAR")
        self._obs_behavior_label = tk.Label(
            content, textvariable=self._obs_behavior_var,
            font=self._font_big, fg=self.GREEN, bg=self.LIGHT_BG,
            anchor=tk.CENTER)
        self._obs_behavior_label.pack(fill=tk.X, pady=5)

        self._obs_detail_var = tk.StringVar(value="Engel tespiti YOK")
        tk.Label(content, textvariable=self._obs_detail_var,
                 font=self._font_small, fg=self.DIM, bg=self.LIGHT_BG,
                 anchor=tk.W, wraplength=400, justify=tk.LEFT).pack(
            fill=tk.X, padx=10, pady=2)

        self._obs_threat_var = tk.StringVar(value="")
        tk.Label(content, textvariable=self._obs_threat_var,
                 font=self._font_small, fg=self.DIM, bg=self.LIGHT_BG,
                 anchor=tk.W).pack(fill=tk.X, padx=10, pady=1)

    def _build_sensor_section(self):
        """Sensör bölümü — hız, IMU, frame sayısı."""
        content = self._sensor_frame.winfo_children()[1]

        self._sensor_speed_var = tk.StringVar(value="Hiz: -- km/h")
        tk.Label(content, textvariable=self._sensor_speed_var,
                 font=self._font_normal, fg=self.TEXT, bg=self.LIGHT_BG,
                 anchor=tk.W).pack(fill=tk.X, padx=10, pady=(10, 2))

        self._sensor_imu_var = tk.StringVar(value="IMU: veri bekleniyor")
        tk.Label(content, textvariable=self._sensor_imu_var,
                 font=self._font_normal, fg=self.TEXT, bg=self.LIGHT_BG,
                 anchor=tk.W).pack(fill=tk.X, padx=10, pady=2)

        self._sensor_frame_var = tk.StringVar(value="Kare: 0")
        tk.Label(content, textvariable=self._sensor_frame_var,
                 font=self._font_normal, fg=self.TEXT, bg=self.LIGHT_BG,
                 anchor=tk.W).pack(fill=tk.X, padx=10, pady=2)

        self._sensor_det_var = tk.StringVar(value="Tespit: --")
        tk.Label(content, textvariable=self._sensor_det_var,
                 font=self._font_normal, fg=self.TEXT, bg=self.LIGHT_BG,
                 anchor=tk.W).pack(fill=tk.X, padx=10, pady=2)

        # Alt bilgi
        tk.Label(content, text="ESC veya [X] ile kapat  |  perception_pipeline_node",
                 font=self._font_small, fg=self.DIM, bg=self.LIGHT_BG,
                 anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)

    def _tick(self):
        """Periyodik: ROS spin + GUI güncelle (tkinter after() ile çağrılır)."""
        if not self._running:
            return

        # ROS callback'lerini işle
        try:
            rclpy.spin_once(self._node, timeout_sec=0.001)
        except Exception:
            pass

        # GUI'yi güncelle
        self._refresh()

        # Tekrar zamanla
        self._root.after(GUI_REFRESH_MS, self._tick)

    def _refresh(self):
        """Tüm GUI elemanlarını node'daki son durumla güncelle."""
        node = self._node

        # ── Bağlantı durumu ──
        now = time.time()
        if node._last_detection_time is not None:
            age = now - node._last_detection_time
            if age < 1.0:
                self._conn_label.config(
                    text=f"● VERI AKTIF  |  Kare #{node._frame_count}  |  {node._detection_count} tespit",
                    fg=self.GREEN)
            elif age < 3.0:
                self._conn_label.config(
                    text=f"◉ GECIKMELI ({age:.1f}s)",
                    fg=self.YELLOW)
            else:
                self._conn_label.config(
                    text=f"◎ VERI YOK ({age:.0f}s)  |  Kamera calisiyor mu?",
                    fg=self.RED)
        else:
            self._conn_label.config(
                text="◌ VERI BEKLENIYOR...  /perception/detections",
                fg=self.ORANGE)

        # ── Trafik Işığı ──
        self._refresh_light_section(node._last_light_state)

        # ── Trafik Tabelası ──
        self._refresh_sign_section(node._last_sign_state)

        # ── Engel ──
        self._refresh_obstacle_section(node._last_obstacle_state)

        # ── Sensör ──
        self._refresh_sensor_section(node)

        # ── Kamera ──
        self._refresh_camera(node)

    def _refresh_light_section(self, ls: TrafficLightState | None):
        # Işıkları sıfırla
        self._light_canvas.itemconfig(self._light_red_circle, fill="#440000")
        self._light_canvas.itemconfig(self._light_yellow_circle, fill="#444400")
        self._light_canvas.itemconfig(self._light_green_circle, fill="#004400")

        if ls is None or not ls.active_color:
            self._light_state_var.set("Isik tespiti YOK")
            self._light_detail_var.set("Kamerada trafik isigi gorulmedi")
            self._light_speed_var.set("")
            return

        color = ls.active_color
        if color == LightColor.RED:
            self._light_canvas.itemconfig(self._light_red_circle, fill=self.RED)
            self._light_state_var.set("KIRMIZI — DUR!")
            self._light_detail_var.set(ls.reason[:80])
        elif color == LightColor.YELLOW:
            self._light_canvas.itemconfig(self._light_yellow_circle, fill=self.YELLOW)
            self._light_state_var.set("SARI — DIKKAT")
            self._light_detail_var.set(ls.reason[:80])
        elif color == LightColor.GREEN:
            self._light_canvas.itemconfig(self._light_green_circle, fill=self.GREEN)
            self._light_state_var.set("YESIL — GEC")
            self._light_detail_var.set(ls.reason[:80])

        dist_str = f"{ls.last_distance_m:.1f}m" if ls.last_distance_m else "?"
        self._light_speed_var.set(
            f"Hiz tavan: {ls.speed_cap_ratio:.0%}  |  Mesafe: {dist_str}  |  "
            f"must_stop={ls.must_stop} can_go={ls.can_go}")

    def _refresh_sign_section(self, ss: TrafficSignState | None):
        # Mevcut sign widget'larını temizle
        content = self._sign_frame.winfo_children()[1]
        for w in content.winfo_children():
            w.destroy()

        if ss is None or not ss.active_signs:
            tk.Label(content, text="Tabela tespiti YOK",
                     font=self._font_normal, fg=self.DIM, bg=self.LIGHT_BG,
                     anchor=tk.W).pack(fill=tk.X, padx=10, pady=10)
            return

        # Aktif tabela listesi
        for sign in ss.active_signs[:5]:
            tk.Label(content, text=f"  ►  {sign}",
                     font=self._font_normal, fg=self.YELLOW, bg=self.LIGHT_BG,
                     anchor=tk.W).pack(fill=tk.X, padx=10, pady=2)

        # Durum satırı
        flags = []
        if ss.must_stop_soon:
            flags.append("DUR")
        if ss.current_area_is_parking:
            flags.append("PARK")
        if ss.approaching_tunnel:
            flags.append("TUNEL")
        if ss.traffic_light_expected:
            flags.append("ISIKLI_KAVSAK")

        status_text = f"Hiz tavan: {ss.speed_cap_ratio:.0%}"
        if flags:
            status_text += f"  |  {' '.join(flags)}"
        tk.Label(content, text=status_text,
                 font=self._font_small, fg=self.BLUE, bg=self.LIGHT_BG,
                 anchor=tk.W).pack(fill=tk.X, padx=10, pady=(8, 2))

        if ss.reasons:
            tk.Label(content, text=f"Sebep: {'; '.join(ss.reasons[:3])}"[:90],
                     font=self._font_small, fg=self.DIM, bg=self.LIGHT_BG,
                     anchor=tk.W, wraplength=400).pack(fill=tk.X, padx=10, pady=1)

    def _refresh_obstacle_section(self, os_: ObstacleState | None):
        if os_ is None:
            self._obs_behavior_var.set("--")
            self._obs_behavior_label.config(fg=self.DIM)
            self._obs_detail_var.set("Engel tespiti YOK")
            self._obs_threat_var.set("")
            return

        behavior = os_.behavior_mode.upper()
        self._obs_behavior_var.set(behavior)

        # Renk
        if os_.emergency_stop:
            self._obs_behavior_label.config(fg=self.RED)
        elif os_.behavior_mode == ObstacleBehavior.CLEAR:
            self._obs_behavior_label.config(fg=self.GREEN)
        else:
            self._obs_behavior_label.config(fg=self.YELLOW)

        # Detay
        closest = f"{os_.closest_obstacle_m:.1f}m" if os_.closest_obstacle_m else "-"
        types = []
        if os_.pedestrian_in_corridor:
            types.append("Yaya")
        if os_.cone_in_corridor:
            types.append("Koni")
        if os_.barrier_in_corridor:
            types.append("Bariyer")

        detail = f"Hiz tavan: {os_.speed_cap_ratio:.0%}  |  En yakin: {closest}"
        if types:
            detail += f"  |  Koridorda: {', '.join(types)}"
        self._obs_detail_var.set(detail)

        # Tehdit
        threat_parts = [f"Tehdit: {os_.threat_level}"]
        if os_.avoidance_direction:
            threat_parts.append(f"Kacinma: {os_.avoidance_direction}")
        if os_.road_blocked:
            threat_parts.append("YOL KAPALI!")
        if os_.suggest_lane_change:
            threat_parts.append("Serit degisimi oneriliyor")
        self._obs_threat_var.set("  |  ".join(threat_parts))

    def _refresh_sensor_section(self, node):
        speed_kmh = node._vehicle_speed_mps * 3.6
        self._sensor_speed_var.set(f"Hiz: {speed_kmh:.1f} km/h")

        if node._imu_heading_deg is not None:
            self._sensor_imu_var.set(f"IMU yon: {node._imu_heading_deg:.1f}°")
        else:
            self._sensor_imu_var.set("IMU: veri bekleniyor")

        self._sensor_frame_var.set(f"Kare: {node._frame_count}")

        frame = node._last_perception_frame
        if frame:
            n_s = len(frame.sign_dets)
            n_l = len(frame.light_dets)
            n_o = len(frame.obstacle_dets)
            self._sensor_det_var.set(f"Son kare: {n_s} tabela, {n_l} isik, {n_o} engel")
        else:
            self._sensor_det_var.set("Tespit: henuz kare islenmedi")

    def _refresh_camera(self, node: 'PerceptionPipelineNode'):
        """Kamera görüntüsünü tkinter label'da göster."""
        frame = node._latest_camera_frame
        if frame is None:
            self._camera_info_var.set("Kamera verisi bekleniyor...")
            return

        try:
            # BGR → RGB
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Yeniden boyutlandır (en-boy oranını koru)
            h, w = rgb.shape[:2]
            scale = min(CAMERA_WIDTH / w, CAMERA_HEIGHT / h)
            new_w, new_h = int(w * scale), int(h * scale)
            rgb = cv2.resize(rgb, (new_w, new_h), interpolation=cv2.INTER_AREA)

            # PIL Image → ImageTk
            pil_img = PILImage.fromarray(rgb)
            self._camera_tk_image = ImageTk.PhotoImage(pil_img)
            self._camera_label.config(image=self._camera_tk_image)

            self._camera_info_var.set(
                f"{w}x{h}  |  Frame #{node._camera_frame_count}  |  "
                f"Tespit: {node._detection_count} nesne")
        except Exception as e:
            self._camera_info_var.set(f"Hata: {e}")

    def _on_close(self):
        """Pencere kapatıldığında."""
        self._running = False
        self._root.destroy()

    def run(self):
        """GUI ana döngüsünü başlat."""
        self._root.mainloop()
        self._running = False


# ═══════════════════════════════════════════════════════════════════
# ROS 2 NODE
# ═══════════════════════════════════════════════════════════════════

class PerceptionPipelineNode(Node):
    """Sensör → Fusion → Logic modüllerini birbirine bağlayan ana düğüm."""

    def __init__(self):
        super().__init__('perception_pipeline_node')

        # ── Parametreler ──
        self.declare_parameter('show_dashboard', True)
        self._show_dashboard = self.get_parameter('show_dashboard').get_parameter_value().bool_value
        # Slalom direksiyon adayı anahtarı: engel-geçme testlerinde slalom weave'i
        # karışabildiğinden kapatılabilir olsun; slalom parkuru çalışılırken launch'tan açılır.
        self.declare_parameter('enable_slalom', True)
        self._enable_slalom = bool(self.get_parameter('enable_slalom').value)

        # ── cv_bridge ──
        self._bridge = CvBridge()

        # ── Algoritma modülü örnekleri (tekil, stateful) ──
        self._traffic_light_logic = TrafficLightLogic()
        self._traffic_sign_logic = TrafficSignLogic()
        self._obstacle_logic = ObstacleLogic()
        self._parking_logic = ParkingLogic()
        self._decision_arbiter = DecisionArbiter()

        # ── Son bilinen durumlar ──
        self._last_light_state: TrafficLightState | None = None
        self._last_sign_state: TrafficSignState | None = None
        self._last_obstacle_state: ObstacleState | None = None
        self._last_park_state: ParkState | None = None
        self._park_mode: bool = False
        self._last_perception_frame: PerceptionFrame | None = None
        self._vehicle_speed_mps: float = 0.0
        self._imu_heading_deg: float | None = None
        # LiDAR engelleri (lidar_obstacle_node'dan JSON) — mimari kalıbıyla aynı
        self._lidar: list[LidarObstacle] = []
        self._lidar_stamp: float = 0.0
        self.LIDAR_TIMEOUT_S = 1.0   # bu süreden eski lidar verisi kullanılmaz
        # LiDAR direk adayları (trafik ışığı mesafe eşleştirmesi için)
        self._poles: list[dict] = []
        self._poles_stamp: float = 0.0
        self._last_detection_time: float | None = None
        self._detection_count: int = 0
        self._frame_count: int = 0
        self._latest_camera_frame: np.ndarray | None = None   # en son kamera karesi (BGR)
        self._camera_frame_count: int = 0

        # ── ROS Abonelikler ──
        self._sub_dets = self.create_subscription(
            Float32MultiArray, '/perception/detections', self._detections_callback, 10)

        # Kamera karesi yalnız dashboard içindir. Dashboard kapalıyken abone olma:
        # her kare 1280x720 kopyalanıp cv_bridge'den geçiyor ve WSLg'de bu yük şerit
        # takip penceresini donduruyor.
        self._sub_camera = None
        if self._show_dashboard:
            self._sub_camera = self.create_subscription(
                Image, '/camera/image', self._camera_callback, 10)

        self._sub_imu = self.create_subscription(
            Imu, '/imu/data', self._imu_callback, 10)

        self._sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        self._sub_lidar = self.create_subscription(
            String, '/perception/lidar_obstacles', self._lidar_callback, 10)

        self._sub_poles = self.create_subscription(
            String, '/perception/pole_candidates', self._poles_callback, 10)

        # Planning → Perception: park arama/manevra modu (mimari planning_park_mode)
        self._sub_park_mode = self.create_subscription(
            Bool, '/planning/park_mode', self._park_mode_callback, 10)

        # Planning → Perception: kısıtın bağlandığı kavşak geçildi — tabela dönüş
        # kısıtlarını temizle (yoksa kısıt yapışkan kalıp sonraki kavşakları da bloklar)
        self._sub_intersection_passed = self.create_subscription(
            Bool, '/planning/intersection_passed', self._intersection_passed_callback, 10)

        # ── ROS Yayıncılar ──
        self._pub_light = self.create_publisher(
            String, '/perception/traffic_light_state', 10)

        self._pub_sign = self.create_publisher(
            String, '/perception/traffic_sign_state', 10)

        self._pub_obstacle = self.create_publisher(
            String, '/perception/obstacle_state', 10)

        # Mimari arayüzü (mission_planning bunlara abone — launch parametreleriyle eşleşik):
        # turn_permissions/decision_debug JSON şeması perception_fusion_node ile birebir aynı.
        self._pub_turn_permissions = self.create_publisher(
            String, '/perception/turn_permissions', 10)
        self._pub_decision_debug = self.create_publisher(
            String, '/perception/decision_debug', 10)
        self._pub_park_complete = self.create_publisher(
            Bool, '/perception/park_complete', 10)

        # ── State publish timer'ı ──
        state_period = 1.0 / STATE_PUBLISH_HZ
        self._state_timer = self.create_timer(state_period, self._publish_states_timer_callback)

        self.get_logger().info(
            f'PerceptionPipelineNode baslatildi. '
            f'Dashboard: {"acik" if self._show_dashboard else "kapali"}')

    # ═══════════════════════════════════════════════════════════════
    # Sensör callback'leri
    # ═══════════════════════════════════════════════════════════════

    def _detections_callback(self, msg: Float32MultiArray) -> None:
        now = time.time()
        self._last_detection_time = now
        self._frame_count += 1

        stereo_bboxes = self._parse_detections(msg.data)
        self._detection_count = len(stereo_bboxes)

        imu_sample = None
        if self._imu_heading_deg is not None:
            imu_sample = ImuSample(heading_deg=self._imu_heading_deg)

        # LiDAR verisi tazeyse füzyona kat (mimari kalıbı: bayat veri kullanılmaz)
        lidar_fresh = (time.monotonic() - self._lidar_stamp) < self.LIDAR_TIMEOUT_S
        lidar = self._lidar if lidar_fresh else []

        frame = fuse(stereo=stereo_bboxes, lidar=lidar, imu=imu_sample)
        self._last_perception_frame = frame

        # Işık mesafesi: bbox'ın bakış yönüne düşen LiDAR direğinden (stereo yoksa)
        self._assign_light_distances_from_poles(frame.light_dets)

        self._last_light_state = self._traffic_light_logic.update(
            frame.light_dets, now=now, vehicle_speed_mps=self._vehicle_speed_mps)

        self._last_sign_state = self._traffic_sign_logic.update(frame.sign_dets, now=now)

        # Kamera + LiDAR engelleri BİRLİKTE (mimari LiDAR-öncelikli çalışır; simde
        # şerit-sınırı filtresi olmadığından iki kaynak toplanır — çift sayım,
        # "en yakın engele göre karar" mantığında zararsızdır)
        self._last_obstacle_state = self._obstacle_logic.update(
            frame.obstacle_dets + frame.lidar_obstacle_dets)

        # Birleşik kaçınma direksiyonu (_compute_avoid_steer) doğrudan lidar
        # engel listesini kullanır; ayrı slalom besleme adımı KALDIRILDI —
        # tek durum makinesi (avoid→return) her statik grubu aynı işler.

        # Park mantığı sadece park modunda ilerletilir (mimariden sapma: fusion node
        # her tick günceller, ama mission_manager.notify_park_completed yapışkan —
        # görevden önce gelen yanlış bir complete=True park beklemesini iptal eder).
        if self._park_mode:
            park_dets = parking_detections_from_signs(frame.sign_dets)
            self._last_park_state = self._parking_logic.update(park_dets)

        self._log_summary()

    def _park_mode_callback(self, msg: Bool) -> None:
        new_mode = bool(msg.data)
        if new_mode and not self._park_mode:
            # Park moduna giriş: önceki faz kalıntısı taşınmasın (mimari reset kalıbı)
            self._parking_logic = ParkingLogic()
            self._last_park_state = None
        self._park_mode = new_mode

    def _poles_callback(self, msg: String) -> None:
        try:
            self._poles = [p for p in json.loads(msg.data) if isinstance(p, dict)]
            self._poles_stamp = time.monotonic()
        except Exception:
            self._poles = []

    def _assign_light_distances_from_poles(self, light_dets: list) -> None:
        """Trafik ışığı tespitlerine LiDAR direk mesafesi ata: direk adayı,
        ön tampon çerçevesinden kamera pikseline projekte edilir (u = cx - fx·y/x)
        ve ışık bbox merkezine en yakın düşen aday (tolerans içinde) kullanılır."""
        if not light_dets or not self._poles:
            return
        if (time.monotonic() - self._poles_stamp) > self.LIDAR_TIMEOUT_S:
            return
        for det in light_dets:
            if det.estimated_distance_m is not None or not det.bbox_px:
                continue
            u_det = 0.5 * (float(det.bbox_px[0]) + float(det.bbox_px[2]))
            best_d = None
            best_du = POLE_MATCH_MAX_PX
            for p in self._poles:
                d = float(p.get("distance_m", -1.0))
                if d <= 0.5 or d > POLE_MATCH_MAX_DIST_M:
                    continue
                x_c = d + POLE_CAM_X_OFF_M
                u_pred = POLE_CAM_CX_PX - POLE_CAM_FX_PX * float(p.get("lateral_m", 0.0)) / x_c
                du = abs(u_pred - u_det)
                if du < best_du:
                    best_du = du
                    best_d = d
            if best_d is not None:
                det.estimated_distance_m = best_d

    def _intersection_passed_callback(self, msg: Bool) -> None:
        if bool(msg.data):
            self._traffic_sign_logic.notify_intersection_passed()
            self.get_logger().info('Kavşak geçildi: tabela dönüş kısıtları temizlendi')

    def _lidar_callback(self, msg: String) -> None:
        """lidar_obstacle_node'un JSON çıktısını LidarObstacle listesine çevirir
        (mimari perception_fusion_node._lidar_cb ile birebir aynı)."""
        try:
            raw: list[dict] = json.loads(msg.data)
            self._lidar = [
                LidarObstacle(
                    kind=d["kind"],
                    confidence=float(d["confidence"]),
                    distance_m=float(d["distance_m"]),
                    lateral_m=float(d["lateral_m"]),
                    bbox_px=tuple(d["bbox_px"]) if d.get("bbox_px") else None,
                )
                for d in raw
            ]
            self._lidar_stamp = time.monotonic()
        except Exception as e:
            self.get_logger().error(f"lidar parse: {e}")

    def _imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._imu_heading_deg = math.degrees(math.atan2(siny, cosy))

    def _odom_callback(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._vehicle_speed_mps = math.sqrt(vx * vx + vy * vy)

    def _camera_callback(self, msg: Image) -> None:
        """Kamera görüntüsünü numpy array olarak sakla (GUI'de gösterilecek)."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._latest_camera_frame = frame
            self._camera_frame_count += 1
        except Exception:
            pass  # Dönüşüm hatası, sessizce geç

    # ═══════════════════════════════════════════════════════════════
    # Veri parse
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _parse_detections(data: list[float]) -> list[StereoBbox]:
        if not data or len(data) < 6:
            return []
        stride = 6
        bboxes: list[StereoBbox] = []
        for i in range(0, len(data) - stride + 1, stride):
            x1, y1, x2, y2, score, cls_id = data[i:i + stride]
            cls_id_int = int(cls_id)
            class_name = CLASS_NAMES.get(cls_id_int, f'class_{cls_id_int}')
            bboxes.append(StereoBbox(
                class_name=class_name,
                confidence=float(score),
                bbox_px=(float(x1), float(y1), float(x2), float(y2)),
            ))
        return bboxes

    # ═══════════════════════════════════════════════════════════════
    # State publish
    # ═══════════════════════════════════════════════════════════════

    def _publish_states_timer_callback(self) -> None:
        self._publish_states()

    def _publish_states(self) -> None:
        # Trafik ışığı
        ls = self._last_light_state
        light_msg = String()
        if ls and ls.active_color:
            light_msg.data = (
                f"color:{ls.active_color}|must_stop:{ls.must_stop}|"
                f"prepare_stop:{ls.prepare_to_stop}|prepare_move:{ls.prepare_to_move}|"
                f"can_go:{ls.can_go}|speed_cap:{ls.speed_cap_ratio:.2f}|"
                f"dist:{ls.last_distance_m}|reason:{ls.reason}"
            )
        else:
            light_msg.data = "color:none|must_stop:False|can_go:True|reason:no_detections"
        self._pub_light.publish(light_msg)

        # Trafik tabelası
        ss = self._last_sign_state
        sign_msg = String()
        if ss:
            sign_msg.data = (
                f"must_stop:{ss.must_stop_soon}|speed_cap:{ss.speed_cap_ratio:.2f}|"
                f"active:{','.join(ss.active_signs) if ss.active_signs else 'none'}|"
                f"parking:{ss.current_area_is_parking}|no_parking:{ss.current_area_no_parking}|"
                f"light_ahead:{ss.traffic_light_expected}|tunnel:{ss.approaching_tunnel}|"
                f"reasons:{'; '.join(ss.reasons) if ss.reasons else 'none'}"
            )
        else:
            sign_msg.data = "must_stop:False|active:none|reasons:no_detections"
        self._pub_sign.publish(sign_msg)

        # Engel
        os_ = self._last_obstacle_state
        obs_msg = String()
        if os_:
            obs_msg.data = (
                f"behavior:{os_.behavior_mode}|emergency:{os_.emergency_stop}|"
                f"speed_cap:{os_.speed_cap_ratio:.2f}|threat:{os_.threat_level}|"
                f"closest_m:{os_.closest_obstacle_m}|"
                f"avoid_dir:{os_.avoidance_direction or 'none'}|"
                f"lane_change:{os_.suggest_lane_change}|road_blocked:{os_.road_blocked}|"
                f"pedestrian:{os_.pedestrian_in_corridor}|cone:{os_.cone_in_corridor}|"
                f"barrier:{os_.barrier_in_corridor}|reason:{os_.reason}"
            )
        else:
            obs_msg.data = "behavior:clear|emergency:False|reason:no_detections"
        self._pub_obstacle.publish(obs_msg)

        self._publish_mission_interface()

    def _publish_mission_interface(self) -> None:
        """Mimari arayüzü: turn_permissions + decision_debug + park_complete
        (perception_fusion_node ile aynı JSON şemaları)."""
        now = time.monotonic()
        ls = self._last_light_state
        ss = self._last_sign_state
        os_ = self._last_obstacle_state
        ps = self._last_park_state

        # ── turn_permissions ──
        tp = ss.turn_permissions if ss is not None else None
        self._pub_turn_permissions.publish(String(data=json.dumps(
            {
                "left": bool(tp.left) if tp is not None else True,
                "straight": bool(tp.straight) if tp is not None else True,
                "right": bool(tp.right) if tp is not None else True,
                "forced_direction": tp.forced_direction if tp is not None else None,
            },
            ensure_ascii=False,
        )))

        # ── decision arbiter: adayları topla, tek karar üret ──
        candidates: list[Candidate] = []
        if ls is not None:
            if ls.must_stop:
                candidates.append(Candidate(name="light", emergency_stop=True, speed_cap=0.0,
                                            reasons=[ReasonCode.LIGHT_MUST_STOP]))
            elif float(ls.speed_cap_ratio) < 1.0:
                lr = (ReasonCode.LIGHT_RED_SLOW if ls.active_color == LightColor.RED
                      else ReasonCode.LIGHT_YELLOW_SLOW)
                candidates.append(Candidate(name="light", emergency_stop=False,
                                            speed_cap=float(ls.speed_cap_ratio), reasons=[lr]))
        if ss is not None:
            if ss.must_stop_soon:
                candidates.append(Candidate(name="sign", emergency_stop=True, speed_cap=0.0,
                                            reasons=[ReasonCode.SIGN_MUST_STOP]))
            elif float(ss.speed_cap_ratio) < 1.0:
                candidates.append(Candidate(name="sign", emergency_stop=False,
                                            speed_cap=float(ss.speed_cap_ratio),
                                            reasons=[ReasonCode.SIGN_SPEED_CAP]))
        # Engel: yalnız emniyet (acil fren + hız kısıtı). Kaçınma direksiyonu bu node'da
        # değil; ayrı bir düğümde ham lidar + şerit-ofset yaklaşımıyla üretilir.
        if os_ is not None:
            if os_.emergency_stop:
                candidates.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0,
                                            reasons=[ReasonCode.OBSTACLE_EMERGENCY_STOP]))
            if os_.road_blocked:
                candidates.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0,
                                            reasons=[ReasonCode.ROAD_BLOCKED]))
            if float(os_.speed_cap_ratio) < 1.0:
                candidates.append(Candidate(name="obstacle", emergency_stop=False,
                                            speed_cap=float(os_.speed_cap_ratio), reasons=[]))
        if self._park_mode and ps is not None and not ps.complete:
            reasons = [ReasonCode.PARK_MODE]
            if bool(ps.no_eligible_spot):
                reasons.append(ReasonCode.PARK_NO_ELIGIBLE)
            candidates.append(Candidate(name="park", emergency_stop=False,
                                        speed_cap=float(ps.speed_ratio),
                                        steer_override=float(ps.steering), reasons=reasons))

        # Simde lane_walls pointcloud'u yok → lane=None. static_avoid steer'i lidar
        # geometrisinden geldiğinden (nişan-noktası) lane şartı aranmaz; şerit
        # sınırı bilgisi olan mimari kurulumda True kalmalı (madde 12/20 notu).
        decision = self._decision_arbiter.arbitrate(
            candidates=candidates, lane=None, lane_required_for_avoidance=False)

        detections_fresh = (self._last_detection_time is not None
                            and (time.time() - self._last_detection_time) < 1.0)
        lidar_fresh = (now - self._lidar_stamp) < self.LIDAR_TIMEOUT_S
        self._pub_decision_debug.publish(String(data=json.dumps(
            {
                "ts_monotonic": now,
                "fresh": {"stereo": bool(detections_fresh), "lidar": bool(lidar_fresh)},
                "lane_bounds": None,
                "entry_blocked": bool(ss.entry_blocked) if ss is not None else False,
                "candidates": [
                    {
                        "name": c.name,
                        "emergency_stop": c.emergency_stop,
                        "speed_cap": c.speed_cap,
                        "steer_override": c.steer_override,
                        "reasons": [r.value for r in c.reasons],
                    }
                    for c in candidates
                ],
                "final": {
                    "emergency_stop": decision.emergency_stop,
                    "speed_cap": decision.speed_cap,
                    "has_steer_override": decision.has_steer_override,
                    "steer_override": decision.steer_override,
                    "reasons": [r.value for r in decision.reasons],
                },
            },
            ensure_ascii=False,
        )))

        # ── park_complete: sadece park modunda anlamlı ──
        self._pub_park_complete.publish(Bool(
            data=bool(self._park_mode and ps is not None and ps.complete)))

    def _log_summary(self) -> None:
        parts = []
        if self._detection_count > 0:
            parts.append(f"Tespit: {self._detection_count}")
        if self._last_light_state and self._last_light_state.active_color:
            _ls = self._last_light_state
            # Teşhis: ışık mesafe kestirimi kritik — commit bandı (6.5 m) tetikleniyor mu,
            # yoksa mesafe hep bandın üstünde mi görünüyor?
            _d = f"{_ls.last_distance_m:.1f}m" if _ls.last_distance_m is not None else "?"
            parts.append(f"Isik: {_ls.active_color} d={_d} stop={_ls.must_stop} "
                         f"cap={_ls.speed_cap_ratio:.2f}")
        if self._last_sign_state and self._last_sign_state.active_signs:
            parts.append(f"Tabela: {len(self._last_sign_state.active_signs)} adet")
        if self._last_obstacle_state:
            parts.append(f"Engel: {self._last_obstacle_state.behavior_mode}")
        if parts:
            self.get_logger().info(' | '.join(parts), throttle_duration_sec=2.0)


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPipelineNode()

    if node._show_dashboard:
        gui = DashboardGUI(node)
        # spin_once GUI'nin _tick() içinde çağrılıyor
        gui.run()
    else:
        # Dashboard yok — normal spin
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
