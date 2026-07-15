#!/usr/bin/env python3
"""Döner kavşak plan-izleme testleri (halkadan geçen alternatif rota)."""
import math
import sys
import types

sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")

from deos_algorithms.route_graph import build_graph_from_centerlines_geojson, load_centerlines_geojson
from deos_algorithms.route_planner import route_remaining_mission_via_graph
from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
from deos_algorithms.waypoint_manager import GpsPosition
from mission_planning_node import MissionPlanningNode

ok = fail = 0


def check(name, cond):
    global ok, fail
    if cond:
        ok += 1
        print(f"  PASS {name}")
    else:
        fail += 1
        print(f"  FAIL {name}")


DL, DO = 40.7899, 29.5089

data = load_centerlines_geojson("/home/bekir/sim2_ws/src/araba/missions/teknofest_centerlines.geojson")
g = build_graph_from_centerlines_geojson(data)
mission = GeoJsonMissionReader().read_file("/home/bekir/sim2_ws/src/araba/missions/test_gorev_kavsak.geojson")

SIGN_LAT = DL + 9.5 / 111320.0
SIGN_LON = DO + (-36.1) / (111320.0 * math.cos(math.radians(DL)))
plan = route_remaining_mission_via_graph(
    mission, g, current_lat=SIGN_LAT, current_lon=SIGN_LON, start_index=2,
    blocked_edges={(47, 38)}, fallback_to_original_on_failure=False)


class LogStub:
    def info(self, *a, **k): pass
    def warn(self, *a, **k): pass
    def error(self, *a, **k): pass


n = object.__new__(MissionPlanningNode)
n.get_logger = lambda: LogStub()
n._plan_points = list(plan.points)
n._route_graph = g
n._turn_active = False
n._turn_is_roundabout = False
n._turn_rb_exit_idx = None
n._turn_wp_index = None
n._turn_path = None
n._turn_path_origin = None
n._turn_arc_s_exit = 0.0
n._turn_arc_s_here = 0.0
n._curve_skip_logged_idx = -1
n._route_unstable_until = 0.0   # rota oturmuş kabul (dönüş bastırma kapalı)
n._no_route_until = 0.0

# rb anahtarları — node init'teki mantığın aynısı
n._rb_keys = set()
by_id = {nd.id: nd for nd in g.nodes}
for edges in g.adj.values():
    for e in edges:
        if str((e.props or {}).get("tur", "")) == "kavsak":
            for nid in (int(e.u), int(e.v)):
                nd = by_id[nid]
                n._rb_keys.add((round(float(nd.lat), 7), round(float(nd.lon), 7)))

check("rb anahtarları bulundu (halka=24 düğüm)", len(n._rb_keys) == 24)

# Halka merkezi + ortalama yarıçap (node __init__'teki mantığın aynısı).
# İz dışarı kaydırma: harita düğümleri halkanın iç kenarına yapışık
# (r≈6.5-6.8); sürülebilir asfalt 5.3-10.7 m. İz r=8.0'e (bandın ortasına) taşınır.
_lats = [k[0] for k in n._rb_keys]
_lons = [k[1] for k in n._rb_keys]
n._rb_center_ll = (sum(_lats) / len(_lats), sum(_lons) / len(_lons))
_rs = [math.hypot(*n._ll_to_enu_m(la, lo, *n._rb_center_ll)) for la, lo in n._rb_keys]
n._rb_node_radius_m = sum(_rs) / len(_rs)
check(f"harita halkası iç kenarda (ölçülen r={n._rb_node_radius_m:.2f} m)",
      6.3 < n._rb_node_radius_m < 7.2)

rb_flags = [n._is_roundabout_pt(p) for p in plan.points]
first_rb = rb_flags.index(True)
run_len = 0
while first_rb + run_len < len(rb_flags) and rb_flags[first_rb + run_len]:
    run_len += 1
check(f"planda halka koşusu bitişik (idx {first_rb}, {run_len} düğüm)", run_len >= 10)
check("halka koşusundan sonra normal yol", not rb_flags[first_rb + run_len])

# Halka girişine yaklaşım: _update_turn_state ile devreye girmeli
wp_state = types.SimpleNamespace(
    wp_index=first_rb,
    distance_to_wp_m=5.0,
    current_wp=plan.points[first_rb],
    next_wp=plan.points[first_rb + 1],
    bearing_error_deg=0.0,
    bearing_to_wp_deg=180.0,
)
n._update_turn_state(wp_state)
check("halka girişinde turn_active", n._turn_active is True)
check("roundabout modu işaretli", n._turn_is_roundabout is True)
check("iz üretildi", n._turn_path is not None and len(n._turn_path) > run_len)
check("çıkış idx = halka sonu + 1", n._turn_rb_exit_idx == first_rb + run_len)

# ── İz dışarı kaydı mı? (ilk düğüm döner kavşağın çok içinde olmasın) ──
# İzdeki halka noktalarının merkeze uzaklığı RB_LANE_RADIUS_M olmalı (harita 6.5-6.8).
import mission_planning_node as _mpn
_lat0, _lon0 = n._turn_path_origin
_ccx, _ccy = n._ll_to_enu_m(n._rb_center_ll[0], n._rb_center_ll[1], _lat0, _lon0)
_ring_r = []
for _p in n._turn_path:
    _r = math.hypot(_p[0] - _ccx, _p[1] - _ccy)
    if 4.0 < _r < 12.0:                      # halka civarındaki iz noktaları
        _ring_r.append(_r)
_on_lane = [r for r in _ring_r if abs(r - _mpn.RB_LANE_RADIUS_M) < 0.3]
check(f"halka izi {_mpn.RB_LANE_RADIUS_M} m'ye kaydırıldı "
      f"({len(_on_lane)}/{len(_ring_r)} nokta)", len(_on_lane) >= run_len - 1)
_ISLAND_R, _CAR_HALF = 5.3, 0.80
check(f"iç tekerlek ADAYA basmıyor (açıklık {_mpn.RB_LANE_RADIUS_M - _CAR_HALF - _ISLAND_R:.2f} m)",
      _mpn.RB_LANE_RADIUS_M - _CAR_HALF - _ISLAND_R > 1.0)
ring_arc_est = 1.9 * run_len
check(f"s_exit makul (~ön koşu + halka: {n._turn_arc_s_exit:.1f} m)",
      ring_arc_est * 0.7 + 10 <= n._turn_arc_s_exit <= ring_arc_est * 1.5 + 18)

# Pure pursuit yönü: halka ortasında (saat yönü tersi dolaşım) sola kırmalı.
# DİKKAT: araç, kaydırılmış İZİN ÜSTÜNE konmalı (r=RB_LANE_RADIUS_M). Eskiden harita
# düğümüne (r=6.8) konuyordu; iz 8.0'e taşınınca araç izin 1.2 m İÇİNDE kalıyor ve
# pure-pursuit haklı olarak DIŞA kırıyordu — testin varsayımı eskimişti.
lat0, lon0 = n._turn_path_origin
mid = plan.points[first_rb + run_len // 2]
nxtp = plan.points[first_rb + run_len // 2 + 1]
import mission_planning_node as mpn


def _enu_to_ll(e, nn, la0, lo0):
    """_ll_to_enu_m'in tersi (aynı eş-dikdörtgen yaklaşım)."""
    lat = la0 + math.degrees(nn / mpn.EARTH_R_M)
    lon = lo0 + math.degrees(e / (mpn.EARTH_R_M * math.cos(math.radians(la0))))
    return lat, lon


_mx, _my = n._ll_to_enu_m(float(mid.lat), float(mid.lon), lat0, lon0)
_dx, _dy = _mx - _ccx, _my - _ccy
_dr = math.hypot(_dx, _dy)
_sx = _ccx + _dx / _dr * mpn.RB_LANE_RADIUS_M      # izin üstündeki karşılık gelen nokta
_sy = _ccy + _dy / _dr * mpn.RB_LANE_RADIUS_M
_slat, _slon = _enu_to_ll(_sx, _sy, lat0, lon0)
heading = mpn._bearing_deg(float(mid.lat), float(mid.lon), float(nxtp.lat), float(nxtp.lon))
pos = GpsPosition(lat=_slat, lon=_slon, heading_deg=heading)
steer = n._turn_arc_steering(pos)
check(f"halka ortasında sola direksiyon (steer={steer:+.2f})", steer is not None and steer < -0.05)

# Erken hizalanma bitirmesin: s_here < s_exit iken turn_active kalmalı
n._turn_arc_s_here = n._turn_arc_s_exit * 0.5
wp_mid = types.SimpleNamespace(
    wp_index=first_rb + run_len // 2, distance_to_wp_m=1.5,
    current_wp=mid, next_wp=nxtp, bearing_error_deg=3.0, bearing_to_wp_deg=heading)
n._update_turn_state(wp_mid)
check("halka ortasında (hizalı olsa da) plan önceliği sürüyor", n._turn_active is True)

# İz sonuna gelince bırakmalı
n._turn_arc_s_here = n._turn_arc_s_exit + 0.1
n._update_turn_state(wp_mid)
check("iz bitince şerit takibine dönüldü", n._turn_active is False and n._turn_is_roundabout is False)

# Çıkış wp'si geçilince de bırakmalı (yedek koşul)
n._update_turn_state(wp_state)  # yeniden gir
n._turn_arc_s_here = 0.0
wp_out = types.SimpleNamespace(
    wp_index=first_rb + run_len + 1, distance_to_wp_m=5.0,
    current_wp=plan.points[first_rb + run_len + 1],
    next_wp=None, bearing_error_deg=0.0, bearing_to_wp_deg=0.0)
n._update_turn_state(wp_out)
check("çıkış wp geçilince bırakıldı", n._turn_active is False)

print(f"\n{ok} PASS / {fail} FAIL")
sys.exit(1 if fail else 0)
