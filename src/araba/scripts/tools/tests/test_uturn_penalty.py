#!/usr/bin/env python3
"""U dönüşü cezası doğrulaması (sola-dönülmez, kenar 47->38 bloklu)."""
import math
import sys

sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")

from deos_algorithms.route_graph import (
    build_graph_from_centerlines_geojson,
    load_centerlines_geojson,
    _bearing_deg,
    _angle_diff_deg,
)
from deos_algorithms.route_planner import route_remaining_mission_via_graph
from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
from deos_algorithms.geo_utils import haversine_m

DL, DO = 40.7899, 29.5089
R = 6371000.0


def enu(lat, lon):
    return (math.radians(lon - DO) * R * math.cos(math.radians(DL)),
            math.radians(lat - DL) * R)


def count_uturns(points):
    """Rota noktalarında etkin-yön mantığıyla ~180° ters dönüş say (kısa kenar yönü taşır)."""
    n = 0
    ref = None
    acc = 0.0
    for p0, p1 in zip(points, points[1:]):
        b = _bearing_deg(p0.lat, p0.lon, p1.lat, p1.lon)
        seg = haversine_m(p0.lat, p0.lon, p1.lat, p1.lon)
        if ref is None:
            ref, acc = b, 0.0
            continue
        dev = abs(_angle_diff_deg(b, ref))
        if dev >= 150.0:
            if acc < 8.0:
                n += 1
            ref, acc = b, 0.0
        elif dev <= 30.0:
            acc = 0.0
        elif acc + seg >= 8.0:
            ref, acc = b, 0.0
        else:
            acc += seg
    return n


data = load_centerlines_geojson("/home/bekir/sim2_ws/src/araba/missions/teknofest_centerlines.geojson")
g = build_graph_from_centerlines_geojson(data)
mission = GeoJsonMissionReader().read_file("/home/bekir/sim2_ws/src/araba/missions/test_gorev_kavsak.geojson")

SIGN_LAT = DL + 9.5 / 111320.0
SIGN_LON = DO + (-36.1) / (111320.0 * math.cos(math.radians(DL)))

for name, blocked in (("KISITSIZ", set()), ("SOLA-DONULMEZ (47->38 bloklu)", {(47, 38)})):
    plan = route_remaining_mission_via_graph(
        mission, g, current_lat=SIGN_LAT, current_lon=SIGN_LON, start_index=2,
        blocked_edges=blocked, fallback_to_original_on_failure=False)
    xy = [enu(p.lat, p.lon) for p in plan.points]
    ut = count_uturns(plan.points)
    print(f"\n== {name}: wp={len(plan.points)} u_donusu={ut}")
    print("  " + " -> ".join(f"({x:.1f},{y:.1f})" for x, y in xy))
