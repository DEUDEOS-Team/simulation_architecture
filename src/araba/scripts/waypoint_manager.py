"""
waypoint_manager.py
-------------------
GPS tabanlı global rota takibi (Waypoint Manager).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from deos_algorithms.geo_utils import EARTH_RADIUS_M, haversine_m
from deos_algorithms.geojson_mission_reader import MissionPlan, MissionPoint, TaskType

BEARING_GAIN = 1.0 / 90.0
XTE_GAIN = 1.0 / 20.0
# Pickup/dropoff görevlerinde şartname gereksinimi: doğru kafa açısında varış
HEADING_TOLERANCE_DEG = 30.0
# Yandan geçiş (pass-by) ilerletmesi: arrival_radius'a girilmeden waypoint'in
# yanından geçildiyse de varılmış say — aksi hâlde hedef araç arkasında kalır,
# steering ±1'e kilitlenir ve rota takibi kopar.
PASS_BY_RADIUS_FACTOR = 2.0   # arrival_radius'un katı (3 m yarıçap -> 6 m bandı)
PASS_BY_BEHIND_DEG = 100.0    # waypoint bearing hatası bunu aşarsa "geçildi"


@dataclass
class GpsPosition:
    lat: float
    lon: float
    heading_deg: float


@dataclass
class WaypointState:
    current_wp: Optional[MissionPoint] = None
    next_wp: Optional[MissionPoint] = None

    bearing_to_wp_deg: float = 0.0
    bearing_error_deg: float = 0.0
    distance_to_wp_m: float = 0.0
    cross_track_error_m: float = 0.0

    steering_ref: float = 0.0
    speed_limit_ratio: float = 1.0

    wp_index: int = 0
    arrived: bool = False
    heading_aligned: bool = True
    mission_complete: bool = False
    current_task: str = ""

    reason: str = ""


class WaypointManager:
    def __init__(self, plan: MissionPlan, auto_advance: bool = True):
        self._plan = plan
        self._auto_advance = auto_advance
        self._wp_idx: int = 0

    def update(self, pos: GpsPosition) -> WaypointState:
        if not self._plan.points:
            return WaypointState(reason="görev planı boş", mission_complete=True)

        if self._wp_idx >= len(self._plan.points):
            return WaypointState(
                reason="tüm waypoint'ler tamamlandı",
                mission_complete=True,
                wp_index=self._wp_idx,
            )

        wp = self._plan.points[self._wp_idx]
        next_wp = self._plan.points[self._wp_idx + 1] if self._wp_idx + 1 < len(self._plan.points) else None

        dist = haversine_m(pos.lat, pos.lon, wp.lat, wp.lon)
        bearing = forward_azimuth_deg(pos.lat, pos.lon, wp.lat, wp.lon)
        b_err = angle_diff(bearing, pos.heading_deg)

        prev_wp = self._plan.points[self._wp_idx - 1] if self._wp_idx > 0 else None
        xte = 0.0
        if prev_wp is not None:
            xte = cross_track_error_m(prev_wp.lat, prev_wp.lon, wp.lat, wp.lon, pos.lat, pos.lon)

        steering = max(-1.0, min(1.0, b_err * BEARING_GAIN - xte * XTE_GAIN))
        dist_ok = dist <= wp.arrival_radius_m
        heading_aligned = True
        _h_note = ""
        if wp.heading_deg is not None and wp.task in {TaskType.PICKUP, TaskType.DROPOFF}:
            _h_err = abs(angle_diff(wp.heading_deg, pos.heading_deg))
            heading_aligned = _h_err <= HEADING_TOLERANCE_DEG
            _h_note = f", heading_err={_h_err:.0f}°"
        # Pass-by: pickup/dropoff/park gerçek varış ister (duraklama/park tetikler),
        # ara rota noktaları ise yakından geçildiyse ilerletilir.
        passed_by = (
            wp.task not in {TaskType.PICKUP, TaskType.DROPOFF, TaskType.PARK, TaskType.PARK_ENTRY}
            and dist <= wp.arrival_radius_m * PASS_BY_RADIUS_FACTOR
            and abs(b_err) >= PASS_BY_BEHIND_DEG
        )
        if passed_by:
            _h_note += ", pass_by"
        arrived = (dist_ok and heading_aligned) or passed_by

        state = WaypointState(
            current_wp=wp,
            next_wp=next_wp,
            bearing_to_wp_deg=bearing,
            bearing_error_deg=b_err,
            distance_to_wp_m=dist,
            cross_track_error_m=xte,
            steering_ref=steering,
            speed_limit_ratio=wp.speed_limit_ratio,
            wp_index=self._wp_idx,
            arrived=arrived,
            heading_aligned=heading_aligned,
            mission_complete=False,
            current_task=wp.task,
            reason=(
                f"wp[{self._wp_idx}] '{wp.name}': mesafe={dist:.1f}m, bearing={bearing:.0f}°, "
                f"hata={b_err:+.1f}°, xte={xte:+.1f}m{_h_note}"
            ),
        )

        if arrived and self._auto_advance:
            self.advance()
            if self._wp_idx >= len(self._plan.points):
                state.mission_complete = True

        return state

    def advance(self) -> None:
        self._wp_idx += 1

    def reset(self, index: int = 0) -> None:
        self._wp_idx = max(0, min(index, len(self._plan.points)))

    def remaining(self) -> list[MissionPoint]:
        return self._plan.points[self._wp_idx:]

    @property
    def current_index(self) -> int:
        return self._wp_idx

    @property
    def is_complete(self) -> bool:
        return self._wp_idx >= len(self._plan.points)


def forward_azimuth_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)

    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return math.degrees(math.atan2(x, y)) % 360.0


def angle_diff(target_deg: float, current_deg: float) -> float:
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def cross_track_error_m(
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
    pos_lat: float,
    pos_lon: float,
) -> float:
    R = EARTH_RADIUS_M
    d13 = haversine_m(start_lat, start_lon, pos_lat, pos_lon)
    theta13 = math.radians(forward_azimuth_deg(start_lat, start_lon, pos_lat, pos_lon))
    theta12 = math.radians(forward_azimuth_deg(start_lat, start_lon, end_lat, end_lon))

    sin_xte = math.sin(d13 / R) * math.sin(theta13 - theta12)
    return math.asin(max(-1.0, min(1.0, sin_xte))) * R

