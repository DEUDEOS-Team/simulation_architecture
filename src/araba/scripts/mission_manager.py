from __future__ import annotations

"""
mission_manager.py
------------------
Şartnameye göre görev-temelli davranışları (pickup/dropoff duraklama gibi)
WaypointManager üstüne ekleyen ince bir katman.
"""

from dataclasses import dataclass
from typing import Optional

from deos_algorithms.geojson_mission_reader import TaskType
from deos_algorithms.waypoint_manager import GpsPosition, MissionPlan, WaypointManager, WaypointState


@dataclass
class MissionDecision:
    speed_cap_ratio: float = 1.0
    hold_reason: str = ""
    hold_remaining_s: float = 0.0
    park_mode: bool = False
    park_remaining_s: float = 0.0


class MissionManager:
    def __init__(self, plan: MissionPlan):
        self.wp = WaypointManager(plan, auto_advance=False)
        self._hold_started_at: Optional[float] = None
        self._holding_task: Optional[str] = None
        self._park_started_at: Optional[float] = None
        self._park_active: bool = False
        self._park_completed: bool = False

    def notify_park_completed(self) -> None:
        self._park_completed = True

    def update(self, pos: GpsPosition, *, now_s: float) -> tuple[WaypointState, MissionDecision]:
        state = self.wp.update(pos)
        dec = MissionDecision()

        task = state.current_task
        is_hold_task = task in {TaskType.PICKUP, TaskType.DROPOFF}
        is_park_entry = task == TaskType.PARK_ENTRY
        is_park_task = task == TaskType.PARK

        # Normal waypoint ilerletme:
        # - START/CHECKPOINT/STOP gibi noktalarda arrived=True ise bir sonraki waypoint'e geç.
        # - PARK_ENTRY/PARK arrived ise park modu başlayacağı için burada advance ETME.
        # - HOLD task'lar kendi içinde (15s) advance eder.
        if state.arrived and not is_hold_task and not self._park_active and not (is_park_entry or is_park_task):
            self.wp.advance()
            return self.wp.update(pos), dec

        # Park modunu, park giriş noktasına varınca başlat (şartname: park_giris sadece giriş tetikleyicisidir)
        if (is_park_entry or is_park_task) and state.arrived and not self._park_active:
            self._park_active = True
            self._park_started_at = now_s

        # Park tamamlandı sinyali gelirse park modunu kapat ve waypoint'i ilerlet
        if self._park_active and self._park_completed:
            self._park_active = False
            self._park_completed = False
            self._park_started_at = None
            # park giriş/park noktasını geçip devam et (varsa)
            self.wp.advance()
            return self.wp.update(pos), dec

        if self._park_active:
            # Şartname: park girişinden sonra 3 dakika içinde doğru park edilmezse park görevi geçersiz.
            max_s = 180.0
            elapsed = now_s - (self._park_started_at if self._park_started_at is not None else now_s)
            remaining = max(0.0, max_s - elapsed)
            dec.park_mode = True
            dec.park_remaining_s = remaining
            # Park arama/manzara modunda hız limitini düşür (kontrol perception ile birleşecek)
            dec.speed_cap_ratio = min(dec.speed_cap_ratio, 0.5)
            if remaining <= 0.0:
                dec.speed_cap_ratio = 0.0
                dec.hold_reason = "park timeout (3dk) — güvenli dur"
                dec.hold_remaining_s = 0.0
            return state, dec

        if not state.arrived or not is_hold_task:
            self._hold_started_at = None
            self._holding_task = None
            return state, dec

        if self._hold_started_at is None or self._holding_task != task:
            self._hold_started_at = now_s
            self._holding_task = task

        elapsed = now_s - (self._hold_started_at if self._hold_started_at is not None else now_s)
        min_hold = 15.0
        target_hold = 20.0

        if elapsed < min_hold:
            dec.speed_cap_ratio = 0.0
            dec.hold_reason = f"{task} hold"
            dec.hold_remaining_s = max(0.0, min_hold - elapsed)
            return state, dec

        self.wp.advance()
        dec.speed_cap_ratio = 1.0
        dec.hold_reason = f"{task} completed ({min(elapsed, target_hold):.1f}s)"
        dec.hold_remaining_s = 0.0
        self._hold_started_at = None
        self._holding_task = None
        return state, dec

