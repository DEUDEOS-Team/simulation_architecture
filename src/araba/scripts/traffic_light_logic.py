"""
traffic_light_logic.py
----------------------
Trafik ışığı (kırmızı/sarı/yeşil) tespitlerini alıp aracın davranışına
dönüşen KISITLAR üretir.

Sarı davranışı (özet):
- **Kırmızıdan sonra sarı** (önce onaylı kırmızı görüldüyse): harekete hazırlık; tam dur zorunluluğu yok.
  Anlık hız ~0 ise (``vehicle_speed_mps`` verildiyse) durmaya devam (tavan 0).
- **Yeşilden sonra sarı** veya **ilk görünen ışık sarı**: yavaşlama; hız ~0 ise durmaya devam.
"""

from dataclasses import dataclass
from typing import Optional
import time


class LightColor:
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"


RED_ALIASES = {
    "red", "kirmizi", "kırmızı",
    "kirmizi isik", "kırmızı ışık", "kirmizi_isik",
    "red_light", "traffic_light_red", "tl_red",
}
YELLOW_ALIASES = {
    "yellow", "amber", "sari", "sarı",
    "sari isik", "sarı ışık", "sari_isik",
    "yellow_light", "traffic_light_yellow", "tl_yellow",
}
GREEN_ALIASES = {
    "green", "yesil", "yeşil",
    "yesil isik", "yeşil ışık", "yesil_isik",
    "green_light", "traffic_light_green", "tl_green",
}


def classify_color(class_name: str) -> Optional[str]:
    if not class_name:
        return None
    low = class_name.strip().lower()
    if low in RED_ALIASES:
        return LightColor.RED
    if low in YELLOW_ALIASES:
        return LightColor.YELLOW
    if low in GREEN_ALIASES:
        return LightColor.GREEN
    return None


def is_traffic_light_class(class_name: str) -> bool:
    return classify_color(class_name) is not None


MIN_CONFIDENCE = 0.45
CONFIRM_FRAMES = 2
LIGHT_VALIDITY_SECONDS = 1.5
YELLOW_SPEED_RATIO = 0.4
# Kırmızı → sarı: yeşile hazırlık; duruyorken 0, hareket halindeyken düşük tavan
PREPARE_AFTER_RED_SPEED_RATIO = 0.25
# Odometri gürültüsü için “duruyor” eşiği (m/s); None verilirse bu dal kullanılmaz
STATIONARY_SPEED_EPS_MPS = 0.08


@dataclass
class LightDetection:
    color: str
    confidence: float
    bbox_px: tuple[float, float, float, float]
    estimated_distance_m: Optional[float] = None


@dataclass
class TrafficLightState:
    must_stop: bool = False
    prepare_to_stop: bool = False
    prepare_to_move: bool = False
    can_go: bool = False
    speed_cap_ratio: float = 1.0
    active_color: Optional[str] = None
    last_distance_m: Optional[float] = None
    reason: str = ""


@dataclass
class _LightMemory:
    color: str
    frames_seen: int
    last_seen_time: float
    last_distance_m: Optional[float]
    last_confidence: float
    confirmed: bool


class TrafficLightLogic:
    def __init__(self):
        self._memories: dict[str, _LightMemory] = {}
        # Son üretilen kararda kırmızı/yeşil hangisi baskındı (sarı bağlamı için)
        self._last_non_yellow: Optional[str] = None

    def update(
        self,
        detections: list[LightDetection],
        now: Optional[float] = None,
        vehicle_speed_mps: Optional[float] = None,
    ) -> TrafficLightState:
        if now is None:
            now = time.time()
        detections = [d for d in detections if d.confidence >= MIN_CONFIDENCE]
        self._update_memory(detections, now)
        self._forget_expired(now)
        active = [m for m in self._memories.values() if m.confirmed]
        return self._decide(active, vehicle_speed_mps=vehicle_speed_mps)

    def reset(self) -> None:
        self._memories.clear()
        self._last_non_yellow = None

    def _update_memory(self, detections: list[LightDetection], now: float) -> None:
        for det in detections:
            color = det.color
            if color not in (LightColor.RED, LightColor.YELLOW, LightColor.GREEN):
                continue
            mem = self._memories.get(color)
            if mem is None:
                self._memories[color] = _LightMemory(
                    color=color,
                    frames_seen=1,
                    last_seen_time=now,
                    last_distance_m=det.estimated_distance_m,
                    last_confidence=det.confidence,
                    confirmed=(CONFIRM_FRAMES <= 1),
                )
            else:
                mem.frames_seen += 1
                mem.last_seen_time = now
                mem.last_confidence = det.confidence
                if det.estimated_distance_m is not None:
                    # Keep nearest distance (distance and confidence are tracked separately).
                    if mem.last_distance_m is None or float(det.estimated_distance_m) < float(mem.last_distance_m):
                        mem.last_distance_m = float(det.estimated_distance_m)
                if mem.frames_seen >= CONFIRM_FRAMES:
                    mem.confirmed = True

    def _forget_expired(self, now: float) -> None:
        expired = [
            color for color, mem in self._memories.items()
            if (now - mem.last_seen_time) > LIGHT_VALIDITY_SECONDS
        ]
        for color in expired:
            del self._memories[color]

    def _stationary(self, vehicle_speed_mps: Optional[float]) -> bool:
        if vehicle_speed_mps is None:
            return False
        return abs(float(vehicle_speed_mps)) <= STATIONARY_SPEED_EPS_MPS

    def _decide(self, active: list[_LightMemory], vehicle_speed_mps: Optional[float] = None) -> TrafficLightState:
        state = TrafficLightState()
        if not active:
            return state

        by_color = {m.color: m for m in active}
        stationary = self._stationary(vehicle_speed_mps)

        # Aynı karede / kısa sürede hem kırmızı hem sarı bellekte kalabilir (son görülme süresi).
        # Sarı daha yeni ise kırmızıdan sarıya geçiş kabul edilir.
        if LightColor.RED in by_color and LightColor.YELLOW in by_color:
            r_mem = by_color[LightColor.RED]
            y_mem = by_color[LightColor.YELLOW]
            if y_mem.last_seen_time >= r_mem.last_seen_time:
                by_color.pop(LightColor.RED)

        if LightColor.RED in by_color:
            mem = by_color[LightColor.RED]
            state.must_stop = True
            state.speed_cap_ratio = 0.0
            state.active_color = LightColor.RED
            state.last_distance_m = mem.last_distance_m
            self._last_non_yellow = LightColor.RED
            dist_txt = f"{mem.last_distance_m:.1f}m" if mem.last_distance_m is not None else "?"
            state.reason = f"RED light at {dist_txt}"
            return state

        if LightColor.YELLOW in by_color:
            mem = by_color[LightColor.YELLOW]
            state.active_color = LightColor.YELLOW
            state.last_distance_m = mem.last_distance_m
            dist = mem.last_distance_m
            dist_txt = f"{dist:.1f}m" if dist is not None else "?"

            after_red = self._last_non_yellow == LightColor.RED
            if after_red:
                # Kırmızıdan sonra sarı: harekete hazırlık (yeşile geçiş öncesi)
                state.prepare_to_move = True
                state.prepare_to_stop = False
                state.can_go = False
                state.must_stop = False
                if stationary:
                    state.speed_cap_ratio = 0.0
                    state.reason = f"YELLOW after RED, prepare (stationary, hold stop, {dist_txt})"
                else:
                    state.speed_cap_ratio = PREPARE_AFTER_RED_SPEED_RATIO
                    state.reason = f"YELLOW after RED, prepare to move ({dist_txt})"
            else:
                # Yeşilden sonra sarı veya ilk görünen sarı: yavaşlama
                state.prepare_to_stop = True
                state.prepare_to_move = False
                state.can_go = False
                state.must_stop = False
                if stationary:
                    state.speed_cap_ratio = 0.0
                    state.reason = f"YELLOW slow down (stationary, hold stop, {dist_txt})"
                else:
                    state.speed_cap_ratio = YELLOW_SPEED_RATIO
                    state.reason = f"YELLOW slow down ({dist_txt})"
            return state

        if LightColor.GREEN in by_color:
            mem = by_color[LightColor.GREEN]
            state.can_go = True
            state.active_color = LightColor.GREEN
            state.last_distance_m = mem.last_distance_m
            self._last_non_yellow = LightColor.GREEN
            state.reason = "GREEN, go"
            return state

        return state

