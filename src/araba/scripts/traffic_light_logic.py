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

from deos_algorithms.safety_logic import (
    DIST_EMERGENCY_STOP,
    DIST_HARD_SLOWDOWN,
    DIST_SOFT_SLOWDOWN,
)


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
# Kırmızı kilidi emniyet supabı: onaylı yeşil bu süre boyunca hiç görülemezse
# kilidi bırak (tam ışık döngüsünden uzun olmalı; araç sonsuza dek beklemesin)
RED_LATCH_TIMEOUT_S = 45.0

# Mono kamerada stereo mesafe üretilemiyor (sim) — bbox yüksekliğinden kestirim.
# Sim traffic_light modeli: 3 lamba kolonu (üst kürenin tepesi - alt kürenin dibi)
# ~0.71 m; kamera fx=fy=917.4 px (1280x720, 69.4° HFOV). d = H*f/h_px.
# Amaç: uzaktaki alakasız kavşağın ışığı aracı durdurmasın — karar, "mesafe
# bilinmiyor->dur" yerine mesafe bantlarıyla verilsin.
LIGHT_STACK_HEIGHT_M = 0.71
CAMERA_FOCAL_PX = 917.4
BBOX_DIST_MAX_M = 80.0


def _distance_from_bbox_h(bbox_px) -> Optional[float]:
    """Bbox piksel yüksekliğinden ışık mesafesi (stereo yoksa yedek kestirim)."""
    try:
        h = float(bbox_px[3]) - float(bbox_px[1])
    except (TypeError, IndexError, ValueError):
        return None
    if h <= 1.0:
        return None
    return min(BBOX_DIST_MAX_M, LIGHT_STACK_HEIGHT_M * CAMERA_FOCAL_PX / h)

# Bayat yeşil: kavşağa yaklaşırken yeşil görülüp ışık görüş dışına çıkarsa (yakında
# direk kadraj dışı kalır) faz değişimi görülemez — kırmızıya dönmüş olabilir. Bellek
# düştükten sonra bu pencere boyunca sürünme tavanı uygula; pencere dolunca kavşak
# geçilmiş sayılır.
GREEN_STALE_WINDOW_S = 6.0
GREEN_STALE_SPEED_RATIO = 0.5


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
    # Şartname: yeşil ışıkta ≤5sn=+40p, 5-30sn=+20p, >30sn=-20p
    green_elapsed_s: Optional[float] = None
    # Kırmızı kilidi: kırmızıda durulduktan sonra onaylı yeşile dek bekleme aktif
    red_latch_active: bool = False
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
    """
    Kırmızı ışık mesafe bantları, ``safety_logic`` ile aynı eşiklerde tutulur (şartname tablosu ile uyum):
    ``<= DIST_EMERGENCY_STOP`` tam dur; üst bantlarda yalnızca hız tavanı; ``> DIST_SOFT_SLOWDOWN`` müdahale yok.
    """

    def __init__(
        self,
        *,
        red_emergency_m: float = DIST_EMERGENCY_STOP,
        red_hard_m: float = DIST_HARD_SLOWDOWN,
        red_soft_m: float = DIST_SOFT_SLOWDOWN,
        red_hard_speed_ratio: float = 0.5,
        red_soft_speed_ratio: float = 0.8,
        red_unknown_must_stop: bool = True,
        red_commit_m: float = 6.5,
    ) -> None:
        # Dur kararı ışık hâlâ görünürken verilmeli: kamera dikey FOV 42.6° ve lamba
        # yerden ~2.8 m olduğundan ışık 4.2-5.5 m kala görüş alanından çıkıyor. must_stop
        # yalnız ≤3 m'de tetiklenseydi hiç çalışmaz, araç kırmızıda geçerdi. red_commit_m
        # (6.5) FOV çıkışının üstünde: karar ışık görünürken verilir, kilit (RED_LATCH)
        # kurulur, araç ~6 m'de durur ve yeşil de o mesafeden görünür (kilit yeşille açılır).
        self._red_commit_m = float(red_commit_m)
        self._red_emergency_m = float(red_emergency_m)
        self._red_hard_m = float(red_hard_m)
        self._red_soft_m = float(red_soft_m)
        self._red_hard_speed_ratio = float(red_hard_speed_ratio)
        self._red_soft_speed_ratio = float(red_soft_speed_ratio)
        self._red_unknown_must_stop = bool(red_unknown_must_stop)

        self._memories: dict[str, _LightMemory] = {}
        # Son üretilen kararda kırmızı/yeşil hangisi baskındı (sarı bağlamı için)
        self._last_non_yellow: Optional[str] = None
        # Yeşil ışığın ilk onaylandığı an (şartname tepki süresi takibi için)
        self._green_started_at: Optional[float] = None
        # Kırmızı kilidi: kırmızıda dur kararı verildikten sonra onaylı YEŞİL'e dek tut
        self._red_latched: bool = False
        self._red_latched_since: Optional[float] = None
        # Bayat yeşil takibi: onaylı yeşilin bellekten düştüğü an
        self._green_lost_at: Optional[float] = None

    def update(
        self,
        detections: list[LightDetection],
        now: Optional[float] = None,
        vehicle_speed_mps: Optional[float] = None,
    ) -> TrafficLightState:
        if now is None:
            now = time.monotonic()
        detections = [d for d in detections if d.confidence >= MIN_CONFIDENCE]
        for d in detections:
            if d.estimated_distance_m is None:
                d.estimated_distance_m = _distance_from_bbox_h(d.bbox_px)
        self._update_memory(detections, now)
        self._forget_expired(now)
        active = [m for m in self._memories.values() if m.confirmed]

        active_colors = {m.color for m in active}
        if LightColor.GREEN in active_colors:
            if self._green_started_at is None:
                self._green_started_at = now
            self._green_lost_at = None
        else:
            if self._green_started_at is not None and self._green_lost_at is None:
                self._green_lost_at = now  # onaylı yeşil görüşten/bellekten düştü
            self._green_started_at = None

        state = self._decide(active, vehicle_speed_mps=vehicle_speed_mps, now=now)
        state = self._apply_red_latch(state, active, now)
        return self._apply_stale_green(state, active, now)

    def reset(self) -> None:
        self._memories.clear()
        self._last_non_yellow = None
        self._green_started_at = None
        self._red_latched = False
        self._red_latched_since = None
        self._green_lost_at = None

    def _apply_stale_green(self, state: TrafficLightState, active: list[_LightMemory], now: float) -> TrafficLightState:
        """
        Yeşil görülüp ışık görüş dışına çıktıysa faz değişimi izlenemez: pencere
        boyunca sürünme tavanı uygula, yeniden onaylı renk gelirse normal karar
        zaten baskındır. Kırmızı kilidi/dur kararlarını asla GEVŞETMEZ.
        """
        if self._green_lost_at is None or self._red_latched or state.must_stop or active:
            return state
        dt = now - self._green_lost_at
        if dt <= GREEN_STALE_WINDOW_S:
            state.speed_cap_ratio = min(state.speed_cap_ratio, GREEN_STALE_SPEED_RATIO)
            extra = "bayat YEŞİL: ışık görüş dışında, kavşak sürünerek geçiliyor"
            state.reason = f"{state.reason}; {extra}" if state.reason else extra
        else:
            self._green_lost_at = None  # pencere doldu: kavşak geçilmiş say
        return state

    def _apply_red_latch(self, state: TrafficLightState, active: list[_LightMemory], now: float) -> TrafficLightState:
        """
        Kırmızı kilidi: kırmızıda "dur" kararı verildikten sonra ışık görüş
        alanından çıksa bile (bellek LIGHT_VALIDITY_SECONDS'ta düşer) onaylı
        YEŞİL görülene kadar durmaya devam et. Aksi hâlde direğin dibinde
        duran araçta ışık kadraj dışına çıkınca tüm kısıtlar düşüyor ve araç
        yeşili beklemeden kalkıyordu. RED_LATCH_TIMEOUT_S emniyet supabıdır:
        yeşil o konumdan hiç görülemiyorsa araç sonsuza dek beklemesin.
        """
        if state.must_stop and state.active_color == LightColor.RED and not self._red_latched:
            self._red_latched = True
            self._red_latched_since = now

        if not self._red_latched:
            return state

        active_colors = {m.color for m in active}
        if LightColor.GREEN in active_colors:
            # Onaylı yeşil: kilit açılır, _decide'ın yeşil kararı (can_go) geçerli
            self._red_latched = False
            self._red_latched_since = None
            return state

        if self._red_latched_since is not None and (now - self._red_latched_since) > RED_LATCH_TIMEOUT_S:
            self._red_latched = False
            self._red_latched_since = None
            state.reason = ((state.reason + " | ") if state.reason else "") + (
                f"RED latch timeout ({RED_LATCH_TIMEOUT_S:.0f}s) — kilit bırakıldı")
            return state

        state.red_latch_active = True
        state.must_stop = True
        state.can_go = False
        state.prepare_to_move = False
        state.speed_cap_ratio = 0.0
        if state.active_color is None:
            state.active_color = LightColor.RED
            state.reason = "RED latch: ışık görüş dışında, onaylı YEŞİL bekleniyor"
        else:
            state.reason = f"RED latch | {state.reason}"
        return state

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

    def _decide(self, active: list[_LightMemory], vehicle_speed_mps: Optional[float] = None, now: float = 0.0) -> TrafficLightState:
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
            state.active_color = LightColor.RED
            state.last_distance_m = mem.last_distance_m
            self._last_non_yellow = LightColor.RED
            dist = mem.last_distance_m

            if dist is None:
                if self._red_unknown_must_stop:
                    state.must_stop = True
                    state.speed_cap_ratio = 0.0
                    state.reason = "RED light distance unknown -> must_stop (conservative)"
                else:
                    state.must_stop = False
                    state.speed_cap_ratio = float(self._red_soft_speed_ratio)
                    state.reason = "RED light distance unknown -> soft slow"
                return state

            d = float(dist)
            # COMMIT: ışık hâlâ görünürken (>FOV çıkışı ~5.5 m) dur kararı ver ve
            # kilitle. Aksi hâlde araç durma bandına girmeden ışığı kaybediyor.
            if d <= self._red_commit_m:
                state.must_stop = True
                state.speed_cap_ratio = 0.0
                band = ("emergency" if d <= self._red_emergency_m else "commit")
                state.reason = f"RED light {band} band (d<={self._red_commit_m:.1f}m) at {d:.1f}m -> STOP+latch"
                return state
            if d <= self._red_hard_m:
                state.must_stop = False
                state.speed_cap_ratio = float(self._red_hard_speed_ratio)
                state.reason = f"RED light hard slow ({self._red_emergency_m:.1f}m<d<={self._red_hard_m:.1f}m) at {d:.1f}m"
                return state
            if d <= self._red_soft_m:
                state.must_stop = False
                state.speed_cap_ratio = float(self._red_soft_speed_ratio)
                state.reason = f"RED light soft slow ({self._red_hard_m:.1f}m<d<={self._red_soft_m:.1f}m) at {d:.1f}m"
                return state

            state.must_stop = False
            state.speed_cap_ratio = 1.0
            state.reason = f"RED light far (d>{self._red_soft_m:.1f}m) ignored at {d:.1f}m"
            return state

        if LightColor.YELLOW in by_color:
            mem = by_color[LightColor.YELLOW]
            state.active_color = LightColor.YELLOW
            state.last_distance_m = mem.last_distance_m
            dist = mem.last_distance_m
            dist_txt = f"{dist:.1f}m" if dist is not None else "?"

            after_red = self._last_non_yellow == LightColor.RED
            # Uzak (alakasız) kavşağın sarısı davranışı etkilemesin — kırmızıyla
            # aynı "soft" bandın ötesindeki sarı yalnız bilgi olarak kalır.
            if dist is not None and float(dist) > self._red_soft_m:
                state.reason = f"YELLOW far (d>{self._red_soft_m:.1f}m) ignored at {float(dist):.1f}m"
                return state
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
            elapsed = (now - self._green_started_at) if self._green_started_at is not None else 0.0
            state.green_elapsed_s = elapsed
            state.reason = f"GREEN, go (elapsed={elapsed:.1f}s)"
            return state

        return state

