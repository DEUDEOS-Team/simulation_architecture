from __future__ import annotations

"""
decision_arbiter.py
-------------------
Perception/decision modüllerinden gelen çıktıları **tek bir karar** haline getirir.

Hedef: Şartnameye uygun deterministik öncelik sırası + anlaşılır gerekçe listesi.

Öncelik (yüksekten düşüğe, özet):
1) Acil durdurma / fail-safe
2) Çarpışma önleme (emergency stop / yol kapalı)
3) Şerit içinde kalma (lane constraint)
4) Trafik ışığı
5) Trafik levhası
6) Park modu / park manevrası
7) Slalom vb. düşük öncelik özel manevralar

Not: Bu modül ROS içermez.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class ReasonCode(str, Enum):
    # Hard stop / safety
    MOTION_ENABLE_STOP = "motion_enable_stop"
    MOTION_ENABLE_TIMEOUT = "motion_enable_timeout"
    ALL_SENSORS_LOST = "all_sensors_lost"
    OBSTACLE_EMERGENCY_STOP = "obstacle_emergency_stop"
    ROAD_BLOCKED = "road_blocked"

    # Lane constraints
    LANE_MISSING_AVOIDANCE_DISABLED = "lane_missing_avoidance_disabled"
    LANE_CLAMP_AVOIDANCE = "lane_clamp_avoidance"

    # Traffic rules
    LIGHT_MUST_STOP = "light_must_stop"
    LIGHT_YELLOW_SLOW = "light_yellow_slow"
    SIGN_MUST_STOP = "sign_must_stop"
    SIGN_SPEED_CAP = "sign_speed_cap"

    # Behaviors
    PARK_MODE = "park_mode"
    PARK_NO_ELIGIBLE = "park_no_eligible"
    STATIC_AVOID = "static_avoid"
    DYNAMIC_AVOID = "dynamic_avoid"
    SLALOM = "slalom"


@dataclass
class LaneBounds:
    """
    Lane sınırlarının base_link frame'de basit temsili.
    left_y_m: sol sınırın y konumu (pozitif)
    right_y_m: sağ sınırın y konumu (negatif)
    """

    left_y_m: float
    right_y_m: float
    # Güvenli tampon: sınırların içine bu kadar pay bırak
    margin_m: float = 0.25

    @property
    def is_valid(self) -> bool:
        return self.left_y_m > self.right_y_m

    def clamp_lateral_target(self, target_y: float) -> float:
        if not self.is_valid:
            return target_y
        lo = self.right_y_m + self.margin_m
        hi = self.left_y_m - self.margin_m
        if lo >= hi:
            # Çok dar / geçersiz: clamp etme (üst katman yavaşlayabilir)
            return target_y
        return max(lo, min(hi, target_y))


def lane_contains_lateral(lane: LaneBounds, lateral_y_m: float) -> bool:
    """Returns True if a lateral position is inside lane bounds (with margin applied)."""
    if lane is None or not lane.is_valid:
        return False
    lo = lane.right_y_m + float(lane.margin_m)
    hi = lane.left_y_m - float(lane.margin_m)
    if lo >= hi:
        return False
    y = float(lateral_y_m)
    return lo <= y <= hi


@dataclass
class Candidate:
    """Tek bir davranış kaynağından gelen aday karar."""

    name: str
    emergency_stop: bool = False
    speed_cap: float = 1.0
    steer_override: Optional[float] = None  # None ise override yok
    reasons: list[ReasonCode] = field(default_factory=list)


@dataclass
class FinalDecision:
    emergency_stop: bool
    speed_cap: float
    has_steer_override: bool
    steer_override: float
    reasons: list[ReasonCode]


def _clamp01(x: float) -> float:
    return max(0.0, min(1.0, float(x)))


def _clamp11(x: float) -> float:
    return max(-1.0, min(1.0, float(x)))


class DecisionArbiter:
    """
    Candidate listesi alıp öncelik tablosuna göre tek karar üretir.

    Tasarım notu:
    - emergency_stop: OR (hard stop baskın)
    - speed_cap: min
    - steer_override: yüksek öncelikli kaynak seçer (varsa)
    """

    def arbitrate(
        self,
        *,
        candidates: list[Candidate],
        lane: LaneBounds | None = None,
        lane_required_for_avoidance: bool = True,
    ) -> FinalDecision:
        # Varsayılan: serbest
        emergency = any(c.emergency_stop for c in candidates)
        speed_cap = _clamp01(min((c.speed_cap for c in candidates), default=1.0))

        # Override seçimi: öncelik sırası
        # Not: PARK > STATIC_AVOID > SLALOM (şartname park sonunda kritik, ama normal sürüşte engel kaçınma daha kritik)
        steer_src = None
        for preferred in ("park", "dynamic_avoid", "static_avoid", "slalom"):
            steer_src = next((c for c in candidates if c.name == preferred and c.steer_override is not None), None)
            if steer_src is not None:
                break

        steer = 0.0
        has_steer = False
        reasons: list[ReasonCode] = []

        # Reasons: tüm adayların reason’larını topla (tekrarları kırp)
        seen: set[ReasonCode] = set()
        for c in candidates:
            for r in c.reasons:
                if r not in seen:
                    seen.add(r)
                    reasons.append(r)

        if steer_src is not None:
            has_steer = True
            steer = _clamp11(float(steer_src.steer_override))

        # Lane constraint: sadece avoidance türü override'larda clamp et (park/slalom farklı bağlam)
        if has_steer and lane is not None and lane.is_valid and steer_src is not None:
            if steer_src.name in {"static_avoid", "dynamic_avoid"}:
                # Basit model: steer sign -> hedef lateral y (±0.6m)
                target_y = 0.6 if steer > 0 else (-0.6 if steer < 0 else 0.0)
                clamped_y = lane.clamp_lateral_target(target_y)
                if clamped_y != target_y:
                    # Clamp sonrası steer'i küçült
                    steer = _clamp11(steer * (abs(clamped_y) / max(0.1, abs(target_y))))
                    if ReasonCode.LANE_CLAMP_AVOIDANCE not in seen:
                        reasons.append(ReasonCode.LANE_CLAMP_AVOIDANCE)

        # Lane yoksa avoidance override'ını tamamen iptal etme seçeneği.
        # Not: Dinamik engelde (yaya) temel davranış (dur/bekle) lane'e bağlı değildir; bu kural sadece
        # steer override'ı kapatır. Speed cap / stop, obstacle candidate'ı ile zaten uygulanır.
        if lane_required_for_avoidance and steer_src is not None and steer_src.name in {"static_avoid", "dynamic_avoid"}:
            if lane is None or not lane.is_valid:
                has_steer = False
                steer = 0.0
                if ReasonCode.LANE_MISSING_AVOIDANCE_DISABLED not in seen:
                    reasons.append(ReasonCode.LANE_MISSING_AVOIDANCE_DISABLED)

        # emergency_stop varsa steer override'ı kapat ve hız 0
        if emergency:
            has_steer = False
            steer = 0.0
            speed_cap = 0.0

        return FinalDecision(
            emergency_stop=bool(emergency),
            speed_cap=_clamp01(speed_cap),
            has_steer_override=bool(has_steer),
            steer_override=_clamp11(steer),
            reasons=reasons,
        )

