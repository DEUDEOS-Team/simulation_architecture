"""
parking_logic.py
----------------
Park etme manevrasını yöneten durum makinesi.

GeoJSON ile araç park alanına geldikten sonra yan yana slotlar görülebilir; her slot
tabela ile ayrılır: **park yeri** (`SignClass.PARKING_AREA`) veya **park yasak**
(`SignClass.NO_PARKING`). Sadece ``parking_allowed=True`` olan adaylar seçilir;
``False`` olanlar (yasak tabela) asla manevra hedefi olmaz. ``None`` (tabela bilgisi
yok) güvenli tarafta parka kalkılmaz.

Görüntü modeli slot kutusu + tabela sınıfını birleştirip ``ParkingDetection.parking_allowed``
alanını doldurmalıdır; ROS tarafında ``perception_fusion.parking_detections_from_signs``
ile yalnızca tabela tespitleri de aday listesine dönüştürülebilir.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


CAMERA_HEIGHT_M = 1.2
CAMERA_FOCAL_PX = 700.0
PRINCIPAL_POINT_Y_PX = 360.0
IMAGE_WIDTH_PX = 1280

MIN_CONFIDENCE = 0.40
APPROACH_TRIGGER_M = 8.0
ALIGN_TRIGGER_M = 3.0
MANEUVER_TRIGGER_M = 1.5

LATERAL_OK_M = 0.30
LATERAL_GAIN = 1.50

SPEED_APPROACH = 0.40
SPEED_ALIGN = 0.20
SPEED_MANEUVER = 0.15
SPEED_SEARCH = 0.20

SEARCH_STEER_AMPL = 0.20
SEARCH_TOGGLE_FRAMES = 25

PARK_CONFIRM_FRAMES = 20


class ParkType:
    PERPENDICULAR = "dik"
    PARALLEL = "paralel"


class ParkPhase:
    WAITING = "bekleme"
    APPROACHING = "yaklasma"
    ALIGNING = "konumlanma"
    MANEUVERING = "manevra"
    PARKED = "park_edildi"


@dataclass
class ParkingDetection:
    bbox_px: tuple[float, float, float, float]
    confidence: float
    park_type: str = ParkType.PERPENDICULAR
    # True: park yeri tabelası / izinli slot; False: park yasak tabelası; None: belirsiz → kullanılmaz
    parking_allowed: bool | None = None


@dataclass
class ParkState:
    phase: str = ParkPhase.WAITING
    steering: float = 0.0
    speed_ratio: float = 0.0
    reverse: bool = False
    distance_m: Optional[float] = None
    lateral_m: Optional[float] = None
    complete: bool = False
    reason: str = ""
    # Uygun (izinli) aday yokken yalnızca park yasak / belirsiz tespit varsa True olabilir
    no_eligible_spot: bool = False


class ParkingLogic:
    def __init__(self):
        self._phase: str = ParkPhase.WAITING
        self._park_confirm: int = 0
        self._search_frames: int = 0

    def update(self, detections: list[ParkingDetection]) -> ParkState:
        valid = [d for d in detections if d.confidence >= MIN_CONFIDENCE]
        eligible = [d for d in valid if d.parking_allowed is True]
        if not eligible:
            return self._no_eligible_spot(valid)

        # Uygun aday görünmeye başladı: arama sayacını sıfırla
        self._search_frames = 0

        spot = max(eligible, key=lambda d: d.bbox_px[3])
        dist = self._estimate_distance(spot)
        lateral = self._estimate_lateral(spot, dist) if dist is not None else None

        return self._transition(dist, lateral)

    def notify_parked(self) -> None:
        self._reset()

    def _transition(self, dist: Optional[float], lateral: Optional[float]) -> ParkState:
        if self._phase == ParkPhase.WAITING:
            if dist is not None and dist <= APPROACH_TRIGGER_M:
                self._phase = ParkPhase.APPROACHING

        if self._phase == ParkPhase.APPROACHING:
            if dist is not None and dist <= ALIGN_TRIGGER_M:
                self._phase = ParkPhase.ALIGNING
            else:
                return ParkState(
                    phase=self._phase,
                    steering=self._lateral_steering(lateral),
                    speed_ratio=SPEED_APPROACH,
                    distance_m=dist,
                    lateral_m=lateral,
                    reason=f"park yerine yaklaşılıyor, mesafe={dist:.1f}m" if dist is not None else "yaklaşılıyor",
                )

        if self._phase == ParkPhase.ALIGNING:
            if lateral is not None and abs(lateral) < LATERAL_OK_M:
                self._phase = ParkPhase.MANEUVERING
            else:
                return ParkState(
                    phase=self._phase,
                    steering=self._lateral_steering(lateral),
                    speed_ratio=SPEED_ALIGN,
                    distance_m=dist,
                    lateral_m=lateral,
                    reason="hizalanıyor",
                )

        if self._phase == ParkPhase.MANEUVERING:
            state = self._maneuver_dik(dist, lateral)
            if state.complete:
                self._phase = ParkPhase.PARKED
            return state

        return ParkState(
            phase=ParkPhase.PARKED,
            steering=0.0,
            speed_ratio=0.0,
            complete=True,
            distance_m=dist,
            reason="park tamamlandı",
        )

    def _maneuver_dik(self, dist: Optional[float], lateral: Optional[float]) -> ParkState:
        if dist is None or dist > MANEUVER_TRIGGER_M:
            self._park_confirm = 0
            return ParkState(
                phase=ParkPhase.MANEUVERING,
                steering=self._lateral_steering(lateral),
                speed_ratio=SPEED_MANEUVER,
                reverse=True,
                distance_m=dist,
                lateral_m=lateral,
                reason="dik park: geri git",
            )

        self._park_confirm += 1
        if self._park_confirm >= PARK_CONFIRM_FRAMES:
            return ParkState(
                phase=ParkPhase.MANEUVERING,
                steering=0.0,
                speed_ratio=0.0,
                reverse=False,
                distance_m=dist,
                complete=True,
                reason="dik park tamamlandı",
            )

        return ParkState(
            phase=ParkPhase.MANEUVERING,
            steering=0.0,
            speed_ratio=0.0,
            reverse=False,
            distance_m=dist,
            reason=f"park yerine konumlanıyor ({self._park_confirm}/{PARK_CONFIRM_FRAMES})",
        )

    def _lateral_steering(self, lateral: Optional[float]) -> float:
        if lateral is None:
            return 0.0
        return max(-1.0, min(1.0, -lateral * LATERAL_GAIN))

    def _estimate_distance(self, det: ParkingDetection) -> Optional[float]:
        y2 = det.bbox_px[3]
        dy = y2 - PRINCIPAL_POINT_Y_PX
        if dy <= 0:
            return None
        return CAMERA_HEIGHT_M * CAMERA_FOCAL_PX / dy

    def _estimate_lateral(self, det: ParkingDetection, distance: float) -> float:
        x1, _, x2, _ = det.bbox_px
        x_center = (x1 + x2) / 2.0
        du = x_center - (IMAGE_WIDTH_PX / 2.0)
        return -(du * distance) / CAMERA_FOCAL_PX

    def _no_eligible_spot(self, valid: list[ParkingDetection]) -> ParkState:
        if self._phase == ParkPhase.PARKED:
            return ParkState(phase=ParkPhase.PARKED, complete=True, reason="park tamamlandı")
        self._search_frames += 1
        if not valid:
            steer = SEARCH_STEER_AMPL if (self._search_frames // SEARCH_TOGGLE_FRAMES) % 2 == 0 else -SEARCH_STEER_AMPL
            return ParkState(
                phase=self._phase,
                steering=float(steer),
                speed_ratio=float(SPEED_SEARCH),
                reason="park yeri/tabela tespit edilmedi: yavaşça arıyor",
                no_eligible_spot=True,
            )
        forbidden = [d for d in valid if d.parking_allowed is False]
        unknown = [d for d in valid if d.parking_allowed is None]
        if forbidden and not unknown:
            return ParkState(
                phase=self._phase,
                steering=0.0,
                speed_ratio=float(SPEED_SEARCH),
                reason="park yasak tabelası: slot atlanıyor, diğer slotlar aranıyor",
                no_eligible_spot=True,
            )
        if unknown and not forbidden:
            return ParkState(
                phase=self._phase,
                steering=float(SEARCH_STEER_AMPL if (self._search_frames // SEARCH_TOGGLE_FRAMES) % 2 == 0 else -SEARCH_STEER_AMPL),
                speed_ratio=float(SPEED_SEARCH),
                reason="park tabelası sınıfı yok: izinli slot seçilemiyor, yavaşça arıyor",
                no_eligible_spot=True,
            )
        return ParkState(
            phase=self._phase,
            steering=0.0,
            speed_ratio=float(SPEED_SEARCH),
            reason="uygun park yeri (izinli) tespit edilmedi: yavaşça arıyor",
            no_eligible_spot=True,
        )

    def _reset(self) -> None:
        self._phase = ParkPhase.WAITING
        self._park_confirm = 0
        self._search_frames = 0

