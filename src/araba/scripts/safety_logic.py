"""
safety_logic.py
---------------
BBox tabanli nesne tespitlerinden temel yavasla / dur karari ureten
saf Python guvenlik modulu.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


CORRIDOR_HALF_WIDTH_M = 1.2
CORRIDOR_LOOKAHEAD_M = 20.0

DIST_EMERGENCY_STOP = 3.0
DIST_HARD_SLOWDOWN = 8.0
DIST_SOFT_SLOWDOWN = 15.0

CLASS_CAUTION = {
    "pedestrian": 1.5,
    "vehicle": 1.2,
    "cone": 1.0,
    "barrier": 1.0,
    "unknown": 1.3,
}

CAMERA_HEIGHT_M = 1.2
CAMERA_PITCH_RAD = 0.0
CAMERA_FOCAL_PX = 800.0
IMAGE_HEIGHT_PX = 720
IMAGE_WIDTH_PX = 1280
PRINCIPAL_POINT_Y_PX = IMAGE_HEIGHT_PX / 2

MIN_CONFIDENCE = 0.4

CONFIRM_FRAMES = 3
FORGET_FRAMES = 5
TRACK_MATCH_DIST_PX = 80.0


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    class_name: str
    confidence: float
    estimated_distance_m: Optional[float] = None
    estimated_lateral_m: Optional[float] = None

    @property
    def center_px(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)

    @property
    def bottom_center_px(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2, self.y2)


class ThreatLevel(Enum):
    NONE = 0
    SOFT_SLOW = 1
    HARD_SLOW = 2
    EMERGENCY = 3


@dataclass
class SafetyDecision:
    emergency_stop: bool
    speed_cap_ratio: float
    threat_level: ThreatLevel
    closest_obstacle_m: Optional[float]
    reason: str

    @classmethod
    def safe(cls) -> "SafetyDecision":
        return cls(
            emergency_stop=False,
            speed_cap_ratio=1.0,
            threat_level=ThreatLevel.NONE,
            closest_obstacle_m=None,
            reason="clear",
        )


@dataclass
class ThreatObservation:
    detection: Detection
    distance_m: float
    lateral_m: float
    effective_distance_m: float


@dataclass
class SafetyAnalysis:
    decision: SafetyDecision
    confirmed_detections: list[Detection] = field(default_factory=list)
    corridor_threats: list[ThreatObservation] = field(default_factory=list)


@dataclass
class Track:
    track_id: int
    last_detection: Detection
    frames_seen: int = 1
    frames_missed: int = 0
    confirmed: bool = False


def estimate_distance_from_bbox(det: Detection) -> Optional[float]:
    if det.estimated_distance_m is not None:
        if 0.1 <= det.estimated_distance_m <= 100.0:
            return det.estimated_distance_m
        return None

    _, v = det.bottom_center_px
    if v <= PRINCIPAL_POINT_Y_PX + 1:
        return None

    dy = v - PRINCIPAL_POINT_Y_PX
    distance = CAMERA_HEIGHT_M * CAMERA_FOCAL_PX / dy
    if distance > 100.0 or distance < 0.1:
        return None
    return distance


def estimate_lateral_offset_from_bbox(det: Detection, distance: float) -> float:
    if det.estimated_lateral_m is not None and -20.0 <= det.estimated_lateral_m <= 20.0:
        return det.estimated_lateral_m
    u, _ = det.bottom_center_px
    du = u - (IMAGE_WIDTH_PX / 2)
    lateral_camera_frame = (du * distance) / CAMERA_FOCAL_PX
    return -lateral_camera_frame


def is_in_corridor(distance_m: float, lateral_m: float) -> bool:
    if distance_m <= 0 or distance_m > CORRIDOR_LOOKAHEAD_M:
        return False
    if abs(lateral_m) > CORRIDOR_HALF_WIDTH_M:
        return False
    return True


class SimpleTracker:
    def __init__(self):
        self._tracks: dict[int, Track] = {}
        self._next_id = 0

    def update(self, detections: list[Detection]) -> list[Track]:
        unmatched_track_ids = set(self._tracks.keys())
        matched_dets = set()

        for det_idx, det in enumerate(detections):
            best_track_id = None
            best_dist = float("inf")
            for tid in unmatched_track_ids:
                tr = self._tracks[tid]
                dist = self._center_distance(det, tr.last_detection)
                if dist < best_dist and dist < TRACK_MATCH_DIST_PX:
                    best_dist = dist
                    best_track_id = tid

            if best_track_id is not None:
                tr = self._tracks[best_track_id]
                tr.last_detection = det
                tr.frames_seen += 1
                tr.frames_missed = 0
                if tr.frames_seen >= CONFIRM_FRAMES:
                    tr.confirmed = True
                unmatched_track_ids.remove(best_track_id)
                matched_dets.add(det_idx)

        for tid in unmatched_track_ids:
            self._tracks[tid].frames_missed += 1

        for tid in [tid for tid, tr in self._tracks.items() if tr.frames_missed >= FORGET_FRAMES]:
            del self._tracks[tid]

        for det_idx, det in enumerate(detections):
            if det_idx in matched_dets:
                continue
            self._tracks[self._next_id] = Track(
                track_id=self._next_id,
                last_detection=det,
                frames_seen=1,
                frames_missed=0,
                confirmed=CONFIRM_FRAMES <= 1,
            )
            self._next_id += 1

        return list(self._tracks.values())

    @staticmethod
    def _center_distance(a: Detection, b: Detection) -> float:
        ax, ay = a.center_px
        bx, by = b.center_px
        return float(np.hypot(ax - bx, ay - by))


class SafetyLogic:
    def __init__(self):
        self.tracker = SimpleTracker()

    def analyze(self, detections: list[Detection]) -> SafetyAnalysis:
        detections = [d for d in detections if d.confidence >= MIN_CONFIDENCE]
        tracks = self.tracker.update(detections)
        confirmed_dets = [t.last_detection for t in tracks if t.confirmed]

        if not confirmed_dets:
            return SafetyAnalysis(decision=SafetyDecision.safe())

        threats: list[ThreatObservation] = []
        for det in confirmed_dets:
            dist = estimate_distance_from_bbox(det)
            if dist is None:
                continue
            lat = estimate_lateral_offset_from_bbox(det, dist)
            if not is_in_corridor(dist, lat):
                continue
            caution = CLASS_CAUTION.get(det.class_name, CLASS_CAUTION["unknown"])
            threats.append(
                ThreatObservation(
                    detection=det,
                    distance_m=dist,
                    lateral_m=lat,
                    effective_distance_m=dist / caution,
                )
            )

        if not threats:
            return SafetyAnalysis(decision=SafetyDecision.safe(), confirmed_detections=confirmed_dets)

        threats.sort(key=lambda item: item.effective_distance_m)
        closest = threats[0]
        decision = self._decision_from_closest(closest)

        return SafetyAnalysis(
            decision=decision,
            confirmed_detections=confirmed_dets,
            corridor_threats=threats,
        )

    def decide(self, detections: list[Detection]) -> SafetyDecision:
        return self.analyze(detections).decision

    @staticmethod
    def _decision_from_closest(closest: ThreatObservation) -> SafetyDecision:
        det = closest.detection
        eff = closest.effective_distance_m
        if eff < DIST_EMERGENCY_STOP:
            return SafetyDecision(
                emergency_stop=True,
                speed_cap_ratio=0.0,
                threat_level=ThreatLevel.EMERGENCY,
                closest_obstacle_m=closest.distance_m,
                reason=f"EMERGENCY: {det.class_name} at {eff:.1f}m",
            )
        if eff < DIST_HARD_SLOWDOWN:
            return SafetyDecision(
                emergency_stop=False,
                speed_cap_ratio=0.5,
                threat_level=ThreatLevel.HARD_SLOW,
                closest_obstacle_m=closest.distance_m,
                reason=f"HARD_SLOW: {det.class_name} at {eff:.1f}m",
            )
        if eff < DIST_SOFT_SLOWDOWN:
            return SafetyDecision(
                emergency_stop=False,
                speed_cap_ratio=0.8,
                threat_level=ThreatLevel.SOFT_SLOW,
                closest_obstacle_m=closest.distance_m,
                reason=f"SOFT_SLOW: {det.class_name} at {eff:.1f}m",
            )
        return SafetyDecision.safe()
"""
safety_logic.py
---------------
BBox tabanli nesne tespitlerinden temel yavasla / dur karari ureten
saf Python guvenlik modulu.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


# ============================================================
# SABITLER
# ============================================================

CORRIDOR_HALF_WIDTH_M = 1.2
CORRIDOR_LOOKAHEAD_M = 20.0

DIST_EMERGENCY_STOP = 3.0
DIST_HARD_SLOWDOWN = 8.0
DIST_SOFT_SLOWDOWN = 15.0

CLASS_CAUTION = {
    "pedestrian": 1.5,
    "vehicle": 1.2,
    "cone": 1.0,
    "barrier": 1.0,
    "unknown": 1.3,
}

CAMERA_HEIGHT_M = 1.2
CAMERA_PITCH_RAD = 0.0
CAMERA_FOCAL_PX = 800.0
IMAGE_HEIGHT_PX = 720
IMAGE_WIDTH_PX = 1280
PRINCIPAL_POINT_Y_PX = IMAGE_HEIGHT_PX / 2

MIN_CONFIDENCE = 0.4

CONFIRM_FRAMES = 3
FORGET_FRAMES = 5
TRACK_MATCH_DIST_PX = 80.0


# ============================================================
# VERI TIPLERI
# ============================================================


@dataclass
class Detection:
    """Modelin tek bir tespiti."""

    x1: float
    y1: float
    x2: float
    y2: float
    class_name: str
    confidence: float
    estimated_distance_m: Optional[float] = None
    # Harici sensör (stereo/LiDAR) ile ölçülmüş yanal ofset (m).
    # Pozitif=sol, negatif=sağ (bu modüldeki işaret konvansiyonu ile aynı).
    estimated_lateral_m: Optional[float] = None

    @property
    def center_px(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)

    @property
    def bottom_center_px(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2, self.y2)


class ThreatLevel(Enum):
    NONE = 0
    SOFT_SLOW = 1
    HARD_SLOW = 2
    EMERGENCY = 3


@dataclass
class SafetyDecision:
    """Tek frame guvenlik cikisi."""

    emergency_stop: bool
    speed_cap_ratio: float
    threat_level: ThreatLevel
    closest_obstacle_m: Optional[float]
    reason: str

    @classmethod
    def safe(cls) -> "SafetyDecision":
        return cls(
            emergency_stop=False,
            speed_cap_ratio=1.0,
            threat_level=ThreatLevel.NONE,
            closest_obstacle_m=None,
            reason="clear",
        )


@dataclass
class ThreatObservation:
    """Koridor icindeki dogrulanmis tek tehdit."""

    detection: Detection
    distance_m: float
    lateral_m: float
    effective_distance_m: float


@dataclass
class SafetyAnalysis:
    """Karar ile birlikte ara tehdit degerlendirmeleri."""

    decision: SafetyDecision
    confirmed_detections: list[Detection] = field(default_factory=list)
    corridor_threats: list[ThreatObservation] = field(default_factory=list)


@dataclass
class Track:
    """Frame-to-frame takip edilen tek bir nesne."""

    track_id: int
    last_detection: Detection
    frames_seen: int = 1
    frames_missed: int = 0
    confirmed: bool = False


# ============================================================
# MESAFE TAHMINI
# ============================================================


def estimate_distance_from_bbox(det: Detection) -> Optional[float]:
    """
    2D bbox alt orta pikselinden mesafe tahmini uretir.
    Depth/harici mesafe verildiyse onu tercih eder.
    """

    if det.estimated_distance_m is not None:
        if 0.1 <= det.estimated_distance_m <= 100.0:
            return det.estimated_distance_m
        return None

    _, v = det.bottom_center_px
    if v <= PRINCIPAL_POINT_Y_PX + 1:
        return None

    dy = v - PRINCIPAL_POINT_Y_PX
    distance = CAMERA_HEIGHT_M * CAMERA_FOCAL_PX / dy
    if distance > 100.0 or distance < 0.1:
        return None
    return distance


def estimate_lateral_offset_from_bbox(det: Detection, distance: float) -> float:
    """Pozitif = sol, negatif = sag olacak sekilde yanal ofset."""

    if det.estimated_lateral_m is not None:
        # Bu değer sensör füzyonundan geldiyse bbox geometrisini kullanma.
        if -20.0 <= det.estimated_lateral_m <= 20.0:
            return det.estimated_lateral_m
    u, _ = det.bottom_center_px
    cx = IMAGE_WIDTH_PX / 2
    du = u - cx
    lateral_camera_frame = (du * distance) / CAMERA_FOCAL_PX
    return -lateral_camera_frame


# ============================================================
# KORIDOR FILTRESI
# ============================================================


def is_in_corridor(distance_m: float, lateral_m: float) -> bool:
    if distance_m <= 0 or distance_m > CORRIDOR_LOOKAHEAD_M:
        return False
    if abs(lateral_m) > CORRIDOR_HALF_WIDTH_M:
        return False
    return True


# ============================================================
# TRACKER
# ============================================================


class SimpleTracker:
    """Basit merkez-mesafesi tabanli tracker."""

    def __init__(self):
        self._tracks: dict[int, Track] = {}
        self._next_id = 0

    def update(self, detections: list[Detection]) -> list[Track]:
        unmatched_track_ids = set(self._tracks.keys())
        matched_dets = set()

        for det_idx, det in enumerate(detections):
            best_track_id = None
            best_dist = float("inf")
            for tid in unmatched_track_ids:
                tr = self._tracks[tid]
                dist = self._center_distance(det, tr.last_detection)
                if dist < best_dist and dist < TRACK_MATCH_DIST_PX:
                    best_dist = dist
                    best_track_id = tid

            if best_track_id is not None:
                tr = self._tracks[best_track_id]
                tr.last_detection = det
                tr.frames_seen += 1
                tr.frames_missed = 0
                if tr.frames_seen >= CONFIRM_FRAMES:
                    tr.confirmed = True
                unmatched_track_ids.remove(best_track_id)
                matched_dets.add(det_idx)

        for tid in unmatched_track_ids:
            self._tracks[tid].frames_missed += 1

        to_delete = [
            tid
            for tid, tr in self._tracks.items()
            if tr.frames_missed >= FORGET_FRAMES
        ]
        for tid in to_delete:
            del self._tracks[tid]

        for det_idx, det in enumerate(detections):
            if det_idx in matched_dets:
                continue
            self._tracks[self._next_id] = Track(
                track_id=self._next_id,
                last_detection=det,
                frames_seen=1,
                frames_missed=0,
                confirmed=CONFIRM_FRAMES <= 1,
            )
            self._next_id += 1

        return list(self._tracks.values())

    @staticmethod
    def _center_distance(a: Detection, b: Detection) -> float:
        ax, ay = a.center_px
        bx, by = b.center_px
        return float(np.hypot(ax - bx, ay - by))


# ============================================================
# ANA KARAR
# ============================================================


class SafetyLogic:
    """Her frame icin temel guvenlik karari uretir."""

    def __init__(self):
        self.tracker = SimpleTracker()

    def analyze(self, detections: list[Detection]) -> SafetyAnalysis:
        detections = [d for d in detections if d.confidence >= MIN_CONFIDENCE]
        tracks = self.tracker.update(detections)
        confirmed_dets = [t.last_detection for t in tracks if t.confirmed]

        if not confirmed_dets:
            return SafetyAnalysis(
                decision=SafetyDecision.safe(),
                confirmed_detections=[],
                corridor_threats=[],
            )

        threats: list[ThreatObservation] = []
        for det in confirmed_dets:
            dist = estimate_distance_from_bbox(det)
            if dist is None:
                continue
            lat = estimate_lateral_offset_from_bbox(det, dist)
            if not is_in_corridor(dist, lat):
                continue

            caution = CLASS_CAUTION.get(det.class_name, CLASS_CAUTION["unknown"])
            threats.append(
                ThreatObservation(
                    detection=det,
                    distance_m=dist,
                    lateral_m=lat,
                    effective_distance_m=dist / caution,
                )
            )

        if not threats:
            return SafetyAnalysis(
                decision=SafetyDecision.safe(),
                confirmed_detections=confirmed_dets,
                corridor_threats=[],
            )

        threats.sort(key=lambda item: item.effective_distance_m)
        closest = threats[0]
        decision = self._decision_from_closest(closest)

        return SafetyAnalysis(
            decision=decision,
            confirmed_detections=confirmed_dets,
            corridor_threats=threats,
        )

    def decide(self, detections: list[Detection]) -> SafetyDecision:
        return self.analyze(detections).decision

    @staticmethod
    def _decision_from_closest(closest: ThreatObservation) -> SafetyDecision:
        det = closest.detection
        eff = closest.effective_distance_m

        if eff < DIST_EMERGENCY_STOP:
            return SafetyDecision(
                emergency_stop=True,
                speed_cap_ratio=0.0,
                threat_level=ThreatLevel.EMERGENCY,
                closest_obstacle_m=closest.distance_m,
                reason=f"EMERGENCY: {det.class_name} at {eff:.1f}m",
            )
        if eff < DIST_HARD_SLOWDOWN:
            return SafetyDecision(
                emergency_stop=False,
                speed_cap_ratio=0.5,
                threat_level=ThreatLevel.HARD_SLOW,
                closest_obstacle_m=closest.distance_m,
                reason=f"HARD_SLOW: {det.class_name} at {eff:.1f}m",
            )
        if eff < DIST_SOFT_SLOWDOWN:
            return SafetyDecision(
                emergency_stop=False,
                speed_cap_ratio=0.8,
                threat_level=ThreatLevel.SOFT_SLOW,
                closest_obstacle_m=closest.distance_m,
                reason=f"SOFT_SLOW: {det.class_name} at {eff:.1f}m",
            )
        return SafetyDecision.safe()
