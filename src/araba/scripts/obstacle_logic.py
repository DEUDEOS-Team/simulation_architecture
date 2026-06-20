"""
obstacle_logic.py
-----------------
Yaya / koni / bariyer tespitlerini davranisa ceviren ust katman.
"""

from dataclasses import dataclass, field
from typing import Optional

from deos_algorithms.safety_logic import (
    SafetyLogic,
    Detection as SafetyDetection,
    ThreatLevel,
    ThreatObservation,
)


class ObstacleKind:
    PEDESTRIAN = "pedestrian"
    CONE = "cone"
    BARRIER = "barrier"
    UNKNOWN = "unknown"


class ObstacleBehavior:
    CLEAR = "clear"
    STATIC_AVOID = "static_avoid"
    DYNAMIC_SLOW = "dynamic_slow"
    DYNAMIC_WAIT = "dynamic_wait"
    EMERGENCY_STOP = "emergency_stop"


PEDESTRIAN_ALIASES = {"pedestrian", "person", "people", "yaya", "insan"}
CONE_ALIASES = {"cone", "traffic_cone", "koni", "trafik konisi"}
BARRIER_ALIASES = {"barrier", "fence", "roadblock", "bariyer", "engel", "blok"}


MIN_CONFIDENCE = 0.4
DYNAMIC_SLOW_DISTANCE_M = 10.0
DYNAMIC_SLOW_SPEED_CAP = 0.4
DYNAMIC_STOP_DISTANCE_M = 5.0
DYNAMIC_CLEAR_FRAMES = 3
STATIC_LANE_CHANGE_TRIGGER_M = 3.0
STATIC_LANE_CHANGE_SPEED_CAP = 0.35
STATIC_EMERGENCY_DISTANCE_M = 1.5

# Sıralı statik engeller için kaçınma stabilizasyonu (zigzag azaltma)
STATIC_AVOID_COMMIT_FRAMES = 6  # yön kararı en az bu kadar frame korunur
STATIC_AVOID_CLEAR_DISTANCE_M = 4.5  # en yakın statik engel bu mesafeden uzaksa commit bırak
STATIC_AVOID_SWITCH_MARGIN_M = 0.6  # zıt tarafa geçmek için "daha belirgin" yakınlık farkı


def classify_obstacle(class_name: str) -> Optional[str]:
    if not class_name:
        return None
    low = class_name.strip().lower()
    if low in PEDESTRIAN_ALIASES:
        return ObstacleKind.PEDESTRIAN
    if low in CONE_ALIASES:
        return ObstacleKind.CONE
    if low in BARRIER_ALIASES:
        return ObstacleKind.BARRIER
    return None


def is_obstacle_class(class_name: str) -> bool:
    return classify_obstacle(class_name) is not None


def is_dynamic_kind(kind: str) -> bool:
    return kind == ObstacleKind.PEDESTRIAN


def is_static_kind(kind: str) -> bool:
    return kind in {ObstacleKind.CONE, ObstacleKind.BARRIER}


@dataclass
class ObstacleDetection:
    kind: str
    confidence: float
    bbox_px: tuple[float, float, float, float]
    estimated_distance_m: Optional[float] = None
    estimated_lateral_m: Optional[float] = None


@dataclass
class ObstacleState:
    emergency_stop: bool = False
    speed_cap_ratio: float = 1.0
    threat_level: ThreatLevel = ThreatLevel.NONE
    closest_obstacle_m: Optional[float] = None

    pedestrian_in_corridor: bool = False
    cone_in_corridor: bool = False
    barrier_in_corridor: bool = False

    suggest_lane_change: bool = False
    road_blocked: bool = False
    waiting_for_dynamic_obstacle: bool = False
    dynamic_avoid_active: bool = False
    dynamic_avoidance_direction: Optional[str] = None
    avoidance_direction: Optional[str] = None
    behavior_mode: str = ObstacleBehavior.CLEAR

    active_kinds: list[str] = field(default_factory=list)
    reason: str = ""


class ObstacleLogic:
    def __init__(self):
        self._safety = SafetyLogic()
        self._dynamic_wait_active = False
        self._dynamic_clear_frames = 0
        self._static_commit_dir: Optional[str] = None
        self._static_commit_frames_left: int = 0
        self._dynamic_stop_started_at: Optional[float] = None
        self._tick_i: int = 0

    def update(self, detections: list[ObstacleDetection]) -> ObstacleState:
        self._tick_i += 1
        safety_dets = [
            SafetyDetection(
                x1=d.bbox_px[0],
                y1=d.bbox_px[1],
                x2=d.bbox_px[2],
                y2=d.bbox_px[3],
                class_name=d.kind,
                confidence=d.confidence,
                estimated_distance_m=d.estimated_distance_m,
                estimated_lateral_m=d.estimated_lateral_m,
            )
            for d in detections
        ]
        analysis = self._safety.analyze(safety_dets)
        decision = analysis.decision

        state = ObstacleState(
            emergency_stop=decision.emergency_stop,
            speed_cap_ratio=decision.speed_cap_ratio,
            threat_level=decision.threat_level,
            closest_obstacle_m=decision.closest_obstacle_m,
            reason=decision.reason,
        )

        corridor_threats = analysis.corridor_threats
        corridor_kinds = {th.detection.class_name for th in corridor_threats}
        state.pedestrian_in_corridor = ObstacleKind.PEDESTRIAN in corridor_kinds
        state.cone_in_corridor = ObstacleKind.CONE in corridor_kinds
        state.barrier_in_corridor = ObstacleKind.BARRIER in corridor_kinds
        state.active_kinds = sorted(corridor_kinds)

        dynamic_threats = [th for th in corridor_threats if is_dynamic_kind(th.detection.class_name)]
        static_threats = [th for th in corridor_threats if is_static_kind(th.detection.class_name)]

        self._apply_dynamic_wait(state, dynamic_threats)
        if not state.waiting_for_dynamic_obstacle:
            self._apply_static_avoidance(state, static_threats)

        if state.emergency_stop:
            state.behavior_mode = ObstacleBehavior.EMERGENCY_STOP
        return state

    def _apply_dynamic_wait(self, state: ObstacleState, dynamic_threats: list[ThreatObservation]) -> None:
        nearest = self._nearest(dynamic_threats)
        should_wait = nearest is not None and nearest.distance_m <= DYNAMIC_STOP_DISTANCE_M

        if should_wait:
            self._dynamic_wait_active = True
            self._dynamic_clear_frames = 0
        elif self._dynamic_wait_active:
            if dynamic_threats:
                self._dynamic_clear_frames = 0
            else:
                self._dynamic_clear_frames += 1
                if self._dynamic_clear_frames >= DYNAMIC_CLEAR_FRAMES:
                    self._dynamic_wait_active = False
                    self._dynamic_clear_frames = 0

        if not self._dynamic_wait_active:
            self._dynamic_stop_started_at = None
            if nearest is not None and nearest.distance_m <= DYNAMIC_SLOW_DISTANCE_M:
                state.speed_cap_ratio = min(state.speed_cap_ratio, DYNAMIC_SLOW_SPEED_CAP)
                state.behavior_mode = ObstacleBehavior.DYNAMIC_SLOW
                state.closest_obstacle_m = nearest.distance_m
                state.reason = f"SLOW_DYNAMIC: {nearest.detection.class_name} at {nearest.distance_m:.1f}m"
            return

        state.waiting_for_dynamic_obstacle = True
        state.behavior_mode = ObstacleBehavior.DYNAMIC_WAIT
        state.speed_cap_ratio = 0.0
        if nearest is not None:
            state.closest_obstacle_m = nearest.distance_m
            state.reason = f"WAIT_DYNAMIC: {nearest.detection.class_name} at {nearest.distance_m:.1f}m"
        else:
            state.reason = f"WAIT_DYNAMIC: clearing {self._dynamic_clear_frames}/{DYNAMIC_CLEAR_FRAMES}"

        # Two-stage dynamic behavior (competition heuristic):
        # 1) Stop (wait) when pedestrian is too close.
        # 2) After a short stop-hold, if LiDAR lateral offset exists, suggest a cautious pass on opposite side.
        #    This is only a suggestion; lane constraint may clamp/disable it.
        if nearest is None:
            self._dynamic_stop_started_at = None
            return

        if self._dynamic_stop_started_at is None:
            self._dynamic_stop_started_at = float(self._tick_i)

        hold_ticks = 8  # ~8 frames in this pure-python loop; ROS node runs 20Hz => ~0.4s
        ticks_waiting = float(self._tick_i) - float(self._dynamic_stop_started_at)
        lat = float(nearest.lateral_m)
        if ticks_waiting >= hold_ticks and abs(lat) >= 0.35:
            state.dynamic_avoid_active = True
            # Opposite side of obstacle lateral: obstacle on left => pass right, etc.
            state.dynamic_avoidance_direction = "right" if lat > 0 else "left"
            state.speed_cap_ratio = min(state.speed_cap_ratio, 0.2)
            state.reason = f"AVOID_DYNAMIC_AFTER_STOP: dir={state.dynamic_avoidance_direction} lat={lat:+.2f}m"

    def _apply_static_avoidance(self, state: ObstacleState, static_threats: list[ThreatObservation]) -> None:
        nearest = self._nearest(static_threats)
        if nearest is None:
            self._static_commit_dir = None
            self._static_commit_frames_left = 0
            state.road_blocked = False
            return

        # Commit bırakma: yol temizlenince
        if nearest.distance_m > STATIC_AVOID_CLEAR_DISTANCE_M:
            self._static_commit_dir = None
            self._static_commit_frames_left = 0

        if nearest.distance_m > STATIC_LANE_CHANGE_TRIGGER_M:
            barrier_count = sum(1 for threat in static_threats if threat.detection.class_name == ObstacleKind.BARRIER)
            state.road_blocked = barrier_count > 1
            return

        # Sıralı engellerde zigzag'ı azaltmak için yönü "commit" et:
        # - commit aktifse, süre bitene kadar aynı yönü koru
        # - commit yoksa, en yakın engelin lateral'ına göre karar ver
        desired_dir = self._avoidance_direction(nearest.lateral_m)

        if self._static_commit_dir is None:
            self._static_commit_dir = desired_dir
            self._static_commit_frames_left = STATIC_AVOID_COMMIT_FRAMES
        else:
            # commit aktifken yön değiştirmeyi zorlaştır:
            # zıt yön ancak daha yakın ve belirgin ise (mesafe marjı) değişsin
            if desired_dir != self._static_commit_dir and self._static_commit_frames_left <= 0:
                # Yakındaki zıt tarafta engel var mı?
                opposite = [t for t in static_threats if self._avoidance_direction(t.lateral_m) == desired_dir]
                nearest_op = self._nearest(opposite)
                if nearest_op is not None and (nearest.distance_m - nearest_op.distance_m) >= STATIC_AVOID_SWITCH_MARGIN_M:
                    self._static_commit_dir = desired_dir
                    self._static_commit_frames_left = STATIC_AVOID_COMMIT_FRAMES

        if self._static_commit_frames_left > 0:
            self._static_commit_frames_left -= 1

        state.suggest_lane_change = True
        state.avoidance_direction = self._static_commit_dir
        state.behavior_mode = ObstacleBehavior.STATIC_AVOID
        state.speed_cap_ratio = min(state.speed_cap_ratio, STATIC_LANE_CHANGE_SPEED_CAP)
        state.reason = (
            f"AVOID_STATIC: {nearest.detection.class_name} -> {state.avoidance_direction} "
            f"at {nearest.distance_m:.1f}m (commit_left={self._static_commit_frames_left})"
        )

        if nearest.distance_m > STATIC_EMERGENCY_DISTANCE_M:
            state.emergency_stop = False
            if state.threat_level == ThreatLevel.EMERGENCY:
                state.threat_level = ThreatLevel.HARD_SLOW
            # SafetyLogic can output speed_cap_ratio=0.0 for <3m.
            # For static avoidance beyond the hard emergency distance, do not keep a full stop cap.
            state.speed_cap_ratio = max(state.speed_cap_ratio, STATIC_LANE_CHANGE_SPEED_CAP)

        barrier_count = sum(1 for threat in static_threats if threat.detection.class_name == ObstacleKind.BARRIER)
        state.road_blocked = barrier_count > 1

    @staticmethod
    def _nearest(threats: list[ThreatObservation]) -> Optional[ThreatObservation]:
        if not threats:
            return None
        return min(threats, key=lambda threat: threat.distance_m)

    @staticmethod
    def _avoidance_direction(lateral_m: float) -> str:
        return "right" if lateral_m >= 0.0 else "left"

