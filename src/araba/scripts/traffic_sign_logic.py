"""
traffic_sign_logic.py
---------------------
Tabela tanıma modelinin çıktılarını alıp, aracın davranışına
dönüşen KISITLAR üretir.

Mimari felsefe:
- Bu modül tabelaları tespit ETMİYOR. Tespit edildiği varsayılıyor.
- Bu modül doğrudan komut üretmiyor ("dur!"), KISIT üretiyor
  ("bir sonraki DUR noktasında tam dur yapılacak").
- Çıktılar waypoint_manager ve controller tarafından okunur,
  onlar kendi kararlarını bu kısıtları göz önünde bulundurarak verir.

safety_logic.py ile aynı tasarım: ROS bağımsız, saf Python, test edilebilir.
"""

from dataclasses import dataclass, field
from typing import Optional
import time


class SignClass:
    STOP = "dur tabelası"
    NO_ENTRY = "girilmez"
    PEDESTRIAN_CROSSING = "yaya gecidi"
    BUS_STOP = "durak"

    NO_RIGHT_TURN = "saga donulmez"
    NO_LEFT_TURN = "sola donulmez"
    KEEP_RIGHT = "sağdan gidiniz"
    KEEP_LEFT = "soldan gidin"
    MUST_RIGHT = "sola mecburi"
    MUST_LEFT = "saga mecburi"
    MUST_STRAIGHT = "ileri mecburi"
    STRAIGHT_OR_RIGHT = "ileri ve saga mecburi yon"
    STRAIGHT_OR_LEFT = "ileri ve sola mecburi yon"
    AHEAD_THEN_RIGHT = "ileriden saga mecburi yon"
    AHEAD_THEN_LEFT = "ileriden sola mecburi yon"
    ROUNDABOUT = "doner kavsak"
    LANE_ARRANGEMENT_H = "sol seridin sonu"
    LANE_ARRANGEMENT_I = "sag seridin sonu"
    TWO_WAY = "iki yonlu yol"

    PARKING_AREA = "park"
    NO_PARKING = "park yapilmaz"

    TRAFFIC_LIGHT_AHEAD = "isikli isaret cihazi"
    TUNNEL = "tunel"


IMMEDIATE_ACTION = {
    SignClass.STOP,
    SignClass.NO_ENTRY,
    SignClass.PEDESTRIAN_CROSSING,
    SignClass.BUS_STOP,
}

TURN_RESTRICTION = {
    SignClass.NO_RIGHT_TURN,
    SignClass.NO_LEFT_TURN,
    SignClass.KEEP_RIGHT,
    SignClass.KEEP_LEFT,
    SignClass.MUST_RIGHT,
    SignClass.MUST_LEFT,
    SignClass.MUST_STRAIGHT,
    SignClass.STRAIGHT_OR_RIGHT,
    SignClass.STRAIGHT_OR_LEFT,
    SignClass.AHEAD_THEN_RIGHT,
    SignClass.AHEAD_THEN_LEFT,
    SignClass.ROUNDABOUT,
    SignClass.LANE_ARRANGEMENT_H,
    SignClass.LANE_ARRANGEMENT_I,
    SignClass.TWO_WAY,
}

PARKING_RELATED = {SignClass.PARKING_AREA, SignClass.NO_PARKING}
INFRASTRUCTURE = {SignClass.TRAFFIC_LIGHT_AHEAD, SignClass.TUNNEL}


MIN_CONFIDENCE = 0.5
CONFIRM_FRAMES = 3
SIGN_VALIDITY_SECONDS = 5.0
STOP_TRIGGER_DISTANCE_M = 10.0
STOP_HOLD_SECONDS = 5.0
STOP_REARM_SECONDS = SIGN_VALIDITY_SECONDS
CROSSWALK_SPEED_RATIO = 0.5
TUNNEL_SPEED_RATIO = 0.7


@dataclass
class SignDetection:
    class_name: str
    confidence: float
    bbox_px: tuple[float, float, float, float]
    estimated_distance_m: Optional[float] = None


@dataclass
class TurnPermissions:
    left: bool = True
    straight: bool = True
    right: bool = True
    forced_direction: Optional[str] = None

    def apply_restriction(self, sign_class: str) -> None:
        if sign_class == SignClass.NO_LEFT_TURN:
            self.left = False
        elif sign_class == SignClass.NO_RIGHT_TURN:
            self.right = False
        elif sign_class == SignClass.MUST_LEFT:
            self.left, self.straight, self.right = True, False, False
            self.forced_direction = "left"
        elif sign_class == SignClass.MUST_RIGHT:
            self.left, self.straight, self.right = False, False, True
            self.forced_direction = "right"
        elif sign_class == SignClass.MUST_STRAIGHT:
            self.left, self.straight, self.right = False, True, False
            self.forced_direction = "straight"
        elif sign_class == SignClass.STRAIGHT_OR_LEFT:
            self.left, self.straight, self.right = True, True, False
        elif sign_class == SignClass.STRAIGHT_OR_RIGHT:
            self.left, self.straight, self.right = False, True, True
        elif sign_class == SignClass.AHEAD_THEN_LEFT:
            self.left, self.straight, self.right = True, False, False
            self.forced_direction = "left"
        elif sign_class == SignClass.AHEAD_THEN_RIGHT:
            self.left, self.straight, self.right = False, False, True
            self.forced_direction = "right"
        elif sign_class == SignClass.KEEP_LEFT:
            self.forced_direction = "pass_left"
        elif sign_class == SignClass.KEEP_RIGHT:
            self.forced_direction = "pass_right"
        elif sign_class == SignClass.ROUNDABOUT:
            self.forced_direction = "roundabout"


@dataclass
class TrafficSignState:
    must_stop_soon: bool = False
    speed_cap_ratio: float = 1.0
    turn_permissions: TurnPermissions = field(default_factory=TurnPermissions)
    traffic_light_expected: bool = False
    approaching_tunnel: bool = False
    current_area_is_parking: bool = False
    current_area_no_parking: bool = False
    active_signs: list[str] = field(default_factory=list)
    reasons: list[str] = field(default_factory=list)


@dataclass
class _SignMemory:
    class_name: str
    frames_seen: int
    last_seen_time: float
    last_distance_m: Optional[float]
    confirmed: bool


class TrafficSignLogic:
    def __init__(self):
        self._memories: dict[str, _SignMemory] = {}
        self._pending_stop: bool = False
        self._stop_started_at: Optional[float] = None
        self._stop_cooldown_until: Optional[float] = None
        self._pending_turn_restrictions: TurnPermissions = TurnPermissions()

    def reset(self) -> None:
        self._memories.clear()
        self._pending_stop = False
        self._stop_started_at = None
        self._stop_cooldown_until = None
        self._pending_turn_restrictions = TurnPermissions()

    def update(self, detections: list[SignDetection], now: Optional[float] = None) -> TrafficSignState:
        if now is None:
            now = time.time()

        detections = [d for d in detections if d.confidence >= MIN_CONFIDENCE]
        self._update_memory(detections, now)
        self._forget_expired(now)
        active = [m for m in self._memories.values() if m.confirmed]
        return self._build_state(active, now)

    def notify_intersection_passed(self) -> None:
        self._pending_turn_restrictions = TurnPermissions()
        for cls in list(self._memories.keys()):
            if cls in TURN_RESTRICTION:
                del self._memories[cls]

    def notify_stop_completed(self, now: Optional[float] = None) -> None:
        if now is None:
            now = time.time()
        self._pending_stop = False
        self._stop_started_at = None
        self._stop_cooldown_until = now + STOP_REARM_SECONDS
        self._memories.pop(SignClass.STOP, None)

    def _update_memory(self, detections: list[SignDetection], now: float) -> None:
        for det in detections:
            mem = self._memories.get(det.class_name)
            if mem is None:
                self._memories[det.class_name] = _SignMemory(
                    class_name=det.class_name,
                    frames_seen=1,
                    last_seen_time=now,
                    last_distance_m=det.estimated_distance_m,
                    confirmed=(CONFIRM_FRAMES <= 1),
                )
            else:
                mem.frames_seen += 1
                mem.last_seen_time = now
                if det.estimated_distance_m is not None:
                    # Keep nearest distance (distance and confidence are tracked separately).
                    if mem.last_distance_m is None or float(det.estimated_distance_m) < float(mem.last_distance_m):
                        mem.last_distance_m = float(det.estimated_distance_m)
                if mem.frames_seen >= CONFIRM_FRAMES:
                    mem.confirmed = True

    def _forget_expired(self, now: float) -> None:
        expired = [
            cls for cls, mem in self._memories.items()
            if (now - mem.last_seen_time) > SIGN_VALIDITY_SECONDS
        ]
        for cls in expired:
            del self._memories[cls]

    def _build_state(self, active_memories: list[_SignMemory], now: float) -> TrafficSignState:
        state = TrafficSignState()
        state.turn_permissions = self._pending_turn_restrictions

        for mem in active_memories:
            cls = mem.class_name
            state.active_signs.append(cls)

            if cls == SignClass.STOP:
                if self._stop_cooldown_until is not None and now < self._stop_cooldown_until:
                    self._stop_cooldown_until = max(self._stop_cooldown_until, now + STOP_REARM_SECONDS)
                    state.reasons.append("STOP cooldown active")
                    continue

                if mem.last_distance_m is None or mem.last_distance_m <= STOP_TRIGGER_DISTANCE_M:
                    if not self._pending_stop:
                        self._pending_stop = True
                        self._stop_started_at = None

                if self._pending_stop:
                    if self._stop_started_at is None:
                        self._stop_started_at = now
                    elapsed = now - self._stop_started_at
                    if elapsed < STOP_HOLD_SECONDS:
                        state.must_stop_soon = True
                        state.reasons.append(f"STOP hold {elapsed:.1f}/{STOP_HOLD_SECONDS:.1f}s")
                    else:
                        self._pending_stop = False
                        self._stop_started_at = None
                        self._stop_cooldown_until = now + STOP_REARM_SECONDS
                        state.reasons.append("STOP completed, entering cooldown")

            elif cls == SignClass.NO_ENTRY:
                state.must_stop_soon = True
                state.reasons.append("NO_ENTRY ahead")

            elif cls == SignClass.PEDESTRIAN_CROSSING:
                state.speed_cap_ratio = min(state.speed_cap_ratio, CROSSWALK_SPEED_RATIO)
                state.reasons.append("pedestrian crossing ahead")

            elif cls == SignClass.BUS_STOP:
                pass

            elif cls in TURN_RESTRICTION:
                self._pending_turn_restrictions.apply_restriction(cls)
                state.reasons.append(f"turn restriction: {cls}")

            elif cls == SignClass.PARKING_AREA:
                state.current_area_is_parking = True
            elif cls == SignClass.NO_PARKING:
                state.current_area_no_parking = True

            elif cls == SignClass.TRAFFIC_LIGHT_AHEAD:
                state.traffic_light_expected = True
                state.reasons.append("traffic light ahead, activate light detection")

            elif cls == SignClass.TUNNEL:
                state.approaching_tunnel = True
                state.speed_cap_ratio = min(state.speed_cap_ratio, TUNNEL_SPEED_RATIO)
                state.reasons.append("tunnel ahead, perception may degrade")

        state.turn_permissions = TurnPermissions(
            left=self._pending_turn_restrictions.left,
            straight=self._pending_turn_restrictions.straight,
            right=self._pending_turn_restrictions.right,
            forced_direction=self._pending_turn_restrictions.forced_direction,
        )
        return state

