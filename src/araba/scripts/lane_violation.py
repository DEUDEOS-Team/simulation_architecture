from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from deos_algorithms.decision_arbiter import LaneBounds


def wheels_outside_lane(*, lane: LaneBounds, half_width_m: float) -> bool:
    """
    Şerit ihlali için basit geometrik kontrol (base_link frame):
    - Araç merkezi y=0 kabul edilir.
    - Tekerlekler y=±half_width_m kabul edilir.
    - lane.left_y_m ve lane.right_y_m aynı frame'de sol/sağ sınırdır.

    "2 tekerlek tamamen şerit dışarı" şartnamesini yaklaşıklar:
    - sol teker ( +half_width ) sol sınırı aşarsa veya
    - sağ teker ( -half_width ) sağ sınırı aşarsa
    ihlal=true.
    """
    if not lane.is_valid:
        return False
    hw = max(0.0, float(half_width_m))
    left_wheel_y = hw
    right_wheel_y = -hw
    return bool(left_wheel_y > float(lane.left_y_m) or right_wheel_y < float(lane.right_y_m))


@dataclass
class LaneViolationTracker:
    """
    Şartnameye yakın sayaç:
    - Şerit dışında geçirilen süreyi biriktirir.
    - Her 10 saniyelik periyot bir ihlal sayılır.

    Not: "Şerit dışındayken durursa tek ihlal" kuralını uygulamak için araç hızı gerekir.
    Bu tracker isteğe bağlı speed_mps ile bunu destekler; speed=None ise sadece süre kuralı uygulanır.
    """

    bucket_s: float = 10.0
    stationary_speed_eps_mps: float = 0.05

    outside_since_s: Optional[float] = None
    outside_accum_s: float = 0.0
    violation_count: int = 0
    _last_bucket_index: int = 0

    def reset(self) -> None:
        self.outside_since_s = None
        self.outside_accum_s = 0.0
        self.violation_count = 0
        self._last_bucket_index = 0

    def update(self, *, now_s: float, outside: bool, speed_mps: float | None = None) -> None:
        now = float(now_s)
        if not outside:
            self.outside_since_s = None
            self.outside_accum_s = 0.0
            self._last_bucket_index = 0
            return

        if self.outside_since_s is None:
            self.outside_since_s = now
            self.outside_accum_s = 0.0
            self._last_bucket_index = 0

        self.outside_accum_s = max(0.0, now - float(self.outside_since_s))

        # Optional: if stationary while outside => count at least 1
        if speed_mps is not None and abs(float(speed_mps)) <= float(self.stationary_speed_eps_mps):
            self.violation_count = max(self.violation_count, 1)

        bucket_index = int(self.outside_accum_s // max(1e-6, float(self.bucket_s)))
        if bucket_index > self._last_bucket_index:
            # each bucket adds 1 violation
            self.violation_count += bucket_index - self._last_bucket_index
            self._last_bucket_index = bucket_index

