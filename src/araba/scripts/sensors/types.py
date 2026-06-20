from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class StereoBbox:
    """
    Stereo/camera detector output for one object.
    Depth can come from stereo disparity or depth camera.
    """

    class_name: str
    confidence: float
    bbox_px: tuple[float, float, float, float]  # x1,y1,x2,y2
    distance_m: Optional[float] = None
    lateral_m: Optional[float] = None


@dataclass(frozen=True)
class LidarObstacle:
    """LiDAR perception output in ego frame."""

    kind: str  # "pedestrian" | "cone" | "barrier" | ...
    confidence: float
    distance_m: float
    lateral_m: float
    # Optional: if you also have a 2D bbox from camera association, pass it through.
    bbox_px: Optional[tuple[float, float, float, float]] = None


@dataclass(frozen=True)
class ImuSample:
    """
    Minimal IMU sample needed by navigation/controller.
    heading_deg: yaw in degrees, north-referenced or map-referenced depending on your stack.
    """

    heading_deg: float
    yaw_rate_dps: float = 0.0

