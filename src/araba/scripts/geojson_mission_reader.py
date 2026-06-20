"""
geojson_mission_reader.py
-------------------------
GeoJSON FeatureCollection dosyasından görev noktalarını okur.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Optional


class TaskType:
    START = "start"
    CHECKPOINT = "checkpoint"
    STOP = "stop"
    PARK = "park"
    PARK_ENTRY = "park_entry"
    PICKUP = "pickup"
    DROPOFF = "dropoff"


VALID_TASKS = {
    TaskType.START,
    TaskType.CHECKPOINT,
    TaskType.STOP,
    TaskType.PARK,
    TaskType.PARK_ENTRY,
    TaskType.PICKUP,
    TaskType.DROPOFF,
}

# Şartname / Türkçe görev GeoJSON: `task` alanında kullanılabilecek eş anlamlılar (küçük harfe çevrilir).
_TASK_SYNONYMS: dict[str, str] = {
    # başlangıç
    "baslangic": TaskType.START,
    "başlangıç": TaskType.START,
    "basla": TaskType.START,
    # durak / zorunlu durma
    "durak": TaskType.STOP,
    "bus_stop": TaskType.STOP,
    # görev noktası
    "gorev": TaskType.CHECKPOINT,
    "görev": TaskType.CHECKPOINT,
    "gorev_yeri": TaskType.CHECKPOINT,
    "görev_yeri": TaskType.CHECKPOINT,
    "checkpoint": TaskType.CHECKPOINT,
    # park
    "park": TaskType.PARK,
    "park_yeri": TaskType.PARK,
    "parkyeri": TaskType.PARK,
    "otopark": TaskType.PARK,
    "park_entry": TaskType.PARK_ENTRY,
    "park_giris": TaskType.PARK_ENTRY,
    "park_giriş": TaskType.PARK_ENTRY,
    "otopark_giris": TaskType.PARK_ENTRY,
    "otopark_giriş": TaskType.PARK_ENTRY,
    # yol üstü ara nokta (rota zorlaması — tünel ekseni vb.)
    "via": TaskType.CHECKPOINT,
    "ara_nokta": TaskType.CHECKPOINT,
    "tunel": TaskType.CHECKPOINT,
    "tünel": TaskType.CHECKPOINT,
}

DEFAULT_ARRIVAL_RADIUS_M = 1.0
DEFAULT_SPEED_LIMIT_RATIO = 1.0


@dataclass
class MissionPoint:
    index: int
    point_id: Optional[int]
    name: str
    lat: float
    lon: float
    task: str
    heading_deg: Optional[float]
    speed_limit_ratio: float
    arrival_radius_m: float


@dataclass
class MissionPlan:
    points: list[MissionPoint] = field(default_factory=list)
    source_file: str = ""
    raw_crs: Optional[str] = None
    # GeoJSON FeatureCollection üzerindeki yabancı üyeler / özellikler (rota tercihi vb.)
    meta: dict[str, Any] = field(default_factory=dict)

    @property
    def start(self) -> Optional[MissionPoint]:
        return next((p for p in self.points if p.task == TaskType.START), None)

    @property
    def goal(self) -> Optional[MissionPoint]:
        return next(
            (p for p in reversed(self.points) if p.task in {TaskType.STOP, TaskType.DROPOFF}),
            None,
        )

    def __len__(self) -> int:
        return len(self.points)

    def __iter__(self):
        return iter(self.points)

    @property
    def prefer_tunnel_routing(self) -> bool:
        v = self.meta.get("prefer_tunnel", self.meta.get("prefer_tunnel_routing", False))
        return v in (True, "true", "True", 1, "1")

    @property
    def tunnel_edge_cost_scale(self) -> float:
        """`tunnel: true` centerline kenarlarında maliyet çarpanı (<1 => tünel tercih edilir)."""
        try:
            return float(self.meta.get("tunnel_edge_cost_scale", 0.3))
        except Exception:
            return 0.3


class GeoJsonMissionReader:
    def read_file(self, path: str) -> MissionPlan:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return self._parse(data, source_file=path)

    def read_string(self, geojson_str: str) -> MissionPlan:
        data = json.loads(geojson_str)
        return self._parse(data, source_file="<string>")

    def _parse(self, data: dict, source_file: str) -> MissionPlan:
        if data.get("type") != "FeatureCollection":
            raise ValueError(
                f"GeoJSON tipi 'FeatureCollection' olmalı, alındı: '{data.get('type')}'"
            )

        plan = MissionPlan(source_file=source_file)
        if "crs" in data:
            plan.raw_crs = str(data["crs"])

        meta: dict[str, Any] = {}
        root_props = data.get("properties")
        if isinstance(root_props, dict):
            meta.update(root_props)
        extra = data.get("deos_mission_meta")
        if isinstance(extra, dict):
            meta.update(extra)
        plan.meta = meta

        for idx, feature in enumerate(data.get("features", [])):
            point = self._parse_feature(idx, feature)
            if point is not None:
                plan.points.append(point)

        return plan

    def _parse_feature(self, idx: int, feature: dict) -> Optional[MissionPoint]:
        if feature.get("type") != "Feature":
            return None

        geometry = feature.get("geometry") or {}
        if geometry.get("type") != "Point":
            return None

        coords = geometry.get("coordinates", [])
        if len(coords) < 2:
            raise ValueError(
                f"Feature index={idx}: 'coordinates' en az [lon, lat] içermeli, alındı: {coords}"
            )

        lon = float(coords[0])
        lat = float(coords[1])

        props = feature.get("properties") or {}
        task_raw = str(props.get("task", "")).strip().lower()
        name = str(props.get("name", f"Nokta-{idx}")).strip()
        low_name = name.strip().lower()

        # Önce eş anlamlı `task` değerlerini iç kod tipine çevir
        task = _TASK_SYNONYMS.get(task_raw, task_raw)
        if task not in VALID_TASKS:
            task = ""

        # Şartname GeoJSON örneğinde `name` alanı start/gorev_*/park_giris gibi geliyor.
        # Repodaki algoritmalar ise `task` alanı üzerinden ilerliyor. Bu yüzden:
        # - task yoksa veya tanımsızsa name ile eşle.
        # - park giriş noktası: park_giris -> PARK_ENTRY
        if not task or task not in VALID_TASKS:
            if low_name == "start":
                task = TaskType.START
            elif low_name.startswith("durak"):
                task = TaskType.STOP
            elif low_name.startswith(("gorev", "görev")):
                task = TaskType.CHECKPOINT
            elif low_name.startswith(("tunel", "tünel", "tunnel")):
                task = TaskType.CHECKPOINT
            elif low_name in {"park_giris", "park giriş", "otopark giris", "otopark_giris", "otopark_giriş"}:
                task = TaskType.PARK_ENTRY
            elif low_name.startswith("park"):
                task = TaskType.PARK
            else:
                task = TaskType.CHECKPOINT

        heading_deg: Optional[float] = None
        if "heading_deg" in props and props["heading_deg"] is not None:
            heading_deg = float(props["heading_deg"]) % 360.0

        return MissionPoint(
            index=idx,
            point_id=int(props["id"]) if "id" in props else None,
            name=name,
            lat=lat,
            lon=lon,
            task=task,
            heading_deg=heading_deg,
            speed_limit_ratio=float(props.get("speed_limit_ratio", DEFAULT_SPEED_LIMIT_RATIO)),
            arrival_radius_m=float(props.get("radius_m", DEFAULT_ARRIVAL_RADIUS_M)),
        )

