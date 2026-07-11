from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
from typing import Optional

from deos_algorithms.geojson_mission_reader import (
    DEFAULT_ARRIVAL_RADIUS_M,
    DEFAULT_SPEED_LIMIT_RATIO,
    MissionPlan,
    MissionPoint,
    TaskType,
)
from deos_algorithms.route_graph import (
    Edge,
    RouteGraph,
    dijkstra,
    dijkstra_mandatory_tunnel,
    edge_is_tunnel,
    graph_has_tunnel_edges,
    haversine_m,
    nearest_node_id,
    path_coords,
)


def route_mission_plan_without_graph(
    plan: MissionPlan,
    *,
    reorder_checkpoints_by_nearest: bool = True,
    keep_park_last: bool = True,
) -> MissionPlan:
    """
    Centerline graph olmadan mission GeoJSON noktalarindan takip edilebilir waypoint sirasi uretir.

    Bu mod yol topolojisi bilmez; Dijkstra yerine mission noktalarini dogrudan kullanir.
    Varsayilan politika START'i basa alir, gorev noktalarini yakinlik sirasi ile dizer,
    PARK_ENTRY/PARK noktalarini sona alir.
    """
    if not plan.points:
        return plan

    pts = list(plan.points)
    original_start = next((p for p in pts if p.task == TaskType.START), None)
    if original_start is None:
        original_start = pts[0]
        start = replace(original_start, task=TaskType.START, name=original_start.name or "start")
    else:
        start = original_start

    park_tasks = {TaskType.PARK_ENTRY, TaskType.PARK}
    middle: list[MissionPoint] = []
    park_points: list[MissionPoint] = []
    for p in pts:
        if p is original_start:
            continue
        if p.task in park_tasks and keep_park_last:
            park_points.append(p)
        else:
            middle.append(p)

    ordered_middle = list(middle)
    if reorder_checkpoints_by_nearest and len(middle) > 1:
        ordered_middle = []
        remaining = list(middle)
        cur = start
        while remaining:
            best_i = min(
                range(len(remaining)),
                key=lambda i: haversine_m(
                    float(cur.lat),
                    float(cur.lon),
                    float(remaining[i].lat),
                    float(remaining[i].lon),
                ),
            )
            cur = remaining.pop(best_i)
            ordered_middle.append(cur)

    ordered = [start] + ordered_middle + park_points
    return MissionPlan(
        points=[replace(p, index=i) for i, p in enumerate(ordered)],
        source_file=plan.source_file,
        raw_crs=plan.raw_crs,
        meta=dict(plan.meta),
    )


def _tunnel_multiplier_from_plan(plan: MissionPlan) -> Callable[[Edge], float] | None:
    """`prefer_tunnel` açıksa `tunnel: true` centerline kenarlarını ucuzlatır (Dijkstra tünelden geçmeyi tercih eder)."""
    if not plan.prefer_tunnel_routing:
        return None
    scale = float(plan.tunnel_edge_cost_scale)

    def fn(e: Edge) -> float:
        v = e.props.get("tunnel")
        if v in (True, "true", "True", 1, "1", "yes", "YES"):
            return scale
        return 1.0

    return fn


def _leg_path(
    graph: RouteGraph,
    a: int,
    b: int,
    *,
    blocked_edges: set[tuple[int, int]] | None,
    edge_mult: Callable[[Edge], float] | None,
    tunnel_mandatory: bool,
) -> list[int]:
    """
    `tunnel_mandatory` ve graph'ta `tunnel: true` kenar varsa rota en az bir tünel kenarı kullanır.
    Görev GeoJSON'unda alan gerekmez; sadece centerlines'ta işaret yeterli.
    """
    use = bool(tunnel_mandatory) and graph_has_tunnel_edges(graph)
    if use:
        return dijkstra_mandatory_tunnel(
            graph,
            start=int(a),
            goal=int(b),
            blocked_edges=blocked_edges,
            edge_cost_multiplier=edge_mult,
        )
    return dijkstra(
        graph,
        start=int(a),
        goal=int(b),
        blocked_edges=blocked_edges,
        edge_cost_multiplier=edge_mult,
    )


def _find_edge(graph: RouteGraph, u: int, v: int) -> Edge | None:
    for e in graph.adj.get(int(u), []):
        if int(e.v) == int(v):
            return e
    return None


def _path_uses_tunnel(graph: RouteGraph, path: list[int]) -> bool:
    for a, b in zip(path, path[1:], strict=False):
        e = _find_edge(graph, a, b)
        if e is not None and edge_is_tunnel(e):
            return True
    return False


def _path_cost(
    graph: RouteGraph,
    path: list[int],
    edge_mult: Callable[[Edge], float] | None,
) -> float:
    total = 0.0
    for a, b in zip(path, path[1:], strict=False):
        e = _find_edge(graph, a, b)
        if e is None:
            return float("inf")
        mult = 1.0 if edge_mult is None else max(float(edge_mult(e)), 1e-9)
        total += float(e.cost) * mult
    return total


def _route_node_sequence(
    graph: RouteGraph,
    node_seq: list[int],
    *,
    blocked_edges: set[tuple[int, int]] | None,
    edge_mult: Callable[[Edge], float] | None,
    tunnel_mandatory: bool,
) -> list[int]:
    """
    Ardışık node hedeflerini bacak bacak en kısa yolla bağlar; `tunnel_mandatory`
    ise tünel şartını ROTANIN TAMAMINDA bir kez sağlar (her bacakta değil): normal
    rota tünelden geçmiyorsa tünel, en düşük ek maliyetli bacağa yerleştirilir.
    Başarısızlıkta boş liste döner.
    """
    legs: list[list[int]] = []
    for a, b in zip(node_seq, node_seq[1:], strict=False):
        leg = _leg_path(
            graph,
            int(a),
            int(b),
            blocked_edges=blocked_edges,
            edge_mult=edge_mult,
            tunnel_mandatory=False,
        )
        if not leg:
            return []
        legs.append(leg)

    if (
        tunnel_mandatory
        and legs
        and graph_has_tunnel_edges(graph)
        and not any(_path_uses_tunnel(graph, leg) for leg in legs)
    ):
        best_i = -1
        best_extra = float("inf")
        best_leg: list[int] = []
        for i, leg in enumerate(legs):
            t_leg = _leg_path(
                graph,
                int(leg[0]),
                int(leg[-1]),
                blocked_edges=blocked_edges,
                edge_mult=edge_mult,
                tunnel_mandatory=True,
            )
            if not t_leg:
                continue
            extra = _path_cost(graph, t_leg, edge_mult) - _path_cost(graph, leg, edge_mult)
            if extra < best_extra:
                best_extra, best_i, best_leg = extra, i, t_leg
        if best_i < 0:
            return []
        legs[best_i] = best_leg

    full_path: list[int] = []
    for leg in legs:
        if not full_path:
            full_path.extend(leg)
        else:
            full_path.extend(leg[1:])  # bağlantı node'u iki bacakta da var
    return full_path


def route_mission_plan_via_graph(
    plan: MissionPlan,
    graph: RouteGraph,
    *,
    fallback_to_original_on_failure: bool = True,
    blocked_edges: set[tuple[int, int]] | None = None,
    tunnel_mandatory: bool = True,
) -> MissionPlan:
    """
    Verilen MissionPlan noktalarını graph'a snap edip ardışık hedefler arasında Dijkstra ile rota üretir.

    Çıktı: Yeni bir MissionPlan:
    - Waypoint listesi graph node path koordinatlarından oluşur.
    - Mission'daki hedef noktalar (start/gorev/park_giris/pickup/dropoff/stop) rotadaki
      en yakın node'a "etiketlenmiş task" olarak aktarılır.

    Not: Bu bir "global routing" katmanı; araç kontrolü yine WaypointManager + lokal arbiter ile yapılır.

    tunnel_mandatory: Centerlines graph'ta en az bir `tunnel: true` kenar varsa, rota GÖREVİN
    TAMAMINDA en az bir tünel kenarı içerir: normal en kısa rota tünelden geçmiyorsa, tünel
    en düşük ek maliyetle eklenebilen TEK bacağa yerleştirilir (her bacağa ayrı ayrı değil —
    aksi hâlde çok noktalı görevlerde aynı yoldan gidiş-gelişler oluşur). `False` ise klasik
    en kısa yol.
    """
    if not plan.points or not graph.nodes:
        return plan

    edge_mult = _tunnel_multiplier_from_plan(plan)

    snapped_ids: list[int] = []
    for p in plan.points:
        snapped_ids.append(int(nearest_node_id(graph, lat=float(p.lat), lon=float(p.lon))))

    full_path = _route_node_sequence(
        graph,
        snapped_ids,
        blocked_edges=blocked_edges,
        edge_mult=edge_mult,
        tunnel_mandatory=tunnel_mandatory,
    )

    if not full_path:
        if fallback_to_original_on_failure:
            return plan
        return MissionPlan(points=[], source_file=plan.source_file, raw_crs=plan.raw_crs, meta=dict(plan.meta))

    coords = path_coords(graph, full_path)  # [(lat,lon), ...]
    if not coords:
        return plan

    # Map snapped node -> original mission point (task/name/speed/radius).
    # If multiple mission points snap to same node, keep the last one (later in sequence).
    node_to_mission: dict[int, MissionPoint] = {}
    for mp, nid in zip(plan.points, snapped_ids, strict=False):
        node_to_mission[int(nid)] = mp

    new_points: list[MissionPoint] = []
    for i, (lat, lon) in enumerate(coords):
        task = TaskType.CHECKPOINT
        name = f"route_{i}"
        speed = DEFAULT_SPEED_LIMIT_RATIO
        radius = DEFAULT_ARRIVAL_RADIUS_M
        heading: Optional[float] = None
        point_id = None

        mid = full_path[i]
        if int(mid) in node_to_mission:
            mp = node_to_mission[int(mid)]
            task = mp.task
            name = mp.name
            speed = float(mp.speed_limit_ratio)
            radius = float(mp.arrival_radius_m)
            heading = mp.heading_deg
            point_id = mp.point_id

        new_points.append(
            MissionPoint(
                index=i,
                point_id=point_id,
                name=name,
                lat=float(lat),
                lon=float(lon),
                task=task,
                heading_deg=heading,
                speed_limit_ratio=float(speed),
                arrival_radius_m=float(radius),
            )
        )

    # Güvenlik: ilk waypoint START olmalı (şartname akışı).
    if new_points and new_points[0].task != TaskType.START:
        # Eğer orijinal planda start varsa onu dayat.
        start_mp = plan.start
        if start_mp is not None:
            new_points[0] = replace(new_points[0], task=TaskType.START, name=start_mp.name)
        else:
            new_points[0] = replace(new_points[0], task=TaskType.START, name="start")

    return MissionPlan(points=new_points, source_file=plan.source_file, raw_crs=plan.raw_crs, meta=dict(plan.meta))


def advance_mission_index_by_position(plan: MissionPlan, *, start_index: int, lat: float, lon: float) -> int:
    """
    Basit ilerleme: araç pozisyonu, sıradaki mission hedefinin arrival radius'u içine girdiyse index'i artır.
    (WaypointManager ile aynı işi, sadece mission hedef listesi için yapar.)
    """
    idx = max(0, int(start_index))
    while idx < len(plan.points):
        p = plan.points[idx]
        dist = haversine_m(float(lat), float(lon), float(p.lat), float(p.lon))
        if dist <= float(p.arrival_radius_m):
            idx += 1
            continue
        break
    return idx


def route_remaining_mission_via_graph(
    plan: MissionPlan,
    graph: RouteGraph,
    *,
    current_lat: float,
    current_lon: float,
    start_index: int,
    blocked_edges: set[tuple[int, int]] | None = None,
    fallback_to_original_on_failure: bool = True,
    tunnel_mandatory: bool = True,
) -> MissionPlan:
    """
    Replanning için: mevcut pozisyondan başlayarak mission hedeflerinin (start_index..end)
    sırasını koruyarak graph üzerinde rota üretir.

    tunnel_mandatory: Kalan rotanın TAMAMINDA bir kez tünel şartı uygular. Araç tünelden
    zaten geçtiyse çağıran taraf `False` vermelidir (mission_planning bunu takip etmeli).
    """
    if not plan.points or not graph.nodes:
        return plan
    if start_index >= len(plan.points):
        return MissionPlan(points=[], source_file=plan.source_file, raw_crs=plan.raw_crs, meta=dict(plan.meta))

    edge_mult = _tunnel_multiplier_from_plan(plan)

    targets = plan.points[start_index:]
    start_node = int(nearest_node_id(graph, lat=float(current_lat), lon=float(current_lon)))
    target_nodes = [int(nearest_node_id(graph, lat=float(p.lat), lon=float(p.lon))) for p in targets]

    full_path = _route_node_sequence(
        graph,
        [start_node, *target_nodes],
        blocked_edges=blocked_edges,
        edge_mult=edge_mult,
        tunnel_mandatory=tunnel_mandatory,
    )

    if not full_path:
        if fallback_to_original_on_failure:
            return plan
        return MissionPlan(points=[], source_file=plan.source_file, raw_crs=plan.raw_crs, meta=dict(plan.meta))

    coords = path_coords(graph, full_path)
    if not coords:
        return plan

    node_to_mission: dict[int, MissionPoint] = {}
    for mp, nid in zip(targets, target_nodes, strict=False):
        node_to_mission[int(nid)] = mp

    new_points: list[MissionPoint] = []
    for i, (lat, lon) in enumerate(coords):
        task = TaskType.CHECKPOINT
        name = f"route_{i}"
        speed = DEFAULT_SPEED_LIMIT_RATIO
        radius = DEFAULT_ARRIVAL_RADIUS_M
        heading: Optional[float] = None
        point_id = None

        mid = full_path[i]
        if int(mid) in node_to_mission:
            mp = node_to_mission[int(mid)]
            task = mp.task
            name = mp.name
            speed = float(mp.speed_limit_ratio)
            radius = float(mp.arrival_radius_m)
            heading = mp.heading_deg
            point_id = mp.point_id

        new_points.append(
            MissionPoint(
                index=i,
                point_id=point_id,
                name=name,
                lat=float(lat),
                lon=float(lon),
                task=task,
                heading_deg=heading,
                speed_limit_ratio=float(speed),
                arrival_radius_m=float(radius),
            )
        )

    return MissionPlan(points=new_points, source_file=plan.source_file, raw_crs=plan.raw_crs, meta=dict(plan.meta))
