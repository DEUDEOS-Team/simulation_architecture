from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from collections.abc import Callable
from typing import Any, Iterable, Optional


@dataclass(frozen=True)
class Node:
    id: int
    lat: float
    lon: float


@dataclass(frozen=True)
class Edge:
    u: int
    v: int
    cost: float
    props: dict[str, Any]


@dataclass
class RouteGraph:
    nodes: list[Node]
    adj: dict[int, list[Edge]]


EARTH_RADIUS_M = 6_371_000.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2.0 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _round_key(lat: float, lon: float, *, decimals: int) -> tuple[float, float]:
    return (round(lat, decimals), round(lon, decimals))


def build_graph_from_centerlines_geojson(
    geojson: dict[str, Any],
    *,
    coord_round_decimals: int = 7,
) -> RouteGraph:
    """
    QGIS ile çizilmiş LineString centerline'larından basit bir graph üretir.

    Beklenti:
    - Feature.geometry.type == "LineString"
    - coordinates: [[lon,lat], [lon,lat], ...]
    - Kavşak birleşimleri için çizgilerin uç noktaları aynı koordinata getirilmeli.

    Graph modeli:
    - Node: unique (lat,lon) (yuvarlama ile)
    - Edge: her LineString içindeki ardışık nokta çiftleri arası çift yönlü (oneway=true ise tek yönlü)
    - cost: varsayılan haversine mesafe (metre); `speed_limit_mps` verilirse süre maliyeti (saniye) kullanılır
    """
    if geojson.get("type") != "FeatureCollection":
        raise ValueError("centerlines geojson FeatureCollection olmalı")

    key_to_id: dict[tuple[float, float], int] = {}
    nodes: list[Node] = []
    adj: dict[int, list[Edge]] = {}

    def get_node_id(lat: float, lon: float) -> int:
        k = _round_key(lat, lon, decimals=coord_round_decimals)
        if k in key_to_id:
            return key_to_id[k]
        nid = len(nodes)
        key_to_id[k] = nid
        nodes.append(Node(id=nid, lat=float(k[0]), lon=float(k[1])))
        adj[nid] = []
        return nid

    feats = geojson.get("features") or []
    for feat in feats:
        if (feat or {}).get("type") != "Feature":
            continue
        geom = (feat or {}).get("geometry") or {}
        if geom.get("type") != "LineString":
            continue
        coords = geom.get("coordinates") or []
        if len(coords) < 2:
            continue

        props = (feat or {}).get("properties") or {}
        if bool(props.get("blocked", False)):
            continue
        oneway = bool(props.get("oneway", False))
        speed_limit_mps = props.get("speed_limit_mps", None)
        try:
            speed_limit_mps_f = float(speed_limit_mps) if speed_limit_mps is not None else None
        except Exception:
            speed_limit_mps_f = None

        # coords are [lon,lat]
        pts: list[tuple[float, float]] = [(float(c[1]), float(c[0])) for c in coords]
        for (lat1, lon1), (lat2, lon2) in zip(pts, pts[1:], strict=False):
            u = get_node_id(lat1, lon1)
            v = get_node_id(lat2, lon2)
            if u == v:
                continue
            dist_m = float(haversine_m(lat1, lon1, lat2, lon2))
            if speed_limit_mps_f is not None and speed_limit_mps_f > 0.05:
                cost = float(dist_m / speed_limit_mps_f)  # seconds
            else:
                cost = float(dist_m)  # meters
            e = Edge(u=u, v=v, cost=cost, props=dict(props))
            adj[u].append(e)
            if not oneway:
                adj[v].append(Edge(u=v, v=u, cost=cost, props=dict(props)))

    return RouteGraph(nodes=nodes, adj=adj)


def load_centerlines_geojson(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    return json.loads(p.read_text(encoding="utf-8"))


def edge_props_tunnel(props: dict[str, Any]) -> bool:
    v = props.get("tunnel")
    return v in (True, "true", "True", 1, "1", "yes", "YES")


def edge_is_tunnel(e: Edge) -> bool:
    return edge_props_tunnel(e.props)


def iter_tunnel_directed_edges(g: RouteGraph):
    """Yönlü tünel kenarları (çift yönlü yolda hem u→v hem v→u ayrı adaydır)."""
    for edges in g.adj.values():
        for e in edges:
            if edge_is_tunnel(e):
                yield e


def graph_has_tunnel_edges(g: RouteGraph) -> bool:
    for _ in iter_tunnel_directed_edges(g):
        return True
    return False


def _dijkstra_all(
    g: RouteGraph,
    *,
    start: int,
    blocked_edges: Optional[set[tuple[int, int]]] = None,
    edge_cost_multiplier: Optional[Callable[[Edge], float]] = None,
) -> tuple[dict[int, float], dict[int, int]]:
    """Kaynak `start` için tüm düğümlerde en kısa mesafe ve önceki düğüm (pozitif ağırlık)."""
    import heapq

    dist: dict[int, float] = {int(start): 0.0}
    prev: dict[int, int] = {}
    pq: list[tuple[float, int]] = [(0.0, int(start))]
    seen: set[int] = set()

    while pq:
        d, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        for e in g.adj.get(u, []):
            if blocked_edges is not None and (int(e.u), int(e.v)) in blocked_edges:
                continue
            mult = 1.0 if edge_cost_multiplier is None else float(edge_cost_multiplier(e))
            if mult < 1e-9:
                mult = 1e-9
            nd = d + float(e.cost) * mult
            if nd < dist.get(int(e.v), float("inf")):
                dist[int(e.v)] = nd
                prev[int(e.v)] = u
                heapq.heappush(pq, (nd, int(e.v)))

    return dist, prev


def dijkstra_mandatory_tunnel(
    g: RouteGraph,
    *,
    start: int,
    goal: int,
    blocked_edges: Optional[set[tuple[int, int]]] = None,
    edge_cost_multiplier: Optional[Callable[[Edge], float]] = None,
) -> list[int]:
    """
    Rota, en az bir `tunnel: true` centerline kenarından geçmek zorunda (yönlü: o kenarı kullanır).

    Tünel işaretli kenar yoksa normal `dijkstra` ile aynı davranır.
    """
    if int(start) == int(goal):
        return [int(start)]
    if not graph_has_tunnel_edges(g):
        return dijkstra(
            g,
            start=int(start),
            goal=int(goal),
            blocked_edges=blocked_edges,
            edge_cost_multiplier=edge_cost_multiplier,
        )

    dist_s, _ = _dijkstra_all(
        g,
        start=int(start),
        blocked_edges=blocked_edges,
        edge_cost_multiplier=edge_cost_multiplier,
    )
    dist_g, _ = _dijkstra_all(
        g,
        start=int(goal),
        blocked_edges=blocked_edges,
        edge_cost_multiplier=edge_cost_multiplier,
    )

    best_cost = float("inf")
    best_u: Optional[int] = None
    best_v: Optional[int] = None

    for e in iter_tunnel_directed_edges(g):
        u, v = int(e.u), int(e.v)
        ds = dist_s.get(u)
        dg = dist_g.get(v)
        if ds is None or dg is None:
            continue
        mult = 1.0 if edge_cost_multiplier is None else float(edge_cost_multiplier(e))
        if mult < 1e-9:
            mult = 1e-9
        cand = float(ds) + float(e.cost) * mult + float(dg)
        if cand < best_cost:
            best_cost = cand
            best_u, best_v = u, v

    if best_u is None or best_v is None:
        return []

    p1 = dijkstra(
        g,
        start=int(start),
        goal=int(best_u),
        blocked_edges=blocked_edges,
        edge_cost_multiplier=edge_cost_multiplier,
    )
    p2 = dijkstra(
        g,
        start=int(best_v),
        goal=int(goal),
        blocked_edges=blocked_edges,
        edge_cost_multiplier=edge_cost_multiplier,
    )
    if not p1 or not p2:
        return []

    # p1: start..u, p2: v..goal — u→v tünel kenarı; v'yi atlamamak için birleştirme:
    if int(p1[-1]) == int(p2[0]):
        return p1 + p2[1:]
    return p1 + p2


def nearest_node_id(g: RouteGraph, *, lat: float, lon: float) -> int:
    if not g.nodes:
        raise ValueError("graph boş")
    best_id = 0
    best_d = float("inf")
    for n in g.nodes:
        d = haversine_m(lat, lon, n.lat, n.lon)
        if d < best_d:
            best_d = d
            best_id = n.id
    return best_id


def dijkstra(
    g: RouteGraph,
    *,
    start: int,
    goal: int,
    blocked_edges: Optional[set[tuple[int, int]]] = None,
    edge_cost_multiplier: Optional[Callable[[Edge], float]] = None,
) -> list[int]:
    """
    Basit Dijkstra (pozitif edge cost varsayılır). Çıktı: node id path (start..goal).

    edge_cost_multiplier: Kenar maliyetini çarpan ile ölçekler (ör. tünel segmentlerini ucuzlatmak için).
    """
    if start == goal:
        return [start]

    import heapq

    dist: dict[int, float] = {start: 0.0}
    prev: dict[int, int] = {}
    pq: list[tuple[float, int]] = [(0.0, start)]
    seen: set[int] = set()

    while pq:
        d, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        if u == goal:
            break
        for e in g.adj.get(u, []):
            if blocked_edges is not None and (int(e.u), int(e.v)) in blocked_edges:
                continue
            mult = 1.0 if edge_cost_multiplier is None else float(edge_cost_multiplier(e))
            if mult < 1e-9:
                mult = 1e-9
            nd = d + float(e.cost) * mult
            if nd < dist.get(e.v, float("inf")):
                dist[e.v] = nd
                prev[e.v] = u
                heapq.heappush(pq, (nd, e.v))

    if goal not in dist:
        return []

    path: list[int] = [goal]
    cur = goal
    while cur != start:
        cur = prev[cur]
        path.append(cur)
    path.reverse()
    return path


def path_coords(g: RouteGraph, node_path: Iterable[int]) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    by_id = {n.id: n for n in g.nodes}
    for nid in node_path:
        n = by_id.get(int(nid))
        if n is not None:
            out.append((n.lat, n.lon))
    return out

