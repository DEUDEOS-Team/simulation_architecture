from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from collections.abc import Callable
from typing import Any, Iterable, Optional

from deos_algorithms.geo_utils import EARTH_RADIUS_M, haversine_m

# U dönüşü caydırma: rota araması, gidiş yönünü kısa mesafede ~180° tersine
# çeviren SIKI dönüşlere büyük ek maliyet biçer. Böylece U dönüşü ancak başka
# hiçbir yol yokken (veya alternatifler bu cezadan daha pahalıyken) seçilir.
# Uzun mesafeye yayılan yön değişimi (ör. döner kavşak yayı) meşrudur, cezalanmaz.
U_TURN_ANGLE_DEG = 150.0
U_TURN_PENALTY_COST = 300.0  # cost birimi cinsinden (tipik: metre)
_U_TURN_WINDOW_M = 8.0  # referans yönden sapma bu mesafeden kısa sürerken 150°'ye ulaşırsa U sayılır
_U_TURN_STRAIGHT_DEV_DEG = 30.0  # bu sapmanın altı "hâlâ düz gidiyor" sayılır (referans korunur)


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _angle_diff_deg(a: float, b: float) -> float:
    return ((a - b + 180.0) % 360.0) - 180.0


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


def _reverse_graph(g: RouteGraph) -> RouteGraph:
    """Yönlü grafın tüm kenarlarını tersine çevirerek yeni bir graf döner."""
    rev_adj: dict[int, list[Edge]] = {nid: [] for nid in g.adj}
    for edges in g.adj.values():
        for e in edges:
            rev_adj[e.v].append(Edge(u=e.v, v=e.u, cost=e.cost, props=e.props))
    return RouteGraph(nodes=g.nodes, adj=rev_adj)


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
    # Yönlü grafta v->goal mesafelerini bulmak için ters grafta goal'dan Dijkstra.
    rev_blocked: Optional[set[tuple[int, int]]] = (
        {(v, u) for u, v in blocked_edges} if blocked_edges else None
    )
    rev_g = _reverse_graph(g)
    rev_mult: Optional[Callable[[Edge], float]] = None
    if edge_cost_multiplier is not None:
        def rev_mult(re: Edge, _ecm=edge_cost_multiplier, _g=g) -> float:
            for e in _g.adj.get(re.v, []):
                if e.v == re.u:
                    return float(_ecm(e))
            return 1.0
    dist_g, _ = _dijkstra_all(
        rev_g,
        start=int(goal),
        blocked_edges=rev_blocked,
        edge_cost_multiplier=rev_mult,
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
    u_turn_penalty: float = U_TURN_PENALTY_COST,
) -> list[int]:
    """
    Yön-farkındalıklı Dijkstra (pozitif edge cost varsayılır). Çıktı: node id path (start..goal).

    edge_cost_multiplier: Kenar maliyetini çarpan ile ölçekler (ör. tünel segmentlerini ucuzlatmak için).
    u_turn_penalty: Referans gidiş yönünden sapma _U_TURN_WINDOW_M'den kısa yol
      içinde U_TURN_ANGLE_DEG'e ulaşırsa (sıkı geri dönüş) eklenen maliyet.
      Çift yönlü yolun 3-4 m'lik dönüş cebi yakalanır; döner kavşak yayı gibi
      geniş dönüşler pencereyi aştığından cezalanmaz. 0 => kapalı.
    """
    if start == goal:
        return [start]

    by_id = {n.id: n for n in g.nodes}

    # Durum: (düğüm, referans gidiş yönü [tam derece] | None, referanstan sapma
    # başladığından beri katedilen yol [dm, üstten sınırlı]). Aynı düğüme farklı
    # yönlerden gelmek farklı maliyet taşıyabildiğinden arama durum uzayında yapılır.
    win_dm = int(round(_U_TURN_WINDOW_M * 10.0))
    start_state: tuple[int, Optional[int], int] = (int(start), None, 0)
    dist: dict[tuple[int, Optional[int], int], float] = {start_state: 0.0}
    prev: dict[tuple[int, Optional[int], int], tuple[int, Optional[int], int]] = {}
    tie = 0
    pq: list[tuple[float, int, int, Optional[int], int]] = [(0.0, tie, int(start), None, 0)]
    seen: set[tuple[int, Optional[int], int]] = set()
    goal_state: Optional[tuple[int, Optional[int], int]] = None

    while pq:
        d, _, u, ref_b, acc = heapq.heappop(pq)
        state = (u, ref_b, acc)
        if state in seen:
            continue
        seen.add(state)
        if u == int(goal):
            goal_state = state
            break
        for e in g.adj.get(u, []):
            if blocked_edges is not None and (int(e.u), int(e.v)) in blocked_edges:
                continue
            mult = 1.0 if edge_cost_multiplier is None else float(edge_cost_multiplier(e))
            if mult < 1e-9:
                mult = 1e-9
            penalty = 0.0
            new_ref = ref_b
            new_acc = acc
            n0 = by_id.get(int(e.u))
            n1 = by_id.get(int(e.v))
            if n0 is not None and n1 is not None:
                b = _bearing_deg(n0.lat, n0.lon, n1.lat, n1.lon)
                b_key = int(round(b)) % 360
                seg_dm = int(round(haversine_m(n0.lat, n0.lon, n1.lat, n1.lon) * 10.0))
                if ref_b is None:
                    new_ref, new_acc = b_key, 0
                else:
                    dev = abs(_angle_diff_deg(b, float(ref_b)))
                    if dev >= U_TURN_ANGLE_DEG:
                        # Sapma penceresi içinde tersine dönüş tamamlandı => U dönüşü
                        if u_turn_penalty > 0.0 and acc < win_dm:
                            penalty = float(u_turn_penalty)
                        new_ref, new_acc = b_key, 0
                    elif dev <= _U_TURN_STRAIGHT_DEV_DEG:
                        new_ref, new_acc = ref_b, 0  # hâlâ düz: sapma sayacı sıfır
                    elif acc + seg_dm >= win_dm:
                        new_ref, new_acc = b_key, 0  # geniş dönüş: referansı tazele
                    else:
                        new_acc = acc + seg_dm
            nd = d + float(e.cost) * mult + penalty
            ns = (int(e.v), new_ref, new_acc)
            if nd < dist.get(ns, float("inf")):
                dist[ns] = nd
                prev[ns] = state
                tie += 1
                heapq.heappush(pq, (nd, tie, int(e.v), new_ref, new_acc))

    if goal_state is None:
        return []

    path: list[int] = [int(goal_state[0])]
    cur = goal_state
    while cur != start_state:
        cur = prev[cur]
        path.append(int(cur[0]))
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

