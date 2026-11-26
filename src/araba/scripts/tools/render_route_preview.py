#!/usr/bin/env python3
"""
render_route_preview.py — missions/ GeoJSON'larından rota önizleme görseli üretir.

map.jpeg üzerine çizer:
  - tüm şerit kenarları (TEK YÖN mavi + akış yönü oku, ÇİFT YÖN kırmızı —
    saha_onizleme ile aynı dil; tünel kenarı mor, rota üstünden geçse
    bile kesikli mor olarak rotanın ÜZERİNE çizilir)
  - route_planner'ın rotası (turuncu, kalın; gidiş yönü oklarla, geçilen
    ara kavşaklar küçük numaralı noktalarla)
  - görev noktaları (numaralı kırmızı daireler + isim etiketi)

Çıktı: missions/rota_onizleme.png

Piksel↔dünya kalibrasyonu dünya nesne pozlarıyla doğrulandı
(map.jpeg 1600×1210, dünyada 97×120 m kutu, 90° dönük):
  u = (35.782 − y) · 13.333   [px]
  v = (48.5  − x) · 12.474    [px]

Datum değişirse önce generate_teknofest_geojson.py yeniden çalıştırılır,
sonra bu script (datum'u geojson properties.datum'dan okur).
"""

import argparse
import json
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.dirname(_TOOLS)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from geojson_mission_reader import GeoJsonMissionReader
from route_graph import build_graph_from_centerlines_geojson, edge_is_tunnel
from route_planner import route_mission_plan_via_graph

# WGS84 (generate_teknofest_geojson.py ile aynı)
_A = 6378137.0
_E2 = 6.69437999014e-3

PKG_DIR = os.path.dirname(_SCRIPTS)                      # src/araba
MAP_JPEG = os.path.join(PKG_DIR, "worlds", "map.jpeg")
MISSIONS = os.path.join(PKG_DIR, "missions")
OUT_PNG = os.path.join(MISSIONS, "rota_onizleme.png")


def make_world_converter(datum_lat: float, datum_lon: float):
    lat0 = math.radians(datum_lat)
    s2 = math.sin(lat0) ** 2
    n_rad = _A / math.sqrt(1.0 - _E2 * s2)
    m_rad = _A * (1.0 - _E2) / (1.0 - _E2 * s2) ** 1.5
    m_per_deg_lat = math.radians(1.0) * m_rad
    m_per_deg_lon = math.radians(1.0) * n_rad * math.cos(lat0)

    def to_world(lat: float, lon: float) -> tuple[float, float]:
        return ((lon - datum_lon) * m_per_deg_lon,
                (lat - datum_lat) * m_per_deg_lat)

    return to_world


def world_to_px(x: float, y: float) -> tuple[float, float]:
    return ((35.782 - y) * 13.333, (48.5 - x) * 12.474)


def main() -> None:
    # --start N: rotayı N. node'dan başlıyormuş gibi çiz (1 = tam rota).
    # N>1 iken ayrı dosyaya yazar; asıl rota_onizleme.png bozulmaz.
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", type=int, default=1,
                    help="rotanın kaçıncı node'undan başlansın (1 = tam rota)")
    args = ap.parse_args()
    start_idx = max(0, int(args.start) - 1)

    cl_path = os.path.join(MISSIONS, "teknofest_centerlines.geojson")
    gj_path = os.path.join(MISSIONS, "teknofest_gorev.geojson")

    with open(cl_path, encoding="utf-8") as f:
        cl = json.load(f)
    datum_lat, datum_lon = cl["properties"]["datum"]
    to_world = make_world_converter(datum_lat, datum_lon)

    graph = build_graph_from_centerlines_geojson(cl, coord_round_decimals=7)
    plan = GeoJsonMissionReader().read_file(gj_path)
    routed = route_mission_plan_via_graph(plan, graph, tunnel_mandatory=True)

    # N. node'dan başla: öncesindeki waypoint'ler atılır, numaralar 1'den başlar
    route_pts = list(routed.points)[start_idx:]
    out_png = OUT_PNG if start_idx == 0 else os.path.join(
        MISSIONS, f"rota_onizleme_node{start_idx + 1}.png")

    img = Image.open(MAP_JPEG).convert("RGB")
    draw = ImageDraw.Draw(img)

    def px(lat: float, lon: float) -> tuple[float, float]:
        return world_to_px(*to_world(lat, lon))

    # ── şerit kenarları (tünel hariç; tünel en üste çizilecek) ──
    directed: set[tuple[int, int]] = set()
    for edges in graph.adj.values():
        for e in edges:
            directed.add((e.u, e.v))
    tunnel_segs: list[tuple[tuple[float, float], tuple[float, float]]] = []
    seen: set[tuple[int, int]] = set()
    for edges in graph.adj.values():
        for e in edges:
            key = (min(e.u, e.v), max(e.u, e.v))
            if key in seen:
                continue
            seen.add(key)
            a, b = graph.nodes[e.u], graph.nodes[e.v]
            seg = (px(a.lat, a.lon), px(b.lat, b.lon))
            if edge_is_tunnel(e):
                tunnel_segs.append(seg)
                continue
            if (e.v, e.u) in directed:      # her iki yön de var → çift yön
                draw.line(list(seg), fill=(220, 40, 40), width=4)
                continue
            (x0, y0), (x1, y1) = seg
            draw.line(list(seg), fill=(40, 110, 230), width=4)
            if math.hypot(x1 - x0, y1 - y0) > 28:   # akış yönü oku (kısa parçalarda atla)
                ang = math.atan2(y1 - y0, x1 - x0)
                mx, my = (x0 + x1) / 2, (y0 + y1) / 2
                al, aw = 13, math.radians(26)
                tip = (mx + al / 2 * math.cos(ang), my + al / 2 * math.sin(ang))
                draw.polygon(
                    [tip,
                     (tip[0] - al * math.cos(ang - aw), tip[1] - al * math.sin(ang - aw)),
                     (tip[0] - al * math.cos(ang + aw), tip[1] - al * math.sin(ang + aw))],
                    fill=(15, 55, 160))

    # ── planlanan rota: turuncu hat + yön okları + ara kavşak numaraları ──
    if route_pts:
        pts = [px(p.lat, p.lon) for p in route_pts]
        draw.line(pts, fill=(255, 140, 0), width=8)
        # her segmentin ortasına gidiş yönü oku
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            mx, my = (x0 + x1) / 2, (y0 + y1) / 2
            ang = math.atan2(y1 - y0, x1 - x0)
            al, aw = 22, math.radians(26)
            tip = (mx + al / 2 * math.cos(ang), my + al / 2 * math.sin(ang))
            left = (tip[0] - al * math.cos(ang - aw), tip[1] - al * math.sin(ang - aw))
            right = (tip[0] - al * math.cos(ang + aw), tip[1] - al * math.sin(ang + aw))
            draw.polygon([tip, left, right], fill=(255, 140, 0),
                         outline=(120, 60, 0))
        # geçilen her waypoint'e küçük sıra numarası (görev noktaları hariç,
        # onlar aşağıda büyük kırmızı daireyle çizilir)
        try:
            small = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
        except OSError:
            small = ImageFont.load_default()
        for i, p in enumerate(route_pts):
            if p.task != "checkpoint":
                continue
            u, v = pts[i]
            r = 11
            draw.ellipse([u - r, v - r, u + r, v + r],
                         fill=(255, 255, 255), outline=(255, 140, 0), width=3)
            draw.text((u, v), str(i + 1), fill=(120, 60, 0),
                      font=small, anchor="mm")

    # ── tünel kenarı: her şeyin üstüne kesikli mor (rota kapatmasın) ──
    for (x0, y0), (x1, y1) in tunnel_segs:
        seg_len = math.hypot(x1 - x0, y1 - y0)
        n_dash = max(int(seg_len / 26), 1)
        for k in range(n_dash):
            t0, t1 = k / n_dash, (k + 0.55) / n_dash
            draw.line([(x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0),
                       (x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1)],
                      fill=(170, 60, 220), width=6)

    # ── görev noktaları ──
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
    except OSError:
        font = ImageFont.load_default()
    # Numara = rotadaki sürüş sırası (ara kavşak numaralarıyla aynı seri)
    for i, mp in enumerate(route_pts):
        if mp.task == "checkpoint":
            continue
        u, v = px(mp.lat, mp.lon)
        r = 16
        draw.ellipse([u - r, v - r, u + r, v + r],
                     fill=(220, 30, 30), outline=(255, 255, 255), width=3)
        draw.text((u, v), str(i + 1), fill=(255, 255, 255),
                  font=font, anchor="mm")
        label = f"{mp.name} ({mp.task})"
        # Etiket dairenin çaprazında (rota hattının üstünde) dursun ki aynı
        # hizadaki ara kavşak numaralarını kapatmasın; sağ kenara yakınsa sola.
        if u > img.width - 380:
            draw.text((u - r - 6, v - r - 4), label, fill=(220, 30, 30), font=font,
                      anchor="rb", stroke_width=2, stroke_fill=(255, 255, 255))
        else:
            draw.text((u + r + 6, v - r - 4), label, fill=(220, 30, 30), font=font,
                      anchor="lb", stroke_width=2, stroke_fill=(255, 255, 255))

    # ── lejant (harita altına eklenen şeritte, yatay tek satır) ──
    legend = [((40, 110, 230), "tek yon serit (ok = akis)"),
              ((220, 40, 40), "cift yon yol"),
              ((170, 60, 220), "tunel"),
              ((255, 140, 0), "planlanan rota"),
              ((220, 30, 30), "gorev noktasi")]
    strip_h = 64
    canvas = Image.new("RGB", (img.width, img.height + strip_h), (255, 255, 255))
    canvas.paste(img, (0, 0))
    draw = ImageDraw.Draw(canvas)
    lx = 20
    yy = img.height + strip_h // 2
    for color, label in legend:
        draw.line([(lx, yy), (lx + 50, yy)], fill=color, width=8)
        draw.text((lx + 62, yy), label, fill=(0, 0, 0), font=font, anchor="lm")
        lx += 62 + int(draw.textlength(label, font=font)) + 40

    canvas.save(out_png)
    print(f"yazildi: {out_png}  (cizilen waypoint: {len(route_pts)}, "
          f"tam rota: {len(routed.points)}, baslangic node: {start_idx + 1})")


if __name__ == "__main__":
    main()
