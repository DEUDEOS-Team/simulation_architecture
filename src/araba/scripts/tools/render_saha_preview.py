#!/usr/bin/env python3
"""
render_saha_preview.py — gerçek saha yol ağının (map_islenmis.geojson) önizlemesi.

Arka plan: sahanın UYDU görüntüsü (Esri World Imagery karoları; internet yoksa
beyaz zemine düşer; indirilen karolar tekrar kullanım için önbelleğe alınır).
Saha kuzeye göre ~45° dönük kurulu olduğu için yol ağı görüntüde "yamuk" durur —
bu gerçeğin kendisidir (kuzey = yukarı).

Çizer:
  - TEK YÖN şeritler: mavi, üzerinde akış yönü okları
  - ÇİFT YÖN kalan çizgiler: kırmızı (ikizi bulunamayan köşegenler, çıkmaz
    onarımıyla geri çevrilenler vb. — veri kontrolünde göze batması için)
  - Deneme görevinin planlanan rotası: turuncu, gidiş yönü oklu
  - Görev noktaları: numaralı koyu kırmızı daireler

Ayrıca deneme görevini missions/saha_deneme_gorev.geojson olarak yazar ki
mission_planning_node aynı dosyayla test edilebilsin.

Çıktı: missions/saha_onizleme.png

Kullanım:
  python3 render_saha_preview.py [--gorev <baska_gorev.geojson>] [--duz-zemin]
"""

import argparse
import json
import math
import os
import sys
import urllib.request

from PIL import Image, ImageDraw, ImageFont

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.dirname(_TOOLS)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from geojson_mission_reader import GeoJsonMissionReader
from route_graph import build_graph_from_centerlines_geojson
from route_planner import route_mission_plan_via_graph

MISSIONS = os.path.join(os.path.dirname(_SCRIPTS), "missions")
MAP_FILE = os.path.join(MISSIONS, "map_islenmis.geojson")
OUT_PNG = os.path.join(MISSIONS, "saha_onizleme.png")
GOREV_FILE = os.path.join(MISSIONS, "saha_deneme_gorev.geojson")

# Deneme görevi — noktalar uydu görüntüsündeki gerçek saha öğelerine oturtuldu.
# Duraklar işaretli ceplerdir: durak_A = sol alt kenardaki cep (ring'in dış
# tarafında), durak_B = alt kenarın sağındaki cep.
# park = sağ üstteki gerçek park alanının girişe yakın cebi (ağ oraya fid=14
# stub'ıyla girer; alan içi çizilmediğinden rota girişte biter).
DEMO_MISSION = [
    ("start",   "baslangic", 29.5084428, 40.7898918),
    ("durak_A", "durak",     29.5087035, 40.7896655),
    ("durak_B", "durak",     29.5090536, 40.7896190),
    ("park",    "park_yeri", 29.5092463, 40.7902655),
]


TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}")
TILE_CACHE = os.path.join(os.path.expanduser("~"), ".cache", "saha_tiles")
ZOOM = 19  # ~0.23 m/px @ 40.79° — z=20 bu bölgede "data not available" karosu döndürüyor


def tile_xy(lon: float, lat: float, z: int) -> tuple[float, float]:
    """Web-Mercator karo koordinatı (kesirli)."""
    tx = (lon + 180.0) / 360.0 * (1 << z)
    ty = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * (1 << z)
    return tx, ty


def fetch_satellite(lon_min, lat_min, lon_max, lat_max):
    """Alanı kaplayan uydu mozaiği + (lon,lat)->piksel fonksiyonu. Hata halinde None."""
    tx0, ty1 = tile_xy(lon_min, lat_min, ZOOM)
    tx1, ty0 = tile_xy(lon_max, lat_max, ZOOM)
    x0, x1 = int(tx0), int(tx1)
    y0, y1 = int(ty0), int(ty1)
    os.makedirs(TILE_CACHE, exist_ok=True)
    mosaic = Image.new("RGB", ((x1 - x0 + 1) * 256, (y1 - y0 + 1) * 256), (200, 200, 200))
    try:
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                cache = os.path.join(TILE_CACHE, f"{ZOOM}_{x}_{y}.jpg")
                if not os.path.exists(cache):
                    req = urllib.request.Request(
                        TILE_URL.format(z=ZOOM, x=x, y=y),
                        headers={"User-Agent": "Mozilla/5.0"})
                    with urllib.request.urlopen(req, timeout=15) as r:
                        data = r.read()
                    with open(cache, "wb") as f:
                        f.write(data)
                mosaic.paste(Image.open(cache).convert("RGB"),
                             ((x - x0) * 256, (y - y0) * 256))
    except Exception as e:
        print(f"UYARI: uydu karosu indirilemedi ({e}) — beyaz zemin kullanilacak")
        return None, None

    def to_px(lon, lat):
        tx, ty = tile_xy(lon, lat, ZOOM)
        return ((tx - x0) * 256.0, (ty - y0) * 256.0)

    return mosaic, to_px


def draw_arrow(dr, x0, y0, x1, y1, color, al=14):
    ang = math.atan2(y1 - y0, x1 - x0)
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    aw = math.radians(25)
    tip = (mx + al / 2 * math.cos(ang), my + al / 2 * math.sin(ang))
    dr.polygon(
        [tip,
         (tip[0] - al * math.cos(ang - aw), tip[1] - al * math.sin(ang - aw)),
         (tip[0] - al * math.cos(ang + aw), tip[1] - al * math.sin(ang + aw))],
        fill=color)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gorev", default="")
    ap.add_argument("--duz-zemin", action="store_true",
                    help="uydu arka planını atla (beyaz zemin)")
    args = ap.parse_args()

    with open(MAP_FILE, encoding="utf-8") as f:
        d = json.load(f)
    feats = d["features"]

    if args.gorev:
        gorev_path = args.gorev
    else:
        gorev_path = GOREV_FILE
        mission = {"type": "FeatureCollection", "features": [
            {"type": "Feature",
             "properties": {"id": i, "name": n, "task": t},
             "geometry": {"type": "Point", "coordinates": [lo, la]}}
            for i, (n, t, lo, la) in enumerate(DEMO_MISSION)]}
        with open(gorev_path, "w", encoding="utf-8") as f:
            json.dump(mission, f, ensure_ascii=False, indent=2)

    g = build_graph_from_centerlines_geojson(d, coord_round_decimals=7)
    plan = GeoJsonMissionReader().read_file(gorev_path)
    routed = route_mission_plan_via_graph(plan, g, tunnel_mandatory=True)

    # ── projeksiyon + arka plan ──
    lats = [c[1] for ft in feats for c in ft["geometry"]["coordinates"]]
    lons = [c[0] for ft in feats for c in ft["geometry"]["coordinates"]]
    lat0 = (min(lats) + max(lats)) / 2
    mdlat = 110574.0
    mdlon = 111320.0 * math.cos(math.radians(lat0))
    pad_lat, pad_lon = 15.0 / mdlat, 15.0 / mdlon  # 15 m kenar payı

    img = None
    sat = False
    if not args.duz_zemin:
        mosaic, to_px_m = fetch_satellite(min(lons) - pad_lon, min(lats) - pad_lat,
                                          max(lons) + pad_lon, max(lats) + pad_lat)
        if mosaic is not None:
            cx0, cy0 = to_px_m(min(lons) - pad_lon, max(lats) + pad_lat)
            cx1, cy1 = to_px_m(max(lons) + pad_lon, min(lats) - pad_lat)
            crop = mosaic.crop((int(cx0), int(cy0), int(cx1), int(cy1)))
            scale = max(1.0, 1400.0 / crop.width)
            img = crop.resize((int(crop.width * scale), int(crop.height * scale)),
                              Image.LANCZOS)
            sat = True

            def px(lon, lat, _x0=int(cx0), _y0=int(cy0), _s=scale):
                x, y = to_px_m(lon, lat)
                return ((x - _x0) * _s, (y - _y0) * _s)

    if img is None:
        # çevrimdışı: beyaz zemin, yerel metre projeksiyonu
        lon0 = (min(lons) + max(lons)) / 2

        def xy(lon, lat):
            return ((lon - lon0) * mdlon, (lat - lat0) * mdlat)

        allxy = [xy(c[0], c[1]) for ft in feats for c in ft["geometry"]["coordinates"]]
        xs = [p[0] for p in allxy]
        ys = [p[1] for p in allxy]
        SC = 16
        W = int((max(xs) - min(xs) + 10) * SC)
        H = int((max(ys) - min(ys) + 10) * SC)

        def px(lon, lat):
            x, y = xy(lon, lat)
            return ((x - min(xs) + 5) * SC, H - (y - min(ys) + 5) * SC)

        img = Image.new("RGB", (W, H), (255, 255, 255))

    dr = ImageDraw.Draw(img)

    # ── yollar ──
    tunnel_paths = []
    for ft in feats:
        props = ft.get("properties") or {}
        oneway = bool(props.get("oneway", False))
        P = [px(c[0], c[1]) for c in ft["geometry"]["coordinates"]]
        if props.get("tunnel"):
            tunnel_paths.append(P)
        if sat:  # uydu üstünde okunabilirlik için beyaz hale
            dr.line(P, fill=(255, 255, 255), width=7)
        dr.line(P, fill=(60, 120, 230) if oneway else (220, 40, 40), width=4)
        if not oneway:
            continue
        acc = 0.0
        for (x0, y0), (x1, y1) in zip(P, P[1:]):
            acc += math.hypot(x1 - x0, y1 - y0)
            if acc < 90:
                continue
            acc = 0.0
            draw_arrow(dr, x0, y0, x1, y1, (20, 60, 160))

    # ── rota ──
    rp = [px(p.lon, p.lat) for p in routed.points]
    if len(rp) > 1:
        dr.line(rp, fill=(255, 140, 0), width=7)
        for (x0, y0), (x1, y1) in list(zip(rp, rp[1:]))[::2]:
            if math.hypot(x1 - x0, y1 - y0) < 12:
                continue
            draw_arrow(dr, x0, y0, x1, y1, (200, 90, 0), al=17)

    # ── tünel (kesikli mor; rota dahil her şeyin ÜSTÜNE, görünür kalsın) ──
    for P in tunnel_paths:
        for (x0, y0), (x1, y1) in zip(P, P[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(L / 14))
            for k in range(n):
                if k % 2:
                    continue
                t0, t1 = k / n, min((k + 1) / n, 1.0)
                dr.line([(x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0),
                         (x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1)],
                        fill=(150, 40, 200), width=6)

    # ── görev noktaları ──
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28)
    except OSError:
        font = ImageFont.load_default()
    for i, p in enumerate(routed.points):
        if p.task == "checkpoint":
            continue
        u, v = px(p.lon, p.lat)
        r = 15
        dr.ellipse([u - r, v - r, u + r, v + r],
                   fill=(160, 20, 20), outline=(255, 255, 255), width=3)
        dr.text((u + r + 6, v - r - 4), p.name, fill=(150, 15, 15), font=font,
                anchor="lb", stroke_width=2, stroke_fill=(255, 255, 255))

    # ── lejant (alt şerit) ──
    legend = [((60, 120, 230), "tek yon serit (ok = akis)"),
              ((220, 40, 40), "cift yon kalan cizgi"),
              ((255, 140, 0), "planlanan rota"),
              ((150, 40, 200), "tunel")]
    strip = 66
    canvas = Image.new("RGB", (img.width, img.height + strip), (255, 255, 255))
    canvas.paste(img, (0, 0))
    dr = ImageDraw.Draw(canvas)
    lx, yy = 24, img.height + strip // 2
    for col, lab in legend:
        dr.line([(lx, yy), (lx + 52, yy)], fill=col, width=8)
        dr.text((lx + 64, yy), lab, fill=(0, 0, 0), font=font, anchor="lm")
        lx += 64 + int(dr.textlength(lab, font=font)) + 44

    canvas.save(OUT_PNG)
    print(f"yazildi: {OUT_PNG}")
    print(f"gorev: {gorev_path}  ({len(plan.points)} hedef -> {len(routed.points)} waypoint)")


if __name__ == "__main__":
    main()
