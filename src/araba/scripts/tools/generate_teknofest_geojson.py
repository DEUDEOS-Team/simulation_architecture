#!/usr/bin/env python3
"""
generate_teknofest_geojson.py — sim dünyası için görev + centerline GeoJSON üretici.

Çift şerit şeması kullanır. Harita ve koordinatlar map.jpeg / benim_dunyam.sdf ile
aynı; yol ağının işaretlenme biçimi gerçek saha çizimiyle (map.geojson) aynı şemadadır:
  - Her yolun gidiş ve gelişi AYRI birer LineString'dir (`oneway: true`).
    Şerit eksenleri yol ekseninin ±OFF yanındadır (map.jpeg'deki kesikli
    ayırıcının iki yanı) ve yönler SAĞDAN trafiğe göre atanır:
    kuzey yönü doğu şeritte, doğu yönü güney şeritte.
  - Döner kavşak (C3) düz kavşak düğümü değil, saat yönünün TERSİNE dönen
    tek yönlü bir dairedir (`tur: kavsak`); giriş/çıkış şeritleri daireye
    tam kesişim noktalarında bağlanır. C sütunu ve 3. satır şeritleri
    dairede KESİLİR (içinden geçemez).
  - Kavşaklarda şeritler ortak köşe noktalarıyla kesişir (route_graph
    düğüm birleştirmesi); U dönüşü grafikte mümkün değildir — gerçekçi.
  - Tünel (B2-C2) her iki yön şeridinde ayrı `tunnel: true` feature'dır.
  - Park bölgesi yolları (erişim + koridor) tek çizgi çift yön kalır
    (arkadaşın haritasındaki ikizsiz yollar gibi).

Kaynak geometri: benim_dunyam.sdf'teki 'resimli_kutu' (map.jpeg, 97×120 m) üzerinden
kalibre edilen yol ızgarası (dünya nesne pozlarıyla doğrulandı).

Çıktılar (missions/ dizinine):
  teknofest_centerlines.geojson — şerit LineString'leri (route_graph girdisi)
  teknofest_gorev.geojson — Görev noktaları (aracın gideceği şeridin üstünde)

Koordinatlar dünya SDF'indeki <spherical_coordinates> datumundan çevrilir
(final_odom_node ile aynı WGS84 formülleri). Datum değişirse:

  python3 generate_teknofest_geojson.py [--datum-lat 40.7899] [--datum-lon 29.5089]
"""

import argparse
import json
import math
import os

# WGS84 (final_odom_node ile aynı)
_A = 6378137.0
_E2 = 6.69437999014e-3

# ── Kalibre edilmiş yol ızgarası (dünya metre) ──
# Kuzey-güney yol eksenleri (sabit x):
XA, XB, XC, XD = 43.3, 21.3, -11.9, -34.7
XP = 11.1            # park erişim yolu (sadece güney bölge)
# Doğu-batı yol eksenleri (sabit y):
Y1, Y2, Y3, Y4 = 22.3, 5.3, -18.3, -43.3
YE = -63.4           # park koridoru; cepler koridorun güneyinde (y −67..−77)
PS = 25.2            # hedef cebin önündeki koridor noktası

OFF = 1.7            # şerit ekseni ofseti (kesikli ayırıcının iki yanı, m)
R_ROND = 6.8         # döner kavşak dolaşım şeridi yarıçapı (map.jpeg'den, m)

COLS = {"A": XA, "B": XB, "C": XC, "D": XD}
ROWS = {"1": Y1, "2": Y2, "3": Y3, "4": Y4}

# ── Görev noktaları (dünya metre) — aracın gideceği ŞERİDİN üstünde ──
MISSION = [
    {"name": "start",      "task": "baslangic",  "xy": (XA + OFF, 16.0),
     "note": "spawn noktasi; A yolu kuzey serit"},
    {"name": "durak_1",    "task": "durak",      "xy": (0.69, Y1 + OFF),
     "note": "durak levhasi (0.69, 26.15) onu; 1. yol bati serit"},
    {"name": "durak_2",    "task": "durak",      "xy": (XD - OFF, 12.55),
     "note": "durak_0 levhasi (-39.01, 12.55) onu; D yolu guney serit"},
    {"name": "park_giris", "task": "park_giris", "xy": (XP, -48.0),
     "note": "park erisim yoluna donusten hemen sonra (tek cizgi cift yon)"},
    {"name": "park",       "task": "park_yeri",  "xy": (PS, -72.0),
     "note": "park_yeri_1 levhasi onundeki cep; koridordan guneye manevra"},
]


def make_latlon_converter(datum_lat: float, datum_lon: float):
    lat0 = math.radians(datum_lat)
    s2 = math.sin(lat0) ** 2
    n_rad = _A / math.sqrt(1.0 - _E2 * s2)
    m_rad = _A * (1.0 - _E2) / (1.0 - _E2 * s2) ** 1.5
    m_per_deg_lat = math.radians(1.0) * m_rad
    m_per_deg_lon = math.radians(1.0) * n_rad * math.cos(lat0)

    def to_lonlat(x: float, y: float) -> list[float]:
        # route_graph düğümleri 7 haneye yuvarlayarak birleştirir;
        # aynı düğüm her kenarda bit-bit aynı çıksın diye burada da yuvarlanır.
        return [round(datum_lon + x / m_per_deg_lon, 7),
                round(datum_lat + y / m_per_deg_lat, 7)]

    return to_lonlat


def build_lane_features() -> list[dict]:
    """Çift şerit yol ağını (dünya metre koordinatlı) feature listesi olarak kurar."""
    h = math.sqrt(R_ROND * R_ROND - OFF * OFF)   # daire-şerit kesişim uzaklığı
    x_min, x_max = XD - OFF, XA + OFF
    y_min, y_max = Y4 - OFF, Y1 + OFF
    eps = 1e-9

    # Dikey şeritlerin kestiği y değerleri / yatay şeritlerin kestiği x değerleri.
    # Görev noktalarının hizasına da düğüm eklenir; yoksa route_planner noktayı
    # en yakın kavşağa snap'ler ve araç durağın 5-11 m uzağında durur.
    mission_ys = {"A": [16.0], "D": [12.55]}          # start, durak_2
    mission_xs = {"1": [0.69]}                        # durak_1
    cross_ys_base = sorted({y + s * OFF for y in ROWS.values() for s in (-1, 1)})

    def cross_ys(col_ad: str) -> list[float]:
        return sorted(set(cross_ys_base) | set(mission_ys.get(col_ad, [])))

    def cross_xs(row_ad: str, row_y: float) -> list[float]:
        xs = {x + s * OFF for x in COLS.values() for s in (-1, 1)}
        if abs(row_y - Y4) < eps:
            xs |= {XA, XP}   # park tek-yollarının T bağlantıları
        xs |= set(mission_xs.get(row_ad, []))
        return sorted(xs)

    def clamp_pts(vals, lo, hi, endpoints=True):
        out = [v for v in vals if lo - eps <= v <= hi + eps]
        if endpoints:
            if not out or out[0] > lo + eps:
                out.insert(0, lo)
            if out[-1] < hi - eps:
                out.append(hi)
        return out

    feats = []

    def add(name, tur, coords, oneway, **extra):
        feats.append({
            "type": "Feature",
            "properties": {"name": name, "tur": tur, "oneway": oneway, **extra},
            "geometry": {"type": "LineString", "coordinates": [list(c) for c in coords]},
        })

    def v_lane(name, col_ad, x, ylo, yhi, northbound, **extra):
        ys = clamp_pts(cross_ys(col_ad), ylo, yhi)
        if not northbound:
            ys = ys[::-1]
        add(name, "yol", [(x, y) for y in ys], True, **extra)

    def h_lane(name, row_ad, lane_y, row_y, xlo, xhi, eastbound, **extra):
        xs = clamp_pts(cross_xs(row_ad, row_y), xlo, xhi)
        if not eastbound:
            xs = xs[::-1]
        add(name, "yol", [(x, lane_y) for x in xs], True, **extra)

    # ── Kuzey-güney sütunlar (C hariç tam boy) ──
    for ad, cx in COLS.items():
        if ad == "C":
            continue
        v_lane(f"{ad}-kuzey", ad, cx + OFF, y_min, y_max, northbound=True)
        v_lane(f"{ad}-guney", ad, cx - OFF, y_min, y_max, northbound=False)
    # C sütunu: döner kavşakta kesilir
    v_lane("C-kuzey-guney_parca", "C", XC + OFF, y_min, Y3 - h, northbound=True)
    v_lane("C-kuzey-kuzey_parca", "C", XC + OFF, Y3 + h, y_max, northbound=True)
    v_lane("C-guney-kuzey_parca", "C", XC - OFF, Y3 + h, y_max, northbound=False)
    v_lane("C-guney-guney_parca", "C", XC - OFF, y_min, Y3 - h, northbound=False)

    # ── Doğu-batı satırlar ──
    # 1 ve 4 tam boy
    for ad in ("1", "4"):
        ry = ROWS[ad]
        h_lane(f"{ad}-dogu", ad, ry - OFF, ry, x_min, x_max, eastbound=True)
        h_lane(f"{ad}-bati", ad, ry + OFF, ry, x_min, x_max, eastbound=False)
    # 2. satır: tünel (B2-C2 arası) ayrı feature
    txw, txe = XC + OFF, XB - OFF
    h_lane("2-dogu-bati_parca", "2", Y2 - OFF, Y2, x_min, txw, eastbound=True)
    h_lane("2-dogu-tunel", "2", Y2 - OFF, Y2, txw, txe, eastbound=True, tunnel=True)
    h_lane("2-dogu-dogu_parca", "2", Y2 - OFF, Y2, txe, x_max, eastbound=True)
    h_lane("2-bati-dogu_parca", "2", Y2 + OFF, Y2, txe, x_max, eastbound=False)
    h_lane("2-bati-tunel", "2", Y2 + OFF, Y2, txw, txe, eastbound=False, tunnel=True)
    h_lane("2-bati-bati_parca", "2", Y2 + OFF, Y2, x_min, txw, eastbound=False)
    # 3. satır: döner kavşakta kesilir
    h_lane("3-dogu-bati_parca", "3", Y3 - OFF, Y3, x_min, XC - h, eastbound=True)
    h_lane("3-dogu-dogu_parca", "3", Y3 - OFF, Y3, XC + h, x_max, eastbound=True)
    h_lane("3-bati-dogu_parca", "3", Y3 + OFF, Y3, XC + h, x_max, eastbound=False)
    h_lane("3-bati-bati_parca", "3", Y3 + OFF, Y3, x_min, XC - h, eastbound=False)

    # ── Döner kavşak: saat yönü tersine (sağdan trafik) kapalı tek yön daire ──
    # Bağlantı noktaları şerit uçlarıyla BİT-BİT aynı olsun diye açı->koordinat
    # tablosunda tam ifadeleriyle tutulur.
    conn = {}
    for dx, dy in ((OFF, -h), (OFF, h), (-OFF, h), (-OFF, -h),
                   (h, -OFF), (h, OFF), (-h, OFF), (-h, -OFF)):
        conn[math.atan2(dy, dx)] = (XC + dx, Y3 + dy)
    angles = sorted(conn)
    filler = [math.radians(a) for a in range(-180, 180, 15)]
    for fa in filler:
        if all(abs(fa - ca) > math.radians(4.0) for ca in angles):
            angles.append(fa)
    angles.sort()
    ring = [conn.get(a, (XC + R_ROND * math.cos(a), Y3 + R_ROND * math.sin(a)))
            for a in angles]
    ring.append(ring[0])   # kapalı halka
    add("doner_kavsak", "kavsak", ring, True)

    # ── Park bölgesi: tek çizgi çift yön (ikizsiz) ──
    add("A-park_erisim", "yol",
        [(XA, Y4 + OFF), (XA, Y4 - OFF), (XA, YE)], False)
    add("P-park_erisim", "yol",
        [(XP, Y4 + OFF), (XP, Y4 - OFF), (XP, -48.0), (XP, YE)], False)
    add("park_koridor", "yol",
        [(XP, YE), (PS, YE), (XA, YE)], False)

    return feats


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--datum-lat", type=float, default=40.7899)
    ap.add_argument("--datum-lon", type=float, default=29.5089)
    default_out = os.path.join(os.path.dirname(__file__), "..", "..", "missions")
    ap.add_argument("--out-dir", default=os.path.abspath(default_out))
    args = ap.parse_args()

    to_lonlat = make_latlon_converter(args.datum_lat, args.datum_lon)
    os.makedirs(args.out_dir, exist_ok=True)

    # ── centerlines ──
    features = build_lane_features()
    for ft in features:
        ft["geometry"]["coordinates"] = [
            to_lonlat(x, y) for x, y in ft["geometry"]["coordinates"]
        ]
    centerlines = {
        "type": "FeatureCollection",
        "properties": {
            "generator": "generate_teknofest_geojson.py",
            "datum": [args.datum_lat, args.datum_lon],
            "coordinate_note": "benim_dunyam.sdf spherical_coordinates datumundan uretildi",
            "schema_note": "cift serit + oneway + doner kavsak dairesi (map.geojson semasi)",
        },
        "features": features,
    }
    cl_path = os.path.join(args.out_dir, "teknofest_centerlines.geojson")
    with open(cl_path, "w", encoding="utf-8") as f:
        json.dump(centerlines, f, ensure_ascii=False, indent=2)

    # ── görev ──
    mission_features = []
    for i, mp in enumerate(MISSION):
        mission_features.append({
            "type": "Feature",
            "properties": {"id": i, "name": mp["name"], "task": mp["task"],
                           "note": mp["note"]},
            "geometry": {"type": "Point", "coordinates": to_lonlat(*mp["xy"])},
        })
    mission = {
        "type": "FeatureCollection",
        "properties": {
            "generator": "generate_teknofest_geojson.py",
            "datum": [args.datum_lat, args.datum_lon],
        },
        "features": mission_features,
    }
    gj_path = os.path.join(args.out_dir, "teknofest_gorev.geojson")
    with open(gj_path, "w", encoding="utf-8") as f:
        json.dump(mission, f, ensure_ascii=False, indent=2)

    n_seg = sum(len(ft["geometry"]["coordinates"]) - 1 for ft in features)
    print(f"yazildi: {cl_path}  ({len(features)} feature, {n_seg} kenar)")
    print(f"yazildi: {gj_path}  ({len(mission_features)} gorev noktasi)")


if __name__ == "__main__":
    main()
