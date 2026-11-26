#!/usr/bin/env python3
"""
generate_saha_ground.py — gerçek saha yol ağından Gazebo zemin dokusu üretir.

Amaç: sim'de aracı GERÇEK saha yerleşiminde sürebilmek. benim_dunyam.sdf'teki
'resimli_kutu' nasıl map.jpeg'i zemine seriyorsa, saha_dunyasi.sdf de bu aracın
ürettiği worlds/saha_zemin.png'yi serer. Doku, map_islenmis.geojson'daki şerit
eksenlerinden map.jpeg ile AYNI görsel dilde çizilir (koyu asfalt + beyaz
kesintisiz yol kenarı çizgileri + ikiz şeritler arasında beyaz kesikli ayırıcı),
böylece şerit algısı (lane_test_node) yeni zeminde de çalışır.

Doku yerleşimi resimli_kutu kalibrasyonuyla AYNI konvansiyondadır
(kutu yaw=0 iken görüntü-u ekseni dünya −y'ye, görüntü-v ekseni dünya −x'e
gider; rota önizleme kalibrasyonundan doğrulandı). Yani:
  u = (cy + sy/2 − y) · ppm ,  v = (cx + sx/2 − x) · ppm
Script sonunda SDF'e yazılacak kutu merkezi/boyutu basılır; saha_dunyasi.sdf
bu değerlerle senkron tutulmalıdır (değişirse ikisini birlikte güncelle).

Kullanım:
  python3 generate_saha_ground.py [--ppm 14] [--pad-m 12]
"""

import argparse
import json
import math
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw

MDLAT = 110574.0

# map.jpeg ile uyumlu renkler
ASPHALT = (58, 58, 60)
LINE = (245, 245, 245)
GREEN = (110, 205, 40)

TWIN_MAX_M = 4.5     # ikiz şerit arama mesafesi (prepare_saha_geojson ile aynı)
LANE_HALF_M = 1.7    # ikizli şeritte bant yarı genişliği (şerit ~3.4 m)
SINGLE_HALF_M = 2.8  # ikizi olmayan çizgide (tek çizilmiş çift yön / kavis)
EDGE_W_M = 0.30      # beyaz kenar çizgisi kalınlığı
DASH_W_M = 0.25      # kesikli ayırıcı kalınlığı
DASH_ON_M, DASH_OFF_M = 2.0, 1.6

# Döner kavşak göbeği: maskedeki en küçük iç delik (ada) yeşile boyanır.
# Ada alanı üst sınırı — blok delikleriyle karışmasın diye (m²).
ISLAND_MAX_M2 = 60.0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.abspath(os.path.join(here, "..", ".."))
    ap.add_argument("--in", dest="inp",
                    default=os.path.join(pkg, "missions", "map_islenmis.geojson"))
    ap.add_argument("--out", default=os.path.join(pkg, "worlds", "saha_zemin.png"))
    ap.add_argument("--datum-lat", type=float, default=40.7899)
    ap.add_argument("--datum-lon", type=float, default=29.5089)
    ap.add_argument("--ppm", type=float, default=14.0, help="piksel / metre")
    ap.add_argument("--pad-m", type=float, default=12.0, help="ağ çevresi asfalt payı")
    args = ap.parse_args()

    with open(args.inp, encoding="utf-8") as f:
        d = json.load(f)
    feats = d["features"]
    mdlon = 111320.0 * math.cos(math.radians(args.datum_lat))

    def xy(c):
        return ((c[0] - args.datum_lon) * mdlon, (c[1] - args.datum_lat) * MDLAT)

    lines = [[xy(c) for c in ft["geometry"]["coordinates"]] for ft in feats]
    xs = [p[0] for ln in lines for p in ln]
    ys = [p[1] for ln in lines for p in ln]
    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    sx = (max(xs) - min(xs)) + 2 * args.pad_m
    sy = (max(ys) - min(ys)) + 2 * args.pad_m
    ppm = args.ppm
    W = int(round(sy * ppm))   # u ekseni ↔ dünya y (kuzey solda)
    H = int(round(sx * ppm))   # v ekseni ↔ dünya x (doğu üstte)

    def px(p):
        return ((cy + sy / 2.0 - p[1]) * ppm, (cx + sx / 2.0 - p[0]) * ppm)

    # ── ikiz eşleştirme (geometrik; oneway işaretinden bağımsız) ──
    def seg_proj(p, a, b):
        bx, by = b[0] - a[0], b[1] - a[1]
        L2 = bx * bx + by * by
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * bx + (p[1] - a[1]) * by) / L2))
        q = (a[0] + t * bx, a[1] + t * by)
        return math.hypot(p[0] - q[0], p[1] - q[1]), q

    def nearest_on(ln, p):
        best = (1e9, None)
        for a, b in zip(ln, ln[1:]):
            dd, q = seg_proj(p, a, b)
            if dd < best[0]:
                best = (dd, q)
        return best

    twin = [None] * len(lines)
    for i, ln in enumerate(lines):
        votes = {}
        for p in ln:
            for j, lo in enumerate(lines):
                if j == i:
                    continue
                dd, _ = nearest_on(lo, p)
                if 1.0 <= dd <= TWIN_MAX_M:
                    votes[j] = votes.get(j, 0) + 1
        if votes:
            j, n = max(votes.items(), key=lambda kv: kv[1])
            if n >= max(2, len(ln) * 0.5):
                twin[i] = j

    # ── yol bandı maskesi ──
    mask = Image.new("L", (W, H), 0)
    md = ImageDraw.Draw(mask)
    for i, ln in enumerate(lines):
        half = LANE_HALF_M if twin[i] is not None else SINGLE_HALF_M
        w = int(round(2 * half * ppm))
        P = [px(p) for p in ln]
        md.line(P, fill=255, width=w, joint="curve")
        r = w / 2.0
        for q in (P[0], P[-1]):
            md.ellipse([q[0] - r, q[1] - r, q[0] + r, q[1] + r], fill=255)

    # ── asfalt + beyaz kenar çizgileri (maske konturları) ──
    img = Image.new("RGB", (W, H), ASPHALT)
    m = np.array(mask)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, hier = cv2.findContours(m, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    arr = np.array(img)
    # Döner kavşak adası = en küçük iç delik → yeşil göbek
    holes = [k for k in range(len(contours))
             if hier[0][k][3] != -1 and cv2.contourArea(contours[k]) < ISLAND_MAX_M2 * ppm * ppm]
    if holes:
        island = min(holes, key=lambda k: cv2.contourArea(contours[k]))
        cv2.drawContours(arr, contours, island, GREEN, -1)
    cv2.drawContours(arr, contours, -1, LINE, int(round(EDGE_W_M * ppm)),
                     lineType=cv2.LINE_AA)
    img = Image.fromarray(arr)
    dr = ImageDraw.Draw(img)

    # ── ikiz çiftleri arasında kesikli ayırıcı ──
    def draw_dashes(pts):
        acc, on = 0.0, True
        seg_start = pts[0]
        for a, b in zip(pts, pts[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            done = 0.0
            while done < L:
                lim = (DASH_ON_M if on else DASH_OFF_M) * ppm - acc
                step = min(lim, L - done)
                t0 = done / L
                t1 = (done + step) / L
                p0 = (a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0)
                p1 = (a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1)
                if on:
                    dr.line([p0, p1], fill=LINE, width=int(round(DASH_W_M * ppm)))
                acc += step
                done += step
                if acc >= (DASH_ON_M if on else DASH_OFF_M) * ppm - 1e-6:
                    acc, on = 0.0, not on

    drawn_pairs = set()
    for i, ln in enumerate(lines):
        j = twin[i]
        if j is None:
            # tek çizilmiş çift yön: ayırıcı çizginin kendisi üzerinde
            if not feats[i]["properties"].get("oneway", False):
                draw_dashes([px(p) for p in ln])
            continue
        key = (min(i, j), max(i, j))
        if key in drawn_pairs:
            continue
        drawn_pairs.add(key)
        mid = []
        for p in ln:
            dd, q = nearest_on(lines[j], p)
            if dd <= TWIN_MAX_M:
                mid.append(((p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0))
        if len(mid) >= 2:
            draw_dashes([px(p) for p in mid])

    img.save(args.out)
    print(f"yazildi: {args.out}  ({W}x{H}px, {ppm} px/m)")
    print(f"ikizli cizgi: {sum(1 for t in twin if t is not None)}/{len(lines)}")
    print("SDF kutu degerleri (saha_dunyasi.sdf ile senkron tutulacak):")
    print(f"  <pose>{cx:.3f} {cy:.3f} 0.01 0 0 0</pose>")
    print(f"  <size>{sx:.3f} {sy:.3f} 0.1</size>")


if __name__ == "__main__":
    main()
