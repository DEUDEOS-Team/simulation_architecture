#!/usr/bin/env python3
"""
prepare_saha_geojson.py — sahadan çizilen ham yol GeoJSON'unu rota grafına hazırlar.

route_graph çizgileri yalnızca ORTAK KÖŞE noktalarında birleştirir. QGIS'te bir
çizginin ucu başka çizginin ortasına snap'lenmişse (T-kavşak) koordinat çizgi
üzerindedir ama karşı çizgide o noktada köşe yoktur → graf kopuk kalır.

Bu araç:
  1. Her çizgi UCUNU diğer çizgilerin segmentlerine karşı test eder; tolerans
     içindeyse karşı çizgiye tam izdüşüm noktasında köşe ekler ve ucu oraya taşır.
  2. Property değerlerindeki baş/son boşlukları kırpar (ör. ' yol' → 'yol').
  3. (--oneway ile) Yön atar: saha yolları hep çift şerit (gidiş/geliş ayrı çizgi)
     ve trafik sağdan akar. Her şeridin yönü,
     karşı yön ikizinin SOLDA kalacağı şekilde seçilir; kavşak dönüş kavislerinin
     yönü ise uçlarının bağlandığı şeritlerin akışından türetilir. Yönü çizim
     sırasının tersi çıkan çizgilerin koordinat dizisi ters çevrilir ve tüm
     yönlendirilenlere `oneway: true` yazılır. Güvenle yönlendirilemeyenler
     çift yön bırakılıp raporlanır.
  4. TUNNEL_FIDS içindeki çizgilere `tunnel: true` yazar (tünel iskeleti uydu
     görüntüsünde fid=41 şeridinin tam üstünde; fid=3 onun karşı yön ikizi).
     route_planner tünel-zorunlu rota için bunu okur.
  5. Sonucu ayrı dosyaya yazar — ORİJİNALE DOKUNMAZ.

Kullanım:
  python3 prepare_saha_geojson.py [--in ../../missions/map.geojson]
                                  [--out ../../missions/map_islenmis.geojson]
                                  [--tolerance-m 0.5] [--oneway]
"""

import argparse
import json
import math
import os

MDLAT = 110574.0  # m / derece enlem (yaklaşık, saha ölçeğinde yeterli)

# Tünel iskeletinin üstünde durduğu şerit çifti (map.geojson 'fid' değerleri;
# uydu görüntüsüyle hizalanarak tespit edildi, sapma ~1 m)
TUNNEL_FIDS = {3, 41}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    here = os.path.dirname(os.path.abspath(__file__))
    missions = os.path.abspath(os.path.join(here, "..", "..", "missions"))
    ap.add_argument("--in", dest="inp", default=os.path.join(missions, "map.geojson"))
    ap.add_argument("--out", default=os.path.join(missions, "map_islenmis.geojson"))
    ap.add_argument("--tolerance-m", type=float, default=0.5)
    ap.add_argument("--oneway", action="store_true",
                    help="çift-şerit + sağdan trafik varsayımıyla şeritlere yön ata")
    args = ap.parse_args()

    with open(args.inp, encoding="utf-8") as f:
        d = json.load(f)
    feats = d["features"]

    lat0 = 0.0
    n = 0
    for ft in feats:
        for lon, lat in ft["geometry"]["coordinates"]:
            lat0 += lat
            n += 1
    lat0 /= max(n, 1)
    mdlon = 111320.0 * math.cos(math.radians(lat0))

    def seg_proj(p, a, b):
        """Nokta p'nin a-b segmentine dik mesafesi (m) ve izdüşüm parametresi t."""
        px = (p[0] - a[0]) * mdlon
        py = (p[1] - a[1]) * MDLAT
        bx = (b[0] - a[0]) * mdlon
        by = (b[1] - a[1]) * MDLAT
        L2 = bx * bx + by * by
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, (px * bx + py * by) / L2))
        return math.hypot(px - t * bx, py - t * by), t

    # ── 1) T-kavşak onarımı ──
    inserted = 0
    for fi, ft in enumerate(feats):
        coords = ft["geometry"]["coordinates"]
        for end in (0, -1):
            tip = coords[end]
            # Uç zaten başka çizginin bir KÖŞESİYLE çakışıyorsa dokunma
            shared = any(
                fj != fi and any(abs(c[0] - tip[0]) < 1e-9 and abs(c[1] - tip[1]) < 1e-9
                                 for c in f2["geometry"]["coordinates"])
                for fj, f2 in enumerate(feats)
            )
            if shared:
                continue
            best = (args.tolerance_m, None, None, None)
            for fj, f2 in enumerate(feats):
                if fj == fi:
                    continue
                cs = f2["geometry"]["coordinates"]
                for si in range(len(cs) - 1):
                    dd, t = seg_proj(tip, cs[si], cs[si + 1])
                    if dd < best[0]:
                        best = (dd, fj, si, t)
            if best[1] is None:
                continue
            _, fj, si, t = best
            cs = feats[fj]["geometry"]["coordinates"]
            a, b = cs[si], cs[si + 1]
            # İzdüşüm noktası; 7 hane yuvarlama route_graph düğüm birleştirmesiyle uyumlu
            pt = [round(a[0] + t * (b[0] - a[0]), 7), round(a[1] + t * (b[1] - a[1]), 7)]
            # Segment ucuna çok yakınsa köşe ekleme, mevcut köşeyi kullan
            if seg_proj(pt, a, a)[0] < 0.05:
                pt = [a[0], a[1]]
            elif seg_proj(pt, b, b)[0] < 0.05:
                pt = [b[0], b[1]]
            else:
                cs.insert(si + 1, list(pt))
            coords[end] = list(pt)
            inserted += 1
            print(f"T-kavsak: feature {fi} ucu -> feature {fj} (sapma {best[0]:.3f} m)")

    # ── 2) property temizliği ──
    trimmed = 0
    for ft in feats:
        props = ft.get("properties") or {}
        ft["properties"] = props
        for k, v in list(props.items()):
            if isinstance(v, str) and v != v.strip():
                props[k] = v.strip()
                trimmed += 1

    # ── 3) tünel işareti ──
    tunneled = 0
    for ft in feats:
        if ft["properties"].get("fid") in TUNNEL_FIDS:
            ft["properties"]["tunnel"] = True
            tunneled += 1
    if tunneled != len(TUNNEL_FIDS):
        print(f"UYARI: TUNNEL_FIDS {TUNNEL_FIDS} icin {tunneled} eslesme bulundu")

    # ── 4) yön atama (çift şerit + sağdan trafik) ──
    if args.oneway:
        assign_oneway(feats, mdlon)

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)
    print(f"yazildi: {args.out}  (T-kavsak eklenen uc: {inserted}, kirpilan property: {trimmed})")


def assign_oneway(feats: list, mdlon: float) -> None:
    """
    Şerit yönlerini türetir (yerinde değiştirir):
      A) İkizli şeritler: karşı yön şeridi solda kalacak yön seçilir (sağdan trafik).
      B) İkizsiz çizgiler (kavşak kavisleri vb.): uçlarının değdiği yönlendirilmiş
         şeritlerin akışıyla hizalanır; birkaç geçişte yayılır.
    Yön çizim sırasının tersiyse koordinatlar ters çevrilir; hepsine oneway=true.
    """

    def xy(c):
        return (c[0] * mdlon, c[1] * MDLAT)

    def norm(vx, vy):
        L = math.hypot(vx, vy)
        return (vx / L, vy / L) if L > 1e-9 else (0.0, 0.0)

    # Her feature için örnek noktalar + birim teğetler (segment ortaları)
    samples = []  # fi -> [(px,py,tx,ty), ...]
    for ft in feats:
        cs = [xy(c) for c in ft["geometry"]["coordinates"]]
        ss = []
        for a, b in zip(cs, cs[1:]):
            tx, ty = norm(b[0] - a[0], b[1] - a[1])
            ss.append(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, tx, ty))
        samples.append(ss)

    # ── A) ikiz tespiti + yön ──
    TWIN_MIN, TWIN_MAX = 1.0, 4.5   # m — karşı şerit merkezine beklenen mesafe
    PARALLEL_COS = 0.7
    oriented: dict[int, bool] = {}  # fi -> True (yönü kesinleşti)
    unresolved: list[int] = []

    for fi, ss in enumerate(samples):
        left_votes = right_votes = 0
        for (px, py, tx, ty) in ss:
            best = None
            for fj, ss2 in enumerate(samples):
                if fj == fi:
                    continue
                for (qx, qy, ux, uy) in ss2:
                    dd = math.hypot(qx - px, qy - py)
                    if not (TWIN_MIN <= dd <= TWIN_MAX):
                        continue
                    if abs(tx * ux + ty * uy) < PARALLEL_COS:
                        continue
                    if best is None or dd < best[0]:
                        best = (dd, qx, qy)
            if best is None:
                continue
            cross_z = tx * (best[2] - py) - ty * (best[1] - px)
            if cross_z > 0:
                left_votes += 1
            else:
                right_votes += 1
        votes = left_votes + right_votes
        if votes >= max(2, len(ss) * 0.5):
            if right_votes > left_votes:
                ft = feats[fi]
                ft["geometry"]["coordinates"].reverse()
                samples[fi] = [(-1, -1, 0, 0)]  # yeniden hesaplanacak
                cs = [xy(c) for c in ft["geometry"]["coordinates"]]
                samples[fi] = [((a[0]+b[0])/2, (a[1]+b[1])/2, *norm(b[0]-a[0], b[1]-a[1]))
                               for a, b in zip(cs, cs[1:])]
            feats[fi]["properties"]["oneway"] = True
            oriented[fi] = True
        else:
            unresolved.append(fi)
    print(f"oneway A: {len(oriented)} serit ikiz kuralıyla yonlendirildi, {len(unresolved)} kaldi")

    # ── B) kavis/ikizsizler: uç akışından yön (yayılmalı, en çok 5 geçiş) ──
    def lane_dir_at(p, skip_fi):
        """p noktasından geçen YÖNLENDİRİLMİŞ çizginin birim yönü (0.6 m tolerans)."""
        best = (0.6, None)
        for fj in oriented:
            if fj == skip_fi:
                continue
            cs = [xy(c) for c in feats[fj]["geometry"]["coordinates"]]
            for a, b in zip(cs, cs[1:]):
                bx, by = b[0] - a[0], b[1] - a[1]
                L2 = bx * bx + by * by
                t = 0 if L2 == 0 else max(0.0, min(1.0, ((p[0]-a[0])*bx + (p[1]-a[1])*by) / L2))
                dd = math.hypot(p[0] - (a[0]+t*bx), p[1] - (a[1]+t*by))
                if dd < best[0]:
                    best = (dd, norm(bx, by))
        return best[1]

    for _pass in range(5):
        progressed = False
        for fi in list(unresolved):
            cs = [xy(c) for c in feats[fi]["geometry"]["coordinates"]]
            if len(cs) < 2:
                unresolved.remove(fi)
                continue
            t_in = norm(cs[1][0]-cs[0][0], cs[1][1]-cs[0][1])
            t_out = norm(cs[-1][0]-cs[-2][0], cs[-1][1]-cs[-2][1])
            score = 0.0
            d0 = lane_dir_at(cs[0], fi)
            d1 = lane_dir_at(cs[-1], fi)
            if d0:
                score += d0[0]*t_in[0] + d0[1]*t_in[1]
            if d1:
                score += t_out[0]*d1[0] + t_out[1]*d1[1]
            if abs(score) < 0.3:
                continue  # bu geçişte karar verme
            if score < 0:
                feats[fi]["geometry"]["coordinates"].reverse()
            feats[fi]["properties"]["oneway"] = True
            oriented[fi] = True
            unresolved.remove(fi)
            progressed = True
        if not progressed:
            break
    print(f"oneway B: kavis akisiyla toplam {len(oriented)} yonlendirildi")
    if unresolved:
        print(f"NOT: {len(unresolved)} cizgi yonlendirilemedi (cift yon birakildi):")
        for fi in unresolved:
            p = feats[fi]["properties"]
            c0 = feats[fi]["geometry"]["coordinates"][0]
            print(f"  feature {fi} (tur={p.get('tur')}, fid={p.get('fid')}) ilk nokta {c0}")

    # ── C) çıkmaz uç onarımı ──
    # Tek yön atanan bir şeridin sonu hiçbir yere bağlanmıyorsa (çıkış derecesi 0)
    # veya başına hiçbir şerit beslemiyorsa (giriş derecesi 0), o şerit tuzak olur:
    # araç girip çıkamaz ya da oraya hiç ulaşılamaz. Bunlar çift yöne geri çevrilir
    # (çıkmazda U-dönüşü gerçek dünyada da tek seçenektir).
    def key(c):
        return (round(c[1], 7), round(c[0], 7))

    for _pass in range(10):
        out_deg: dict = {}
        in_deg: dict = {}
        for ft in feats:
            cs = ft["geometry"]["coordinates"]
            ow = bool(ft["properties"].get("oneway", False))
            for a, b in zip(cs, cs[1:]):
                ka, kb = key(a), key(b)
                out_deg[ka] = out_deg.get(ka, 0) + 1
                in_deg[kb] = in_deg.get(kb, 0) + 1
                if not ow:
                    out_deg[kb] = out_deg.get(kb, 0) + 1
                    in_deg[ka] = in_deg.get(ka, 0) + 1
        changed = 0
        for fi, ft in enumerate(feats):
            if not ft["properties"].get("oneway", False):
                continue
            cs = ft["geometry"]["coordinates"]
            k_end, k_start = key(cs[-1]), key(cs[0])
            # kendi kenarları hariç dış bağlantı var mı?
            end_dead = out_deg.get(k_end, 0) == 0
            start_dead = in_deg.get(k_start, 0) == 0
            if end_dead or start_dead:
                ft["properties"]["oneway"] = False
                changed += 1
        if changed == 0:
            break
        print(f"cikmaz onarim gecisi: {changed} serit cift yone cevrildi")


if __name__ == "__main__":
    main()
