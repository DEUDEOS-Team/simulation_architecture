#!/usr/bin/env python3
"""Boşluk-takibi kaçınması (gap_avoid_logic) testleri.

FİZİK MODELİ (önemli): engeller DÜNYADA sabittir; lidar onları ARAÇ çerçevesinde
ölçer. Araç ofset komutunu izleyip yana kaydıkça engelin araç-göreli laterali de
kayar:  lateral_araç = lateral_dünya − ofset.  Kapanan döngü budur — manevranın
kendi kendine bitmesini (b -> 0) sağlayan da bu. Testler bu modeli simüle eder
(araç ofseti kusursuz izliyor varsayımı; gerçekte şerit takipçisi biraz geriden
gelir, hız sınırı zaten bunu tolere eder).

Kapsam: tek engel, yan yana 5 koni, art arda gruplar (slalom), şeride dönüş,
banttan taşmama, yol kapalı, hız sınırı, menzil.
Konvansiyon: lateral/ofset + = SOL.
"""
import sys

sys.path.insert(0, "/home/bekir/sim2_ws/src/araba/scripts")

from gap_avoid_logic import (  # noqa: E402
    GapAvoidConfig, GapAvoidState, GapObstacle, plan_offset)

ok = fail = 0


def check(name, cond):
    global ok, fail
    if cond:
        ok += 1
        print(f"  PASS {name}")
    else:
        fail += 1
        print(f"  FAIL {name}")


CFG = GapAvoidConfig()
DT = 0.1
INFLATE = CFG.car_half_width_m + CFG.safety_margin_m   # 0.95 m


def world_cone(dist, lat_world, half=0.15):
    """Dünyada sabit koni tanımı (araç çerçevesine run() çevirir)."""
    return (dist, lat_world - half, lat_world + half)


def run(scene_fn, steps, state=None, dt=DT, tracking=True, lag=1.0):
    """scene_fn(i) -> DÜNYA çerçevesinde engel listesi.

    tracking=True : araç komutu izler (lag=1.0 kusursuz; <1 gecikmeli takip).
    tracking=False: araç komutu hiç izlemez (car_y sabit 0) — kontrol PLAN modundayken
                    ofset uygulanmıyor. Komut bu durumda sarmamalı (band sınırına dayanıp
                    titrememeli).
    Ölçülen yanal konum (kamera) her karede plan_offset'e verilir.
    """
    st = state or GapAvoidState()
    car_y = st.offset_m if tracking else 0.0
    trace = []
    for i in range(steps):
        obs = [GapObstacle(distance_m=d,
                           lateral_min_m=lo - car_y,
                           lateral_max_m=hi - car_y)
               for (d, lo, hi) in scene_fn(i)]
        st = plan_offset(obs, st, dt, CFG, measured_offset_m=car_y)
        trace.append(st.offset_m)
        if tracking:                       # araç komuta doğru hareket eder
            car_y += (st.offset_m - car_y) * lag
    return st, trace


print("── 1) Boş yol: ofset üretilmez ──")
st, _ = run(lambda i: [], 20)
check("engel yok -> ofset 0", abs(st.offset_m) < 1e-6)
check("engel yok -> reason=clear", st.reason == "clear")
check("engel yok -> aktif değil", not st.active)

print("\n── 2) Tam önde tek koni: kaçınır, yeterince kaçınca durur ──")
st, trace = run(lambda i: [world_cone(8.0, 0.0)], 60)
check("önde engel -> ofset üretti", abs(st.offset_m) > 0.9)
check("gereğinden fazla kaçmaz (~engel yarısı + pay)",
      abs(st.offset_m) <= INFLATE + 0.35)
check("ofset monoton (zikzak yok)",
      all(abs(b) >= abs(a) - 1e-9 for a, b in zip(trace, trace[1:])))
check("temizlenince tutar (pass)", st.reason == "pass")
check("bandın içinde", abs(st.offset_m) <= CFG.band_half_width_m + 1e-9)

print("\n── 3) SAĞDAKİ engel -> SOLA, SOLDAKİ engel -> SAĞA ──")
st, _ = run(lambda i: [world_cone(7.0, -1.0)], 40)
check("engel sağda -> sola kaçtı", st.offset_m > 0.2)
st, _ = run(lambda i: [world_cone(7.0, +1.0)], 40)
check("engel solda -> sağa kaçtı", st.offset_m < -0.2)

print("\n── 4) Yan yana 5 koni (dünyadaki 1. engel grubu) ──")
row = [world_cone(9.0, lat) for lat in (-0.7, -0.35, 0.0, 0.35, 0.7)]
st, trace = run(lambda i: row, 80)
# Grubun uçları ±0.85; temiz geçiş için |ofset| >= 0.85 + 0.95 = 1.80
check("5'li sıra -> grubun tamamen dışına çıktı", abs(st.offset_m) >= 1.80)
check("5'li sıra -> bandın içinde", abs(st.offset_m) <= CFG.band_half_width_m)
check("5'li sıra -> yol kapalı sayılmadı", not st.blocked)
check("5'li sıra -> tek yöne, zikzaksız",
      all(abs(b) >= abs(a) - 1e-9 for a, b in zip(trace, trace[1:])))

print("\n── 5) Engel geçildi: kendi şeridine döner ──")
st = GapAvoidState(offset_m=2.0)
st, trace = run(lambda i: [], 60, state=st)
check("engel gidince ofset 0'a döner", abs(st.offset_m) < 1e-6)
check("dönüş hız-sınırlı (ani değil)", trace[0] > 1.9)
check("dönüş bitince reason=clear", st.reason == "clear")

print("\n── 6) Yanından geçerken geri dönmez (pass) ──")
st = GapAvoidState(offset_m=1.8)
# Engel dünyada -0.0'da; araç 1.8 solda -> araca göre -1.8, önü açık.
st2, _ = run(lambda i: [world_cone(4.0, 0.0)], 1, state=st)
check("önü açık ama engel menzilde -> reason=pass", st2.reason == "pass")
check("önü açık ama engel menzilde -> ofset korunur", abs(st2.offset_m - 1.8) < 1e-9)

print("\n── 7) Art arda gruplar = SLALOM (ofset işaret değiştirir) ──")
def slalom(i):
    if i < 40:
        return [world_cone(7.0, -1.2)]     # 1. grup dünyada SAĞDA -> araç sola
    if i < 110:
        return [world_cone(7.0, +1.2)]     # 2. grup dünyada SOLDA -> araç sağa
    return []


st, trace = run(slalom, 160)
peak_left = max(trace[:40])
# NOT: algoritma ASGARİ SAPMA yapar — 1.2 m yanda duran koni için tam şerit
# değiştirmez, sadece pay kadar (≈0.35 m) kayar. İstenen davranış budur;
# önemli olan İŞARETİN doğru ve geçişin temiz olması.
check("1. grup (sağda) -> araç SOLA kaydı", peak_left > 0.1)
check("2. grup (solda) -> araç SAĞA geçti (işaret döndü)", min(trace[40:110]) < -0.1)
check("slalom sonu -> kendi şeridine döndü", abs(trace[-1]) < 1e-6)

print("\n── 7c) DÜNYADAKİ SLALOM PARKURU: dönüşümlü koniler, araç zikzak çizer ──")
# benim_dunyam.sdf: şerit merkezi y=24.0; koniler dönüşümlü 24.8 / 22.8 (±0.8),
# 8-12 m aralıklı. Şerit merkezine göre yanal: +0.8 (kuzey) / -0.8 (güney).
# Araç BATIYA gider; koniler sırayla yaklaşır. Burada "ofset" = şeride göre yanal.
# benim_dunyam.sdf'teki GERÇEK koniler, şerit merkezine (y=24.0) göre YANAL:
#   x=28 y=24.8 -> 0.8 m KUZEY;  x=16 y=22.8 -> 1.2 m GÜNEY;  x=8 y=24.8 -> 0.8 m KUZEY
# Araç BATIYA gidiyor: sol(+) = GÜNEY, sağ(−) = KUZEY.
#   kuzeydeki koni -> lateral −0.8   |   güneydeki koni -> lateral +1.2
# (3 tekli koni; 4. ve 5. koni dünyadan kaldırıldı.)
CONES = [(-0.8,), (+1.2,), (-0.8,)]
SPACING = 10.0


def course(i):
    """i. karede aracın önündeki koniler (mesafe azalarak)."""
    travelled = i * 0.30                        # ~3 m/s, dt=0.1
    out = []
    for k, (lat,) in enumerate(CONES):
        d = 30.0 + k * SPACING - travelled      # ilk koni 30 m ileride
        if d > 0.3:
            out.append((d, lat - 0.15, lat + 0.15))
    return out


st = GapAvoidState()
car_y = 0.0
seq = []          # her koniyi geçerken aracın yanal konumu
passed = 0
for i in range(700):
    obs = [GapObstacle(distance_m=d, lateral_min_m=lo - car_y, lateral_max_m=hi - car_y)
           for (d, lo, hi) in course(i)]
    st = plan_offset(obs, st, DT, CFG, measured_offset_m=car_y)
    car_y = st.offset_m                          # kusursuz takip varsayımı
    n_left = len(course(i))
    if n_left < len(CONES) - passed:             # bir koni geçildi
        seq.append(round(car_y, 2))
        passed += 1
check(f"3 koninin de yanından geçildi (geçiş yanalları: {seq})", passed == len(CONES))
if len(seq) >= 3:
    check("araç DÖNÜŞÜMLÜ kırdı (zikzak)",
          all((a < 0) != (b < 0) for a, b in zip(seq, seq[1:])))
    check("zikzak şerit bandında kaldı",
          all(abs(s) <= CFG.band_half_width_m for s in seq))

print("\n── 7d) Titreme: ölçüm yokken b zıplamaz ──")
# Kamera şerit noktaları hiç gelmeyince (measured=None) ofset integratörü -2.7 m'ye
# sarabiliyor; band kısıtı ofset komutuna uygulandığında b kare-kare +2.50 ↔ -0.30
# zıplayıp direksiyon titriyor ve araç koniye giriyordu.
st = GapAvoidState(offset_m=-2.7)          # sarmış (anlamsız) komut
bs = []
car_y = 0.0                                 # araç aslında şerit merkezinde
for i in range(40):
    obs = [GapObstacle(distance_m=6.0 - i * 0.05,
                       lateral_min_m=-1.2 - 0.15, lateral_max_m=-1.2 + 0.15)]
    st = plan_offset(obs, st, DT, CFG, measured_offset_m=None)
    bs.append(st.required_b_m)
signs = [1 if b > 0 else (-1 if b < 0 else 0) for b in bs if abs(b) > 1e-3]
check(f"b işaret değiştirmiyor (b aralığı {min(bs):+.2f}..{max(bs):+.2f})",
      len(set(signs)) <= 1)
check("engel SAĞDA -> b SOLA (+)", all(s > 0 for s in signs))

print("\n── 7e) Yoldan çıkma: sağ band aşılmaz ──")
# Koşu-34: araç 2. koniden sonra sağda kalmıştı; 3. koni (sağda/kuzeyde) gelince
# taraf-değiştirme cezası yüzünden onu da SAĞDAN dolaşmayı seçti ve asfaltın
# dışına kadar kaydı. Sağ band (band_right_m) bunu yapısal olarak engellemeli.
st = GapAvoidState(offset_m=0.5)
car_y = 0.5                                  # araç zaten sağda değil, hafif solda
bs, ys = [], []
for i in range(120):
    # koni ARAÇ TARAFINDA (sol/+): sağdan dolaşmak asfalt dışına iterdi
    lat_world = 0.8
    obs = [GapObstacle(distance_m=max(1.0, 12.0 - i * 0.12),
                       lateral_min_m=lat_world - 0.15 - car_y,
                       lateral_max_m=lat_world + 0.15 - car_y)]
    st = plan_offset(obs, st, DT, CFG, measured_offset_m=car_y)
    car_y = st.offset_m
    ys.append(car_y)
check(f"araç sağ bandı (−{CFG.band_right_m} m) ASLA aşmadı (min {min(ys):+.2f} m)",
      min(ys) >= -CFG.band_right_m - 1e-6)
check("araç sol bandı aşmadı", max(ys) <= CFG.band_left_m + 1e-6)

print("\n── 7b) Slalom boyunca hiçbir koniye ÇARPMAZ ──")
# Açıklık YALNIZ koninin YANINDAN GEÇERKEN ölçülür (boylamsal olarak hizadayken).
# İlk sürüm bunu atlıyordu: 24 m ileride, aracın henüz kaymadığı bir koniyle yanal
# örtüşmeyi "çarpma" sayıyordu — testin kendi hatasıydı, algoritmanınki değil.
PASS_BY_M = 1.5          # koni bu kadar önümüzdeyken fiilen yanımızdan geçiyor
worst = float("inf")
measured_frames = 0
st = GapAvoidState()
car_y = 0.0
for i in range(700):
    scene = course(i)    # 7c'deki GERÇEK parkur (koniler yaklaşıyor ve geçiliyor)
    for (d, lo, hi) in scene:
        if d > PASS_BY_M:
            continue                          # daha uzakta: kaçınmak için zaman var
        gap = min(abs(car_y - CFG.car_half_width_m - hi),
                  abs(lo - (car_y + CFG.car_half_width_m)))
        if lo - CFG.car_half_width_m <= car_y <= hi + CFG.car_half_width_m:
            gap = -gap                        # gövde koninin üstünde = çarpma
        worst = min(worst, gap)
        measured_frames += 1
    obs = [GapObstacle(distance_m=d, lateral_min_m=lo - car_y, lateral_max_m=hi - car_y)
           for (d, lo, hi) in scene]
    st = plan_offset(obs, st, DT, CFG, measured_offset_m=car_y)
    car_y = st.offset_m
check(f"geçiş anı gerçekten ölçüldü ({measured_frames} kare)", measured_frames > 0)
check(f"yanından geçerken gövde-koni açıklığı > 0 (ölçülen {worst:+.2f} m)", worst > 0.0)

print("\n── 8) Bandın dışına ASLA taşmaz ──")
wide = [world_cone(8.0, lat) for lat in (-2.5, -2.0, -1.5, -1.0, -0.5, 0.0)]
st, trace = run(lambda i: wide, 120)
check("geniş engel -> ofset banda kırpıldı",
      max(abs(t) for t in trace) <= CFG.band_half_width_m + 1e-9)

print("\n── 9) Yol kapalı: blocked ──")
wall = [world_cone(6.0, lat, half=0.3) for lat in
        (-3.5, -2.8, -2.1, -1.4, -0.7, 0.0, 0.7, 1.4, 2.1, 2.8, 3.5)]
st, _ = run(lambda i: wall, 10)
check("baştan başa bariyer -> blocked", st.blocked and st.reason == "blocked")
check("blocked -> ofset dondu", abs(st.offset_m) < 1e-6)

print("\n── 10) Menzil dışı engel dikkate alınmaz ──")
st, _ = run(lambda i: [world_cone(18.0, 0.0)], 20)
check("18 m'deki engel (tetik 12) -> ofset yok",
      abs(st.offset_m) < 1e-6 and st.reason == "clear")

print("\n── 11) Sarma: araç komutu izlemezse komut patlamaz ──")
# Koşu-26: kontrol PLAN modundaydı, ofset uygulanmıyordu. Eski (açık döngü) kod
# ofseti 3 saniyede 0 -> -2.9 m'ye (band sınırı) sürükleyip iki taraf arası titretti.
st, trace = run(lambda i: [world_cone(6.0, 0.0)], 100, tracking=False)
check("izlenmeyen komut band sınırına dayanmaz",
      max(abs(t) for t in trace) < CFG.band_half_width_m - 0.5)
check("komut 'gereken kaçış' civarında durur (sarma yok)",
      abs(abs(trace[-1]) - abs(st.required_b_m)) < 0.35)
check("titreme yok (işaret değiştirmiyor)",
      all(t >= -1e-9 for t in trace) or all(t <= 1e-9 for t in trace))

print("\n── 11b) Gecikmeli takip (gerçekçi): yine de aşırı kaçmaz ──")
st, trace = run(lambda i: [world_cone(8.0, 0.0)], 120, tracking=True, lag=0.15)
check("gecikmeli araçta bile ofset makul kalır", max(abs(t) for t in trace) <= 1.9)

print("\n── 12) Hız sınırı ──")
st = plan_offset([GapObstacle(8.0, -0.15, 0.15)], GapAvoidState(), 0.1, CFG)
check("tek karede en fazla max_rate*dt kayar",
      abs(st.offset_m) <= CFG.max_rate_mps * 0.1 + 1e-9)
st_big = plan_offset([GapObstacle(8.0, -0.15, 0.15)], GapAvoidState(), 5.0, CFG)
check("dev dt kırpılır (0.5 s)",
      abs(st_big.offset_m) <= CFG.max_rate_mps * 0.5 + 1e-9)

print(f"\n{'='*46}\nSONUÇ: {ok} PASS, {fail} FAIL\n{'='*46}")
sys.exit(1 if fail else 0)
