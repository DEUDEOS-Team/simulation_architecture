#!/usr/bin/env python3
"""
gap_avoid_logic.py — boşluk-takibi (gap following / VFH benzeri) engelden kaçınma.

Bu modül direksiyon üretmez. Tek çıktısı bir sayıdır: **yanal ofset (metre, + = SOL)**.
Direksiyonu her zaman kamera şerit takipçisi (autonomous_control_node) üretir; biz
sadece onun hedefini yanal olarak kaydırırız ("şerit merkezini 1.4 m sola kaymış gibi izle").

Neden böyle:
  * Harita/GPS/başlık HİÇ kullanılmaz -> "engel hangi şeritte, ben hangi şeritteyim"
    hesabı yok; yanlış olamaz. Her şey ham lidar + ARAÇ ÇERÇEVESİ.
  * Kaçınma ile şerit takibi aynı direksiyonu iki yerden çekmez (eski hatanın kökü:
    haritadan mutlak direksiyon üretip virajda kamerayla kavga etmek -> araç engele kırdı).
  * Ofset ±band_half_width_m'ye kırpılır (bir şerit) -> yoldan çıkmak MÜMKÜN DEĞİL.
  * Engel kalmayınca ofset 0'a sönümlenir -> "eski şeridine dön" ayrı bir durum
    makinesi değil, VARSAYILAN davranış.
  * Slalom bedavaya çıkar: her koni grubu farklı tarafı kapatır, ofset kendiliğinden
    +/- salınır. Ayrı slalom modülü, engel sayacı, yapışkan commit YOK.

ÇERÇEVE KURALI (kritik): tüm engel ölçümleri ARAÇ çerçevesindedir (lidar). Ofset ise
ŞERİDE göredir. İkisini asla birbirine çevirmeyiz. Onun yerine ofset bir İNTEGRATÖRdür:
araç çerçevesinde "şu an ne kadar yana kaçmalıyım" (b) hesaplanır ve ofsete eklenir.
Araç kaydıkça engelin araç-göreli laterali de kayar, b kendiliğinden 0'a iner ve
manevra kendi kendine biter. Böylece komut ile gerçek konum arasındaki gecikme
güvenlik hesabını BOZMAZ (araç çerçevesindeki ölçüm her zaman gerçektir).
"""

from __future__ import annotations

from dataclasses import dataclass, replace

# Aynı manevra içinde taraf değiştirmenin maliyeti (m). Kare-kare zikzağı keser;
# aynı taraf gerçekten kapandıysa geçiş yine serbesttir (slalom bozulmaz).
SIDE_FLIP_PENALTY_M = 0.5
# Şerit bandı sınırına bırakılacak pay (m) — sınırı yalayan çözümler seçilmez.
BAND_MARGIN_M = 0.15


@dataclass(frozen=True)
class GapObstacle:
    """Araç çerçevesinde bir engel. lateral: + = SOL (ROS/lidar konvansiyonu)."""
    distance_m: float       # araç merkezinden (base_link) ileri mesafe
    lateral_min_m: float    # kümenin en sağ ucu  (küçük = sağ)
    lateral_max_m: float    # kümenin en sol ucu


@dataclass(frozen=True)
class GapAvoidConfig:
    # Ofsetin izinli bandı asimetrik. Bu haritada komşu (karşı) şerit her zaman soldadır
    # (sağdan trafik, şerit çiftleri 3.39 m). Sola taşmak = karşı şeridi kullanmak (kaçınmada
    # izinli), sağa taşmak = asfaltı bitirip yoldan çıkmak (yasak). Simetrik band araç sağa
    # kaçarken onu asfaltın dışına kadar bırakıyordu.
    band_left_m: float = 3.0     # karşı şerit merkezine kadar
    # Şerit merkezinden sağ asfalt kenarına 1.7 m var. Araç yarı genişliği 0.80 -> tam asfalt
    # üstünde kalmak için sınır 0.90 olurdu; engeli geniş geçmek için biraz gevşetildi.
    # 1.30'da tekerler en fazla ~0.4 m bankete taşar, engel bitince ofset zaten 0'a döner.
    band_right_m: float = 1.30
    # Geriye dönük uyumluluk / genel kırpma sınırı
    band_half_width_m: float = 3.0
    bin_size_m: float = 0.10
    # Araç yarı genişliği. Gövde yarısı 0.60 ama tekerler/aynalar daha dışarıda ve lidar
    # kümesi koninin yalnız görünen yüzünü verir (arka yüzü eksik) — ikisi de gerçek açıklığı
    # hesaplanandan küçük yapıyor, o yüzden 0.80.
    car_half_width_m: float = 0.80
    # Engele bırakılacak ek pay. Algoritma asgari sapma yapar (gerektiği kadar kaçar, fazlasını
    # değil), dolayısıyla geçiş açıklığını belirleyen tek şey bu paydır. Şişirme toplamı
    # 0.80+0.55 = 1.35 m -> koni merkezine 1.50 m ile geçilir, gövdeyle koni arasında gerçek
    # ~0.55 m hava kalır. Daha da büyütmek bu yolda çalışmaz: şerit 3.4 m, koninin iki yanı
    # birden kapanıp "yol kapalı" çıkar.
    safety_margin_m: float = 0.55
    # Bu mesafeden yakın engeller manevraya sokar. 5'li koni sırası lidar'da tam 12 m'de
    # belirdiğinden 12 m'de manevra için pay kalmıyordu; 15 m gerekli kaçışa (~2 m, 1.2 m/s
    # ile ~1.7 s -> 4 m/s'de ~7 m yol) yer bırakır.
    trigger_distance_m: float = 15.0
    min_distance_m: float = 0.3         # bundan yakını gürültü/araç gövdesi
    # Planlama derinliği: yalnız en yakın engelden itibaren bu derinlikteki engeller hesaba
    # katılır. Slalom için şart: iki koni birden pencereye girerse (biri sağı biri solu
    # kapatır) "ikisini birden çevreleyen" tek boşluk aranır ve araç şeritten fırlar. İnsan da
    # slalomda en yakın koniye göre kırar, sonraki koni yaklaşınca yeniden karar verir. Aynı
    # hizadaki gruplar (yan yana koniler) bu derinlik içinde kaldığından tek engel sayılır.
    plan_depth_m: float = 3.0
    # Kayma hızı. Şişirme büyüyünce gereken kayma da büyüdü; daha yavaş bir değer koniler
    # arasındaki ~10 m'de yetişmiyor ve araç ikinci koniye yarım kaymış hâlde giriyor.
    max_rate_mps: float = 1.8
    return_rate_mps: float = 0.6        # engel bitince 0'a dönüş hızı (daha yumuşak)


@dataclass(frozen=True)
class GapAvoidState:
    offset_m: float = 0.0       # KOMUT: şerit merkezine göre yanal kayma (+ = sol)
    required_b_m: float = 0.0   # bu karede araç çerçevesinde gereken kaçış (teşhis)
    blocked: bool = False       # hiçbir boşluk yok -> yol kapalı (acil fren üst katmanda)
    active: bool = False        # manevra sürüyor mu
    reason: str = "clear"       # clear | avoid_left | avoid_right | pass | blocked | return


def _blocked_intervals(obstacles: list[GapObstacle],
                       cfg: GapAvoidConfig) -> list[tuple[float, float]]:
    """Her engeli araç yarı genişliği + pay kadar şişir. Araç merkezi bu aralıkların
    içindeyse çarpar; dışındaysa geçer. (Şişirme sayesinde 'araç noktadır' varsayımı
    geçerli olur — sonraki adımda genişlik tekrar hesaba katılmaz.)"""
    inflate = cfg.car_half_width_m + cfg.safety_margin_m
    return [(o.lateral_min_m - inflate, o.lateral_max_m + inflate) for o in obstacles]


def _is_free(b: float, intervals: list[tuple[float, float]]) -> bool:
    """Aralıklar KAPALI: tam sınırda duran aday serbest SAYILMAZ. Böylece 'payı
    sıfırlayan' teğet geçişler seçilmez; gerçekten temiz boşluk yoksa blocked'a düşeriz."""
    return all(not (lo <= b <= hi) for lo, hi in intervals)


def _required_shift(intervals: list[tuple[float, float]],
                    measured_offset_m: float | None,
                    prev_sign: float,
                    cfg: GapAvoidConfig) -> float | None:
    """Araç çerçevesinde EN UCUZ serbest yanal konumu bul (b, + = sol).

    Maliyet = |b| + (taraf değiştiriyorsa SIDE_FLIP_PENALTY). b = 0 serbestse 0
    (önü açık). Hiçbir aday serbest değilse None (= yol kapalı).

    Şerit bandı kısıtı yalnız ölçüm varken uygulanır: kısıt ofset komutuna göre
    olursa, ölçüm yokken komut kör integratör gibi sarıp band kısıtı bir kare sağı bir
    kare solu yasaklıyor, b zıplıyor ve direksiyon titriyor. Şeridin nerede olduğunu
    ölçemiyorsak banda göre kısıtlamak anlamsız; o durumda yalnız engelden kaçarız
    (düzeltme zaten sınırlı ve b sıfırlanınca biter).

    Taraf cezası aynı manevra içinde kare-kare taraf değiştirmeyi engeller; o taraf
    gerçekten kapandıysa yine de geçiş yapılır — slalomda sıradaki koni ters taraftaysa
    dönüş serbest kalmalı.
    """
    step = cfg.bin_size_m
    n = int(round(2.0 * max(cfg.band_left_m, cfg.band_right_m) / step))
    best_b: float | None = None
    best_cost = float("inf")
    for k in range(n + 1):
        for b in ((0.0,) if k == 0 else (-k * step, k * step)):
            if measured_offset_m is not None:
                pos = measured_offset_m + b          # ölçülü konumdan varılacak yer
                # BAND_MARGIN: sınıra sıfır paylı çözüm seçilmez. Aksi hâlde araç o
                # yöne bir kare kayınca çözüm bandın dışına düşüyor ve ters tarafa
                # dönmek zorunda kalıyor (bir karelik zikzak).
                if (pos > cfg.band_left_m - BAND_MARGIN_M
                        or pos < -cfg.band_right_m + BAND_MARGIN_M):
                    continue        # sola: karşı şeridi aşar / sağa: asfalttan çıkar
            if not _is_free(b, intervals):
                continue
            cost = abs(b)
            if prev_sign != 0.0 and b != 0.0 and (b > 0) != (prev_sign > 0):
                cost += SIDE_FLIP_PENALTY_M
            if cost < best_cost:
                best_cost, best_b = cost, b
        # |b| taramanın ötesine geçtiyse daha iyisi çıkamaz (maliyet |b| ile artar)
        if best_b is not None and (k * step) > best_cost:
            break
    return best_b


def plan_offset(obstacles: list[GapObstacle],
                state: GapAvoidState,
                dt_s: float,
                cfg: GapAvoidConfig = GapAvoidConfig(),
                measured_offset_m: float | None = None) -> GapAvoidState:
    """Bir lidar karesi -> yeni ofset komutu.

    measured_offset_m: aracın ŞERİT MERKEZİNE göre ölçülen yanal konumu (+ = sol).
    Kaynağı avoidance_node'da: BİRİNCİL harita (GPS + merkez çizgileri, her zaman var),
    yedek kamera. Bu ölçüm ŞERİTTEN ÇIKMAMA kısıtını besler (band_left/band_right);
    ölçüm yoksa kısıt uygulanamaz. Koşu-34: ölçüm yalnız kameradan geliyordu, şerit
    segmentasyonu düşünce kısıt tamamen kalktı ve araç 3. koniyi sağdan dolaşmaya
    çalışırken asfaltın dışına kadar kaydı.

    Dört durum:
      1. Menzilde engel yok            -> ofset 0'a sönümlenir (kendi şeridine dönüş)
      2. Engel var, önü KAPALI (b != 0) -> hedef = ölçülen + b (hız sınırlı)
      3. Engel var, önü AÇIK  (b == 0)  -> ofset TUTULUR (engelin yanından geçiyoruz;
                                           şimdi geri dönmek engele kırmak olurdu)
      4. Hiç boşluk yok                 -> blocked (üst katman acil fren yapar)
    """
    dt = max(0.0, min(float(dt_s), 0.5))   # bir karede sıçrama olmasın

    rel = [o for o in obstacles
           if cfg.min_distance_m <= o.distance_m <= cfg.trigger_distance_m]

    if not rel:
        # (1) Yol temiz: eve dön.
        step = cfg.return_rate_mps * dt
        new_off = state.offset_m
        if abs(new_off) <= step:
            new_off = 0.0
        else:
            new_off -= step if new_off > 0 else -step
        homing = abs(new_off) > 1e-6
        return GapAvoidState(
            offset_m=new_off, required_b_m=0.0, blocked=False, active=homing,
            reason="return" if homing else "clear")

    # Yalnız EN YAKIN grup (bkz. plan_depth_m) — slalomda alternatif kırış bunu gerektirir.
    nearest = min(o.distance_m for o in rel)
    rel = [o for o in rel if o.distance_m <= nearest + cfg.plan_depth_m]

    intervals = _blocked_intervals(rel, cfg)
    # b, ofset komutuna (integratöre) asla bakmaz — yalnız lidar ölçümüne ve (varsa)
    # gerçek yanal konuma. Bkz. _required_shift docstring'i.
    prev_sign = 0.0
    if state.reason in ("avoid_left", "avoid_right") and abs(state.required_b_m) > 1e-3:
        prev_sign = 1.0 if state.required_b_m > 0 else -1.0
    b = _required_shift(intervals, measured_offset_m, prev_sign, cfg)

    if b is None:
        # (4) Hiçbir yere sığmıyoruz. Ofseti dondur, yol kapalı bildir.
        return replace(state, required_b_m=0.0, blocked=True, active=True,
                       reason="blocked")

    if abs(b) < cfg.bin_size_m / 2.0:
        # (3) Önümüz açık ama engel hâlâ menzilde -> ofseti KORU.
        return replace(state, required_b_m=0.0, blocked=False, active=True,
                       reason="pass")

    # (2) Kaç. offset_m artık yalnız TELEMETRİ/şerit-takip ofseti içindir; direksiyonu
    # avoidance_node doğrudan b'den üretir (bkz. /planning/avoid_steer).
    base = state.offset_m if measured_offset_m is None else float(measured_offset_m)
    target = max(-cfg.band_right_m, min(cfg.band_left_m, base + b))
    step = cfg.max_rate_mps * dt
    delta = max(-step, min(step, target - state.offset_m))
    new_off = max(-cfg.band_right_m, min(cfg.band_left_m, state.offset_m + delta))
    return GapAvoidState(
        offset_m=new_off, required_b_m=b, blocked=False, active=True,
        reason="avoid_left" if b > 0 else "avoid_right")
