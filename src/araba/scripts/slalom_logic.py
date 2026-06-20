"""
slalom_logic.py
---------------
Koni engelleri arasında slalom manevrasını yönetir.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleKind
from deos_algorithms.safety_logic import IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX


MIN_CONFIDENCE = 0.4
MERKEZ_ESIK = 0.15
VARSAYILAN_TARAF = "sol"
STEERING_GAIN = 1.2
MERKEZ_ITME = 0.35

CENTER_WEAVE_ENABLE = True
CENTER_WEAVE_AMPLITUDE = 0.55
CENTER_WEAVE_AVOID_WEIGHT = 0.20

S_WEAVE_ENABLE = True
S_WEAVE_SINGLE_SIDE_FRAMES = 6
S_WEAVE_AMPLITUDE = 0.45
S_WEAVE_AVOID_WEIGHT = 0.35
GECIS_YAKINLIK_ESIK = 0.80
BITTI_FRAME_ESIK = 15

_HIZ = {"uzak": 1.0, "orta": 0.6, "yakin": 0.3}
DIST_UZAK_M = 3.0
DIST_ORTA_M = 1.5


@dataclass
class _KoniHedef:
    lateral_offset: float
    taraf: str
    yakinlik: float
    mesafe_m: Optional[float]


@dataclass
class SlalomState:
    aktif: bool = False
    steering: float = 0.0
    hiz_katsayisi: float = 1.0
    hedef_taraf: str = "belirsiz"
    gecilen_koni: int = 0
    gorunen_koni: int = 0
    faz: str = "bekleme"
    sebep: str = ""


class SlalomLogic:
    def __init__(self, goruntu_genislik: int = IMAGE_WIDTH_PX, goruntu_yukseklik: int = IMAGE_HEIGHT_PX):
        self._g_w = goruntu_genislik
        self._g_h = goruntu_yukseklik

        self._faz: str = "bekleme"
        self._son_gecis_taraf: str = VARSAYILAN_TARAF
        self._gecilen_koni: int = 0
        self._bitti_sayac: int = 0
        self._onceki_yakinlik: float = 0.0
        self._s_weave_sign: float = 1.0
        self._tek_taraf_sayac: int = 0

    def update(self, detections: list[ObstacleDetection]) -> SlalomState:
        koniler = self._filtrele(detections)
        if not koniler:
            return self._koni_yok_isle()

        koniler.sort(key=self._yakinlik_skoru, reverse=True)
        kapi_offset = self._baslangic_kapisi_offset(koniler) if self._gecilen_koni == 0 else None
        en_yakin = koniler[0]
        hedef = self._gecis_hedefi_hesapla(en_yakin)

        self._gecis_kontrol(hedef.yakinlik)

        if kapi_offset is not None:
            steering = self._kapi_steering_hesapla(kapi_offset)
            hedef_taraf = "kapi"
            sebep = f"başlangıç kapısı (mid_offset={kapi_offset:+.2f}, yakinlik={hedef.yakinlik:.2f})"
        else:
            steering = self._steering_hesapla(hedef, koniler)
            hedef_taraf = hedef.taraf
            sebep = f"koni {hedef.taraf} tarafta (offset={hedef.lateral_offset:+.2f}, yakinlik={hedef.yakinlik:.2f})"

        hiz = self._hiz_hesapla(hedef)

        self._faz = "aktif"
        self._bitti_sayac = 0
        self._son_gecis_taraf = hedef.taraf

        return SlalomState(
            aktif=True,
            steering=steering,
            hiz_katsayisi=hiz,
            hedef_taraf=hedef_taraf,
            gecilen_koni=self._gecilen_koni,
            gorunen_koni=len(koniler),
            faz="aktif",
            sebep=sebep,
        )

    def reset(self) -> None:
        self._reset()

    def _filtrele(self, detections: list[ObstacleDetection]) -> list[ObstacleDetection]:
        return [d for d in detections if d.kind == ObstacleKind.CONE and d.confidence >= MIN_CONFIDENCE]

    def _yakinlik_skoru(self, d: ObstacleDetection) -> float:
        if d.estimated_distance_m is not None and d.estimated_distance_m > 0:
            return 1.0 / (1.0 + d.estimated_distance_m)
        return d.bbox_px[3] / self._g_h

    def _lateral_offset(self, d: ObstacleDetection) -> float:
        x_merkez = (d.bbox_px[0] + d.bbox_px[2]) / 2.0
        return (x_merkez - self._g_w / 2.0) / (self._g_w / 2.0)

    def _gecis_hedefi_hesapla(self, d: ObstacleDetection) -> _KoniHedef:
        offset = self._lateral_offset(d)
        yakinlik = self._yakinlik_skoru(d)

        if abs(offset) > MERKEZ_ESIK:
            taraf = "sag" if offset < 0 else "sol"
        else:
            taraf = "sol" if self._son_gecis_taraf == "sag" else "sag"

        return _KoniHedef(lateral_offset=offset, taraf=taraf, yakinlik=yakinlik, mesafe_m=d.estimated_distance_m)

    def _s_weave_aktif_mi(self, koniler: list[ObstacleDetection]) -> bool:
        if not S_WEAVE_ENABLE:
            self._tek_taraf_sayac = 0
            return False

        offsets = [self._lateral_offset(k) for k in koniler]
        offsets = [o for o in offsets if abs(o) > MERKEZ_ESIK]
        if not offsets:
            self._tek_taraf_sayac = max(0, self._tek_taraf_sayac - 1)
            return self._tek_taraf_sayac >= S_WEAVE_SINGLE_SIDE_FRAMES

        same_sign = all(o > 0 for o in offsets) or all(o < 0 for o in offsets)
        self._tek_taraf_sayac = self._tek_taraf_sayac + 1 if same_sign else 0
        return self._tek_taraf_sayac >= S_WEAVE_SINGLE_SIDE_FRAMES

    def _baslangic_kapisi_offset(self, koniler: list[ObstacleDetection]) -> Optional[float]:
        if len(koniler) < 2:
            return None
        k1, k2 = koniler[0], koniler[1]
        o1, o2 = self._lateral_offset(k1), self._lateral_offset(k2)
        if abs(o1) <= MERKEZ_ESIK or abs(o2) <= MERKEZ_ESIK:
            return None
        if (o1 > 0) == (o2 > 0):
            return None
        y1, y2 = self._yakinlik_skoru(k1), self._yakinlik_skoru(k2)
        if abs(y1 - y2) > 0.12:
            return None
        return (o1 + o2) / 2.0

    def _kapi_steering_hesapla(self, kapi_mid_offset: float) -> float:
        steering = -kapi_mid_offset * STEERING_GAIN
        return max(-1.0, min(1.0, steering))

    def _steering_hesapla(self, hedef: _KoniHedef, koniler: list[ObstacleDetection]) -> float:
        merkezde = abs(hedef.lateral_offset) <= MERKEZ_ESIK
        if not merkezde:
            avoid = -hedef.lateral_offset * STEERING_GAIN
        else:
            isaret = 1.0 if hedef.taraf == "sag" else -1.0
            avoid = isaret * MERKEZ_ITME * hedef.yakinlik

        if merkezde and CENTER_WEAVE_ENABLE:
            amp = CENTER_WEAVE_AMPLITUDE * (0.25 + 0.75 * hedef.yakinlik)
            weave = self._s_weave_sign * amp
            steering = (CENTER_WEAVE_AVOID_WEIGHT * avoid) + ((1.0 - CENTER_WEAVE_AVOID_WEIGHT) * weave)
            return max(-1.0, min(1.0, steering))

        if self._s_weave_aktif_mi(koniler):
            amp = S_WEAVE_AMPLITUDE * (0.25 + 0.75 * hedef.yakinlik)
            weave = self._s_weave_sign * amp
            steering = (S_WEAVE_AVOID_WEIGHT * avoid) + ((1.0 - S_WEAVE_AVOID_WEIGHT) * weave)
        else:
            steering = avoid

        return max(-1.0, min(1.0, steering))

    def _hiz_hesapla(self, hedef: _KoniHedef) -> float:
        if hedef.mesafe_m is not None:
            if hedef.mesafe_m > DIST_UZAK_M:
                return _HIZ["uzak"]
            if hedef.mesafe_m > DIST_ORTA_M:
                return _HIZ["orta"]
            return _HIZ["yakin"]
        if hedef.yakinlik < 0.5:
            return _HIZ["uzak"]
        if hedef.yakinlik < 0.8:
            return _HIZ["orta"]
        return _HIZ["yakin"]

    def _gecis_kontrol(self, mevcut_yakinlik: float) -> None:
        if self._onceki_yakinlik >= GECIS_YAKINLIK_ESIK and mevcut_yakinlik < 0.5:
            self._gecilen_koni += 1
            self._s_weave_sign *= -1.0
        self._onceki_yakinlik = mevcut_yakinlik

    def _koni_yok_isle(self) -> SlalomState:
        if self._faz == "bekleme":
            return SlalomState(faz="bekleme", sebep="koni tespit edilmedi")

        self._bitti_sayac += 1
        if self._bitti_sayac >= BITTI_FRAME_ESIK:
            gecilen = self._gecilen_koni
            self._reset()
            return SlalomState(
                faz="bitti",
                gecilen_koni=gecilen,
                sebep=f"{BITTI_FRAME_ESIK} frame boyunca koni görülmedi — manevra tamamlandı",
            )

        return SlalomState(
            aktif=True,
            steering=0.0,
            hiz_katsayisi=_HIZ["orta"],
            gecilen_koni=self._gecilen_koni,
            faz="aktif",
            sebep=f"geçici koni kaybı ({self._bitti_sayac}/{BITTI_FRAME_ESIK})",
        )

    def _reset(self) -> None:
        self._faz = "bekleme"
        self._son_gecis_taraf = VARSAYILAN_TARAF
        self._gecilen_koni = 0
        self._bitti_sayac = 0
        self._onceki_yakinlik = 0.0
        self._s_weave_sign = 1.0
        self._tek_taraf_sayac = 0

