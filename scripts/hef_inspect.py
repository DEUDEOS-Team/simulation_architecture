#!/usr/bin/env python3
"""
HEF model çıktı katmanlarını dökme aracı.

decode_yolov8_seg_tensors içindeki SABİT katman isimleri (conv44/45/46 ...) yalnızca
belirli bir derlenmiş modele aittir. Bu betik, eldeki HEF'in GERÇEK giriş/çıkış
vstream'lerini (isim + şekil) yazar; böylece her katmanın rolünü kanal sayısından
doğru eşleyebiliriz:

  • cls (sınıf skoru) katmanı  -> kanal = sınıf sayısı (örn. 2)
  • bbox (DFL) katmanı         -> kanal = 64  (4 kenar × 16 bin)
  • mask-coef katmanı          -> kanal = 32
  • proto (prototip) katmanı   -> kanal = 32, ama EN BÜYÜK uzaysal boyut (örn. 160×160)

ÇALIŞTIRMA (araçta):
    python3 hef_inspect.py            # varsayılan: model.hef
    python3 hef_inspect.py yol.hef    # başka dosya
"""

import sys
from hailo_platform import HEF

HEF_PATH = sys.argv[1] if len(sys.argv) > 1 else "model.hef"


def main():
    hef = HEF(HEF_PATH)
    print(f"=== HEF: {HEF_PATH} ===\n")

    print("--- GİRİŞ (input) vstream'leri ---")
    for vi in hef.get_input_vstream_infos():
        print(f"  IN   name={vi.name!r}  shape={tuple(vi.shape)}")

    print("\n--- ÇIKIŞ (output) vstream'leri ---")
    print("  (şekil genelde (H, W, C) — NHWC; C = kanal sayısı = rolün anahtarı)\n")
    for vo in hef.get_output_vstream_infos():
        shape = tuple(vo.shape)
        c = shape[-1] if len(shape) >= 1 else None
        # Kanal sayısından rol tahmini
        if c == 64:
            role = "bbox (DFL, 4x16)"
        elif c == 32:
            role = "mask-coef VEYA proto (uzaysal en büyük olan = proto)"
        elif c is not None and c <= 16:
            role = f"cls? (kanal={c} -> sınıf sayısı olabilir)"
        else:
            role = "?"
        print(f"  OUT  name={vo.name!r}  shape={shape}  -> {role}")

    print("\nNot: cls katmanlarının kanalı = SINIF SAYISI. 'her yer yol' sorununda")
    print("cls sanılan katmanın aslında 64-kanallı bbox olması en olası nedendir.")


if __name__ == "__main__":
    main()
