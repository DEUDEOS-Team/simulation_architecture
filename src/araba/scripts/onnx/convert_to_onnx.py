#!/usr/bin/env python3
"""
smallmodel.pt dosyasını ONNX formatına çevirir.
Kullanım: python3 convert_to_onnx.py
"""
import os
from ultralytics import YOLO

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PT_PATH   = os.path.join(SCRIPT_DIR, 'smallmodel.pt')
ONNX_PATH = os.path.join(SCRIPT_DIR, 'model.onnx')

model = YOLO(PT_PATH)
model.export(format='onnx', imgsz=640, opset=17, dynamic=False)

# ultralytics çıktı adını <isim>.onnx olarak oluşturur, rename edelim
exported = PT_PATH.replace('.pt', '.onnx')
if exported != ONNX_PATH and os.path.exists(exported):
    os.rename(exported, ONNX_PATH)

print(f'Dönüşüm tamamlandı: {ONNX_PATH}')
