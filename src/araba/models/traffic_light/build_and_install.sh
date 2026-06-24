#!/bin/bash

echo "Eski derleme dosyalari temizleniyor..."
rm -rf build
mkdir build
cd build

echo "CMake yapilandirmasi basliyor..."
cmake ..

echo "Eklenti derleniyor..."
make -j4

echo "Eklenti Gazebo'nun eklenti klasorune kopyalaniyor (~/.gz/sim/plugins)..."
mkdir -p ~/.gz/sim/plugins
cp libTrafficLightPlugin.so ~/.gz/sim/plugins/

echo "=========================================================="
echo " ✅ DERLEME VE KURULUM BASARIYLA TAMAMLANDI!"
echo " Lutfen su komutla simulasyonu baslatin: gz sim -r model.sdf"
echo "=========================================================="