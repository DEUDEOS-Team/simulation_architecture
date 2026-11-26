cmake_minimum_required(VERSION 3.16)
project(TrafficLightPlugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Gazebo Harmonic (8) kütüphanelerini bul
find_package(gz-sim8 REQUIRED)

# Eklenti kütüphanesini oluştur
add_library(TrafficLightPlugin SHARED TrafficLightPlugin.cc)

# Gazebo kütüphanelerini eklentimize bağla
target_link_libraries(TrafficLightPlugin
  PUBLIC
    gz-sim8::gz-sim8
)