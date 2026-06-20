cmake_minimum_required(VERSION 3.8)
project(araba_v2)

# Ament CMake
find_package(ament_cmake REQUIRED)

# URDF, mesh, launch, config dosyalarını kur
install(DIRECTORY config launch meshes textures urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
