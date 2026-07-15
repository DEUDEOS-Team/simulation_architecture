cmake_minimum_required(VERSION 3.8)
project(araba)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  meshes
  missions
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  scripts/onnx
  DESTINATION share/${PROJECT_NAME}/models/
)

install(DIRECTORY
  scripts/deos_algorithms
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/sensors/
  DESTINATION lib/${PROJECT_NAME}/sensors
  FILES_MATCHING PATTERN "*.py"
)

# ── ROS wrapper nodeları (algı + kontrol altyapısı) ──
install(PROGRAMS
  scripts/camera_perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_perception_node
)

install(PROGRAMS
  scripts/lane_perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lane_perception_node
)

install(PROGRAMS
  scripts/perception_pipeline_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_pipeline_node
)

install(PROGRAMS
  scripts/speed_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME speed_controller_node
)

install(PROGRAMS
  scripts/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_teleop
)

# NOT: DENEME node'ları — RENAME YOK. Böylece --symlink-install gerçek symlink kurar ve
# kod değişiklikleri yeniden derleme olmadan anında geçerli olur.
# Çalıştırma: ros2 run araba <ad>.py  (launch'ta executable='<ad>.py')
install(PROGRAMS
  scripts/lane_test_node.py
  scripts/lane_yolo_node.py
  scripts/autonomous_control_node.py
  scripts/lidar_obstacle_node.py
  scripts/avoidance_node.py
  scripts/final_odom_node.py
  scripts/mission_planning_node.py
  scripts/vehicle_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Algoritma modülleri (pure Python, import edilebilir) ──
install(PROGRAMS
  scripts/geo_utils.py
  scripts/ros_topic_layout.py
  scripts/safety_logic.py
  scripts/obstacle_logic.py
  scripts/gap_avoid_logic.py
  scripts/traffic_light_logic.py
  scripts/traffic_sign_logic.py
  scripts/parking_logic.py
  scripts/slalom_logic.py
  scripts/decision_arbiter.py
  scripts/perception_fusion.py
  scripts/lane_violation.py
  scripts/geojson_mission_reader.py
  scripts/route_graph.py
  scripts/route_planner.py
  scripts/waypoint_manager.py
  scripts/mission_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
