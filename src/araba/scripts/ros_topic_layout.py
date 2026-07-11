"""
DEOS tutarlı ROS 2 topic hiyerarşisi.

Tüm yollar ``{deos_root}/...`` şeklindedir (varsayılan ``deos_root=/deos``).

Özet ağaç::

    {root}/sensors/camera/...
    {root}/sensors/imu/data
    {root}/sensors/gps/fix | filtered
    {root}/sensors/lidar/points_downsampled

    {root}/perception/stereo/detections
    {root}/perception/lidar/obstacles
    {root}/perception/lane/...
    {root}/perception/fusion/...

    {root}/planning/...

    {root}/lane/...

    {root}/localization/odom/ekf|gps|icp|final

    {root}/hardware/motion_enable | autonomy_enable

    {root}/control/cmd_vel

    {root}/safety/...

    {root}/actuators/stm32/...

    {root}/failsafe/out/... | in/fsm_reset
"""


def _j(root: str, *segments: str) -> str:
    r = root.strip().rstrip("/")
    return "/".join((r,) + segments)


def build_deos_topics(deos_root: str) -> dict[str, str]:
    """Tek kök altında tüm graf topic tam yolları."""
    r = deos_root.strip().rstrip("/")
    j = lambda *p: _j(r, *p)
    return {
        "sensors_camera_color": j("sensors", "camera", "color", "image_raw"),
        "sensors_camera_depth": j("sensors", "camera", "depth", "image_raw"),
        "sensors_camera_color_info": j("sensors", "camera", "color", "camera_info"),
        "sensors_camera_depth_info": j("sensors", "camera", "depth", "camera_info"),
        "sensors_imu": j("sensors", "imu", "data"),
        "sensors_gps_fix": j("sensors", "gps", "fix"),
        "sensors_gps_filtered": j("sensors", "gps", "filtered"),
        "sensors_lidar_points_downsampled": j("sensors", "lidar", "points_downsampled"),
        "sensors_lidar_cloud_unstructured_fullframe": j("sensors", "lidar", "cloud_unstructured_fullframe"),
        "perception_stereo_detections": j("perception", "stereo", "detections"),
        "perception_lidar_obstacles": j("perception", "lidar", "obstacles"),
        "perception_lane_center_pts": j("perception", "lane", "center_pts"),
        "perception_lane_debug": j("perception", "lane", "debug"),
        "perception_lane_walls": j("perception", "lane", "walls"),
        "perception_fusion_emergency_stop": j("perception", "fusion", "emergency_stop"),
        "perception_fusion_speed_cap": j("perception", "fusion", "speed_cap"),
        "perception_fusion_steering_override": j("perception", "fusion", "steering_override"),
        "perception_fusion_has_steering_override": j("perception", "fusion", "has_steering_override"),
        "perception_fusion_park_complete": j("perception", "fusion", "park_complete"),
        "perception_fusion_turn_permissions": j("perception", "fusion", "turn_permissions"),
        "perception_fusion_decision_debug": j("perception", "fusion", "decision_debug"),
        "perception_fusion_green_elapsed_s": j("perception", "fusion", "green_elapsed_s"),
        "planning_steering_ref": j("planning", "steering_ref"),
        "planning_speed_limit": j("planning", "speed_limit"),
        "planning_current_task": j("planning", "current_task"),
        "planning_arrived": j("planning", "arrived"),
        "planning_park_mode": j("planning", "park_mode"),
        "planning_park_remaining_s": j("planning", "park_remaining_s"),
        "lane_steering_ref": j("lane", "steering_ref"),
        "lane_speed_limit": j("lane", "speed_limit"),
        "localization_odom_gps": j("localization", "odom", "gps"),
        "localization_odom_ekf": j("localization", "odom", "ekf"),
        # pcl_localization_ros2 varsayılanı hâlâ kökte; ileride remap ile deos altına alınabilir
        "localization_odom_icp": "/odometry/icp",
        "control_intent": j("control", "intent"),
        "localization_odom_final": j("localization", "odom", "final"),
        "hardware_motion_enable": j("hardware", "motion_enable"),
        "hardware_autonomy_enable": j("hardware", "autonomy_enable"),
        "control_cmd_vel": j("control", "cmd_vel"),
        "safety_emergency_stop": j("safety", "emergency_stop"),
        "safety_lane_violation": j("safety", "lane", "violation"),
        "safety_lane_violation_count": j("safety", "lane", "violation_count"),
        "safety_lane_violation_seconds": j("safety", "lane", "violation_seconds"),
        "actuators_stm32_speed_delta_mps": j("actuators", "stm32", "speed_delta_mps"),
        "actuators_stm32_speed_target_mps": j("actuators", "stm32", "speed_target_mps"),
        "actuators_stm32_steering_deg": j("actuators", "stm32", "steering_deg"),
        "failsafe_out_emergency_stop": j("failsafe", "out", "emergency_stop"),
        "failsafe_out_speed_cap": j("failsafe", "out", "speed_cap"),
        "failsafe_out_diagnostics": j("failsafe", "out", "diagnostics"),
        "failsafe_in_fsm_reset": j("failsafe", "in", "fsm_reset"),
    }
