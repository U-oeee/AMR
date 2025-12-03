-- my_robot_2d.lua
-- RB-35GM + RPi4 + ROS2 Humble + YDLIDAR + wheel_odometry(odom.py) 용 Cartographer 2D 설정

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- === frame 설정 ===
  map_frame = "map",

  -- odom.py에서 이미 IMU까지 반영된 base_link를 쓰므로
  -- Cartographer의 tracking_frame은 base_link로 둔다.
  tracking_frame = "base_link",

  -- Cartographer가 publish하는 로봇 포즈의 child_frame
  published_frame = "base_link",

  -- odom.py에서 쓰는 odom frame과 일치시킨다.
  odom_frame = "odom",

  -- odom frame은 이미 odom.py가 제공하므로 Cartographer는 새로 만들지 않는다.
  provide_odom_frame = false,

  publish_frame_projected_to_2d = true,

  -- /odom 을 사용해서 wheel odom 정보를 활용
  use_odometry = true,

  -- GNSS, Landmarks는 사용하지 않음
  use_nav_sat = false,
  use_landmarks = false,

  -- LaserScan 1개 사용
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 2D Trajectory Builder 사용
MAP_BUILDER.use_trajectory_builder_2d = true

-- === Laser 설정 (YDLIDAR X4 Pro 기준 튜닝 예시) ===
-- ydlidar_ros2_driver에서 range_min=0.1, range_max=12.0 으로 설정했다고 가정
TRAJECTORY_BUILDER_2D.min_range = 0.10
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0

-- LaserScan을 몇 개 누적해서 한 번에 처리할지 (1이면 실시간성이 좋음)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 현재 구조에선 IMU yaw를 odom.py 내부에서 이미 사용하므로
-- Cartographer에는 IMU raw 데이터를 사용하지 않는다.
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 온라인 코릴레이티브 스캔 매칭: 초기 정합에 도움 (CPU 조금 더 사용)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 너무 작은 자세 변화는 무시해서 노이즈 필터링 (TB3 기본값과 비슷하게)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- === Local SLAM 스캔 매처 튜닝 (필요하면 조절) ===
-- 아래 값들은 일반적인 실내/소형 로봇 기준 예시 튜닝 값.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0

-- === Submap / Pose Graph 튜닝 (기본 TB3 값과 유사) ===
POSE_GRAPH.optimize_every_n_nodes = 35

POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

POSE_GRAPH.optimization_problem.huber_scale = 1e1

return options

