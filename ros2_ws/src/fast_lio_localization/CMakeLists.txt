cmake_minimum_required(VERSION 3.8)
project(fast_lio_localization)

find_package(ament_cmake REQUIRED)

# Only global_localization.py (the actual ICP relocalization-against-a-
# saved-PCD engine) and publish_initial_pose.py (a CLI alternative to
# RViz's "2D Pose Estimate") are used here -- this package originally
# also bundled its own full second copy of FAST-LIO2 (src/laserMapping.cpp,
# needing livox_ros_driver2 + PCL + a generated Pose6D.msg) and
# transform_fusion.py (broadcasts map->odom directly, conflated with this
# project's own odom frame in a way that doesn't fit how
# fastlio_map_odom_bridge.py already separates FAST-LIO2's frames from
# the EKF's) -- both dropped. libbot_localization's bridge subscribes to
# global_localization.py's /map_to_odom output directly instead of
# running its own (less robust) single-scale ICP.
install(PROGRAMS
  fast_lio_localization/global_localization.py
  fast_lio_localization/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
