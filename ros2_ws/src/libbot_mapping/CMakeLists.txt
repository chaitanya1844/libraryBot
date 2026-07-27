cmake_minimum_required(VERSION 3.8)
project(libbot_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pcd_to_2d_map.py
  scripts/mapping_preview_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
