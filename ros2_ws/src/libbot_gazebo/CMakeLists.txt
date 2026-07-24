cmake_minimum_required(VERSION 3.8)
project(libbot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch models config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/strip_urdf_for_gzcontrol.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
