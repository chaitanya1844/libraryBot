cmake_minimum_required(VERSION 3.8)
project(libbot_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AttachLink.srv"
  "srv/DetachLink.srv"
)

# rosidl_generate_interfaces only produces a target name in newer distros
# via this helper; Humble exposes the C++ typesupport target as
# "${PROJECT_NAME}__rosidl_typesupport_cpp".
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(libbot_link_attacher SHARED
  src/link_attacher_plugin.cpp
)
target_include_directories(libbot_link_attacher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(libbot_link_attacher
  gazebo_dev
  gazebo_ros
  rclcpp
)
target_link_libraries(libbot_link_attacher "${cpp_typesupport_target}")

install(TARGETS libbot_link_attacher
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(libbot_link_attacher)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
