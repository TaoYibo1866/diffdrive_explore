cmake_minimum_required(VERSION 3.5)
project(diffdrive_webots)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

add_library(
  diffdrive_driver
  SHARED
  src/diffdrive_driver.cpp
)
target_include_directories(
  diffdrive_driver
  PRIVATE
  include
)
ament_target_dependencies(
  diffdrive_driver
  pluginlib
  rclcpp
  webots_ros2_driver
)

pluginlib_export_plugin_description_file(webots_ros2_driver plugins.xml)

install(TARGETS
  diffdrive_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  diffdrive_driver
)

install(FILES launch/robot_launch.py DESTINATION share/${PROJECT_NAME}/launch)
install(FILES protos/DiffDrive.proto DESTINATION share/${PROJECT_NAME}/protos)
install(FILES resource/diffdrive_webots.urdf DESTINATION share/${PROJECT_NAME}/resource)
install(FILES worlds/diffdrive_navigation.wbt DESTINATION share/${PROJECT_NAME}/worlds)

ament_package()
