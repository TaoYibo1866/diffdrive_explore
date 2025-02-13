cmake_minimum_required(VERSION 3.5)
project(diffdrive_webots)

set(CMAKE_BUILD_TYPE "Release")

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# plugins
add_library(
  diffdrive_webots_plugin
  SHARED
  src/differential_control.cpp
  src/wheel_encoder.cpp
  src/imu.cpp
  src/depth_camera.cpp
  src/camera.cpp
  src/laser_scan.cpp
)
target_include_directories(
  diffdrive_webots_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  diffdrive_webots_plugin
  pluginlib
  rclcpp
  webots_ros2_driver
  geometry_msgs
  sensor_msgs
)

pluginlib_export_plugin_description_file(webots_ros2_driver plugins.xml)

install(TARGETS diffdrive_webots_plugin DESTINATION lib)

ament_export_include_directories(
  include
)
ament_export_libraries(
  diffdrive_webots_plugin
)
# plugins end

add_executable(point_cloud_subscribe test/point_cloud_subscribe.cpp)
ament_target_dependencies(
  point_cloud_subscribe
  rclcpp 
  sensor_msgs
)

install(TARGETS point_cloud_subscribe DESTINATION lib/${PROJECT_NAME})

# install regular files
install(
  DIRECTORY launch protos resource worlds
  DESTINATION share/${PROJECT_NAME}
)
# install regular files end

ament_package()
