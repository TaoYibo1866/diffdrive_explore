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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# joy_control
add_executable(joy_control src/joy_control.cpp)
ament_target_dependencies(
  joy_control
  rclcpp 
  geometry_msgs
  sensor_msgs
)

install(TARGETS joy_control DESTINATION lib/${PROJECT_NAME})
# joy_control end

# diffdrive_driver
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
  geometry_msgs
  sensor_msgs
)

pluginlib_export_plugin_description_file(webots_ros2_driver plugins.xml)

install(TARGETS diffdrive_driver DESTINATION lib)

ament_export_include_directories(
  include
)
ament_export_libraries(
  diffdrive_driver
)
# diffdrive_driver end

# install regular files
install(FILES launch/robot_launch.py DESTINATION share/${PROJECT_NAME}/launch)
install(FILES protos/DiffDrive.proto DESTINATION share/${PROJECT_NAME}/protos)
install(FILES resource/diffdrive_webots.urdf DESTINATION share/${PROJECT_NAME}/resource)
install(FILES worlds/diffdrive_navigation.wbt DESTINATION share/${PROJECT_NAME}/worlds)
# install regular files end

ament_package()
