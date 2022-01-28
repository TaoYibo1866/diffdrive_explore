#ifndef DIFFDRIVE_WEBOTS_IMU_HPP
#define DIFFDRIVE_WEBOTS_IMU_HPP

#include <webots/Supervisor.hpp>

#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace diffdrive_webots_plugin
{
  class IMU : public webots_ros2_driver::PluginInterface
  {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
  private:
    webots_ros2_driver::WebotsNode *node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    sensor_msgs::msg::Imu imu_msg_;
    
    webots::Supervisor *robot_;
    webots::Accelerometer *accelerometer_;
    webots::Gyro *gyro_;
    webots::InertialUnit *inertial_unit_;

    int period_;
  };
}

#endif