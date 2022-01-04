#ifndef WHEEL_ENCODER_HPP
#define WHEEL_ENCODER_HPP

#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace diffdrive_webots_plugin
{
  class WheelEncoder : public webots_ros2_driver::PluginInterface
  {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
  private:
    webots_ros2_driver::WebotsNode *node_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr wheel_speed_pub_;
    
    webots::Supervisor *robot_;
    webots::PositionSensor *left_encoder_;
    webots::PositionSensor *right_encoder_;

    int encoder_period_;
    double prev_left_angle_;
    double prev_right_angle;
  };
}

#endif