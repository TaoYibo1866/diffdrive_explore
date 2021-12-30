#ifndef DIFFDRIVE_DRIVER_HPP
#define DIFFDRIVE_DRIVER_HPP

#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace diffdrive_webots
{
  class DiffDriveDriver : public webots_ros2_driver::PluginInterface
  {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
  private:
    webots_ros2_driver::WebotsNode *node_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr wheel_speed_pub_;
    geometry_msgs::msg::Twist cmd_vel_;
    
    webots::Supervisor *robot_;
    webots::PositionSensor *encoder1_;
    webots::PositionSensor *encoder2_;
    webots::Motor *motor1_;
    webots::Motor *motor2_;

    int encoder_period_;
    double prev_angle1_;
    double prev_angle2_;
    double wheel_radius_;
    double half_distance_between_wheels_;
  };
}

#endif