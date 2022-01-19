#ifndef DIFFDRIVE_WEBOTS_DIFFERENTIAL_CONTROL_HPP
#define DIFFDRIVE_WEBOTS_DIFFERENTIAL_CONTROL_HPP

#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace diffdrive_webots_plugin
{
  class DifferentialControl : public webots_ros2_driver::PluginInterface
  {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
  private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    webots_ros2_driver::WebotsNode *node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    webots::Supervisor *robot_;
    webots::Motor *left_motor_;
    webots::Motor *right_motor_;

    double wheel_radius_;
    double half_distance_between_wheels_;
    double forward_speed_;
    double angular_speed_;
  };
}

#endif