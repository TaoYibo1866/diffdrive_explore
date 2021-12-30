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
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    webots_ros2_driver::WebotsNode *node_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr wheel_speed_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    webots::Supervisor *robot_;
    webots::PositionSensor *left_encoder_;
    webots::PositionSensor *right_encoder_;
    webots::Motor *left_motor_;
    webots::Motor *right_motor_;

    int encoder_period_;
    double prev_left_angle_;
    double prev_right_angle;
    double wheel_radius_;
    double half_distance_between_wheels_;
    double forward_speed_;
    double angular_speed_;
  };
}

#endif