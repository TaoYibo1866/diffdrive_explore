#ifndef DIFFDRIVE_WEBOTS_CAMERA_HPP
#define DIFFDRIVE_WEBOTS_CAMERA_HPP

#include <webots/Supervisor.hpp>
#include <webots/Camera.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace diffdrive_webots_plugin
{
  class Camera : public webots_ros2_driver::PluginInterface
  {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
  private:
    webots_ros2_driver::WebotsNode *node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    sensor_msgs::msg::Image image_msg_;

    webots::Supervisor *robot_;
    webots::Camera *camera_;

    int camera_period_;
  };
}

#endif