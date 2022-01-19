#ifndef DIFFDRIVE_WEBOTS_DEPTH_CAMERA_HPP
#define DIFFDRIVE_WEBOTS_DEPTH_CAMERA_HPP

#include <webots/Supervisor.hpp>
#include <webots/RangeFinder.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace diffdrive_webots_plugin
{
  class DepthCamera : public webots_ros2_driver::PluginInterface
  {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
  private:
    webots_ros2_driver::WebotsNode *node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    sensor_msgs::msg::Image depth_image_;

    webots::Supervisor *robot_;
    webots::RangeFinder *depth_camera_;

    int depth_camera_period_;
  };
}

#endif