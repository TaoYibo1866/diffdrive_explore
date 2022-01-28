#ifndef DIFFDRIVE_WEBOTS_DEPTH_CAMERA_HPP
#define DIFFDRIVE_WEBOTS_DEPTH_CAMERA_HPP

#include <webots/Supervisor.hpp>
#include <webots/RangeFinder.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    sensor_msgs::msg::PointCloud2 point_cloud_msg_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    webots::Supervisor *robot_;
    webots::RangeFinder *range_finder_;

    int period_;
  };
}

#endif