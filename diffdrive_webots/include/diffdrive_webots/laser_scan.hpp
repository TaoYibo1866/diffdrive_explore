#ifndef DIFFDRIVE_WEBOTS_LASER_SCAN_HPP
#define DIFFDRIVE_WEBOTS_LASER_SCAN_HPP

#include <webots/Supervisor.hpp>
#include <webots/Lidar.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace diffdrive_webots_plugin
{
  class LaserScan : public webots_ros2_driver::PluginInterface
  {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
  private:
    webots_ros2_driver::WebotsNode *node_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    sensor_msgs::msg::LaserScan scan_msg_;

    webots::Supervisor *robot_;
    webots::Lidar *lidar_;

    int period_;
  };
}

#endif