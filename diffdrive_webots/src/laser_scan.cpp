#include "diffdrive_webots/laser_scan.hpp"
#include <sensor_msgs/image_encodings.hpp>

using std::placeholders::_1;

namespace diffdrive_webots_plugin
{
  void LaserScan::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // initialize member variables
    node_ = node;
    robot_ = node->robot();
    lidar_ = NULL;

    // retrieve tags
    if (parameters.count("laserScanPeriodMs"))
      period_ = std::stoi(parameters["laserScanPeriodMs"]);
    else
      throw std::runtime_error("Must set laserScanPeriodMs tag");

    std::string laser_scan_name;
    if (parameters.count("laserScanName"))
      laser_scan_name = parameters["laserScanName"];
    else
      throw std::runtime_error("Must set laserScanName tag");
    
    std::string frame_id;
    if (parameters.count("frameID"))
      frame_id = parameters["frameID"];
    else
      throw std::runtime_error("Must set frameID tag");

    // set webots device
    lidar_ = robot_->getLidar(laser_scan_name);
    if (lidar_ == NULL)
      throw std::runtime_error("Cannot find laserScan with name " + laser_scan_name);
    
    
    int timestep = (int)robot_->getBasicTimeStep();
    if (period_ % timestep != 0)
      throw std::runtime_error("laserScanPeriodMs must be integer multiple of basicTimeStep");

    lidar_->enable(period_);

    // LaserScan publisher
    scan_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS().reliable());
    scan_msg_.header.frame_id = frame_id;
    scan_msg_.scan_time = (double)period_ / 1e3;
    const int resolution = lidar_->getHorizontalResolution();
    scan_msg_.angle_increment = -lidar_->getFov() / resolution;
    scan_msg_.time_increment = (double)period_ / (1e3 * resolution);
    scan_msg_.angle_min = lidar_->getFov() / 2.0 - scan_msg_.angle_increment;
    scan_msg_.angle_max =-lidar_->getFov() / 2.0;
    scan_msg_.ranges.resize(resolution);

  }

  void LaserScan::step()
  {
    int64_t sim_time = (int64_t)(robot_->getTime() * 1e3);

    if (sim_time % period_ == 0)
    {
      auto scan = lidar_->getLayerRangeImage(0);
      if (scan)
      {
        scan_msg_.header.stamp = node_->get_clock()->now();
        scan_msg_.range_min = lidar_->getMinRange();
        scan_msg_.range_max = lidar_->getMaxRange();
        memcpy(scan_msg_.ranges.data(), scan, scan_msg_.ranges.size() * sizeof(float));
        scan_pub_->publish(scan_msg_);
      }
    }

  }

}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots_plugin::LaserScan, webots_ros2_driver::PluginInterface)