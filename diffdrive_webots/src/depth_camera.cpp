#include "diffdrive_webots/depth_camera.hpp"
#include <sensor_msgs/image_encodings.hpp>

using std::placeholders::_1;

namespace diffdrive_webots_plugin
{
  void DepthCamera::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // initialize member variables
    node_ = node;
    robot_ = node->robot();
    depth_camera_ = NULL;

    // retrieve tags
    if (parameters.count("depthCameraPeriodMs"))
      depth_camera_period_ = std::stoi(parameters["depthCameraPeriodMs"]);
    else
      throw std::runtime_error("Must set depthCameraPeriodMs tag");

    std::string depth_camera_name;
    if (parameters.count("depthCameraName"))
      depth_camera_name = parameters["depthCameraName"];
    else
      throw std::runtime_error("Must set depthCameraName tag");
    
    std::string frame_id;
    if (parameters.count("frameID"))
      frame_id = parameters["frameID"];
    else
      throw std::runtime_error("Must set frameID tag");

    // set webots device
    depth_camera_ = robot_->getRangeFinder(depth_camera_name);
    if (depth_camera_ == NULL)
      throw std::runtime_error("Cannot find DepthCamera with name " + depth_camera_name);
    
    
    int timestep = (int)robot_->getBasicTimeStep();
    if (depth_camera_period_ % timestep != 0)
      throw std::runtime_error("depthCameraPeriodMs must be integer multiple of basicTimeStep");

    depth_camera_->enable(depth_camera_period_);

    depth_image_.header.frame_id = frame_id;
    depth_image_.width = depth_camera_->getWidth();
    depth_image_.height = depth_camera_->getHeight();
    depth_image_.is_bigendian = false;
    depth_image_.step = 4 * depth_camera_->getWidth();
    depth_image_.data.resize(4 * depth_camera_->getWidth() * depth_camera_->getHeight());
    depth_image_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    depth_image_pub_ = node->create_publisher<sensor_msgs::msg::Image>("depth_image", rclcpp::SensorDataQoS().reliable());
  }

  void DepthCamera::step()
  {
    int64_t sim_time = (int64_t)(robot_->getTime() * 1e3);

    if (sim_time % depth_camera_period_ == 0)
    {
      depth_image_.header.stamp = node_->get_clock()->now();
      memcpy(depth_image_.data.data(), depth_camera_->getRangeImage(), depth_image_.data.size());
      depth_image_pub_->publish(depth_image_);
    }

  }

}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots_plugin::DepthCamera, webots_ros2_driver::PluginInterface)