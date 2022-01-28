#include "diffdrive_webots/camera.hpp"
#include <sensor_msgs/image_encodings.hpp>

using std::placeholders::_1;

namespace diffdrive_webots_plugin
{
  void Camera::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // initialize member variables
    node_ = node;
    robot_ = node->robot();
    camera_ = NULL;

    // retrieve tags
    if (parameters.count("cameraPeriodMs"))
      period_ = std::stoi(parameters["cameraPeriodMs"]);
    else
      throw std::runtime_error("Must set cameraPeriodMs tag");

    std::string camera_name;
    if (parameters.count("cameraName"))
      camera_name = parameters["cameraName"];
    else
      throw std::runtime_error("Must set cameraName tag");
    
    std::string frame_id;
    if (parameters.count("frameID"))
      frame_id = parameters["frameID"];
    else
      throw std::runtime_error("Must set frameID tag");

    // set webots device
    camera_ = robot_->getCamera(camera_name);
    if (camera_ == NULL)
      throw std::runtime_error("Cannot find Camera with name " + camera_name);
    
    
    int timestep = (int)robot_->getBasicTimeStep();
    if (period_ % timestep != 0)
      throw std::runtime_error("cameraPeriodMs must be integer multiple of basicTimeStep");

    camera_->enable(period_);

    // CameraInfo publisher
    rclcpp::QoS camera_info_qos(1);
    camera_info_qos.reliable();
    camera_info_qos.transient_local();
    camera_info_qos.keep_last(1);
    camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", camera_info_qos);
    camera_info_msg_.header.stamp = node_->get_clock()->now();
    camera_info_msg_.header.frame_id = frame_id;
    camera_info_msg_.height = camera_->getHeight();
    camera_info_msg_.width = camera_->getWidth();
    camera_info_msg_.distortion_model = "plumb_bob";
    const double focal_length = 0.5 * camera_->getWidth() * (1 / tan(0.5 * camera_->getFov()));
    camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info_msg_.k = {
        focal_length, 0.0, (double)camera_->getWidth() / 2,
        0.0, focal_length, (double)camera_->getHeight() / 2,
        0.0, 0.0, 1.0};
    camera_info_msg_.p = {
        focal_length, 0.0, (double)camera_->getWidth() / 2, 0.0,
        0.0, focal_length, (double)camera_->getHeight() / 2, 0.0,
        0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(camera_info_msg_);

    // Image publisher
    image_pub_ = node->create_publisher<sensor_msgs::msg::Image>("image", rclcpp::SensorDataQoS().reliable());
    image_msg_.header.frame_id = frame_id;
    image_msg_.is_bigendian = false;
    image_msg_.encoding = sensor_msgs::image_encodings::BGRA8;
    image_msg_.width = camera_->getWidth();
    image_msg_.height = camera_->getHeight();
    image_msg_.step = 4 * camera_->getWidth();
    image_msg_.data.resize(4 * camera_->getWidth() * camera_->getHeight());

  }

  void Camera::step()
  {
    int64_t sim_time = (int64_t)(robot_->getTime() * 1e3);

    if (sim_time % period_ == 0)
    {
      auto image = camera_->getImage();
      if (image)
      {
        image_msg_.header.stamp = node_->get_clock()->now();
        memcpy(image_msg_.data.data(), image, image_msg_.data.size() * sizeof(uint8_t));
        image_pub_->publish(image_msg_);
      }
    }

  }

}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots_plugin::Camera, webots_ros2_driver::PluginInterface)