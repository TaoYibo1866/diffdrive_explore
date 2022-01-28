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
    cx_ = 0;
    cy_ = 0;
    fx_ = 0;
    fy_ = 0;

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
    
    int width = depth_camera_->getWidth();
    int height = depth_camera_->getHeight();
    float hfov = depth_camera_->getFov(); 
    cx_ = width / 2.0;
    cy_ = height / 2.0;
    fx_ = 0.5 * width * (1 / tan(0.5 * hfov));
    fy_ = fx_;
    point_cloud_.header.frame_id = frame_id;
    point_cloud_.width = width;
    point_cloud_.height = height;
    point_cloud_.fields.resize(3);
    point_cloud_.fields[0].name = "x";
    point_cloud_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_.fields[0].count = 1;
    point_cloud_.fields[0].offset = 0;
    point_cloud_.fields[1].name = "y";
    point_cloud_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_.fields[1].count = 1;
    point_cloud_.fields[1].offset = 4;
    point_cloud_.fields[2].name = "z";
    point_cloud_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_.fields[2].count = 1;
    point_cloud_.fields[2].offset = 8;
    point_cloud_.is_bigendian = false;
    point_cloud_.point_step = 3 * sizeof(float);
    point_cloud_.row_step = width * 3 * sizeof(float);
    point_cloud_.data.resize(width * height * 3 * sizeof(float));
    point_cloud_.is_dense = false;
    point_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SensorDataQoS().reliable());
  }

  void DepthCamera::step()
  {
    int64_t sim_time = (int64_t)(robot_->getTime() * 1e3);
    if (sim_time % depth_camera_period_ == 0)
    {
      point_cloud_.header.stamp = node_->get_clock()->now();
      int width = depth_camera_->getWidth();
      int height = depth_camera_->getHeight();
      const float* range_image = depth_camera_->getRangeImage();
      float* data = (float*)point_cloud_.data.data();
      for (int j = 0; j < height; j++)
      {
        for (int i = 0; i < width; i++)
        {
          int idx = j * width + i;
          float x = range_image[idx];
          float y = -(i - cx_) * x / fx_;
          float z = -(j - cy_) * x / fy_;
          memcpy(data + idx * 3    , &x, sizeof(float));
          memcpy(data + idx * 3 + 1, &y, sizeof(float));
          memcpy(data + idx * 3 + 2, &z, sizeof(float));
        }
      }
      point_cloud_pub_->publish(point_cloud_);
    }

  }

}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots_plugin::DepthCamera, webots_ros2_driver::PluginInterface)