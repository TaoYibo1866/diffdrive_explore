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
    range_finder_ = NULL;

    // retrieve tags
    if (parameters.count("depthCameraPeriodMs"))
      period_ = std::stoi(parameters["depthCameraPeriodMs"]);
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
    range_finder_ = robot_->getRangeFinder(depth_camera_name);
    if (range_finder_ == NULL)
      throw std::runtime_error("Cannot find DepthCamera with name " + depth_camera_name);
    
    
    int timestep = (int)robot_->getBasicTimeStep();
    if (period_ % timestep != 0)
      throw std::runtime_error("depthCameraPeriodMs must be integer multiple of basicTimeStep");

    range_finder_->enable(period_);
    
    // CameraInfo publisher
    rclcpp::QoS camera_info_qos(1);
    camera_info_qos.reliable();
    camera_info_qos.transient_local();
    camera_info_qos.keep_last(1);
    camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", camera_info_qos);
    camera_info_msg_.header.stamp = node_->get_clock()->now();
    camera_info_msg_.header.frame_id = frame_id;
    camera_info_msg_.height = range_finder_->getHeight();
    camera_info_msg_.width = range_finder_->getWidth();
    camera_info_msg_.distortion_model = "plumb_bob";
    const double focal_length = 0.5 * range_finder_->getWidth() * (1 / tan(0.5 * range_finder_->getFov()));
    camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info_msg_.k = {
        focal_length, 0.0, (double)range_finder_->getWidth() / 2,
        0.0, focal_length, (double)range_finder_->getHeight() / 2,
        0.0, 0.0, 1.0};
    camera_info_msg_.p = {
        focal_length, 0.0, (double)range_finder_->getWidth() / 2, 0.0,
        0.0, focal_length, (double)range_finder_->getHeight() / 2, 0.0,
        0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(camera_info_msg_);

    // PointCloud publisher
    point_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SensorDataQoS().reliable());
    point_cloud_msg_.header.frame_id = frame_id;
    point_cloud_msg_.fields.resize(3);
    point_cloud_msg_.fields[0].name = "x";
    point_cloud_msg_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg_.fields[0].count = 1;
    point_cloud_msg_.fields[0].offset = 0;
    point_cloud_msg_.fields[1].name = "y";
    point_cloud_msg_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg_.fields[1].count = 1;
    point_cloud_msg_.fields[1].offset = 4;
    point_cloud_msg_.fields[2].name = "z";
    point_cloud_msg_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg_.fields[2].count = 1;
    point_cloud_msg_.fields[2].offset = 8;
    point_cloud_msg_.is_bigendian = false;
    point_cloud_msg_.is_dense = false;
    const int width = range_finder_->getWidth();
    const int height = range_finder_->getHeight();
    point_cloud_msg_.width = width;
    point_cloud_msg_.height = height;
    point_cloud_msg_.point_step = 3 * sizeof(float);
    point_cloud_msg_.row_step = width * 3 * sizeof(float);
    point_cloud_msg_.data.resize(width * height * 3 * sizeof(float));
  }

  void DepthCamera::step()
  {
    int64_t sim_time = (int64_t)(robot_->getTime() * 1e3);

    if (sim_time % period_ == 0)
    {
      auto image = range_finder_->getRangeImage();
      if (image)
      {
        point_cloud_msg_.header.stamp = node_->get_clock()->now();

        const int width = camera_info_msg_.width;
        const int height = camera_info_msg_.height;
        const float cx = camera_info_msg_.k[2];
        const float cy = camera_info_msg_.k[5];
        const float fx = camera_info_msg_.k[0];
        const float fy = camera_info_msg_.k[4];

        float* data = (float*)point_cloud_msg_.data.data();
        for (int j = 0; j < height; j++)
        {
          for (int i = 0; i < width; i++)
          {
            int idx = j * width + i;
            float x = image[idx];
            float y = -(i - cx) * x / fx;
            float z = -(j - cy) * x / fy;
            memcpy(data + idx * 3    , &x, sizeof(float));
            memcpy(data + idx * 3 + 1, &y, sizeof(float));
            memcpy(data + idx * 3 + 2, &z, sizeof(float));
          }
        }
        point_cloud_pub_->publish(point_cloud_msg_);
      }
    }

  }

}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots_plugin::DepthCamera, webots_ros2_driver::PluginInterface)