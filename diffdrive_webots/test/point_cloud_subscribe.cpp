#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;

rclcpp::Node::SharedPtr nh;

void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(nh->get_logger(), "got it, %ld", msg->data.size());
  float* data = (float*)msg->data.data();
  for (int i = 0; i < msg->width * msg->height; i++)
  {
    float x = data[i * 3];
    float y = data[i * 3 + 1];
    float z = data[i * 3 + 2];
    RCLCPP_INFO(nh->get_logger(), "x=%f, y=%f, z=%f", x, y, z);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  nh = std::make_shared<rclcpp::Node>("point_cloud_subscriber");
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub
   = nh->create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SensorDataQoS().reliable(), pointCloudCallback);
  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}