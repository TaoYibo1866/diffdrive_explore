#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

using sensor_msgs::msg::Joy;
using geometry_msgs::msg::Twist;
using std::placeholders::_1;

class JoyControlNode : public rclcpp::Node
{
public:
  JoyControlNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node(node_name, options)
  {
    joy_sub_ = this->create_subscription<Joy>(
      "joy", 
      rclcpp::SensorDataQoS().reliable(),
      std::bind(&JoyControlNode::joyCallback, this, _1)
    );

    cmd_pub_ = this->create_publisher<Twist>(
      "cmd_vel",
      rclcpp::SensorDataQoS().reliable()
    );
  }

private:
  void joyCallback(const Joy::SharedPtr msg)
  {
    double forward_speed = 0;
    double angular_speed = 0;

    if (msg->axes.size() != 6)
    {
      RCLCPP_WARN(this->get_logger(), "msg->axes.size() != 6");
    }
    else
    {
      forward_speed = 2 * msg->axes[3];
      angular_speed = 90 * msg->axes[2] * M_PI / 180.0;
    }
    
    Twist cmd_vel;
    cmd_vel.linear.set__x(forward_speed);
    cmd_vel.angular.set__z(angular_speed);
    cmd_pub_->publish(cmd_vel);
  }
  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyControlNode>("joy_control"));
  rclcpp::shutdown();
  return 0;
}