#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using sensor_msgs::msg::JointState;
using std::placeholders::_1;

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node(node_name, options)
  {
    joint_state_sub_ = this->create_subscription<JointState>(
      "joint_states",
      rclcpp::SensorDataQoS().reliable(),
      std::bind(&WheelOdometryNode::jointStateCallback, this, _1)
    );
  }
private:
  void jointStateCallback(const JointState::SharedPtr msg)
  {
    return;
  }
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryNode>("wheel_odometry"));
  rclcpp::shutdown();
  return 0;
}