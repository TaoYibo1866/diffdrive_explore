#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_broadcaster.h>
#include <cmath>

#define WHEEL_RADIUS 0.03
#define HALF_DISTANCE_BETWEEN_WHEELS 0.1
#define WHEEL_ENCODER_PERIOD 0.004

using sensor_msgs::msg::JointState;
using nav_msgs::msg::Odometry;
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

    odom_pub_ = this->create_publisher<Odometry>(
      "wheel_odometry",
      rclcpp::SensorDataQoS().reliable()
    );

    // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    position_x_ = 0;
    position_y_ = 0;
    attitude_yaw_ = 0;
  }
private:
  void jointStateCallback(const JointState::SharedPtr msg)
  {
    if (msg->velocity.size() != 2)
    {
      RCLCPP_WARN(this->get_logger(), "msg->velocity.size() != 2");
      return;
    }
    
    double left_wheel_speed = msg->velocity[0];
    double right_wheel_speed = msg->velocity[1];
    double forward_speed = (right_wheel_speed * WHEEL_RADIUS + left_wheel_speed * WHEEL_RADIUS) / 2;
    double angular_speed = (right_wheel_speed * WHEEL_RADIUS - left_wheel_speed * WHEEL_RADIUS) / 2 / HALF_DISTANCE_BETWEEN_WHEELS;
    double velocity_x = forward_speed * cos(attitude_yaw_);
    double velocity_y = forward_speed * sin(attitude_yaw_);
    position_x_ += velocity_x * WHEEL_ENCODER_PERIOD;
    position_y_ += velocity_y * WHEEL_ENCODER_PERIOD;
    attitude_yaw_ += angular_speed * WHEEL_ENCODER_PERIOD;

    Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.header.stamp = msg->header.stamp;
    odom.pose.pose.position
      .set__x(position_x_)
      .set__y(position_y_)
      .set__z(0.0);
    odom.pose.pose.orientation
      .set__x(0.0)
      .set__y(0.0)
      .set__z(sin(attitude_yaw_ / 2.0))
      .set__w(cos(attitude_yaw_ / 2.0));
    odom.twist.twist.linear
      .set__x(forward_speed)
      .set__y(0.0)
      .set__z(0.0);
    odom.twist.twist.angular
      .set__x(0.0)
      .set__y(0.0)
      .set__z(angular_speed);
    odom.pose.covariance =  {1e-6  , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ,
                             0.0   , 1e-6  , 0.0   , 0.0   , 0.0   , 0.0   ,
                             0.0   , 0.0   , 1e-6  , 0.0   , 0.0   , 0.0   ,
                             0.0   , 0.0   , 0.0   , 1e-6  , 0.0   , 0.0   ,
                             0.0   , 0.0   , 0.0   , 0.0   , 1e-6  , 0.0   ,
                             0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 1e-6  };
                             
    odom.twist.covariance = {1e-3  , 0.0   , 0.0   , 0.0   , 0.0   , 0.0   ,
                             0.0   , 1e-3  , 0.0   , 0.0   , 0.0   , 0.0   ,
                             0.0   , 0.0   , 1e-3  , 0.0   , 0.0   , 0.0   ,
                             0.0   , 0.0   , 0.0   , 1e-3  , 0.0   , 0.0   ,
                             0.0   , 0.0   , 0.0   , 0.0   , 1e-3  , 0.0   ,
                             0.0   , 0.0   , 0.0   , 0.0   , 0.0   , 1e-3  };
    odom_pub_->publish(odom);

    // geometry_msgs::msg::TransformStamped tf;
    // tf.header.frame_id = "wheel_odom";
    // tf.child_frame_id = "base_link";
    // tf.header.stamp = msg->header.stamp;
    // tf.transform.translation
    //   .set__x(position_x_)
    //   .set__y(position_y_)
    //   .set__z(0.0);
    // tf.transform.rotation
    //   .set__x(0.0)
    //   .set__y(0.0)
    //   .set__z(sin(attitude_yaw_ / 2.0))
    //   .set__w(cos(attitude_yaw_ / 2.0));
    // tf_broadcaster_->sendTransform(tf);
  }
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double position_x_;
  double position_y_;
  double attitude_yaw_;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryNode>("wheel_odometry"));
  rclcpp::shutdown();
  return 0;
}