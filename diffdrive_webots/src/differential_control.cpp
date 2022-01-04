#include "diffdrive_webots/differential_control.hpp"

using std::placeholders::_1;

namespace diffdrive_webots_plugin
{
  void DifferentialControl::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // initialize member variables
    node_ = node;
    robot_ = node->robot();
    left_motor_ = NULL;
    right_motor_ = NULL;
    forward_speed_ = 0;
    angular_speed_ = 0;
    wheel_radius_ = 0;
    half_distance_between_wheels_ = 0;

    // retrieve tags
    std::string left_motor_name;
    if (parameters.count("leftMotorName"))
      left_motor_name = parameters["leftMotorName"];
    else
      throw std::runtime_error("Must set leftMotorName tag");

    std::string right_motor_name;
    if (parameters.count("rightMotorName"))
      right_motor_name = parameters["rightMotorName"];
    else
      throw std::runtime_error("Must set rightMotorName tag");

    if (parameters.count("wheelRadius"))
      wheel_radius_ = std::stod(parameters["wheelRadius"]);
    else
      throw std::runtime_error("Must set wheelRadius tag");
    
    if (parameters.count("halfDistanceBetweenWheels"))
      half_distance_between_wheels_ = std::stod(parameters["halfDistanceBetweenWheels"]);
    else
      throw std::runtime_error("Must set halfDistanceBetweenWheels tag");

    // set webots device
    left_motor_ = robot_->getMotor(left_motor_name);
    if (left_motor_ == NULL)
      throw std::runtime_error("Cannot find Motor with name " + left_motor_name);
    
    right_motor_ = robot_->getMotor(right_motor_name);
    if (right_motor_ == NULL)
      throw std::runtime_error("Cannot find Motor with name " + right_motor_name);
    
    left_motor_->setPosition(INFINITY);
    left_motor_->setVelocity(0);
    right_motor_->setPosition(INFINITY);
    right_motor_->setVelocity(0);

    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS().reliable(), std::bind(&DifferentialControl::cmdVelCallback, this, _1));
  }

  void DifferentialControl::step()
  {
    double left_wheel_speed_cmd = (forward_speed_ - half_distance_between_wheels_ * angular_speed_) / wheel_radius_;
    double right_wheel_speed_cmd = (forward_speed_ + half_distance_between_wheels_ * angular_speed_) / wheel_radius_;
    left_motor_->setVelocity(left_wheel_speed_cmd);
    right_motor_->setVelocity(right_wheel_speed_cmd);
  }

  void DifferentialControl::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    forward_speed_ = msg->linear.x;
    angular_speed_ = msg->angular.z;
  }
}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots_plugin::DifferentialControl, webots_ros2_driver::PluginInterface)