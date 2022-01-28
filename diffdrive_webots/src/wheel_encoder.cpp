#include "diffdrive_webots/wheel_encoder.hpp"

using std::placeholders::_1;

namespace diffdrive_webots_plugin
{
  void WheelEncoder::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // initialize member variables
    node_ = node;
    robot_ = node->robot();
    left_encoder_ = NULL;
    right_encoder_ = NULL;
    period_ = (int)robot_->getBasicTimeStep();
    prev_left_angle_ = 0;
    prev_right_angle = 0;

    // retrieve tags
    if (parameters.count("wheelEncoderPeriodMs"))
      period_ = std::stoi(parameters["wheelEncoderPeriodMs"]);
    else
      throw std::runtime_error("Must set wheelEncoderPeriodMs tag");

    std::string left_wheel_encoder_name;
    if (parameters.count("leftWheelEncoderName"))
      left_wheel_encoder_name = parameters["leftWheelEncoderName"];
    else
      throw std::runtime_error("Must set leftWheelEncoderName tag");

    std::string right_wheel_encoder_name;
    if (parameters.count("rightWheelEncoderName"))
      right_wheel_encoder_name = parameters["rightWheelEncoderName"];
    else
      throw std::runtime_error("Must set rightWheelEncoderName tag");
    
    std::string left_wheel_joint_name;
    if (parameters.count("leftWheelJointName"))
      left_wheel_joint_name = parameters["leftWheelJointName"];
    else
      throw std::runtime_error("Must set leftWheelJointName tag");

    std::string right_wheel_joint_name;
    if (parameters.count("rightWheelJointName"))
      right_wheel_joint_name = parameters["rightWheelJointName"];
    else
      throw std::runtime_error("Must set rightWheelJointName tag");

    // set webots device
    left_encoder_ = robot_->getPositionSensor(left_wheel_encoder_name);
    if (left_encoder_ == NULL)
      throw std::runtime_error("Cannot find PositionSensor with name " + left_wheel_encoder_name);
    
    right_encoder_ = robot_->getPositionSensor(right_wheel_encoder_name);
    if (right_encoder_ == NULL)
      throw std::runtime_error("Cannot find PositionSensor with name " + right_wheel_encoder_name);
    
    int timestep = (int)robot_->getBasicTimeStep();
    if (period_ % timestep != 0)
      throw std::runtime_error("wheelEncoderPeriodMs must be integer multiple of basicTimeStep");

    left_encoder_->enable(period_);
    right_encoder_->enable(period_);

    prev_left_angle_ = left_encoder_->getValue();
    prev_right_angle = right_encoder_->getValue();

    // JointState publisher
    joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS().reliable());
    joint_state_msg_.name.resize(2);
    joint_state_msg_.position.resize(2);
    joint_state_msg_.velocity.resize(2);
    joint_state_msg_.name[0] = left_wheel_joint_name;
    joint_state_msg_.name[1] = right_wheel_joint_name;
    joint_state_msg_.position[0] = 0.0;
    joint_state_msg_.position[1] = 0.0;
    joint_state_msg_.velocity[0] = 0.0;
    joint_state_msg_.velocity[1] = 0.0;

  }

  void WheelEncoder::step()
  {
    int64_t sim_time = (int64_t)(robot_->getTime() * 1e3);

    if (sim_time % period_ == 0)
    {
      double curr_left_angle = left_encoder_->getValue();
      double curr_right_angle = right_encoder_->getValue();
      double left_wheel_speed = 1e3 * (curr_left_angle - prev_left_angle_) / period_;
      double right_wheel_speed = 1e3 * (curr_right_angle - prev_right_angle) / period_;
      prev_left_angle_ = curr_left_angle;
      prev_right_angle = curr_right_angle;

      joint_state_msg_.header.stamp = node_->get_clock()->now();
      joint_state_msg_.position[0] = curr_left_angle;
      joint_state_msg_.position[1] = curr_right_angle;
      joint_state_msg_.velocity[0] = left_wheel_speed;
      joint_state_msg_.velocity[1] = right_wheel_speed;
      joint_state_pub_->publish(joint_state_msg_);
    }

  }

}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots_plugin::WheelEncoder, webots_ros2_driver::PluginInterface)