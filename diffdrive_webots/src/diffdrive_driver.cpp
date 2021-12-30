#include "diffdrive_webots/diffdrive_driver.hpp"

using std::placeholders::_1;

namespace diffdrive_webots
{
  void DiffDriveDriver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // initialize member variables
    node_ = node;
    robot_ = node->robot();
    left_encoder_ = NULL;
    right_encoder_ = NULL;
    left_motor_ = NULL;
    right_motor_ = NULL;
    encoder_period_ = (int)robot_->getBasicTimeStep();
    prev_left_angle_ = 0;
    prev_right_angle = 0;
    forward_speed_ = 0;
    angular_speed_ = 0;
    wheel_radius_ = 0;
    half_distance_between_wheels_ = 0;

    // retrieve tags
    if (parameters.count("wheelEncoderPeriodMs"))
      encoder_period_ = std::stoi(parameters["wheelEncoderPeriodMs"]);
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
    left_encoder_ = robot_->getPositionSensor(left_wheel_encoder_name);
    if (left_encoder_ == NULL)
      throw std::runtime_error("Cannot find PositionSensor with name " + left_wheel_encoder_name);
    
    right_encoder_ = robot_->getPositionSensor(right_wheel_encoder_name);
    if (right_encoder_ == NULL)
      throw std::runtime_error("Cannot find PositionSensor with name " + right_wheel_encoder_name);
    
    left_motor_ = robot_->getMotor(left_motor_name);
    if (left_motor_ == NULL)
      throw std::runtime_error("Cannot find Motor with name " + left_motor_name);
    
    right_motor_ = robot_->getMotor(right_motor_name);
    if (right_motor_ == NULL)
      throw std::runtime_error("Cannot find Motor with name " + right_motor_name);
    
    int timestep = (int)robot_->getBasicTimeStep();
    if (encoder_period_ % timestep != 0)
      throw std::runtime_error("wheelEncoderPeriodMs must be integer multiple of basicTimeStep");

    left_encoder_->enable(encoder_period_);
    right_encoder_->enable(encoder_period_);
    left_motor_->setPosition(INFINITY);
    left_motor_->setVelocity(0);
    right_motor_->setPosition(INFINITY);
    right_motor_->setVelocity(0);

    prev_left_angle_ = left_encoder_->getValue();
    prev_right_angle = right_encoder_->getValue();

    wheel_speed_pub_ = node->create_publisher<sensor_msgs::msg::Joy>("wheel_speed", rclcpp::SensorDataQoS().reliable());
    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS().reliable(), std::bind(&DiffDriveDriver::cmdVelCallback, this, _1));
  }

  void DiffDriveDriver::step()
  {
    int64_t sim_time = (int64_t)(robot_->getTime() * 1e3);

    if (sim_time % encoder_period_ == 0)
    {
      double curr_left_angle = left_encoder_->getValue();
      double curr_right_angle = right_encoder_->getValue();
      double left_wheel_speed = 1e3 * (curr_left_angle - prev_left_angle_) / encoder_period_;
      double right_wheel_speed = 1e3 * (curr_right_angle - prev_right_angle) / encoder_period_;
      prev_left_angle_ = curr_left_angle;
      prev_right_angle = curr_right_angle;

      sensor_msgs::msg::Joy wheel_speed;
      wheel_speed.header.stamp = node_->get_clock()->now();
      wheel_speed.axes.resize(2);
      wheel_speed.axes[0] = left_wheel_speed;
      wheel_speed.axes[1] = right_wheel_speed;
      
      wheel_speed_pub_->publish(wheel_speed);
    }
    
    double left_wheel_speed_cmd = (forward_speed_ - half_distance_between_wheels_ * angular_speed_) / wheel_radius_;
    double right_wheel_speed_cmd = (forward_speed_ + half_distance_between_wheels_ * angular_speed_) / wheel_radius_;
    left_motor_->setVelocity(left_wheel_speed_cmd);
    right_motor_->setVelocity(right_wheel_speed_cmd);
  }

  void DiffDriveDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    forward_speed_ = msg->linear.x;
    angular_speed_ = msg->angular.z;
  }
}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots::DiffDriveDriver, webots_ros2_driver::PluginInterface)