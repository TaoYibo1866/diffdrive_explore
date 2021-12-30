#include "diffdrive_webots/diffdrive_driver.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace diffdrive_webots
{
  void DiffDriveDriver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // initialize member variables
    node_ = node;
    robot_ = node->robot();
    encoder1_ = NULL;
    encoder2_ = NULL;
    motor1_ = NULL;
    motor2_ = NULL;
    encoder_period_ = (int)robot_->getBasicTimeStep();
    prev_angle1_ = 0;
    prev_angle2_ = 0;
    cmd_vel_.linear.x = 0;
    cmd_vel_.angular.z = 0;
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
      throw std::runtime_error("Must set ightWheelEncoderName tag");

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
    encoder1_ = robot_->getPositionSensor(left_wheel_encoder_name);
    if (encoder1_ == NULL)
      throw std::runtime_error("Cannot find PositionSensor with name " + left_wheel_encoder_name);
    
    encoder2_ = robot_->getPositionSensor(right_wheel_encoder_name);
    if (encoder2_ == NULL)
      throw std::runtime_error("Cannot find PositionSensor with name " + right_wheel_encoder_name);
    
    motor1_ = robot_->getMotor(left_motor_name);
    if (motor1_ == NULL)
      throw std::runtime_error("Cannot find Motor with name " + left_motor_name);
    
    motor2_ = robot_->getMotor(right_motor_name);
    if (motor2_ == NULL)
      throw std::runtime_error("Cannot find Motor with name " + right_motor_name);
    
    int timestep = (int)robot_->getBasicTimeStep();
    if (encoder_period_ % timestep != 0)
      throw std::runtime_error("wheelEncoderPeriodMs must be integer multiple of basicTimeStep");

    encoder1_->enable(encoder_period_);
    encoder2_->enable(encoder_period_);
    motor1_->setPosition(INFINITY);
    motor1_->setVelocity(0);
    motor2_->setPosition(INFINITY);
    motor2_->setVelocity(0);

    prev_angle1_ = encoder1_->getValue();
    prev_angle2_ = encoder1_->getValue();

    wheel_speed_pub_ = node->create_publisher<sensor_msgs::msg::Joy>("wheel_speed", rclcpp::SensorDataQoS().reliable());
  }

  void DiffDriveDriver::step()
  {
    int64_t sim_time = (int64_t)(robot_->getTime() * 1e3);

    if (sim_time % encoder_period_ == 0)
    {
      double curr_angle1 = encoder1_->getValue();
      double curr_angle2 = encoder2_->getValue();
      double wheel_speed1 = 1e3 * (curr_angle1 - prev_angle1_) / encoder_period_;
      double wheel_speed2 = 1e3 * (curr_angle2 - prev_angle2_) / encoder_period_;
      prev_angle1_ = curr_angle1;
      prev_angle2_ = curr_angle2;

      sensor_msgs::msg::Joy wheel_speed;
      wheel_speed.header.stamp = node_->get_clock()->now();
      wheel_speed.axes.resize(2);
      wheel_speed.axes[0] = wheel_speed1;
      wheel_speed.axes[1] = wheel_speed2;
      
      wheel_speed_pub_->publish(wheel_speed);
    }
    
  }
}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots::DiffDriveDriver, webots_ros2_driver::PluginInterface)