#include "diffdrive_webots/imu.hpp"

using std::placeholders::_1;

namespace diffdrive_webots_plugin
{
  void IMU::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // initialize member variables
    node_ = node;
    robot_ = node->robot();
    accelerometer_ = NULL;
    gyro_ = NULL;
    inertial_unit_ = NULL;

    // retrieve tags
    if (parameters.count("imuPeriodMs"))
      period_ = std::stoi(parameters["imuPeriodMs"]);
    else
      throw std::runtime_error("Must set imuPeriodMs tag");

    std::string accelerometer_name;
    if (parameters.count("accelerometerName"))
      accelerometer_name = parameters["accelerometerName"];
    else
      throw std::runtime_error("Must set accelerometerName tag");

    std::string gyro_name;
    if (parameters.count("gyroName"))
      gyro_name = parameters["gyroName"];
    else
      throw std::runtime_error("Must set  gyroName tag");
    
    std::string inertial_unit_name;
    if (parameters.count("inertialUnitName"))
      inertial_unit_name = parameters["inertialUnitName"];
    else
      throw std::runtime_error("Must set inertialUnitName tag");
    
    std::string frame_id;
    if (parameters.count("frameID"))
      frame_id = parameters["frameID"];
    else
      throw std::runtime_error("Must set frameID tag");

    // set webots device
    accelerometer_ = robot_->getAccelerometer(accelerometer_name);
    if (accelerometer_ == NULL)
      throw std::runtime_error("Cannot find Accelerometer with name " + accelerometer_name);
    
    gyro_ = robot_->getGyro(gyro_name);
    if (gyro_ == NULL)
      throw std::runtime_error("Cannot find Gyro with name " + gyro_name);
    
    inertial_unit_ = robot_->getInertialUnit(inertial_unit_name);
    if (inertial_unit_ == NULL)
      throw std::runtime_error("Cannot find inertialUnit with name " + inertial_unit_name);
    
    int timestep = (int)robot_->getBasicTimeStep();
    if (period_ % timestep != 0)
      throw std::runtime_error("imuPeriodMs must be integer multiple of basicTimeStep");

    accelerometer_->enable(period_);
    gyro_->enable(period_);
    inertial_unit_->enable(period_);

    // Imu publisher
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS().reliable());
    imu_msg_.header.frame_id = frame_id;
    imu_msg_.orientation_covariance          = {1e-6   , 0.0    , 0.0    ,
                                                0.0    , 1e-6   , 0.0    ,
                                                0.0    , 0.0    , 1e-6   };

    imu_msg_.angular_velocity_covariance     = {1e-6   , 0.0    , 0.0    ,
                                                0.0    , 1e-6   , 0.0    ,
                                                0.0    , 0.0    , 1e-6   };

    imu_msg_.linear_acceleration_covariance  = {1e-6   , 0.0    , 0.0    ,
                                                0.0    , 1e-6   , 0.0    ,
                                                0.0    , 0.0    , 1e-6   };

  }

  void IMU::step()
  {
    int64_t sim_time = (int64_t)(robot_->getTime() * 1e3);

    if (sim_time % period_ == 0)
    {
      const double* acc = accelerometer_->getValues();
      const double* ome = gyro_->getValues();
      const double* att = inertial_unit_->getQuaternion();

      imu_msg_.header.stamp = node_->get_clock()->now();
      imu_msg_.orientation
        .set__x(att[0])
        .set__y(att[1])
        .set__z(att[2])
        .set__w(att[3]);
      imu_msg_.angular_velocity
        .set__x(ome[0])
        .set__y(ome[1])
        .set__z(ome[2]);
      imu_msg_.linear_acceleration
        .set__x(acc[0])
        .set__y(acc[1])
        .set__z(acc[2]);
      imu_pub_->publish(imu_msg_);
    }

  }

}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_webots_plugin::IMU, webots_ros2_driver::PluginInterface)