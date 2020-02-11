#ifndef DSCHUBBA_GAZEBO_GAZEBO_ROS_MAGNETIC_H
#define DSCHUBBA_GAZEBO_GAZEBO_ROS_MAGNETIC_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>

#include <gazebo/sensors/MagnetometerSensor.hh>

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

namespace gazebo {
namespace sensors {
class MagnetometerSensor;
}
class GazeboRosMagneticSensor : public SensorPlugin {
public:
  GazeboRosMagneticSensor();
  virtual ~GazeboRosMagneticSensor();

protected:
  virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

protected:
  virtual void UpdateChild(const gazebo::common::UpdateInfo &/*_info*/);

private:
  bool LoadParameters();

  ros::NodeHandle *nh;

  ros::Publisher pub;
  sensor_msgs::MagneticField magnetic_field_msg;
  common::Time last_time;

  event::ConnectionPtr connection;
  sensors::MagnetometerSensor* sensor;
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string body_name;
  double update_rate;
};

} // namespace gazebo

#endif // DSCHUBBA_GAZEBO_GAZEBO_ROS_MAGNETIC_H
