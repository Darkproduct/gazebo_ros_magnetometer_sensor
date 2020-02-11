#include <gazebo_ros_magnetometer/gazebo_ros_magnetometer.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMagneticSensor)

GazeboRosMagneticSensor::GazeboRosMagneticSensor() : SensorPlugin(), sensor(nullptr), nh(nullptr) {
}

GazeboRosMagneticSensor::~GazeboRosMagneticSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = event::ConnectionPtr();
  }

  if (nh != nullptr)
  {
    nh->shutdown();
    delete nh;
  }
}

void GazeboRosMagneticSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf    = sdf_;
  sensor = dynamic_cast<gazebo::sensors::MagnetometerSensor*>(sensor_.get());

  if (sensor == nullptr)
  {
    ROS_FATAL("Error: Sensor pointer is NULL!");
    return;
  }

  sensor->SetActive(true);

  if (!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  if (!ros::isInitialized())  // check if ros is initialized properly
  {
    ROS_FATAL("ROS has not been initialized!");
    return;
  }

  nh = new ros::NodeHandle(this->robot_namespace);

  pub = nh->advertise<sensor_msgs::MagneticField>(topic_name, 1);

  connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosMagneticSensor::UpdateChild, this, _1));

  last_time = sensor->LastUpdateTime();
}

void GazeboRosMagneticSensor::UpdateChild([[maybe_unused]] const gazebo::common::UpdateInfo& msg)
{
  common::Time current_time = sensor->LastUpdateTime();

  if (update_rate > 0 && (current_time - last_time).Double() < 1.0 / update_rate)  // update rate check
    return;

  if (pub.getNumSubscribers() > 0)
  {
    ignition::math::Vector3d field = sensor->MagneticField();

    magnetic_field_msg.magnetic_field.x = field.X();
    magnetic_field_msg.magnetic_field.y = field.Y();
    magnetic_field_msg.magnetic_field.z = field.Z();

    // preparing message header
    magnetic_field_msg.header.frame_id   = body_name;
    magnetic_field_msg.header.stamp.sec  = current_time.sec;
    magnetic_field_msg.header.stamp.nsec = current_time.nsec;

    // publishing data
    pub.publish(magnetic_field_msg);

    ros::spinOnce();
  }

  last_time = current_time;
}

bool GazeboRosMagneticSensor::LoadParameters()
{
  // loading parameters from the sdf file

  // NAMESPACE
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace = sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace);
  }
  else
  {
    std::string scoped_name = sensor->ParentName();
    std::size_t it          = scoped_name.find("::");

    robot_namespace = "/" + scoped_name.substr(0, it) + "/";
    ROS_WARN_STREAM("missing <robotNamespace>, set to default: " << robot_namespace);
  }

  // TOPIC
  if (sdf->HasElement("topicName"))
  {
    topic_name = robot_namespace + sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: " << topic_name);
  }
  else
  {
    topic_name = robot_namespace + "/mag_data";
    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
  }

  // BODY NAME
  if (sdf->HasElement("frameName"))
  {
    body_name = sdf->Get<std::string>("frameName");
    ROS_INFO_STREAM("<frameName> set to: " << body_name);
  }
  else
  {
    ROS_FATAL("missing <frameName>, cannot proceed");
    return false;
  }

  // UPDATE RATE
  if (sdf->HasElement("updateRateHZ"))
  {
    update_rate = sdf->Get<double>("updateRateHZ");
    ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
  }
  else
  {
    update_rate = 1.0;
    ROS_WARN_STREAM("missing <updateRateHZ>, set to default: " << update_rate);
  }

  return true;
}

}  // namespace gazebo
