#include "gps_plugin.hh"
#include <gazebo/sensors/GaussianNoiseModel.hh>
#include <gazebo/sensors/GpsSensor.hh>
#include <iostream>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosGpsSensor)

gazebo::GazeboRosGpsSensor::GazeboRosGpsSensor() : SensorPlugin() { sensor = nullptr; }

void gazebo::GazeboRosGpsSensor::Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf = sdf_;
  sensor = dynamic_cast<gazebo::sensors::GpsSensor*>(sensor_.get());

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

  if (!ros::isInitialized()) // check if ros is initialized properly
  {
    ROS_FATAL("ROS has not been initialized!");
    return;
  }

  using SNT = gazebo::sensors::SensorNoiseType;
  gps_msg.position_covariance[0] = NoiseVariance(sensor_->Noise(SNT::GPS_POSITION_LATITUDE_NOISE_METERS));
  gps_msg.position_covariance[4] = NoiseVariance(sensor_->Noise(SNT::GPS_POSITION_LONGITUDE_NOISE_METERS));
  gps_msg.position_covariance[8] = NoiseVariance(sensor_->Noise(SNT::GPS_POSITION_ALTITUDE_NOISE_METERS));
  gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  node = new ros::NodeHandle(this->robot_namespace);
  gps_data_publisher = node->advertise<sensor_msgs::NavSatFix>(topic_name, 10);
  gps_vel_data_publisher = node->advertise<geometry_msgs::Vector3Stamped>(velocity_topic_name, 10);
  connection = sensor->ConnectUpdated(std::bind(&GazeboRosGpsSensor::OnUpdate, this));
  last_time = sensor->LastUpdateTime();

  gps_msg.header.frame_id = body_name;
  vel_gps_msg.header.frame_id = body_name;
  for (size_t i = 0; i < 3; i++)
  {
    currDrift[i] = GaussianKernel(0, drift);
    currVelDrift[i] = GaussianKernel(0, velocity_drift);
    currError[i] = 0;
    currVelError[i] = 0;
  }
}

void gazebo::GazeboRosGpsSensor::OnUpdate()
{
  common::Time current_time = sensor->LastUpdateTime();

  if (gps_data_publisher.getNumSubscribers() > 0)
  {
    gps_msg.header.stamp.sec = current_time.sec;
    gps_msg.header.stamp.nsec = current_time.nsec;
    vel_gps_msg.header.stamp.sec = current_time.sec;
    vel_gps_msg.header.stamp.nsec = current_time.nsec;

    double lat = sensor->Latitude().Degree();
    double lon = sensor->Longitude().Degree();
    double alt = sensor->Altitude();
    double vel_x = sensor->VelocityNorth();
    double vel_y = sensor->VelocityEast();
    double vel_z = -sensor->VelocityUp();

    double dt = (current_time - last_time).Double();
    for (size_t i = 0; i < 3; i++)
    {
      currDrift[i] =
          exp(-dt * drift_frequency) * currDrift[i] + dt * GaussianKernel(0, sqrt(2 * drift_frequency) * drift);
      currError[i] = offset + currDrift[i] + GaussianKernel(0, gaussian_noise);
      currVelDrift[i] = exp(-dt * velocity_drift_frequency) * currVelDrift[i] +
                        dt * GaussianKernel(0, sqrt(2 * velocity_drift_frequency) * velocity_drift);
      currVelError[i] = velocity_offset + currVelDrift[i] + GaussianKernel(0, velocity_gaussian_noise);
      currDrift[i] += dt * currVelDrift[i];
    }
    double lat_, lon_;
    convert_local_to_global_coords(currError[0], currError[1], lat, lon, lat_, lon_);

    gps_msg.latitude = lat_;
    gps_msg.longitude = lon_;
    gps_msg.altitude = alt + currError[2];
    vel_gps_msg.vector.x = vel_x + currVelError[0];
    vel_gps_msg.vector.y = vel_y + currVelError[1];
    vel_gps_msg.vector.z = vel_z + currVelError[2];

    gps_data_publisher.publish(gps_msg);
    gps_vel_data_publisher.publish(vel_gps_msg);
  }

  last_time = current_time;
}

bool gazebo::GazeboRosGpsSensor::LoadParameters()
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
    std::size_t it = scoped_name.find("::");

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
    topic_name = robot_namespace + "/fix";
    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
  }

  // VELOCITY TOPIC
  if (sdf->HasElement("velocityTopicName"))
  {
    velocity_topic_name = robot_namespace + sdf->Get<std::string>("velocityTopicName");
    ROS_INFO_STREAM("<topicName> set to: " << velocity_topic_name);
  }
  else
  {
    topic_name = robot_namespace + "/fix_velocity";
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

  // PARAMETERS
  offset = sdf->Get<double>("offset");
  drift = sdf->Get<double>("drift");
  drift_frequency = sdf->Get<double>("driftFrequency");
  gaussian_noise = sdf->Get<double>("gaussianNoise");
  velocity_offset = sdf->Get<double>("velocityOffset");
  velocity_drift = sdf->Get<double>("velocityDrift");
  velocity_drift_frequency = sdf->Get<double>("velocityDriftFrequency");
  velocity_gaussian_noise = sdf->Get<double>("velocityGaussianNoise");

  return true;
}

gazebo::GazeboRosGpsSensor::~GazeboRosGpsSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

double gazebo::GazeboRosGpsSensor::NoiseVariance(const gazebo::sensors::Noise& _noise)
{
  if (gazebo::sensors::Noise::NoiseType::GAUSSIAN == _noise.GetNoiseType())
  {
    auto gm = dynamic_cast<const gazebo::sensors::GaussianNoiseModel&>(_noise);
    return gm.GetStdDev() * gm.GetStdDev();
  }
  else if (gazebo::sensors::Noise::NoiseType::NONE == _noise.GetNoiseType())
  {
    return 0.;
  }
  return -1.;
}

double gazebo::GazeboRosGpsSensor::NoiseVariance(const gazebo::sensors::NoisePtr& _noise_ptr)
{
  if (!_noise_ptr)
  {
    return 0.;
  }
  return NoiseVariance(*_noise_ptr);
}

double gazebo::GazeboRosGpsSensor::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard normally distributed normal variables
  double U = double(rand()) / double(RAND_MAX);
  double V = double(rand()) / double(RAND_MAX);
  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  X = sigma * X + mu;
  return X;
}
