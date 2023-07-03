#include "imu_plugin.hh"

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosImuSensor)

gazebo::GazeboRosImuSensor::GazeboRosImuSensor() : SensorPlugin() { sensor = nullptr; }

void gazebo::GazeboRosImuSensor::Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf = sdf_;
  sensor = dynamic_cast<gazebo::sensors::ImuSensor*>(sensor_.get());

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
  imu_msg.angular_velocity_covariance[0] = NoiseVariance(sensor_->Noise(SNT::IMU_ANGVEL_X_NOISE_RADIANS_PER_S));
  imu_msg.angular_velocity_covariance[4] = NoiseVariance(sensor_->Noise(SNT::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S));
  imu_msg.angular_velocity_covariance[8] = NoiseVariance(sensor_->Noise(SNT::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S));
  imu_msg.linear_acceleration_covariance[0] = NoiseVariance(sensor_->Noise(SNT::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
  imu_msg.linear_acceleration_covariance[4] = NoiseVariance(sensor_->Noise(SNT::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
  imu_msg.linear_acceleration_covariance[8] = NoiseVariance(sensor_->Noise(SNT::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));

  node = new ros::NodeHandle(this->robot_namespace);
  imu_data_publisher = node->advertise<sensor_msgs::Imu>(topic_name, 10);
  connection = sensor->ConnectUpdated(std::bind(&GazeboRosImuSensor::OnUpdate, this));
  last_time = sensor->LastUpdateTime();

  imu_msg.header.frame_id = body_name;
  for (size_t i = 0; i < 3; i++)
  {
    currAccelDrift[i] = GaussianKernel(0, accel_drift);
    currRateDrift[i] = GaussianKernel(0, rate_drift);
    currAccelError[i] = 0;
    currRateError[i] = 0;
  }
}

void gazebo::GazeboRosImuSensor::OnUpdate()
{
  common::Time current_time = sensor->LastUpdateTime();

  if (imu_data_publisher.getNumSubscribers() > 0)
  {
    imu_msg.header.stamp.sec = current_time.sec;
    imu_msg.header.stamp.nsec = current_time.nsec;
    double dt = (current_time - last_time).Double();

    for (size_t i = 0; i < 3; i++)
    {
      currAccelDrift[i] = exp(-dt * accel_drift_frequency) * currAccelDrift[i] +
                          dt * GaussianKernel(0, sqrt(2 * accel_drift_frequency) * accel_drift);
      currAccelError[i] = accel_offset + currAccelDrift[i] + GaussianKernel(0, accel_gaussian_noise);
      currRateDrift[i] = exp(-dt * rate_drift_frequency) * currRateDrift[i] +
                         dt * GaussianKernel(0, sqrt(2 * rate_drift_frequency) * rate_drift);
      currRateError[i] = rate_offset + currRateDrift[i] + GaussianKernel(0, rate_gaussian_noise);
    }
    ignition::math::Vector3d accelDrift = currAccelDrift + accel_offset;

   imu_msg.linear_acceleration.x = sensor->LinearAcceleration().X() + currAccelError[0];
    imu_msg.linear_acceleration.y = -(sensor->LinearAcceleration().Y() + currAccelError[1]);
   imu_msg.linear_acceleration.z = -(sensor->LinearAcceleration().Z() + currAccelError[2]);
    imu_msg.angular_velocity.x = sensor->AngularVelocity().X() + currRateError[0];
    imu_msg.angular_velocity.y = -(sensor->AngularVelocity().Y() + currRateError[1]);
   imu_msg.angular_velocity.z = -(sensor->AngularVelocity().Z() + currRateError[2]);
// Orientation
    imu_msg.orientation.x = sensor->Orientation().X();
    imu_msg.orientation.y = -sensor->Orientation().Y();
    imu_msg.orientation.z = -sensor->Orientation().Z();
    imu_msg.orientation.w = sensor->Orientation().W();

    // Publish data
    imu_data_publisher.publish(imu_msg);
  }

  last_time = current_time;
}

bool gazebo::GazeboRosImuSensor::LoadParameters()
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
    topic_name = robot_namespace + "/imu_data";
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
  accel_offset = sdf->Get<double>("accelOffset");
  accel_drift = sdf->Get<double>("accelDrift");
  accel_drift_frequency = sdf->Get<double>("accelDriftFrequency");
  accel_gaussian_noise = sdf->Get<double>("accelGaussianNoise");
  rate_offset = sdf->Get<double>("rateOffset");
  rate_drift = sdf->Get<double>("rateDrift");
  rate_drift_frequency = sdf->Get<double>("rateDriftFrequency");
  rate_gaussian_noise = sdf->Get<double>("rateGaussianNoise");

  return true;
}

gazebo::GazeboRosImuSensor::~GazeboRosImuSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

double gazebo::GazeboRosImuSensor::NoiseVariance(const gazebo::sensors::Noise& _noise)
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

double gazebo::GazeboRosImuSensor::NoiseVariance(const gazebo::sensors::NoisePtr& _noise_ptr)
{
  if (!_noise_ptr)
  {
    return 0.;
  }
  return NoiseVariance(*_noise_ptr);
}
double gazebo::GazeboRosImuSensor::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard normally distributed normal variables
  double U = double(rand()) / double(RAND_MAX);
  double V = double(rand()) / double(RAND_MAX);
  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  X = sigma * X + mu;
  return X;
}
