#include "magnet_plugin.hh"

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosMagnetSensor)

gazebo::GazeboRosMagnetSensor::GazeboRosMagnetSensor() : SensorPlugin() { sensor = nullptr; }

void gazebo::GazeboRosMagnetSensor::Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf = sdf_;
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

  if (!ros::isInitialized()) // check if ros is initialized properly
  {
    ROS_FATAL("ROS has not been initialized!");
    return;
  }

  node = new ros::NodeHandle(this->robot_namespace);
  magnet_data_publisher = node->advertise<sensor_msgs::MagneticField>(topic_name, 10);
  connection = sensor->ConnectUpdated(std::bind(&GazeboRosMagnetSensor::OnUpdate, this));
  last_time = sensor->LastUpdateTime();

  magnet_msg.header.frame_id = body_name;
  for (size_t i = 0; i < 3; i++)
  {
    currDrift[i] = GaussianKernel(0, drift);
    currError[i] = 0;
  }
}

void gazebo::GazeboRosMagnetSensor::OnUpdate()
{
  common::Time current_time = sensor->LastUpdateTime();

  if (magnet_data_publisher.getNumSubscribers() > 0)
  {
    magnet_msg.header.stamp.sec = current_time.sec;
    magnet_msg.header.stamp.nsec = current_time.nsec;
    double dt = (current_time - last_time).Double();

    for (size_t i = 0; i < 3; i++)
    {
      currDrift[i] =
          exp(-dt * drift_frequency) * currDrift[i] + dt * GaussianKernel(0, sqrt(2 * drift_frequency) * drift);
      currError[i] = offset + currDrift[i] + GaussianKernel(0, gaussian_noise);
    }

    magnet_msg.magnetic_field.x = sensor->MagneticField().X() + currError[0];
    magnet_msg.magnetic_field.y = sensor->MagneticField().Y() + currError[1];
    magnet_msg.magnetic_field.z = sensor->MagneticField().Z() + currError[2];

    // Publish data
    magnet_data_publisher.publish(magnet_msg);
  }

  last_time = current_time;
}

bool gazebo::GazeboRosMagnetSensor::LoadParameters()
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
    topic_name = robot_namespace + "/magnet_data";
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

  return true;
}

gazebo::GazeboRosMagnetSensor::~GazeboRosMagnetSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

double gazebo::GazeboRosMagnetSensor::NoiseVariance(const gazebo::sensors::Noise& _noise)
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

double gazebo::GazeboRosMagnetSensor::NoiseVariance(const gazebo::sensors::NoisePtr& _noise_ptr)
{
  if (!_noise_ptr)
  {
    return 0.;
  }
  return NoiseVariance(*_noise_ptr);
}
double gazebo::GazeboRosMagnetSensor::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard normally distributed normal variables
  double U = double(rand()) / double(RAND_MAX);
  double V = double(rand()) / double(RAND_MAX);
  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  X = sigma * X + mu;
  return X;
}
