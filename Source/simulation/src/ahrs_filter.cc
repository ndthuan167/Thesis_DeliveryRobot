#include "ahrs_filter.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(AHRSFilter);

AHRSFilter::AHRSFilter() {}

AHRSFilter::~AHRSFilter()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

void AHRSFilter::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model = _parent;
  world = _parent->GetWorld();
  sdf = _sdf;

  if (!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  node = new ros::NodeHandle(this->robot_namespace);
  pubOdom = node->advertise<Odometry>("odom_1", 10);
  subGPS = node->subscribe("gps/fix", 10, &AHRSFilter::onGPSCallBack, this);
  subIMU = new Subscriber<Imu>(*node, "imu", 10);
  subMagnet = new Subscriber<MagneticField>(*node, "ins/magnet", 10);
  synINS = new Synchronizer<SynINSPolicy>(SynINSPolicy(10), *subIMU, *subMagnet);
  synINS->registerCallback(boost::bind(&AHRSFilter::onSyncINSCallBack, this, _1, _2));
  sub_amcl = node->subscribe("/amcl_pose", 10, &AHRSFilter::CallbackAMCL, this);
  sub_hed = node->subscribe("/heading_plot", 10, &AHRSFilter::Callbackhead, this);
  sub_mode = node->subscribe("/mode_indoor", 10, &AHRSFilter::CallbackMode, this);

  odom_msg.header.frame_id = "ned_link";
  odom_msg.child_frame_id = body_name;

  connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AHRSFilter::OnUpdate, this));
}

void AHRSFilter::OnUpdate()
{
}

void AHRSFilter::onGPSCallBack(const NavSatFix::ConstPtr& msg)
{
  odom_msg.latitude = msg->latitude;
  odom_msg.longitude = msg->longitude;
  odom_msg.altitude = msg->altitude;
  convert_global_to_local_coords(msg->latitude, msg->longitude, ned_lat, ned_lon, odom_msg.position.x,
                                 odom_msg.position.y);
  odom_msg.position.z = -msg->altitude;
}

void AHRSFilter::CallbackAMCL(const PoseWithCovarianceStamped::ConstPtr& pose ){
    CurrX_indoor = pose->pose.pose.position.x;
    CurrY_indoor = pose->pose.pose.position.y;
       tf::Quaternion q(
    CurrHeading_indoor_x = pose ->pose.pose.orientation.x,
    CurrHeading_indoor_y = pose ->pose.pose.orientation.y,
    CurrHeading_indoor_z = pose ->pose.pose.orientation.z,
    CurrHeading_indoor_w = pose ->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    CurrOrient_indoor = - yaw ;

}

void AHRSFilter::CallbackMode(const mode_indoor::ConstPtr& mod ){
    mode_in = mod ->mode_indoor ;
}

void AHRSFilter::Callbackhead(const heading_plot::ConstPtr& hed ){
  exit_indoor = hed->exit_indoor ;
  if( exit_indoor == true){
    CurrX_indoor = 0.0 ;
   // ROS_INFO_STREAM("fdsfsd");
}
}

void AHRSFilter::onSyncINSCallBack(const Imu::ConstPtr& msg1, const MagneticField::ConstPtr& msg2)
{
  auto orientation = link->WorldPose().Rot();
  odom_msg.orientation.x = orientation.Roll();
  odom_msg.orientation.y = -orientation.Pitch();
   if ( mode_in == 1.0){
     odom_msg.orientation.z = CurrOrient_indoor;
   }
   else {
    odom_msg.orientation.z = -orientation.Yaw();
   }
  odom_msg.angular_velocity.x = msg1->angular_velocity.x;
  odom_msg.angular_velocity.y = msg1->angular_velocity.y;
  odom_msg.angular_velocity.z = msg1->angular_velocity.z;
  odom_msg.linear_acceleration.x = msg1->linear_acceleration.x;
  odom_msg.linear_acceleration.y = msg1->linear_acceleration.y;
  odom_msg.linear_acceleration.z = msg1->linear_acceleration.z;

  odom_msg.x_indoor = CurrX_indoor;
  odom_msg.y_indoor = CurrY_indoor;  
  odom_msg.mode_in = mode_in ;

  pubOdom.publish(odom_msg);
 
}

bool AHRSFilter::LoadParameters()
{
  // loading parameters from the sdf file

  // NAMESPACE
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace = sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace);
  }

  // TOPIC
  if (sdf->HasElement("topicName"))
  {
    topic_name = sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: " << topic_name);
  }
  else
  {
    topic_name = "/ahrs";
    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
  }

  // BODY NAME
  if (sdf->HasElement("bodyName"))
  {
    body_name = sdf->GetElement("bodyName")->Get<std::string>();
    link = model->GetLink(body_name);
    ROS_INFO_STREAM("<bodyName> set to: " << body_name);
  }
  else
  {
    ROS_FATAL("missing <bodyName>, cannot proceed");
    return false;
  }

  // NED_LAT_LON
  if (sdf->HasElement("nedLat"))
  {
    ned_lat = sdf->GetElement("nedLat")->Get<double>();
  }
  else
  {
    ROS_FATAL("missing <nedLat>, cannot proceed");
    return false;
  }
  if (sdf->HasElement("nedLon"))
  {
    ned_lon = sdf->GetElement("nedLon")->Get<double>();
  }
  else
  {
    ROS_FATAL("missing <nedLon>, cannot proceed");
    return false;
  }

  return true;
}
} // namespace gazebo
