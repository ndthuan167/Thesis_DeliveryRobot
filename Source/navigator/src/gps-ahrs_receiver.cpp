#include "gps-ahrs_receiver.h"

GPSAHRSReceiverNode::GPSAHRSReceiverNode()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("gps_enabled", gps_enabled);
  private_nh.getParam("ahrs_enabled", ahrs_enabled);
  private_nh.getParam("gps_port", gps_port);
  private_nh.getParam("ahrs_port", ahrs_port);
  private_nh.getParam("gps_baudrate", gps_baudrate);
  private_nh.getParam("ahrs_baudrate", ahrs_baudrate);

  ros::NodeHandle nh;
  nh.getParam("ned_lat", ned_lat);
  nh.getParam("ned_lon", ned_lon);

  pubOdom = nh.advertise<Odometry>("odom_1", 10);
  pubIMU = nh.advertise<sensor_msgs::Imu>("imu", 10);

}

GPSAHRSReceiverNode::~GPSAHRSReceiverNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void GPSAHRSReceiverNode::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}



GPSAHRSReceiver::GPSAHRSReceiver()
{
  ros::NodeHandle n;
  sub_pose = n.subscribe("/amcl_pose", 10, &GPSAHRSReceiver::onCallbackPose, this );
  sub_hed = n.subscribe("/heading_plot", 10, &GPSAHRSReceiver::onCallbackhead, this );
  sub_mode = n.subscribe("/mode_indoor", 10, &GPSAHRSReceiver::onCallbackmode, this);
  if (node.gps_enabled)
  {
    gpsDevice.setPortName(QString::fromStdString(node.gps_port));
    gpsDevice.setBaudRate(node.gps_baudrate);
    gpsDevice.setParity(QSerialPort::NoParity);
    gpsDevice.open(QIODevice::ReadOnly);
    connect(&gpsDevice, &QSerialPort::readyRead, this, &GPSAHRSReceiver::processGPSFrame);
  }
  if (node.ahrs_enabled)
  {
    ahrsDevice.setPortName(QString::fromStdString(node.ahrs_port));
    ahrsDevice.setBaudRate(node.ahrs_baudrate);
    ahrsDevice.setParity(QSerialPort::NoParity);
    ahrsDevice.open(QIODevice::ReadOnly);
    connect(&ahrsDevice, &QSerialPort::readyRead, this, &GPSAHRSReceiver::processAHRSFrame);
  }

  node.start();
}
void GPSAHRSReceiver::onCallbackmode(const mode_indoor::ConstPtr& mod){
    mode_in = mod->mode_indoor ;

}
void GPSAHRSReceiver::onCallbackPose(const PoseWithCovarianceStamped::ConstPtr& pos){
  CurrX_indoor = pos ->pose.pose.position.x;
  CurrY_indoor = pos ->pose.pose.position.y ;
    tf::Quaternion q(
  CurrHeading_indoor_x = pos ->pose.pose.orientation.x,
  CurrHeading_indoor_y = pos ->pose.pose.orientation.y,
  CurrHeading_indoor_z = pos ->pose.pose.orientation.z,
  CurrHeading_indoor_w = pos ->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    CurrOrient_indoor = - yaw ;
 //   ROS_INFO_STREAM("Orient_z = " << CurrOrient_indoor);

}

void GPSAHRSReceiver::onCallbackhead(const heading_plot::ConstPtr& hed){
	exit_indoor = hed->exit_indoor;
if(exit_indoor == true){
	CurrX_indoor = 0.0 ;	
}
}

GPSAHRSReceiver::~GPSAHRSReceiver()
{
  if (gpsDevice.isOpen())
    gpsDevice.close();
  if (ahrsDevice.isOpen())
    ahrsDevice.close();
}

void GPSAHRSReceiver::processGPSFrame()
{
  QByteArray payload,checksum;
  payload.resize(4);
  QByteArray payload_rvs;
  payload_rvs.resize(4);
  checksum.resize(40);
  NAV_POSLLH posllh;
 
  auto buffer = gpsDevice.readAll();
  if(buffer[0] == 0xB5 && buffer[1] == 0x62){
  // unsigned char CK[2]={buffer[42],buffer[43]};
  // memcpy(checksum.data(),buffer.data() + 2,40);
  // calcChecksum(CK,checksum);
// Get lon
  memcpy(payload.data(),buffer.data() + 14,4);
  std::copy(payload.crbegin(),payload.crend(),payload_rvs.begin());
  memcpy(&posllh.lon,payload.data(),4);
// Get lat
  memcpy(payload.data(),buffer.data() + 18,4);
  std::copy(payload.crbegin(),payload.crend(),payload_rvs.begin());
  memcpy(&posllh.lat,payload.data(),4);
// Get altitute
  memcpy(payload.data(),buffer.data() + 26,4);
  std::copy(payload.crbegin(),payload.crend(),payload_rvs.begin());
  memcpy(&posllh.hMSL,payload.data(),4);
// Get Hlon/ Hlat
  memcpy(payload.data(),buffer.data() + 30,1);
  std::copy(payload.crbegin(),payload.crend(),payload_rvs.begin());
  memcpy(&posllh.hlon,payload.data(),1);

  memcpy(payload.data(),buffer.data() + 31,1);
  std::copy(payload.crbegin(),payload.crend(),payload_rvs.begin());
  memcpy(&posllh.hlat,payload.data(),1);
  // Get accuracy 2D
  memcpy(payload.data(),buffer.data() + 34,4);
  std::copy(payload.crbegin(),payload.crend(),payload_rvs.begin());
  memcpy(&posllh.hAcc,payload.data(),4);


  double lon = (long int)posllh.lon * 1e-7;
  node.odomMsg.latitude = (double) (( int)posllh.lat + posllh.hlat * 1e-2) * 1e-7;
  node.odomMsg.longitude =(double) (( int)posllh.lon + posllh.hlon * 1e-2) * 1e-7;
  node.odomMsg.altitude = (double) ( int)posllh.hMSL * 1e-3;
  convert_global_to_local_coords(node.odomMsg.latitude, node.odomMsg.longitude, node.ned_lat, node.ned_lon,
                                     node.odomMsg.position.x, node.odomMsg.position.y);
  node.odomMsg.position.z = -node.odomMsg.altitude;
  ROS_INFO("Accuracy :%f",(int)posllh.hAcc/10000.0f);

  }
  
  QByteArray tmpBuffer;
  tmpBuffer.resize(74);
  memcpy(tmpBuffer.data(),buffer.data() + 44,73);

  
//  auto tmpBuffer = gpsDevice.readAll();
  vector<string> data_lines;
  boost::split(data_lines, tmpBuffer, boost::is_any_of(" "));
  for (auto it : data_lines)
  {
    vector<string> data;
    boost::split(data, it, boost::is_any_of(","));
    if (data[0] == "$GNGGA")
    {
      // node.odomMsg.latitude = CalLat2Deg(QString::fromStdString(data[2]).toDouble());
      // node.odomMsg.longitude = CalLong2Deg(QString::fromStdString(data[4]).toDouble());
      // node.odomMsg.altitude = QString::fromStdString(data[9]).toDouble();
      // convert_global_to_local_coords(node.odomMsg.latitude, node.odomMsg.longitude, node.ned_lat, node.ned_lon,
      //                                node.odomMsg.position.x, node.odomMsg.position.y);
      // node.odomMsg.position.z = -node.odomMsg.altitude;
      ROS_INFO("Mode: %i",QString::fromStdString(data[6]).toInt());
      ROS_INFO("Satellites Used: %i",QString::fromStdString(data[7]).toInt());
    }
  }

  if (!ahrsDevice.isOpen())
  {
    node.odomMsg.header.stamp = ros::Time::now();
    node.pubOdom.publish(node.odomMsg);
  }
}

void GPSAHRSReceiver::processAHRSFrame()
{
  auto tmpBuffer = ahrsDevice.readAll();

  if (isAhrsFirst)
  {
    ROS_INFO("Waiting for correct header.");
    if (tmpBuffer[0] == '$')
      isAhrsFirst = false;
    else
      return;
  }

  auto tmpBufferSize = tmpBuffer.length();
  for (int i = ahrsBytesReceived; i < ahrsBytesReceived + tmpBufferSize; i++)
  {
    ahrsBuffer[i]= tmpBuffer.data()[i - ahrsBytesReceived];
  }
//  ahrsBytesReceived += tmpBufferSize;
//  if (ahrsBytesReceived < BUFFER_SIZE)
//    return;
//  ahrsBytesReceived = 0;

  char header[4] = {ahrsBuffer[0], ahrsBuffer[1], ahrsBuffer[2]};
  if (strcmp(header, "$RQ"))
  {
    isAhrsFirst = true;
    return;
  }

  vector<string> data;
  boost::split(data, ahrsBuffer, boost::is_any_of(","));
  node.odomMsg.orientation.x = (stod(data[1].data()) - 4300) * 0.1 * M_PI / 180;
  node.odomMsg.orientation.y = (stod(data[2].data()) - 4300) * 0.1 * M_PI / 180;
  if(mode_in == 1.0){
	ROS_INFO_STREAM(CurrX_indoor);
    	node.odomMsg.orientation.z = CurrOrient_indoor;}
  else {
  	node.odomMsg.orientation.z = (-stod(data[3].data()) + 4300) * 0.1 * M_PI / 180;
  }
 // node.odomMsg.linear_acceleration.x = stod(data[4].data()) * 9.8065 / 16384.0;
 // node.odomMsg.linear_acceleration.y = stod(data[5].data()) * 9.8065 / 16384.0;
 // node.odomMsg.linear_acceleration.z = stod(data[6].data()) * 9.8065 / 16384.0;

 // node.odomMsg.angular_velocity.x = stod(data[7].data()) * M_PI / (180 * 65.5);
 // node.odomMsg.angular_velocity.y = stod(data[8].data()) * M_PI / (180 * 65.5);
 // node.odomMsg.angular_velocity.z = stod(data[9].data()) * M_PI / (180 * 65.5);

  node.odomMsg.header.stamp = ros::Time::now();

  node.odomMsg.x_indoor = CurrX_indoor;
  node.odomMsg.y_indoor = CurrY_indoor;
  node.odomMsg.mode_in = mode_in ;

  node.pubOdom.publish(node.odomMsg);

 // node.imu_msg.orientation.x = stod(data[1].data()) / 100000;
 // node.imu_msg.orientation.y = stod(data[2].data()) / 100000;
 // node.imu_msg.orientation.z = stod(data[3].data()) / 100000;
 // node.imu_msg.orientation.w = stod(data[4].data()) / 100000;

  node.imu_msg.linear_acceleration.x = stod(data[4].data()) * 9.8065 / 16384.0;
  node.imu_msg.linear_acceleration.y = stod(data[5].data()) * 9.8065 / 16384.0;
  node.imu_msg.linear_acceleration.z = stod(data[6].data()) * 9.8065 / 16384.0;

  node.imu_msg.angular_velocity.x = stod(data[7].data()) * M_PI/ (180 * 65.5);
  node.imu_msg.angular_velocity.y = stod(data[8].data()) * M_PI/ (180 * 65.5);
  node.imu_msg.angular_velocity.z = stod(data[9].data()) * M_PI/ (180 * 65.5);

  node.imu_msg.header.frame_id = "gps_ins_point_link";
  node.imu_msg.header.stamp = ros::Time::now();
  node.pubIMU.publish(node.imu_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps-ahrs_receiver");
  QCoreApplication a(argc, argv);
  GPSAHRSReceiver receiver;
  return a.exec();
}

