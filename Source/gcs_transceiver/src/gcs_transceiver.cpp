#include "gcs_transceiver.h"

GCSTransceiver::GCSTransceiver()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("main_period", main_period);
  private_nh.getParam("main_url", main_url);

  int autopilot_sysid_, gcs_sysid_;
  private_nh.getParam("autopilot_sysid", autopilot_sysid_);
  private_nh.getParam("gcs_sysid", gcs_sysid_);
  autopilot_sysid = static_cast<uint8_t>(autopilot_sysid_);
  gcs_sysid = static_cast<uint8_t>(gcs_sysid_);

  autopilot_compid = static_cast<uint8_t>(MAV_COMPONENT::COMP_ID_AUTOPILOT1);
  gcs_compid = static_cast<uint8_t>(MAV_COMPONENT::COMP_ID_ALL);

  MainLink = MAVConnInterface::open_url(main_url);
  MainLink->set_system_id(autopilot_sysid);
  MainLink->set_component_id(autopilot_compid);
  MainLink->message_received_cb = boost::bind(&GCSTransceiver::onMainLinkCallBack, this, _1, _2);

  missionProtocol = new MissionProtocol(this, MainLink);
  commandProtocol = new CommandProtocol(this, MainLink);
  parameterProtocol = new ParameterProtocol(this, MainLink);

  setupHeartbeat();

  ros::NodeHandle nh;
  loopMainLink = nh.createTimer(ros::Duration(main_period), &GCSTransceiver::onMainLinkLoop, this);
  loopGlobalPos = nh.createTimer(ros::Duration(0.1), &GCSTransceiver::onGlobalPosLoop, this);
  subOdom = nh.subscribe("odom_1", 10, &GCSTransceiver::onOdomCallBack, this);
  subSetpoint = nh.subscribe("setpoint", 10, &GCSTransceiver::onSetpointCallBack, this);
  subDiffVel = nh.subscribe("diff_motor/vel", 10, &GCSTransceiver::onDiffVelCallBack, this);
  subError = nh.subscribe("error", 10, &GCSTransceiver::onErrorCallBack, this);
}

GCSTransceiver::~GCSTransceiver()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void GCSTransceiver::setupHeartbeat()
{
  Heartbeat.type = static_cast<uint8_t>(MAV_TYPE::GROUND_ROVER);
  Heartbeat.autopilot = static_cast<uint8_t>(MAV_AUTOPILOT::GENERIC);
  Heartbeat.base_mode = static_cast<uint8_t>(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);
}

void GCSTransceiver::onMainLinkLoop(const ros::TimerEvent& /*event*/)
{
  pack_and_send_mavlink_message_t(Heartbeat, MainLink);
}

void GCSTransceiver::onGlobalPosLoop(const ros::TimerEvent &event)
{
  GLOBAL_POSITION_INT pack;
  pack.time_boot_ms = static_cast<uint32_t>(event.current_real.toNSec() * 1e6);
  pack.lat = static_cast<int32_t>(lat * 1e7);
  pack.lon = static_cast<int32_t>(lon * 1e7);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onMainLinkCallBack(const mavlink_message_t* msg, const Framing framing)
{
  if (framing == mavconn::Framing::ok)
  {
    ROS_INFO_STREAM("msgid = " << int(msg->msgid) << ", sysid = " << int(msg->sysid)
                               << ", compid = " << int(msg->compid));

    if (msg->sysid == gcs_sysid && msg->compid == gcs_compid)
    {
      switch (msg->msgid)
      {
      case 44:
      case 73:
      case 43:
      case 51:
      case 41:
      case 45:
        missionProtocol->handleMission(msg);
        break;
      case 76:
      case 75:
      case 11:
        commandProtocol->handleCommand(msg);
        break;
      case 21:
      case 20:
      case 23:
        parameterProtocol->handleParameter(msg);
        break;
      default:
        break;
      }
    }
  }
}

void GCSTransceiver::onOdomCallBack(const Odometry::ConstPtr& msg)
{
  VIAM_ODOMETRY pack;
  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e6);
  pack.lat = static_cast<int32_t>(msg->latitude * 1e7);
  pack.lon = static_cast<int32_t>(msg->longitude * 1e7);
  pack.alt = static_cast<float>(msg->altitude);
  pack.x = static_cast<float>(msg->position.x);
  pack.y = static_cast<float>(msg->position.y);
  pack.z = static_cast<float>(msg->position.z);
  pack.vx = static_cast<float>(msg->linear_velocity.x);
  pack.vy = static_cast<float>(msg->linear_velocity.y);
  pack.vz = static_cast<float>(msg->linear_velocity.z);
  pack.roll = static_cast<float>(msg->orientation.x);
  pack.pitch = static_cast<float>(msg->orientation.y);
  pack.yaw = static_cast<float>(msg->orientation.z);
  pack.vroll = static_cast<float>(msg->angular_velocity.x);
  pack.vpitch = static_cast<float>(msg->angular_velocity.y);
  pack.vyaw = static_cast<float>(msg->angular_velocity.z);
  pack.ax = static_cast<float>(msg->linear_acceleration.x);
  pack.ay = static_cast<float>(msg->linear_acceleration.y);
  pack.az = static_cast<float>(msg->linear_acceleration.z);
  pack.aroll = static_cast<float>(msg->angular_acceleration.x);
  pack.apitch = static_cast<float>(msg->angular_acceleration.y);
  pack.ayaw = static_cast<float>(msg->angular_acceleration.z);
  pack_and_send_mavlink_message_t(pack, MainLink);

  lat = msg->latitude;
  lon = msg->longitude;
}

void GCSTransceiver::onSetpointCallBack(const Setpoint::ConstPtr& msg)
{
  VIAM_SETPOINT pack;
  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e6);
  pack.x = static_cast<float>(msg->position.x);
  pack.y = static_cast<float>(msg->position.y);
  pack.z = static_cast<float>(msg->position.z);
  pack.vx = static_cast<float>(msg->linear_velocity.x);
  pack.vy = static_cast<float>(msg->linear_velocity.y);
  pack.vz = static_cast<float>(msg->linear_velocity.z);
  pack.roll = static_cast<float>(msg->orientation.x);
  pack.pitch = static_cast<float>(msg->orientation.y);
  pack.yaw = static_cast<float>(msg->orientation.z);
  pack.vroll = static_cast<float>(msg->angular_velocity.x);
  pack.vpitch = static_cast<float>(msg->angular_velocity.y);
  pack.vyaw = static_cast<float>(msg->angular_velocity.z);
  pack.ax = static_cast<float>(msg->linear_acceleration.x);
  pack.ay = static_cast<float>(msg->linear_acceleration.y);
  pack.az = static_cast<float>(msg->linear_acceleration.z);
  pack.aroll = static_cast<float>(msg->angular_acceleration.x);
  pack.apitch = static_cast<float>(msg->angular_acceleration.y);
  pack.ayaw = static_cast<float>(msg->angular_acceleration.z);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onDiffVelCallBack(const DiffVel::ConstPtr& msg)
{
  VIAM_DIFF_VEL pack;
  pack.left_vel = static_cast<float>(msg->left_vel);
  pack.right_vel = static_cast<float>(msg->right_vel);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onErrorCallBack(const Error::ConstPtr& msg)
{
  VIAM_ERROR pack;
  pack.along_track = static_cast<float>(msg->along_track);
  pack.cross_track = static_cast<float>(msg->cross_track);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gcs_transceiver");
  GCSTransceiver trans;
  ros::spin();
}
