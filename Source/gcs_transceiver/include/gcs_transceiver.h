#ifndef GCS_TRANSCEIVER_H
#define GCS_TRANSCEIVER_H

#include <ros/ros.h>

#include <utils/Odometry.h>
#include <utils/Setpoint.h>
#include <utils/DiffVel.h>
#include <utils/Error.h>

#include "utils/geo.h"
#include "mavconn/interface.h"
#include "mavconn/mavlink_dialect.h"

#include "msg_handler.h"
#include "mission_protocol.h"
#include "command_protocol.h"
#include "parameter_protocol.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace mavlink::viamlab_agv;
using namespace mavlink::viamlab_agv::msg;
using namespace utils;

class GCSTransceiver
{
public:
  GCSTransceiver();
  ~GCSTransceiver();

  MAVConnInterface::Ptr MainLink;

  HEARTBEAT Heartbeat;

  uint8_t autopilot_sysid;
  uint8_t autopilot_compid;
  uint8_t gcs_sysid;
  uint8_t gcs_compid;

  double main_period;
  std::string main_url;

  ros::Timer loopMainLink;
  ros::Timer loopGlobalPos;
  ros::Subscriber subOdom;
  ros::Subscriber subSetpoint;
  ros::Subscriber subDiffVel;
  ros::Subscriber subError;

  MissionProtocol* missionProtocol = nullptr;
  CommandProtocol* commandProtocol = nullptr;
  ParameterProtocol* parameterProtocol = nullptr;

  double lat, lon;

  void setupHeartbeat();

  void onMainLinkLoop(const ros::TimerEvent& event);
  void onGlobalPosLoop(const ros::TimerEvent& event);
  void onMainLinkCallBack(const mavlink_message_t* msg, const Framing framing);

  void onOdomCallBack(const Odometry::ConstPtr& msg);
  void onSetpointCallBack(const Setpoint::ConstPtr& msg);
  void onDiffVelCallBack(const DiffVel::ConstPtr& msg);
  void onErrorCallBack(const Error::ConstPtr& msg);
};

#endif // GCS_TRANSCEIVER_H
