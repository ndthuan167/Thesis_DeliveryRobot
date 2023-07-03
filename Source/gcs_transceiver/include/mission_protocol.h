#ifndef MISSION_PROTOCOL_H
#define MISSION_PROTOCOL_H

#include <ros/ros.h>
#include <utils/WaypointList.h>

#include "msg_handler.h"
#include "mavconn/interface.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace utils;

class GCSTransceiver;

class MissionProtocol
{
public:
  MissionProtocol();
  MissionProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link);

  void handleMission(const mavlink_message_t* msg);

private:
  ros::Publisher pubItemList;

  MAVConnInterface::Ptr link;
  GCSTransceiver* trans;

  WaypointList itemList;
  uint16_t itemCount;
  uint8_t missionType;

  void uploadMissionToVehicle(const mavlink_message_t* msg);
  void downloadMissionFromVehicle(const mavlink_message_t* msg);
  void setCurrentMissionItem(const mavlink_message_t* msg);
  void clearMission(const mavlink_message_t* msg);
};

#endif // MISSION_PROTOCOL_H
