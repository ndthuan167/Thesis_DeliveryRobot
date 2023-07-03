#include "mission_protocol.h"
#include "gcs_transceiver.h"

MissionProtocol::MissionProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link)
    : link(link), trans(trans), itemCount(0)
{
  ros::NodeHandle nh;
  pubItemList = nh.advertise<WaypointList>("mission/item_list", 1);
}

void MissionProtocol::handleMission(const mavlink_message_t* msg)
{
  switch (msg->msgid)
  {
  case 44: // MISSION_COUNT
  case 73: // MISSION_ITEM_INT
    uploadMissionToVehicle(msg);
    break;
  case 43: // MISSION_REQUEST_LIST
  case 51: // MISSION_REQUEST_INT
    downloadMissionFromVehicle(msg);
    break;
  case 41: // MISSION_SET_CURRENT
    setCurrentMissionItem(msg);
    break;
  case 45: // MISSION_CLEAR_ALL
    clearMission(msg);
    break;
  }
}

void MissionProtocol::uploadMissionToVehicle(const mavlink_message_t* msg)
{
  switch (msg->msgid)
  {
  case 44: // MISSION_COUNT
  {
    MISSION_COUNT reqPack;
    unpack_mavlink_message_t(msg, reqPack);

    if (check_for_target_id_matching(reqPack, trans->autopilot_sysid, trans->autopilot_compid))
    {
      ROS_INFO_STREAM(reqPack.to_yaml());

      itemCount = reqPack.count;
      missionType = static_cast<uint8_t>(reqPack.mission_type);
      itemList.waypoints.clear();
      itemList.waypoints.resize(itemCount);

      MISSION_REQUEST_INT resPack;
      resPack.seq = 0;
      resPack.mission_type = missionType;
      pack_and_send_mavlink_message_t(resPack, link);
    }
  }
  break;
  case 73: // MISSION_ITEM_INT
  {
    MISSION_ITEM_INT reqPack;
    unpack_mavlink_message_t(msg, reqPack);

    if (check_for_target_id_matching(reqPack, trans->autopilot_sysid, trans->autopilot_compid))
    {
      ROS_INFO_STREAM(reqPack.to_yaml());

      uint16_t currSeq = reqPack.seq;
      itemList.waypoints[currSeq].frame = reqPack.frame;
      itemList.waypoints[currSeq].command = reqPack.command;
      itemList.waypoints[currSeq].is_current = reqPack.current;
      itemList.waypoints[currSeq].autocontinue = reqPack.autocontinue;
      itemList.waypoints[currSeq].param1 = reqPack.param1;
      itemList.waypoints[currSeq].param2 = reqPack.param2;
      itemList.waypoints[currSeq].param3 = reqPack.param3;
      itemList.waypoints[currSeq].param4 = reqPack.param4;
      itemList.waypoints[currSeq].x_lat = static_cast<double>(reqPack.x);
      itemList.waypoints[currSeq].y_long = static_cast<double>(reqPack.y);
      itemList.waypoints[currSeq].z_alt = static_cast<double>(reqPack.z);

      if (currSeq < itemCount - 1)
      {
        MISSION_REQUEST_INT resPack;
        resPack.seq = currSeq + 1;
        resPack.mission_type = missionType;
        pack_and_send_mavlink_message_t(resPack, link);
      }
      else
      {
        MISSION_ACK resPack;
        resPack.type = static_cast<uint8_t>(MAV_MISSION_RESULT::ACCEPTED);
        resPack.mission_type = missionType;
        pack_and_send_mavlink_message_t(resPack, link);

        pubItemList.publish(itemList);
      }
    }
  }
  break;
  default:
    break;
  }
}

void MissionProtocol::downloadMissionFromVehicle(const mavlink_message_t* /*msg*/) {}

void MissionProtocol::setCurrentMissionItem(const mavlink_message_t* /*msg*/) {}

void MissionProtocol::clearMission(const mavlink_message_t* /*msg*/) {}
