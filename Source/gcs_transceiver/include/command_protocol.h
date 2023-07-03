#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H

#include <ros/ros.h>

#include <utils/CommandInt.h>
#include <utils/CommandLong.h>
#include <utils/SetMode.h>

#include "msg_handler.h"
#include "mavconn/interface.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace utils;

class GCSTransceiver;

class CommandProtocol
{
public:
  CommandProtocol();
  CommandProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link);

  void handleCommand(const mavlink_message_t* msg);

private:
  ros::ServiceClient reqSetArming;
  ros::ServiceClient reqStartMission;
  ros::ServiceClient reqSetMode;

  MAVConnInterface::Ptr link;
  GCSTransceiver* trans;

  void handleCommandLong(const mavlink_message_t* msg);
  void handleCommandInt(const mavlink_message_t* msg);

  bool requestCommandLong(CommandLong& srv);
  bool requestCommandInt(CommandInt& srv);

  void updateSystemStatus(const CommandLong& srv);
  void updateSystemStatus(const CommandInt& srv);

  void updateArmStatus(const CommandLong& srv);

  void handleSetMode(const mavlink_message_t* msg);
};

#endif // COMMAND_PROTOCOL_H
