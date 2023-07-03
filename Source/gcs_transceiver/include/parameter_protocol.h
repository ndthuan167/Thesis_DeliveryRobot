#ifndef PARAMETER_PROTOCOL_H
#define PARAMETER_PROTOCOL_H

#include <ros/ros.h>

#include <utils/ParamGet.h>
#include <utils/ParamSet.h>

#include "msg_handler.h"
#include "mavconn/interface.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace utils;

class GCSTransceiver;

class ParameterProtocol
{
public:
  using StringList = std::list<std::string>;

  ParameterProtocol();
  ParameterProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link);

  void handleParameter(const mavlink_message_t* msg);

private:
  ros::ServiceClient reqSetHeadingPID;
  ros::ServiceClient reqGetHeadingPID;
  ros::ServiceClient reqSetSetpointParams;
  ros::ServiceClient reqGetSetpointParams;
  ros::ServiceClient reqSetLOSParams;
  ros::ServiceClient reqGetLOSParams;

  MAVConnInterface::Ptr link;
  GCSTransceiver* trans;

  StringList headingPID;
  StringList setpointParams;
  StringList LOSParams;

  void readAllParameters(const mavlink_message_t* msg);
  void readParameter(const mavlink_message_t* msg);
  void writeParameter(const mavlink_message_t* msg);

  bool requestSetParameter(ParamSet& srv);
  bool requestGetParameter(ParamGet& srv, const StringList& paramList);

  void readParamList(const StringList& paramList, const uint16_t& numParams, uint16_t& paramId);

  inline bool paramListMatched(const StringList& paramList, const char* paramId)
  {
    for (auto it = paramList.begin(); it != paramList.end(); it++)
      if (!strncmp(paramId, it->data(), it->size()))
        return true;
    return false;
  }
};

#endif // PARAMETER_PROTOCOL_H
