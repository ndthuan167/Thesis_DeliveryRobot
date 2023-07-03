#include "parameter_protocol.h"
#include "gcs_transceiver.h"

ParameterProtocol::ParameterProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link) : link(link), trans(trans)
{
  headingPID = {"heading_Kp", "heading_Ki", "heading_Kd"};
  setpointParams = {"set_speed", "set_heading"};
  LOSParams = {"LOS_radius", "LOS_min_delta", "LOS_max_delta", "LOS_beta"};

  ros::NodeHandle nh;
  reqSetHeadingPID = nh.serviceClient<ParamSet>("parameter/set_heading_PID");
  reqGetHeadingPID = nh.serviceClient<ParamGet>("parameter/get_heading_PID");
  reqSetSetpointParams = nh.serviceClient<ParamSet>("parameter/set_setpoint_params");
  reqGetSetpointParams = nh.serviceClient<ParamGet>("parameter/get_setpoint_params");
  reqSetLOSParams = nh.serviceClient<ParamSet>("parameter/set_LOS_params");
  reqGetLOSParams = nh.serviceClient<ParamGet>("parameter/get_LOS_params");
}

void ParameterProtocol::handleParameter(const mavlink_message_t* msg)
{
  switch (msg->msgid)
  {
  case 21: // PARAM_REQUEST_LIST
    readAllParameters(msg);
    break;
  case 20: // PARAM_REQUEST_READ
    readParameter(msg);
    break;
  case 23: // PARAM_SET
    writeParameter(msg);
    break;
  }
}

void ParameterProtocol::readAllParameters(const mavlink_message_t* msg)
{
  PARAM_REQUEST_LIST reqPack;
  unpack_mavlink_message_t(msg, reqPack);

  if (check_for_target_id_matching(reqPack, trans->autopilot_sysid, trans->autopilot_compid))
  {
    ROS_INFO_STREAM(reqPack.to_yaml());

    uint16_t numParams = static_cast<uint16_t>(headingPID.size() + setpointParams.size() + LOSParams.size());
    uint16_t paramId = 0;

    readParamList(headingPID, numParams, paramId);
    readParamList(setpointParams, numParams, paramId);
    readParamList(LOSParams, numParams, paramId);
  }
}

void ParameterProtocol::readParameter(const mavlink_message_t* /*msg*/) {}

void ParameterProtocol::writeParameter(const mavlink_message_t* msg)
{
  PARAM_SET reqPack;
  unpack_mavlink_message_t(msg, reqPack);

  if (check_for_target_id_matching(reqPack, trans->autopilot_sysid, trans->autopilot_compid))
  {
    ROS_INFO_STREAM(reqPack.to_yaml());

    ParamSet srv;
    srv.request.param_id = reqPack.param_id.data();
    if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::INT32))
      memcpy(&srv.request.value.integer, &reqPack.param_value, sizeof(float));
    else if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32))
      srv.request.value.real = static_cast<double>(reqPack.param_value);

    if (requestSetParameter(srv))
    {
      PARAM_VALUE resPack;
      resPack.param_id = reqPack.param_id;
      if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::INT32))
        memcpy(&resPack.param_value, &srv.response.value.integer, sizeof(float));
      else if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32))
        resPack.param_value = static_cast<float>(srv.response.value.real);
      resPack.param_type = reqPack.param_type;
      pack_and_send_mavlink_message_t(resPack, link);
    }
  }
}

void ParameterProtocol::readParamList(const StringList& paramList, const uint16_t& numParams, uint16_t& paramId)
{
  ParamGet srv;
  PARAM_VALUE resPack;

  for (auto it = paramList.begin(); it != paramList.end(); it++)
  {
    srv.request.param_id = it->data();

    if (requestGetParameter(srv, paramList))
    {
      strcpy(resPack.param_id.data(), it->data());

      if (paramList == headingPID || paramList == setpointParams || paramList == LOSParams)
      {
        resPack.param_value = static_cast<float>(srv.response.value.real);
        resPack.param_type = static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32);
      }

      resPack.param_count = numParams;
      resPack.param_index = paramId++;
      pack_and_send_mavlink_message_t(resPack, link);
    }
  }
}

bool ParameterProtocol::requestSetParameter(ParamSet& srv)
{
  if (paramListMatched(headingPID, srv.request.param_id.data()))
    return reqSetHeadingPID.call(srv);
  if (paramListMatched(setpointParams, srv.request.param_id.data()))
    return reqSetSetpointParams.call(srv);
  if (paramListMatched(LOSParams, srv.request.param_id.data()))
    return reqSetLOSParams.call(srv);

  return false;
}

bool ParameterProtocol::requestGetParameter(ParamGet& srv, const StringList& paramList)
{
  if (paramList == headingPID)
    return reqGetHeadingPID.call(srv);
  if (paramList == setpointParams)
    return reqGetSetpointParams.call(srv);
  if (paramList == LOSParams)
    return reqGetLOSParams.call(srv);

  return false;
}
