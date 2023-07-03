#ifndef MSG_HANDLER_H
#define MSG_HANDLER_H

#include "mavconn/interface.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;

template <typename T> inline void pack_and_send_mavlink_message_t(const T& pack, const MAVConnInterface::Ptr link)
{
  mavlink_message_t msg;
  MsgMap map(msg);
  pack.serialize(map);
  mavlink_finalize_message(&msg, link->get_system_id(), link->get_component_id(), pack.MIN_LENGTH, pack.LENGTH,
                           pack.CRC_EXTRA);

  link->send_message(&msg);
}

template <typename T, typename K> inline void unpack_mavlink_message_t(const K& msg, T& pack)
{
  MsgMap map(msg);
  pack.deserialize(map);
}

template <typename T>
inline bool check_for_target_id_matching(const T& pack, const uint8_t& target_sysid, const uint8_t& target_compid)
{
  return pack.target_system == target_sysid &&
         (pack.target_component == target_compid ||
          pack.target_component == static_cast<uint8_t>(MAV_COMPONENT::COMP_ID_ALL));
}

#endif // MSG_HANDLER_H
