#ifndef CYBERDOG_OTA_PROTOCOL_SERVER_MESSAGE_HPP_
#define CYBERDOG_OTA_PROTOCOL_SERVER_MESSAGE_HPP_

#include "cyberdog_ota/protocol_app_message.hpp"

#include <tuple>

namespace cyberdog
{

enum CommandID
{
  kVersionQuery,
  kStatusQuery,
  kStartDownload,
  kStartUpdate,
  kAbortDownload,
  kUpdateTime,
  kUpdateProcessQuery,
  kDownloadProcessQuery
};

struct ServerRequestMessage
{
  CommandID command;
};

struct ServerResponseMessage 
{ 
  SoftInfo mcu_sinfo;
  SoftInfo nx_sinfo;
 
  HardwareInfo nx_hdinfo;
  HardwareInfo mcu_hdinfo;
};

using ServerMessage = std::tuple<ServerRequestMessage, ServerResponseMessage>;

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_PROTOCOL_SERVER_MESSAGE_HPP_