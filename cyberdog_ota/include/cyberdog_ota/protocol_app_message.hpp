#ifndef CYBERDOG_OTA_PROTOCOL_APP_MESSAGE_HPP_
#define CYBERDOG_OTA_PROTOCOL_APP_MESSAGE_HPP_

#include <string>

namespace cyberdog
{

struct SoftInfo
{
  std::string version;
  double update_time;
};

struct HardwareInfo
{
  std::string version;
  std::string chip;
};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_PROTOCOL_APP_MESSAGE_HPP_