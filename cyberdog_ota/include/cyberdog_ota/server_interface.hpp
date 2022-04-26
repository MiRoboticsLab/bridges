#ifndef CYBERDOG_OTA_SERVER_INTERFACE_HPP_
#define CYBERDOG_OTA_SERVER_INTERFACE_HPP_

#include <stddef.h>

namespace cyberdog
{

class ServerInterface
{
public:
  ServerInterface() {};
  virtual ~ServerInterface() {};

  ServerInterface(const ServerInterface&) = delete;
  ServerInterface& operator=(const ServerInterface&) = delete;

  virtual bool Initialize() = 0;
  virtual bool SendMessage(char* buf, size_t size) = 0;
  virtual bool ReceiveMessage(char* buf, size_t size) = 0;
};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_SERVER_INTERFACE_HPP_