#ifndef CYBERDOG_OTA_CLIENT_INTERFACE_HPP_
#define CYBERDOG_OTA_CLIENT_INTERFACE_HPP_


namespace cyberdog
{

class ClientInterface
{
public:
  ClientInterface() {};
  virtual ~ClientInterface() {};

  ClientInterface(const ClientInterface&) = delete;
  ClientInterface& operator=(const ClientInterface&) = delete;

  virtual bool Initialize() = 0;
};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_CLIENT_INTERFACE_HPP_
