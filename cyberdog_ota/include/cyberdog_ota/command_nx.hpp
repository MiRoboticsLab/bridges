#ifndef CYBERDOG_OTA_COMMAND_NX_HPP_
#define CYBERDOG_OTA_COMMAND_NX_HPP_

#include "cyberdog_ota/threading.hpp"
#include "cyberdog_ota/command_interface.hpp"
#include "cyberdog_ota/protocol_app_message.hpp"

#include <tuple>

namespace cyberdog
{

class NxCommand : public CommandInterface 
{
public:
  NxCommand();
  virtual ~NxCommand();

  NxCommand(const NxCommand&) = delete;
  NxCommand& operator=(const NxCommand&) = delete;

  virtual bool Execute(const CommandType& type) override;

  bool Execute(const CommandType& type, std::tuple<SoftInfo, HardwareInfo>& info);

  virtual ExecuteState GetExecuteState() override;

private:
  bool QueryVersion(std::tuple<SoftInfo, HardwareInfo>& info);
  bool StartUpgrade(std::tuple<SoftInfo, HardwareInfo>& info, bool force_upgrade = false);
};


}  // namespace cyberdog

#endif  // CYBERDOG_OTA_COMMAND_NX_HPP_