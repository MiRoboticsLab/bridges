#ifndef CYBERDOG_OTA_COMMAND_MCU_HPP_
#define CYBERDOG_OTA_COMMAND_MCU_HPP_

#include "cyberdog_ota/threading.hpp"
#include "cyberdog_ota/command_interface.hpp"
#include "cyberdog_ota/protocol_app_message.hpp"

#include <tuple>

namespace cyberdog
{

class McuCommand : public CommandInterface
{
public:
  McuCommand();
  virtual ~McuCommand();

  McuCommand(const McuCommand&) = delete;
  McuCommand& operator=(const McuCommand&) = delete;

  virtual bool Execute(const CommandType& type) override;

  bool Execute(const CommandType& type, std::tuple<SoftInfo, HardwareInfo>& info);

  virtual ExecuteState GetExecuteState() override;

  std::tuple<SoftInfo, HardwareInfo>& GetSoftHardwareInfo();

private:
  bool QueryVersion();
  bool StartUpgrade(bool force = false);

  std::tuple<SoftInfo, HardwareInfo> soft_hardware_info_;

};


}  // namespace cyberdog

#endif  // CYBERDOG_OTA_COMMAND_MCU_HPP_