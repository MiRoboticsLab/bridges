#include "cyberdog_ota/command_mcu.hpp"

#include <chrono>
#include <iostream>

namespace cyberdog
{

McuCommand::McuCommand()
{

}

McuCommand::~McuCommand()
{

}

bool McuCommand::Execute(const CommandType& type)
{
  if (type == CommandType::kQueryVerion) {
    return QueryVersion();
  } else if (type == CommandType::kForceUpgrade) {
    return StartUpgrade(true);
  } 
  return true;
}

ExecuteState McuCommand::GetExecuteState()
{
  return ExecuteState::kUnknown;
}

bool McuCommand::QueryVersion()
{
  return true;
}

std::tuple<SoftInfo, HardwareInfo>& McuCommand::GetSoftHardwareInfo()
{
  // SoftInfo soft;
  return soft_hardware_info_;
}

bool McuCommand::StartUpgrade(bool force)
{
  return true;
}

}  // namespace cyberdog

