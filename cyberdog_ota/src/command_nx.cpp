#include "cyberdog_ota/command_nx.hpp"

#include <chrono>
#include <iostream>

namespace cyberdog
{

NxCommand::NxCommand()
{

}

NxCommand::~NxCommand()
{

}

bool NxCommand::Execute(const CommandType& type)
{
  
  return true;
}

bool NxCommand::Execute(const CommandType& type, std::tuple<SoftInfo, HardwareInfo>& info)
{
  if (type == CommandType::kQueryVerion) {
    return QueryVersion(info);
  } else if (type == CommandType::kForceUpgrade) {
    return StartUpgrade(info, true);
  } 

  return true;
}

ExecuteState NxCommand::GetExecuteState()
{
  return ExecuteState::kUnknown;
}

bool NxCommand::QueryVersion(std::tuple<SoftInfo, HardwareInfo>& info)
{
  return true;
}

bool NxCommand::StartUpgrade(
  std::tuple<SoftInfo, HardwareInfo>& info, bool force_upgrade)
{
  return true;
}

}  // namespace cyberdog

