#include "cyberdog_ota/factory.hpp"
#include "cyberdog_ota/command_nx.hpp"
#include "cyberdog_ota/command_mcu.hpp"

#include <iostream>

namespace cyberdog
{

// const std::string kMonitorMCUName = "monitor_mcu";
// const std::string kMonitorNXName = "monitor_nx";
// const std::string kMonitorMR813Name = "monitor_MR823";
// const std::string kMonitorM329Name = "monitor_R329";


std::shared_ptr<CommandInterface> Factory::CreateNxCommand()
{
  return std::make_shared<NxCommand>();
}

std::shared_ptr<CommandInterface> Factory::CreateMcuCommand()
{
  return std::make_shared<McuCommand>();
}

}  // namespace cyberdog