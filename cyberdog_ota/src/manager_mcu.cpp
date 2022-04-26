#include "cyberdog_ota/manager_mcu.hpp"

#include <iostream>
#include <chrono>

namespace cyberdog
{

McuManager::McuManager(std::shared_ptr<Factory> factory)
  : factory_(factory)
{
  command_ = factory_->CreateMcuCommand();
  if (command_ == nullptr) {
    std::cout << "Create MCU comand object error." << std::endl;
  }

  std::cout << "intialize McuManager() funciton success." << std::endl;
}

McuManager::~McuManager()
{

}

void McuManager::Run()
{
  while (true) {
    std::cout << " McuManager::Run() " << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

McuManager::ReturnCommandResult McuManager::RunCommand(const Mode& mode)
{
  McuManager::ReturnCommandResult result;
  if (mode == Mode::kForceUpgrade || mode == Mode::kStartUpgrade) {
     result.success = command_->Execute(CommandType::kForceUpgrade) ? true : false;
  } else if (mode == Mode::kQueryVersion) {
     result.success = command_->Execute(CommandType::kQueryVerion) ? true : false;
  } 
  return result;
}

}  // namespace cyberdog