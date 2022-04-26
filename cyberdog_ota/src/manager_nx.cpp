#include "cyberdog_ota/manager_nx.hpp"

#include <iostream>

namespace cyberdog
{

NxManager::NxManager(std::shared_ptr<Factory> factory)
  : factory_(factory)
{
  command_ = factory_->CreateNxCommand();
  if (command_ == nullptr) {
    std::cout << "Create NX comand object error." << std::endl;
  }

  std::cout << "intialize NxManager() funciton success." << std::endl;
}


NxManager::~NxManager()
{

}

void NxManager::Run()
{
  while (true) {
    std::cout << " NxManager::Run() " << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

}  // namespace cyberdog