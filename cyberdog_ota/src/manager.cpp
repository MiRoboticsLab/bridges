#include "cyberdog_ota/manager.hpp"
#include "cyberdog_ota/command_nx.hpp"


#include <iostream>

namespace cyberdog
{


Manager::Manager()
{
  std::cout << "This Manager() funciton..." << std::endl;

  // Server
  server_ = std::make_shared<Server>(this);
  http_client_ = std::make_shared<Client>();

  // Create 
  factory_ = std::make_shared<Factory>();
  nx_ = std::make_shared<NxManager>(factory_);
  mcu_ = std::make_shared<McuManager>(factory_);

  // Start thread monitoring
  nx_->Start();
  mcu_->Start();

  std::cout << "Manager initialize success." << std::endl;
}

Manager::~Manager()
{
  if (nx_->IsRunning()) {
    nx_->Stop();
  }
  
  if (mcu_->IsRunning()) {
    mcu_->Stop();
  }
}

bool Manager::RunVersionQueryCommand(std::string& json_result)
{
  return true;
}

bool Manager::RunStatusQueryCommand(std::string& json_result)
{
  return true;
}

bool Manager::RunProcessQueryCommand(std::string& json_result)
{
  return true;
}

bool Manager::RunStartUpgradeCommand(std::string& json_result)
{
  return true;
}

bool Manager::RunStartDownloadCommand(std::string& json_result)
{
  return true;
}

bool Manager::RunEstimateUpgradeTimeQueryCommand(std::string& json_result)
{
  return true;
}


bool Manager::RunDownloadFilesCommand()
{

  std::string url = "https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag";

  std::string url1 = "https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag";

  std::string path = "/home/quan/Downloads/b3-2016-04-05-14-14-00.bag";
  http_client_->Download(nullptr, url1, path, false, path.c_str());
  return true;
}

std::shared_ptr<NxManager> Manager::GetNxManager()
{
  return nx_;
}

std::shared_ptr<McuManager> Manager::GetMcuManager()
{
  return mcu_;
}

}  // namespace cyberdog