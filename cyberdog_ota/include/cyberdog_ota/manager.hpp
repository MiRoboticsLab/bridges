#ifndef CYBERDOG_OTA_MANAGER_HPP_
#define CYBERDOG_OTA_MANAGER_HPP_

#include "cyberdog_ota/factory.hpp"
#include "cyberdog_ota/manager_nx.hpp"
#include "cyberdog_ota/manager_mcu.hpp"
#include "cyberdog_ota/server.hpp"
#include "cyberdog_ota/client.hpp"

#include <map>
#include <memory>
#include <string>

namespace cyberdog
{

class Manager
{
public:
  Manager();
  ~Manager();
  
  Manager(const Manager&) = delete;
  Manager& operator=(const Manager&) = delete;

  bool RunVersionQueryCommand(std::string& json_result);
  bool RunStatusQueryCommand(std::string& json_result);
  bool RunProcessQueryCommand(std::string& json_result);
  bool RunStartUpgradeCommand(std::string& json_result);
  bool RunStartDownloadCommand(std::string& json_result);
  bool RunEstimateUpgradeTimeQueryCommand(std::string& json_result);


  // Update 
  //  -- MCU Update
  //  -- NX Update
  //  -- R329 Update
  //  -- MR813 Update
  bool RunStartUpdateCommand();

  // http download
  bool RunDownloadFilesCommand(); 


  std::shared_ptr<NxManager> GetNxManager();
  std::shared_ptr<McuManager> GetMcuManager();

  

private:
  std::shared_ptr<Server> server_;
  std::shared_ptr<Client> http_client_;

  std::shared_ptr<Factory> factory_;
  std::shared_ptr<NxManager> nx_;
  std::shared_ptr<McuManager> mcu_;
};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_MANAGER_HPP_

