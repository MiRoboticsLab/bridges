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


// #define STATUS_QUERY_ID                   0x0001
// #define VERSION_QUERY_ID                  0x0002
// #define START_DOWNLOAD_ID                 0x0003
// #define START_UPDATE_ID                   0x0004
// #define DOWNLOAD_PROGRESS_QUERY_ID        0x0005
// #define UPDATE_PROGRESS_QUERY_ID          0x0006
// #define UPDATE_TIME_ID                    0x0007
// #define ABORT_DOWNLOAD_ID                 0x0008

  bool RunVersionQueryCommand();
  bool RunStatusQueryCommand();

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

