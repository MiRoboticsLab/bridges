#ifndef CYBERDOG_OTA_MANAGER_MCU_HPP_
#define CYBERDOG_OTA_MANAGER_MCU_HPP_

#include "cyberdog_ota/command_mcu.hpp"
#include "cyberdog_ota/threading.hpp"
#include "cyberdog_ota/factory.hpp"

#include <memory>
#include <string>

namespace cyberdog
{

class McuManager : public Thread
{
public:
  McuManager(std::shared_ptr<Factory> factory);
  ~McuManager();

  McuManager(const McuManager&) = delete;
  McuManager& operator=(const McuManager&) = delete;

  virtual void Run() override;

  enum class Mode
  {
    kStartUpgrade,
    kForceUpgrade,
    kQueryVersion
  };

  struct ReturnCommandResult
  {
    bool success;
    std::string version;
    double time;
  };

  // Run comamnd and get it's return result
  ReturnCommandResult RunCommand(const Mode& mode);

private:
  std::shared_ptr<Factory> factory_;
  std::shared_ptr<CommandInterface> command_;
};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_MANAGER_MCU_HPP_
