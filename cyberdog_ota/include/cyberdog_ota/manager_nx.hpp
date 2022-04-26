#ifndef CYBERDOG_OTA_MANAGER_NX_HPP_
#define CYBERDOG_OTA_MANAGER_NX_HPP_

#include "cyberdog_ota/threading.hpp"
#include "cyberdog_ota/factory.hpp"

#include <memory>
#include <string>

namespace cyberdog
{

class NxManager : public Thread
{
public:
  NxManager(std::shared_ptr<Factory> factory);
  ~NxManager();
  
  NxManager(const NxManager&) = delete;
  NxManager& operator=(const NxManager&) = delete;

  virtual void Run() override;

private:
  std::shared_ptr<Factory> factory_;
  std::shared_ptr<CommandInterface> command_;
};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_MANAGER_NX_HPP_
