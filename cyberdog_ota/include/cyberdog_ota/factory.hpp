#ifndef CYBERDOG_OTA_FACTORY_HPP_
#define CYBERDOG_OTA_FACTORY_HPP_

#include "cyberdog_ota/command_interface.hpp"
#include "cyberdog_ota/factory_interface.hpp"

#include <memory>

namespace cyberdog
{

class Factory : public FactoryInteface
{
public:
  Factory() {};
  virtual ~Factory() {};

  Factory(const Factory&) = delete;
  Factory& operator=(const Factory&) = delete;

  std::shared_ptr<CommandInterface> CreateNxCommand();
  std::shared_ptr<CommandInterface> CreateMcuCommand();
};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_FACTORY_HPP_