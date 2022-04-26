#ifndef CYBERDOG_OTA_FACTORY_INTERFACE_HPP_
#define CYBERDOG_OTA_FACTORY_INTERFACE_HPP_

#include <string>

namespace cyberdog
{

class FactoryInteface
{
public:
  FactoryInteface() {};
  virtual ~FactoryInteface() {};

  FactoryInteface(const FactoryInteface&) = delete;
  FactoryInteface& operator=(const FactoryInteface&) = delete;

};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_FACTORY_INTERFACE_HPP_