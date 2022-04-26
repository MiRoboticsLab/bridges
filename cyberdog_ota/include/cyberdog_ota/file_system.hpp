#ifndef CYBERDOG_OTA_FILE_SYSTEM_HPP_
#define CYBERDOG_OTA_FILE_SYSTEM_HPP_

#include "cyberdog_ota/system_info.hpp"

namespace cyberdog
{

class FileSystem
{
public:
  FileSystem();
  ~FileSystem();

  FileSystem(const FileSystem&) = delete;
  FileSystem& operator=(const FileSystem&) = delete;

  
private:

  SystemInfo system_info_;
};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_FILE_SYSTEM_HPP_