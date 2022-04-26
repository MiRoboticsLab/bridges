#ifndef CYBERDOG_OTA_CLIENT_HPP_
#define CYBERDOG_OTA_CLIENT_HPP_

#include "cyberdog_ota/client_interface.hpp"

#include <string>


#include "curl/curl.h"

namespace cyberdog
{
  
class Client : public ClientInterface
{
public:
  Client();
  virtual ~Client();

  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;

  virtual bool Initialize() override;

  static int Download(void* user_data, const std::string& url, const std::string& path, bool resume, const char* ca_path = NULL);
  static int GetFileSize(const std::string& url, long long& file_size, const char* ca_path = NULL);
  static int ProgressCallback(void* clientp, double dltotal, double dlnow, double ultotal, double ulnow);

  // 0表示正常继续， CURL_PROGRESSFUNC_CONTINUE继续下载，其它abort下载
  static int abort_download;
};

}  // namespace cyberdog

#endif  // CYBERDOG_OTA_CLIENT_HPP_

