// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string>
#include <array>
#include <memory>

#include "./file_uploading/file_uploading.h"
#include "back_end_http.hpp"

#ifdef __cplusplus
extern "C" {
#endif

std::string myCommand(const std::string & cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(& pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    return result;
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get())) {
    result += buffer.data();
  }
  return result;
}


int uploadFile(const char * file_name, int id, int * http_result_code)
{
  std::string sn = myCommand("factory-tool -f /usr/share/factory_cmd_config/system.xml  -i \"SN\"");
  sn = sn.substr(0, sn.find('\n'));
  if (sn.find("can not open") != std::string::npos) {
    ERROR("Please use sudo to get SN.");
    return cyberdog::bridge::Backend_Http::ErrorCode::INVALID_SN;
  }
  INFO("sn %s", sn.c_str());
  cyberdog::bridge::Backend_Http http_client(std::string("http://10.38.204.220:8091"), "");
  std::string url("device/system/log");
  http_client.SetInfo(sn, "");
  int error_code = cyberdog::bridge::Backend_Http::ErrorCode::OK;
  std::string body = http_client.SendFile(
    1, url, file_name, "application/x-tar", 60000, error_code);
  if (http_result_code) {
    Document json_doc(kObjectType);
    json_doc.Parse<0>(body.c_str());
    if (json_doc.HasParseError()) {
      ERROR("Parse json Error");
    } else {
      std::string code_string;
      CyberdogJson::Get(json_doc, "code", code_string);
      *http_result_code = std::atoi(code_string.c_str());
    }
  }
  return error_code;
}

#ifdef __cplusplus
}
#endif
