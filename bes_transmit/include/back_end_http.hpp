// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef BACK_END_HTTP_HPP_
#define BACK_END_HTTP_HPP_

#define CPPHTTPLIB_OPENSSL_SUPPORT

#include <uuid/uuid.h>
#include <shared_mutex>
#include <cstdlib>
#include <string>
#include <sstream>
#include "cpp_httplib/httplib.h"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#define CHUNK_SIZE 4194304

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace bridge
{
/**
 * @brief Class for calling HTTP methods
 */
class Backend_Http final
{
public:
  /**
   * @brief Construct a new Backend_Http object
   * @param b_url Server IP or region name
   * @param config_file Config file that can get url
   */
  Backend_Http(
    const std::string & b_url = "http://10.38.204.220:8091",
    const std::string & config_file = "/toml_config/manager/settings.json")
  : base_url(b_url)
  {
    if (!config_file.empty()) {
      auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
      auto path = local_share_dir + std::string(config_file);
      Document json_document(kObjectType);
      auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
      if (result) {
        std::string url;
        bool result = CyberdogJson::Get(json_document, "bes_url", url);
        if (result) {
          base_url = url;
        }
      }
    }
    INFO("bes url:%s", base_url.c_str());
  }
  enum ErrorCode
  {
    OK = 6000,
    EMPTY_URL = 6021,
    INFO_SERVICE_ERROR = 6022,
    INVALID_SN = 6023,
    JSON_ERROR = 6024,
    HTTP_REQUEST_ERROR = 6025,
    OPEN_FILE_ERROR = 6026,
    CONNECTION_ERROR = 6027
  };
  /**
   * @brief Get the Default Response body
   * @param message Error message
   * @return Response string
   */
  static std::string GetDefaultResponse(const std::string & message)
  {
    uuid_t uu;
    uuid_generate(uu);
    char uuid_str[37];
    uuid_unparse_lower(uu, uuid_str);
    std::string default_str("{\"code\": \"-1\", \"data\": \"[]\", \"message\": \"");
    default_str += message + "\", \"request_id\": \"" + std::string(uuid_str) + "\"}";
    return default_str;
  }

public:
  /**
   * @brief HTTP GET method
   * @param url URL without server ip and port
   * @param params Request params
   * @param millsecs Timeout
   * @param error_code Error code from enum ErrorCode
   * @return Response body
   */
  const std::string get(
    const std::string & url, const std::string & params,
    const uint16_t & millsecs, int & error_code)
  {
    if (checkConnection() != 0) {
      error_code = ErrorCode::CONNECTION_ERROR;
      return GetDefaultResponse("connection error");
    }
    httplib::Client cli_(base_url);
    cli_.set_read_timeout(std::chrono::milliseconds(millsecs));
    std::string request_url = "/v1";
    if (url.length() > 0 && url[0] == '/') {
      request_url += url;
    } else {
      request_url = request_url + "/" + url;
    }
    std::string body(GetDefaultResponse("http method error"));
    request_url += "?";
    if (!params.empty()) {
      rapidjson::Document doc;
      doc.SetObject();
      doc.Parse<rapidjson::kParseDefaultFlags>(params.c_str());
      if (doc.HasParseError()) {
        ERROR("doc should be json::kObjectType.");
        body = GetDefaultResponse("json format error");
        error_code = ErrorCode::JSON_ERROR;
        return body;
      }
      for (rapidjson::Value::MemberIterator iter = doc.MemberBegin(); iter != doc.MemberEnd();
        iter++)
      {
        const char * key = iter->name.GetString();
        const rapidjson::Value & val = iter->value;
        if (val.IsString()) {
          std::string val_str = val.GetString();
          request_url += key;
          request_url += "=";
          request_url += val_str;
          request_url += "&";
        } else {
          WARN("json params format must be string, if not ignore it.");
        }
      }
    }
    std::string sn, uid;
    GetInfo(sn, uid);
    request_url += "account=" + uid + "&number=" + sn;
    INFO("base_url:%s, request url:%s", base_url.c_str(), request_url.c_str());
    auto res = cli_.Get(request_url);
    if (res) {
      body = res->body;
      error_code = ErrorCode::OK;
    } else {
      error_code = ErrorCode::HTTP_REQUEST_ERROR;
    }
    INFO("response body: %s\n", body.c_str());
    return body;
  }
  /**
   * @brief HTTP POST method
   * @param url URL without server ip and port or region name
   * @param params Request params
   * @param millsecs Timeout
   * @param error_code Error code from enum ErrorCode
   * @return Response body
   */
  const std::string post(
    const std::string & url, const std::string & params,
    const uint16_t & millsecs, int & error_code)
  {
    if (checkConnection() != 0) {
      error_code = ErrorCode::CONNECTION_ERROR;
      return GetDefaultResponse("connection error");
    }
    httplib::Client cli_(base_url);
    cli_.set_read_timeout(std::chrono::milliseconds(millsecs));
    cli_.set_write_timeout(std::chrono::milliseconds(millsecs));
    std::string request_url = "/v1";
    if (url.length() > 0 && url[0] == '/') {
      request_url += url;
    } else {
      request_url = request_url + "/" + url;
    }
    INFO(
      "base_url:%s, request url:%s, params:%s",
      base_url.c_str(), request_url.c_str(), params.c_str());
    std::string sn, uid;
    GetInfo(sn, uid);
    request_url += "?account=" + uid + "&number=" + sn;
    std::string body(GetDefaultResponse("http method error"));
    auto res = cli_.Post(request_url, params, "application/json");
    if (res) {
      body = res->body;
      error_code = ErrorCode::OK;
    } else {
      error_code = ErrorCode::HTTP_REQUEST_ERROR;
    }
    INFO("response body: %s\n", body.c_str());
    return body;
  }
  /**
   * @brief Call HTTP methods to upload files
   * @param method 0 for GET, 1 for POST
   * @param url URL without server ip and port or region name
   * @param file_name File name with full path
   * @param content_type Type of the file
   * @param millsecs Timeout
   * @param error_code Error code from enum ErrorCode
   * @return Response body
   */
  const std::string SendFile(
    unsigned char method, const std::string & url, const std::string & file_name,
    const std::string & content_type, const uint16_t & millsecs, int & error_code)
  {
    if (checkConnection() != 0) {
      error_code = ErrorCode::CONNECTION_ERROR;
      return GetDefaultResponse("connection error");
    }
    std::string body(GetDefaultResponse("http method error"));
    std::ifstream infile;
    infile.open(file_name, std::ifstream::in | std::ifstream::binary);
    if (!infile.is_open()) {
      ERROR_STREAM("file " << file_name << " cannot be opened");
      body = Backend_Http::GetDefaultResponse("file cannot be opened");
      error_code = ErrorCode::OPEN_FILE_ERROR;
      return body;
    }
    infile.seekg(0, infile.end);
    size_t file_size = infile.tellg();
    infile.seekg(0, infile.beg);
    httplib::Client cli_(base_url);
    cli_.set_read_timeout(std::chrono::milliseconds(millsecs));
    cli_.set_write_timeout(std::chrono::milliseconds(millsecs));
    std::string request_url = "/v1";
    if (url.length() > 0 && url[0] == '/') {
      request_url += url;
    } else {
      request_url = request_url + "/" + url;
    }
    INFO(
      "base_url:%s, request url:%s, file_name:%s",
      base_url.c_str(), request_url.c_str(), file_name.c_str());
    size_t position_of_slash = file_name.find_last_of("/");
    std::string file_name_to_set = file_name;
    if (position_of_slash != std::string::npos) {
      file_name_to_set = file_name.substr(position_of_slash + 1);
    }
    request_url += "?file_name=" + file_name_to_set;
    std::string sn, uid;
    GetInfo(sn, uid);
    request_url += "&account=" + uid + "&number=" + sn;
    char data_to_be_sent[CHUNK_SIZE];
    if (method == 1) {
      auto res = cli_.Post(
        request_url, file_size,
        [&](size_t, size_t, httplib::DataSink & sink) {
          infile.read(data_to_be_sent, CHUNK_SIZE);
          sink.write(data_to_be_sent, infile.gcount());
          return true;
        },
        content_type);
      if (res) {
        body = res->body;
      } else {
        error_code = ErrorCode::HTTP_REQUEST_ERROR;
      }
    } else if (method == 2) {
      auto res = cli_.Put(
        request_url, file_size,
        [&](size_t, size_t, httplib::DataSink & sink) {
          infile.read(data_to_be_sent, CHUNK_SIZE);
          sink.write(data_to_be_sent, infile.gcount());
          return true;
        },
        content_type);
      if (res) {
        body = res->body;
        error_code = ErrorCode::OK;
      } else {
        error_code = ErrorCode::HTTP_REQUEST_ERROR;
      }
    }
    infile.close();
    INFO("response body: %s\n", body.c_str());
    return body;
  }
  /**
   * @brief Set SN of the device and user ID
   * @param sn SN of the device
   * @param uid User ID
   */
  void SetInfo(const std::string & sn, const std::string & uid)
  {
    std::unique_lock<std::shared_mutex> lock(info_mutex_);
    sn_ = sn;
    uid_ = uid;
  }
  /**
   * @brief Get SN of the device and user ID
   * @param sn SN of the device
   * @param uid User ID
   */
  void GetInfo(std::string & sn, std::string & uid) const
  {
    std::shared_lock<std::shared_mutex> lock(info_mutex_);
    sn = sn_;
    uid = uid_;
  }
  LOGGER_MINOR_INSTANCE("Backend_Http");

private:
  /**
   * @brief Activate ping command to check connection
   * @return Result of ping command
   */
  int checkConnection()
  {
    std::string cmd("ping -c 1 -W 2 ");
    std::string ip = base_url.substr(base_url.find("://") + 3);
    size_t port_position = ip.find(":");
    if (port_position != std::string::npos) {
      ip = ip.substr(0, port_position);
    }
    cmd += ip;
    return system(cmd.c_str());
  }
  std::string base_url;
  std::string sn_, uid_;
  mutable std::shared_mutex info_mutex_;
};  // Backend_Http
}  // namespace bridge
}  // namespace cyberdog

#endif  // BACK_END_HTTP_HPP_
