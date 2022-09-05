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
#ifndef BACK_END_HTTP_HPP_
#define BACK_END_HTTP_HPP_

#include <shared_mutex>
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
class Backend_Http final
{
public:
  Backend_Http()
  : base_url("http://10.38.204.220:8091")
  {
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager/settings.json");
    Document json_document(kObjectType);
    auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
    if (result) {
      std::string url;
      bool result = CyberdogJson::Get(json_document, "bes_url", url);
      if (result) {
        base_url = url;
        INFO("bes url:%s", base_url.c_str());
      }
    }
  }
  ~Backend_Http()
  {
  }

public:
  const std::string get(
    const std::string & url, const std::string & params,
    const uint16_t & millsecs)
  {
    httplib::Client cli_(base_url);
    cli_.set_read_timeout(std::chrono::milliseconds(millsecs));
    std::string request_url = "/v1";
    if (url.length() > 0 && url[0] == '/') {
      request_url += url;
    } else {
      request_url = request_url + "/" + url;
    }
    std::string body("{\"code\": -1, \"message\": \"http method error\"}");
    request_url += "?";
    if (!params.empty()) {
      rapidjson::Document doc;
      doc.SetObject();
      doc.Parse<rapidjson::kParseDefaultFlags>(params.c_str());
      if (doc.HasParseError()) {
        ERROR("doc should be json::kObjectType.");
        body = "{\"code\": -1, \"message\": \"json format error\"}";
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
    request_url += "account:" + uid + "&number:" + sn;
    INFO("base_url:%s, request url:%s", base_url.c_str(), request_url.c_str());
    auto res = cli_.Get(request_url);
    if (res) {
      body = res->body;
    }
    return body;
  }
  const std::string post(
    const std::string & url, const std::string & params,
    const uint16_t & millsecs)
  {
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
    request_url += "?account:" + uid + "&number:" + sn;
    std::string body("{\"code\": -1, \"message\": \"http method error\"}");
    auto res = cli_.Post(request_url, params, "application/json");
    if (res) {
      body = res->body;
    }
    return body;
  }
  const std::string SendFile(
    unsigned char method, const std::string & url, const std::string & file_name,
    const std::string & content_type, const uint16_t & millsecs)
  {
    std::string body("{\"code\": -1, \"message\": \"http method error\"}");
    std::ifstream infile;
    infile.open(file_name, std::ifstream::in | std::ifstream::binary);
    if (!infile.is_open()) {
      ERROR_STREAM("file " << file_name << " cannot be opened");
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
    std::string sn, uid;
    GetInfo(sn, uid);
    request_url += "?account:" + uid + "&number:" + sn;
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
      }
    }
    infile.close();
    return body;
  }
  void SetInfo(const std::string & sn, const std::string & uid)
  {
    std::unique_lock<std::shared_mutex> lock(info_mutex_);
    sn_ = sn;
    uid_ = uid;
  }
  void GetInfo(std::string & sn, std::string & uid) const
  {
    std::shared_lock<std::shared_mutex> lock(info_mutex_);
    sn = sn_;
    uid = uid_;
  }

private:
  std::string base_url;
  std::string sn_, uid_;
  mutable std::shared_mutex info_mutex_;
};  // Backend_Http
}  // namespace bridge
}  // namespace cyberdog

#endif  // BACK_END_HTTP_HPP_
