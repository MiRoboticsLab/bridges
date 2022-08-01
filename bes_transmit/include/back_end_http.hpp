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

#include <string>
#include "cpp_httplib/httplib.h"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

namespace cyberdog
{
namespace bridge
{
class Backend_Http final
{
public:
  Backend_Http()
  : base_url("http://10.167.23.60:8091")
  {
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
    std::string body("{\"code\": 404}");
    if (!params.empty()) {
      rapidjson::Document doc;
      doc.SetObject();
      doc.Parse<rapidjson::kParseDefaultFlags>(params.c_str());
      if (doc.HasParseError()) {
        ERROR("doc should be json::kObjectType.");
        body = "{\"code\": 369000, \"message\": \"json format error\"}";
        return body;
      }
      request_url += "?";
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
        } else {
          WARN("json params format must be string, if not ignore it.");
        }
      }
    }
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
    std::string body("{\"code\": 404}");
    auto res = cli_.Post(request_url, params, "application/json");
    if (res) {
      body = res->body;
    }
    return body;
  }

private:
  std::string base_url;
};  // Backend_Http
}  // namespace bridge
}  // namespace cyberdog

#endif  // BACK_END_HTTP_HPP_
