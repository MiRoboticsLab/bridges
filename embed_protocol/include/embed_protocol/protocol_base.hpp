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

#ifndef EMBED_PROTOCOL__PROTOCOL_BASE_HPP_
#define EMBED_PROTOCOL__PROTOCOL_BASE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <functional>
#include <stack>
#include <mutex>

#include "toml/toml.hpp"
#include "embed_protocol/common.hpp"

namespace cyberdog
{
namespace embed
{
#define MIN_TIME_OUT_US     1'000L  // 1ms
#define MAX_TIME_OUT_US 3'000'000L  // 3s

template<typename TDataClass>
class ProtocolBase
{
public:
  std::shared_ptr<TDataClass> GetData() {return protocol_data_;}

  void SetDataCallback(std::function<void(std::string &, std::shared_ptr<TDataClass>)> callback)
  {
    if (for_send_) {
      WARN(
        "[PROTOCOL][WARN][%s] for_send protocol not need callback function, "
        "please check the code", name_.c_str());
    }
    if (callback != nullptr) {protocol_data_callback_ = callback;}
  }

  void SetDataCallback(std::function<void(DataLabel &, std::shared_ptr<TDataClass>)> callback)
  {
    if (for_send_) {
      WARN(
        "[PROTOCOL][WARN][%s] for_send protocol not need callback function, "
        "please check the code", name_.c_str());
    }
    if (callback != nullptr) {protocol_data_parse_callback_ = callback;}
  }

  void LinkVar(const std::string & name, const ProtocolData & var)
  {
    std::unique_lock<std::mutex> lock(data_lock_);
    if (protocol_data_map_.find(name) != protocol_data_map_.end()) {
      error_clct_->LogState(ErrorCode::RUNTIME_SAMELINK_ERROR);
      ERROR(
        "[PROTOCOL][%s] LINK_VAR error, get same name:\"%s\"",
        name_.c_str(), name.c_str());
      return;
    }
    protocol_data_map_.insert(std::pair<std::string, ProtocolData>(name, var));
  }

  void BreakVar(const std::string & name)
  {
    std::unique_lock<std::mutex> lock(data_lock_);
    for (auto iter = protocol_data_map_.begin(); iter != protocol_data_map_.end(); ++iter) {
      if (iter->first == name) {
        protocol_data_map_.erase(iter);
        return;
      }
    }
    error_clct_->LogState(ErrorCode::RUNTIME_NOLINK_ERROR);
    ERROR(
      "[PROTOCOL][%s] BREAK_VAR error, not find same name:\"%s\". erase error",
      name_.c_str(), name.c_str());
  }

  virtual bool Operate(
    const std::string & CMD,
    const std::vector<uint8_t> & data = std::vector<uint8_t>()) = 0;
  virtual bool SendSelfData() = 0;

  virtual int GetInitErrorNum() = 0;
  virtual int GetInitWarnNum() = 0;

  virtual bool IsRxTimeout() = 0;
  virtual bool IsTxTimeout() = 0;
  bool IsRxError() {return rx_error_;}
  std::string GetName()
  {
    return name_;
  }

protected:
  ProtocolBase()
  {
    protocol_data_ = std::make_shared<TDataClass>();
    protocol_data_map_ = PROTOCOL_DATA_MAP();
  }
  ~ProtocolBase() {}

  bool for_send_;
  bool rx_error_;
  std::string name_;
  CHILD_STATE_CLCT error_clct_;
  PROTOCOL_DATA_MAP protocol_data_map_;
  std::mutex data_lock_;
  std::shared_ptr<TDataClass> protocol_data_;
  std::function<void(std::string &, std::shared_ptr<TDataClass>)> protocol_data_callback_ = nullptr;
  std::function<void(DataLabel &,
    std::shared_ptr<TDataClass>)> protocol_data_parse_callback_ = nullptr;
};  // class ProtocolBase
}  // namespace embed
}  // namespace cyberdog

#endif  // EMBED_PROTOCOL__PROTOCOL_BASE_HPP_
