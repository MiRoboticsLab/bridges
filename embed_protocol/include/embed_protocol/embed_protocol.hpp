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

#ifndef EMBED_PROTOCOL__EMBED_PROTOCOL_HPP_
#define EMBED_PROTOCOL__EMBED_PROTOCOL_HPP_

#include <string>
#include <memory>
#include <vector>
#include <utility>

#include "embed_protocol/protocol_base.hpp"
#include "embed_protocol/can_protocol.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

#define XNAME(x) (#x)
#define LINK_VAR(var) LinkVar( \
    XNAME(var), \
    cyberdog::embed::ProtocolData(sizeof((var)), static_cast<void *>(&(var))))
#define BREAK_VAR(var) BreakVar( \
    XNAME(var))

namespace cyberdog
{
namespace embed
{
template<typename TDataClass>
class Protocol
{
public:
  Protocol(const Protocol &) = delete;
  explicit Protocol(const std::string & protocol_toml_path, bool for_send = false)
  {
    toml::value toml_config;
    if (toml_parse(toml_config, protocol_toml_path) == false) {
      error_clct_.LogState(ErrorCode::INIT_ERROR);
      ERROR(
        "[PROTOCOL] toml file:\"%s\" error, load prebuilt file",
        protocol_toml_path.c_str());
      // TBD toml_config = LoadPrebuilt();
    }

    Init(toml_config, for_send, protocol_toml_path);
    if (base_ == nullptr || base_->GetInitErrorNum() != 0) {
      error_clct_.LogState(ErrorCode::INIT_ERROR);
      ERROR(
        "[PROTOCOL] toml file:\"%s\" init error, load prebuilt file",
        protocol_toml_path.c_str());
      // TBD Init(LoadPrebuilt(), for_send);
    }
  }

  std::shared_ptr<TDataClass> GetData()
  {
    if (base_ != nullptr) {return base_->GetData();}
    if (tmp_data_ == nullptr) {tmp_data_ = std::make_shared<TDataClass>();}
    return tmp_data_;
  }

  // please use "#define LINK_VAR(var)" instead
  void LinkVar(const std::string & origin_name, const ProtocolData & var)
  {
    if (base_ != nullptr) {base_->LinkVar(get_var_name(origin_name, error_clct_), var);}
  }

  // please use "#define BREAK_VAR(var)" instead
  void BreakVar(const std::string & origin_name)
  {
    if (base_ != nullptr) {base_->BreakVar(get_var_name(origin_name, error_clct_));}
  }

  bool Operate(const std::string & CMD, const std::vector<uint8_t> & data = std::vector<uint8_t>())
  {
    if (base_ != nullptr) {return base_->Operate(CMD, data);}
    return false;
  }

  bool SendSelfData()
  {
    if (base_ != nullptr) {return base_->SendSelfData();}
    return false;
  }

  void SetDataCallback(std::function<void(std::string &, std::shared_ptr<TDataClass>)> callback)
  {
    if (base_ != nullptr) {base_->SetDataCallback(callback);}
  }

  void SetDataCallback(std::function<void(DataLabel &, std::shared_ptr<TDataClass>)> callback)
  {
    if (base_ != nullptr) {base_->SetDataCallback(callback);}
  }
  bool IsRxTimeout()
  {
    if (base_ != nullptr) {return base_->IsRxTimeout();}
    return true;
  }

  bool IsTxTimeout()
  {
    if (base_ != nullptr) {return base_->IsTxTimeout();}
    return true;
  }

  bool IsRxError()
  {
    if (base_ != nullptr) {return base_->IsRxError();}
    return true;
  }

  StateCollector & GetErrorCollector()
  {
    return error_clct_;
  }
  std::string GetName()
  {
    if (base_ != nullptr) {return base_->GetName();}
    return "unknown";
  }

private:
  std::shared_ptr<ProtocolBase<TDataClass>> base_;
  std::shared_ptr<TDataClass> tmp_data_;
  StateCollector error_clct_ = StateCollector();
  void Init(toml::value & toml_config, bool for_send, const std::string & protocol_toml_path = "")
  {
    auto protocol = toml::find_or<std::string>(toml_config, "protocol", "#unknow");
    auto name = toml::find_or<std::string>(toml_config, "name", "#unknow");
    if (protocol_toml_path != "") {
      INFO(
        "[PROTOCOL][INFO] Creat embed protocol[%s], protocol:\"%s\", path:\"%s\"",
        name.c_str(), protocol.c_str(), protocol_toml_path.c_str());
    }

    if (protocol == "can") {
      base_ = std::make_shared<CanProtocol<TDataClass>>(
        error_clct_.CreatChild(), name, toml_config, for_send);
    } else if (protocol == "spi") {
      // todo when need
      error_clct_.LogState(ErrorCode::ILLEGAL_PROTOCOL);
      ERROR("[PROTOCOL] protocol:\"%s\" not support yet", protocol.c_str());
    } else if (protocol == "iic" || protocol == "i2c") {
      // todo when need
      error_clct_.LogState(ErrorCode::ILLEGAL_PROTOCOL);
      ERROR("[PROTOCOL] protocol:\"%s\" not support yet", protocol.c_str());
    } else {
      error_clct_.LogState(ErrorCode::ILLEGAL_PROTOCOL);
      ERROR(
        "[PROTOCOL][%s] protocol:\"%s\" not support, parser path=\"%s\"",
        name.c_str(), protocol.c_str(), protocol_toml_path.c_str());
    }
  }
};  // class Protocol
}  // namespace embed
}  // namespace cyberdog

#endif  // EMBED_PROTOCOL__EMBED_PROTOCOL_HPP_
