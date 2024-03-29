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

#ifndef EMBED_PROTOCOL__CAN_PROTOCOL_HPP_
#define EMBED_PROTOCOL__CAN_PROTOCOL_HPP_

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <stack>

#include "embed_protocol/common.hpp"
#include "embed_protocol/protocol_base.hpp"
#include "embed_parser/can_parser.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace embed
{
template<typename TDataClass>
class CanProtocol : public ProtocolBase<TDataClass>
{
public:
  CanProtocol(
    CHILD_STATE_CLCT error_clct,
    const std::string & name,
    const toml::value & toml_config,
    bool for_send)
  {
    this->name_ = name;
    this->error_clct_ = (error_clct == nullptr) ? std::make_shared<StateCollector>() : error_clct;
    this->for_send_ = for_send;

    auto can_interface = toml::find_or<std::string>(toml_config, "can_interface", "can0");
    auto extended_frame = toml::find_or<bool>(toml_config, "extended_frame", false);
    auto canfd_enable = toml::find_or<bool>(toml_config, "canfd_enable", false);
    auto timeout_us = toml::find_or<int64_t>(toml_config, "timeout_us", MAX_TIME_OUT_US);
    timeout_us = std::clamp(timeout_us, MIN_TIME_OUT_US, MAX_TIME_OUT_US);

    can_parser_ = std::make_shared<CanParser>(
      this->error_clct_->CreatChild(), toml_config, this->name_);
    INFO(
      "[CAN_PROTOCOL][INFO] Creat can protocol[%s]: %d error, %d warning",
      this->name_.c_str(), can_parser_->GetInitErrorNum(), can_parser_->GetInitWarnNum());
    auto recv_list = can_parser_->GetRecvList();
    int recv_num = recv_list.size();
    bool send_only = (recv_num == 0) ? true : this->for_send_;

    if (send_only) {
      INFO(
        "[CAN_PROTOCOL][INFO][%s] No recv canid, enable send-only mode",
        this->name_.c_str());
      can_op_ = std::make_shared<CanDev>(
        can_interface,
        this->name_,
        extended_frame,
        canfd_enable,
        timeout_us * 1000);
    } else {
      can_op_ = canfd_enable ? std::make_shared<CanDev>(
        can_interface,
        this->name_,
        extended_frame,
        std::bind(&CanProtocol::recv_callback_fd, this, std::placeholders::_1),
        timeout_us * 1000) :
        std::make_shared<CanDev>(
        can_interface,
        this->name_,
        extended_frame,
        std::bind(&CanProtocol::recv_callback_std, this, std::placeholders::_1),
        timeout_us * 1000);
    }
    // set can_filter
    if (can_op_ != nullptr && send_only == false) {
      auto filter = new struct can_filter[recv_num];
      for (int a = 0; a < recv_num; a++) {
        filter[a].can_id = recv_list[a];
        filter[a].can_mask = CAN_EFF_MASK;
      }
      can_op_->set_filter(filter, recv_num * sizeof(struct can_filter));
      delete[] filter;
    }
  }
  ~CanProtocol() {}

  bool Operate(
    const std::string & CMD,
    const std::vector<uint8_t> & data = std::vector<uint8_t>()) override
  {
    if (can_parser_->IsCanfd() == false) {
      can_frame tx_frame;
      if (can_parser_->Encode(tx_frame, CMD, data) &&
        can_op_ != nullptr && can_op_->send_can_message(tx_frame))
      {
        return true;
      } else {
        this->error_clct_->LogState(ErrorCode::RUNTIME_OPERATE_ERROR);
        ERROR(
          "[CAN_PROTOCOL][%s] Operate CMD:\"%s\" sending data error 0",
          this->name_.c_str(), CMD.c_str());
      }
    } else {
      canfd_frame tx_frame;
      if (can_parser_->Encode(tx_frame, CMD, data) &&
        can_op_ != nullptr && can_op_->send_can_message(tx_frame))
      {
        return true;
      } else {
        this->error_clct_->LogState(ErrorCode::RUNTIME_OPERATE_ERROR);
        ERROR(
          "[CAN_PROTOCOL][%s] Operate CMD:\"%s\" sending data error 1",
          this->name_.c_str(), CMD.c_str());
      }
    }
    return false;
  }

  bool SendSelfData() override
  {
    if (this->for_send_ == false) {
      WARN(
        "[CAN_PROTOCOL][WARN][%s] Protocol not in sending mode, "
        "should not send data from data class, except for test",
        this->name_.c_str());
    }
    return can_parser_->Encode(this->protocol_data_map_, can_op_);
  }

  int GetInitErrorNum() override {return can_parser_->GetInitErrorNum();}
  int GetInitWarnNum() override {return can_parser_->GetInitWarnNum();}

  bool IsRxTimeout() override {return can_op_->is_rx_timeout();}
  bool IsTxTimeout() override {return can_op_->is_tx_timeout();}

private:
  std::shared_ptr<CanParser> can_parser_;
  std::shared_ptr<CanDev> can_op_;
  const std::string name_unkown_ = "unkown";
  void recv_callback_std(std::shared_ptr<can_frame> recv_frame)
  {
    std::string id_name = name_unkown_;
    std::unique_lock<std::mutex> lock(this->data_lock_);
    if (this->protocol_data_parse_callback_ != nullptr) {
      DataLabel label;
      label.group_name = this->GetName();
      label.is_full = false;

      if (can_parser_->Decode(this->protocol_data_map_, recv_frame, this->rx_error_, label)) {
        this->rx_error_ = false;
        lock.unlock();
        this->protocol_data_parse_callback_(label, this->protocol_data_);
      }
    } else if (this->protocol_data_callback_ != nullptr) {
      if (can_parser_->Decode(this->protocol_data_map_, recv_frame, this->rx_error_, id_name)) {
        this->rx_error_ = false;
        lock.unlock();
        this->protocol_data_callback_(id_name, this->protocol_data_);
      }
    }
  }
  void recv_callback_fd(std::shared_ptr<canfd_frame> recv_frame)
  {
    std::string id_name = name_unkown_;
    std::unique_lock<std::mutex> lock(this->data_lock_);
    if (this->protocol_data_parse_callback_ != nullptr) {
      DataLabel label;
      label.group_name = this->GetName();
      label.is_full = false;
      if (can_parser_->Decode(this->protocol_data_map_, recv_frame, this->rx_error_, label)) {
        this->rx_error_ = false;
        lock.unlock();
        this->protocol_data_parse_callback_(label, this->protocol_data_);
      }
    } else if (this->protocol_data_callback_ != nullptr) {
      if (can_parser_->Decode(this->protocol_data_map_, recv_frame, this->rx_error_, id_name)) {
        this->rx_error_ = false;
        lock.unlock();
        this->protocol_data_callback_(id_name, this->protocol_data_);
      }
    }
  }
};  // class CanProtocol
}  // namespace embed
}  // namespace cyberdog

#endif  // EMBED_PROTOCOL__CAN_PROTOCOL_HPP_
