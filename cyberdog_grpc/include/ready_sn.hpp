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
#ifndef READY_SN_HPP_
#define READY_SN_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace bridges
{

class ReadySnNode
{
public:
  ReadySnNode()
  : dog_sn("")
  {
    ready_sn_node_ = rclcpp::Node::make_shared("grpc_get_sn");
    sn_notify_sub_ =
      ready_sn_node_->create_subscription<std_msgs::msg::String>(
      "dog_sn", rclcpp::SystemDefaultsQoS(),
      std::bind(&ReadySnNode::DogsnCallback, this, std::placeholders::_1));
    audio_sn_ger_client_ =
      ready_sn_node_->create_client<std_srvs::srv::Trigger>("get_dog_sn");
    std::thread(
      [this]() {
        rclcpp::spin(ready_sn_node_);
      }).detach();
  }

public:
  const std::string & WaitSn()
  {
    std::thread(
      [this]() {
        while (!GetDogsn()) {
        }
      }).join();
    return dog_sn;
  }

private:
  void DogsnCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    dog_sn = msg->data;
  }
  bool GetDogsn()
  {
    if (!dog_sn.empty()) {
      return true;
    }
    if (!audio_sn_ger_client_->wait_for_service(std::chrono::seconds(1))) {
      INFO("wait sn get service not ready.");
      return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_result = audio_sn_ger_client_->async_send_request(req);
    std::chrono::seconds timeout(3);
    std::future_status status = future_result.wait_for(timeout);
    if (status == std::future_status::ready) {
      dog_sn = future_result.get()->message;
    } else {
      INFO("async send sn request no return.");
      return false;
    }
    return true;
  }

private:
  std::string dog_sn;
  rclcpp::Node::SharedPtr ready_sn_node_ {nullptr};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sn_notify_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_sn_ger_client_;
};

}  // namespace bridges
}  // namespace cyberdog

#endif  // READY_SN_HPP_
