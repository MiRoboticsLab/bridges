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
#ifndef BES_TRANSMIT_CENTER_HPP_
#define BES_TRANSMIT_CENTER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "protocol/srv/bes_http.hpp"
#include "protocol/srv/bes_http_send_file.hpp"
#include "protocol/srv/device_info.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "back_end_publisher.hpp"
#include "back_end_subscriber.hpp"
#include "back_end_http.hpp"


namespace cyberdog
{
namespace bridge
{
class Transmit_Waiter final
{
public:
  Transmit_Waiter();
  ~Transmit_Waiter();
  void Run();

private:
  void MqttPubCallback(const std_msgs::msg::String::SharedPtr msg);
  void MqttSubCallback(const std::string & msg);
  void BesHttpCallback(const protocol::srv::BesHttp::Request::SharedPtr,
    protocol::srv::BesHttp::Response::SharedPtr);
  void BesHttpSendFileCallback(
    const protocol::srv::BesHttpSendFile::Request::SharedPtr request,
    protocol::srv::BesHttpSendFile::Response::SharedPtr respose);
  bool getDevInf(std::string & sn, std::string & uid);

private:
  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Node::SharedPtr tpub_node_ptr_ {nullptr};
  rclcpp::Node::SharedPtr tsub_node_ptr_ {nullptr};
  rclcpp::Node::SharedPtr http_node_ptr_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr http_node_cb_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr be_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr be_sub_;
  rclcpp::Service<protocol::srv::BesHttp>::SharedPtr http_srv_;
  rclcpp::Service<protocol::srv::BesHttpSendFile>::SharedPtr http_send_file_srv_;
  std::unique_ptr<Backend_Publisher> bpub_ptr_ {nullptr};
  std::unique_ptr<Backend_Subscriber> bsub_ptr_ {nullptr};
  std::unique_ptr<Backend_Http> bhttp_ptr_ {nullptr};
  rclcpp::Client<protocol::srv::DeviceInfo>::SharedPtr device_info_client_ {nullptr};
  bool bpub_is_ready_ {false};
  LOGGER_MINOR_INSTANCE("Transmit_Waiter");
};  // Transmit_Waiter
}  // namespace bridge
}  // namespace cyberdog

#endif  // BES_TRANSMIT_CENTER_HPP_
