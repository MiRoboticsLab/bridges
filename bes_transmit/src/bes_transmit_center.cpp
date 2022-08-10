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
#include <memory>
#include <algorithm>
#include "bes_transmit_center.hpp"

#define   BTW_PUB_NODE_NAME   "bes_transmit_pub_waiter"
#define   BTW_SUB_NODE_NAME   "bes_transmit_sub_waiter"
#define   BT_HTTP_NODE_NAME   "bes_transmit_http_srv"

namespace cyberdog
{
namespace bridge
{

void reconnect_client(struct mqtt_client * client, void ** reconnect_state_vptr)
{
  struct reconnect_state_t * reconnect_state =
    *((struct reconnect_state_t **) reconnect_state_vptr);
  if (client->error != MQTT_ERROR_INITIAL_RECONNECT) {
    close(client->socketfd);
  }
  if (client->error != MQTT_ERROR_INITIAL_RECONNECT) {
    WARN(
      "reconnect_client: called while client was in error state \"%s\"\n",
      mqtt_error_str(client->error));
  }
  int sockfd = PosixSocket::open_nb_socket(reconnect_state->hostname, reconnect_state->port);
  if (sockfd == -1) {
    ERROR("Failed to open socket: ");
    close(sockfd);
    std::this_thread::sleep_for(std::chrono::microseconds(100000U));
    reconnect_client(client, reconnect_state_vptr);
    return;
  }
  mqtt_reinit(
    client, sockfd,
    reconnect_state->sendbuf, reconnect_state->sendbufsz,
    reconnect_state->recvbuf, reconnect_state->recvbufsz
  );
  const char * client_id = NULL;
  uint8_t connect_flags = MQTT_CONNECT_CLEAN_SESSION;
  mqtt_connect(client, client_id, NULL, NULL, 0, NULL, NULL, connect_flags, 400);
  mqtt_subscribe(client, reconnect_state->topic, 0);
}

void subcribe_callback(void **, struct mqtt_response_publish * published)
{
  char * topic_name = reinterpret_cast<char *>(malloc(published->topic_name_size + 1));
  memcpy(topic_name, published->topic_name, published->topic_name_size);
  topic_name[published->topic_name_size] = '\0';
  INFO("Received publish('%s'): %s\n", topic_name, (const char *) published->application_message);
  free(topic_name);
}

void publish_callback(void **, struct mqtt_response_publish *)
{
}

}  // namespace bridge
}  // namespace cyberdog

cyberdog::bridge::Transmit_Waiter::Transmit_Waiter()
{
  tpub_node_ptr_ = rclcpp::Node::make_shared(BTW_PUB_NODE_NAME);
  tsub_node_ptr_ = rclcpp::Node::make_shared(BTW_SUB_NODE_NAME);
  http_node_ptr_ = rclcpp::Node::make_shared(BT_HTTP_NODE_NAME);
  executor_.add_node(tpub_node_ptr_);
  executor_.add_node(tsub_node_ptr_);
  executor_.add_node(http_node_ptr_);
  bpub_ptr_ = std::make_unique<Backend_Publisher>();
  be_pub_ =
    tpub_node_ptr_->create_publisher<std_msgs::msg::String>(
    "bes_to_dog",
    rclcpp::SystemDefaultsQoS());
  bsub_ptr_ = std::make_unique<Backend_Subscriber>();
  be_sub_ =
    tsub_node_ptr_->create_subscription<std_msgs::msg::String>(
    "dog_to_bes", rclcpp::SystemDefaultsQoS(),
    std::bind(&Transmit_Waiter::MqttPubCallback, this, std::placeholders::_1));
  bhttp_ptr_ = std::make_unique<Backend_Http>();
  http_srv_ =
    http_node_ptr_->create_service<protocol::srv::BesHttp>(
    "bes_http_srv",
    std::bind(
      &Transmit_Waiter::BesHttpCallback, this, std::placeholders::_1,
      std::placeholders::_2));
}

cyberdog::bridge::Transmit_Waiter::~Transmit_Waiter()
{
}

void cyberdog::bridge::Transmit_Waiter::Run()
{
  executor_.spin();
  rclcpp::shutdown();
}

void cyberdog::bridge::Transmit_Waiter::MqttPubCallback(const std_msgs::msg::String::SharedPtr msg)
{
  bpub_ptr_->Publish(msg->data.c_str());
}

void cyberdog::bridge::Transmit_Waiter::MqttSubCallback(const std::string & msg)
{
  std_msgs::msg::String msg_data;
  msg_data.data = msg;
  be_pub_->publish(msg_data);
}

void cyberdog::bridge::Transmit_Waiter::BesHttpCallback(
  const protocol::srv::BesHttp::Request::SharedPtr request,
  protocol::srv::BesHttp::Response::SharedPtr respose)
{
  std::string params("");
  if (!request->params.empty()) {
    params = request->params;
  }
  int mill_seconds = std::min(static_cast<int>(request->milsecs), 6000);
  mill_seconds = std::max(0, mill_seconds);
  mill_seconds = (mill_seconds == 0) ? 3000 : mill_seconds;
  respose->data = "{\"code\": 369003, \"message\": \"http method error\"}";
  if (request->method == protocol::srv::BesHttp::Request::HTTP_METHOD_GET) {
    respose->data = bhttp_ptr_->get(request->url, request->params, mill_seconds);
  } else if (request->method == protocol::srv::BesHttp::Request::HTTP_METHOD_POST) {
    respose->data = bhttp_ptr_->post(request->url, request->params, mill_seconds);
  }
}