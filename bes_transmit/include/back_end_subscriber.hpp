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
#ifndef BACK_END_SUBSCRIBER_HPP_
#define BACK_END_SUBSCRIBER_HPP_

#include <mqttc/mqtt.h>
#include <string>
#include <memory>
#include <thread>
#include <functional>
#include "posix_sockets.hpp"


namespace cyberdog
{
namespace bridge
{
struct reconnect_state_t
{
  const char * hostname;
  const char * port;
  const char * topic;
  uint8_t * sendbuf;
  size_t sendbufsz;
  uint8_t * recvbuf;
  size_t recvbufsz;
};

void reconnect_client(struct mqtt_client * client, void ** reconnect_state_vptr);
void subcribe_callback(void ** unused, struct mqtt_response_publish * published);

class Backend_Subscriber final
{
public:
  Backend_Subscriber()
  : addr_("127.0.0.1"), port_("8080"), topic_("bes_to_dog")
  {
  }
  ~Backend_Subscriber()
  {
    if (thread_ && thread_->joinable()) {
      thread_->join();
    }
  }

  void Init()
  {
    reconnect_state.hostname = addr_.c_str();
    reconnect_state.port = port_.c_str();
    reconnect_state.topic = topic_.c_str();
    reconnect_state.sendbuf = sendbuf;
    reconnect_state.sendbufsz = sizeof(sendbuf);
    reconnect_state.recvbuf = recvbuf;
    reconnect_state.recvbufsz = sizeof(recvbuf);
    mqtt_init_reconnect(&client, reconnect_client, &reconnect_state, subcribe_callback);
    auto func = [this]() {
        while (rclcpp::ok()) {
          mqtt_sync(&client);
          std::this_thread::sleep_for(std::chrono::microseconds(100000U));
        }
      };
    thread_ = std::make_unique<std::thread>(func);
  }

private:
  struct reconnect_state_t reconnect_state;
  struct mqtt_client client;
  uint8_t sendbuf[2048];
  uint8_t recvbuf[1024];
  std::string addr_;
  std::string port_;
  std::string topic_;
  std::unique_ptr<std::thread> thread_;
};  // Backend_Subscriber
}  // namespace bridge
}  // namespace cyberdog

#endif  // BACK_END_SUBSCRIBER_HPP_
