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
#ifndef BACK_END_PUBLISHER_HPP_
#define BACK_END_PUBLISHER_HPP_

#include <mqttc/mqtt.h>
#include <thread>
#include <memory>
#include <string>
#include "cyberdog_common/cyberdog_log.hpp"
#include "posix_sockets.hpp"

namespace cyberdog
{
namespace bridge
{

void publish_callback(void ** unused, struct mqtt_response_publish * published);

class Backend_Publisher final
{
public:
  Backend_Publisher(const std::string & topic_name = "cyberdog/base_info/submit")
  : sockfd(-1), addr_("10.38.205.52"),
    port_("1883"), topic_(topic_name), is_stop_(false)
  {
  }
  ~Backend_Publisher()
  {
    if (thread_sync_->joinable()) {
      thread_sync_->join();
    }
  }

  bool Init()
  {
    sockfd = PosixSocket::open_nb_socket(addr_.c_str(), port_.c_str());
    if (sockfd == -1) {
      ERROR("Failed to open socket: ");
      return false;
    }
    mqtt_init(
      &client, sockfd, sendbuf, sizeof(sendbuf), recvbuf, sizeof(recvbuf),
      publish_callback);
    const char * client_id = NULL;
    uint8_t connect_flags = MQTT_CONNECT_CLEAN_SESSION;
    mqtt_connect(&client, client_id, NULL, NULL, 0, NULL, NULL, connect_flags, 400);
    if (client.error != MQTT_OK) {
      ERROR("error: %s\n", mqtt_error_str(client.error));
      if (sockfd != -1) {
        close(sockfd);
      }
      return false;
    }
    auto func = [this]() {
        while (!is_stop_ && rclcpp::ok()) {
          mqtt_sync(&client);
          std::this_thread::sleep_for(std::chrono::microseconds(100000U));
        }
      };
    thread_sync_ = std::make_unique<std::thread>(func);
    return true;
  }

  bool Publish(const char * application_message)
  {
    time_t timer;
    time(&timer);
    struct tm tm_info;
    localtime_r(&timer, &tm_info);
    char timebuf[26];
    strftime(timebuf, 26, "%Y-%m-%d %H:%M:%S", &tm_info);
    INFO("published : \"%s\"", application_message);
    mqtt_publish(
      &client, topic_.c_str(), application_message, strlen(
        application_message) + 1, MQTT_PUBLISH_QOS_0);
    if (client.error != MQTT_OK) {
      ERROR("error: %s", mqtt_error_str(client.error));
      if (sockfd != -1) {
        close(sockfd);
      }
      is_stop_ = true;
      if (thread_sync_->joinable()) {
        thread_sync_->join();
      }
      return false;
    }
    return true;
  }

private:
  int sockfd;
  struct mqtt_client client;
  uint8_t sendbuf[2048];
  uint8_t recvbuf[1024];
  std::string addr_;
  std::string port_;
  std::string topic_;
  bool is_stop_;
  std::unique_ptr<std::thread> thread_sync_;
};  // Backend_Publisher
}  // namespace bridge
}  // namespace cyberdog

#endif  // BACK_END_PUBLISHER_HPP_
