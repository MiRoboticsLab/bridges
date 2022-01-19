// Copyright (c) 2021 Xiaomi Corporation
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

#ifndef CYBERDOG_APP_HPP_
#define CYBERDOG_APP_HPP_

#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <utility>

// Interfaces

#include "rclcpp/rclcpp.hpp"
#include "cyberdog_app_client.hpp"
#include "threadsafe_queue.hpp"
#include "msgdispatcher.hpp"
#include "net_avalible.hpp"
#include "time_interval.hpp"
using namespace std;
namespace athena_cyberdog_app
{
class Cyberdog_app : public rclcpp::Node
{
public:
  Cyberdog_app();
  void closeBtAdv();
  void pingDog(const ::cyberdogapp::Result* request, ::cyberdogapp::DogInfo * respond);
  void findDog();
  void connectDog(const ::cyberdogapp::ConnectRequest* request, ::cyberdogapp::ConnectResult* respond);
  std::string getServiceIp();

private:
  uint32_t ticks_;
  bool can_process_messages;
  void RunServer();
  std::shared_ptr<std::thread> app_server_thread_;
  std::shared_ptr<std::thread> heart_beat_thread_;
  std::shared_ptr<std::thread> destory_grpc_server_thread_;
  std::shared_ptr<std::thread> dog_walk_thread_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ip_subscriber;
  void destroyGrpcServer();
  std::string getDogIp(const string str, const string & split);
  std::string getPhoneIp(const string str, const string & split);
  std::shared_ptr<Cyberdog_App_Client> app_stub;
  std::shared_ptr<std::string> server_ip;
  std::shared_ptr<grpc::Server> server_;
  void subscribeIp(const std_msgs::msg::String::SharedPtr msg);
  void HeartBeat();
  void destroyGrpc();
  void createGrpc();
  string GetFileConecxt(string path);
  NetChecker net_checker;
  uint32_t heartbeat_err_cnt;
  bool app_disconnected;
  std::string local_ip;
  TimeInterval timer_interval;
};
}  // namespace athena_cyberdog_app

#endif  // CYBERDOG_APP_HPP_
