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

#include "cyberdog_app.hpp"
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "cyberdog_app_server.hpp"
#include "std_msgs/msg/string.hpp"

#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "cyberdog_common/cyberdog_json.hpp"
#define gettid() syscall(SYS_gettid)

using grpc::ServerWriter;
#define RECV_URL "udpm://239.255.76.67:7667?ttl=1"
#define SEND_URL "udpm://239.255.76.67:7667?ttl=2"
#define DES_V_MAX 3.0
#define DES_V_MIN -3.0
#define DES_ANG_R_MAX 0.5
#define DES_ANG_R_MIN -0.5
#define DES_ANG_P_MAX 0.5
#define DES_ANG_P_MIN -0.5
#define DES_ANG_Y_MAX 3.0
#define DES_ANG_Y_MIN -3.0
#define DES_BODY_H_MAX 0.5
#define DES_BODY_H_MIN 0.0
#define DES_GAIT_H_MAX 0.2
#define DES_GAIT_H_MIN 0.0
#define SEND_CMD_TTL 2
#define RECV_CMD_TTL 12
#define APP_CONNECTED_FAIL_CNT 3
#define CYBER_CLIENT_LED 1
using std::placeholders::_1;
using ms = std::literals::chrono_literals::operator""ms;
using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;
#define CON_TO_CHAR(a) (reinterpret_cast<const char *>(a))
namespace athena_cyberdog_app
{
static int64_t requestNumber;
Cyberdog_app::Cyberdog_app()
: Node("motion_test_server"), ticks_(0), can_process_messages(false),
  heartbeat_err_cnt(0), heart_beat_thread_(nullptr), app_server_thread_(nullptr),
  server_(nullptr), app_stub(nullptr), app_disconnected(true),
  destory_grpc_server_thread_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Cyberdog_app Configuring");
  server_ip = std::make_shared<std::string>("0.0.0.0");

  ip_subscriber = this->create_subscription<std_msgs::msg::String>(
    "ip_notify", rclcpp::SystemDefaultsQoS(), std::bind(&Cyberdog_app::subscribeIp, this, _1));

  timer_interval.init();

  RCLCPP_INFO(get_logger(), "Create server");
  if (server_ == nullptr) {
    app_server_thread_ = std::make_shared<std::thread>(&Cyberdog_app::RunServer, this);
  }
  heart_beat_thread_ = std::make_shared<std::thread>(&Cyberdog_app::HeartBeat, this);
}

void Cyberdog_app::HeartBeat()
{
  rclcpp::WallRate r(500ms);
  std::string ipv4;
  while (true) {
    if (can_process_messages && app_stub) {
      if (!app_stub->SetHeartBeat(local_ip)) {
        if (heartbeat_err_cnt++ >= APP_CONNECTED_FAIL_CNT) {
          if (!app_disconnected) {
            destroyGrpc();
            createGrpc();
          }
        }
      }
    }
    r.sleep();
  }
}

std::string Cyberdog_app::getServiceIp()
{
  return *server_ip;
}

void Cyberdog_app::RunServer()
{
  RCLCPP_INFO(get_logger(), "run_server thread id is %d", gettid());
  std::string server_address("0.0.0.0:50051");
  CyberdogAppImpl service(server_address);
  service.SetRequesProcess(this);
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS, 1000);
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 1000);
  builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS, 500);
  builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_SENT_PING_INTERVAL_WITHOUT_DATA_MS, 1000);
  builder.RegisterService(&service);
  server_ = std::move(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  RCLCPP_INFO(get_logger(), "server thread id is %d", gettid());
  server_->Wait();
  RCLCPP_INFO(get_logger(), "after wait");
}

std::string Cyberdog_app::getDogIp(const string str, const string & split)
{
  string result;
  int pos = str.find(split);
  if (pos != -1) {
    result = str.substr(pos + split.size(), str.size());
  }
  return result;
}

std::string Cyberdog_app::getPhoneIp(const string str, const string & split)
{
  string result;
  int pos = str.find(split);
  if (pos != -1) {
    result = str.substr(0, pos);
  }
  return result;
}
void Cyberdog_app::subscribeIp(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "get ip :%s", msg->data.c_str());
  RCLCPP_INFO(get_logger(), "old phoneip is:%s", (*server_ip).c_str());
  app_disconnected = false;
  local_ip = getDogIp(msg->data, ":");
  RCLCPP_INFO(get_logger(), "local_ip ip :%s", local_ip.c_str());
  std::string phoneIp = getPhoneIp(msg->data, ":");
  RCLCPP_INFO(get_logger(), "phoneIp ip :%s", phoneIp.c_str());
  if (*server_ip != phoneIp) {
    server_ip = std::make_shared<std::string>(phoneIp);
    destroyGrpc();
    createGrpc();
  }
}
void Cyberdog_app::destroyGrpcServer()
{
  if (server_ != nullptr) {
    RCLCPP_INFO(get_logger(), "close server");
    server_->Shutdown();
    RCLCPP_INFO(get_logger(), "join server");
    app_server_thread_->join();
    server_ = nullptr;
  }
}

void Cyberdog_app::destroyGrpc()
{
  can_process_messages = false;
  if (app_stub != nullptr) {
    app_stub = nullptr;
  }
  net_checker.pause();
}

void Cyberdog_app::createGrpc()
{
  RCLCPP_INFO(get_logger(), "Create server");
  if (server_ == nullptr) {
    app_server_thread_ = std::make_shared<std::thread>(&Cyberdog_app::RunServer, this);
  }
  RCLCPP_INFO(get_logger(), "Create client");
  grpc::string ip = *server_ip + std::string(":8980");
  can_process_messages = false;
  heartbeat_err_cnt = 0;
  net_checker.set_ip(*server_ip);
  RCLCPP_INFO(get_logger(), "before channel");
  auto channel_ = grpc::CreateChannel(ip, grpc::InsecureChannelCredentials());
  RCLCPP_INFO(get_logger(), "after channel");
  app_stub = std::make_shared<Cyberdog_App_Client>(channel_);
  RCLCPP_INFO(get_logger(), "end channel");
  can_process_messages = true;
  if (app_disconnected) {
    destroyGrpc();
  }
}

string Cyberdog_app::GetFileConecxt(string path)
{
  char buffer[32];
  bool result = false;
  int bufflen = sizeof(buffer) / sizeof(char);
  if (path.empty()) {
    return "";
  }

  FILE * _file = NULL;
  _file = fopen(path.c_str(), "r");
  memset(buffer, 0, bufflen);
  if (NULL == _file) {
    RCLCPP_INFO(get_logger(), "open failed");
    return "";
  }

  fgets(buffer, bufflen, _file);
  RCLCPP_INFO(get_logger(), "get file content:%s", buffer);
  fclose(_file);
  return string(buffer);
}

void Cyberdog_app::ProcessMsg(
  const ::grpcapi::SendRequest * request,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  RCLCPP_INFO(get_logger(), "ProcessMsg %d", request->namecode());
  ::grpcapi::RecResponse respond;
  switch (request->namecode()) {
    case ::grpcapi::SendRequest::GET_DEVICE_INFO:
      RCLCPP_INFO(get_logger(), "ProcessMsg %s", request->params().c_str());
      Document d;
      d.Parse<0>(request->params().c_str());
      if (d.HasParseError()) {
        return;
      }

      respond.set_namecode(request->namecode());
      Document res(kObjectType);
      CyberdogJson::Add(res, "ip", "192.168.55.1");
      std::string after;
      if (!CyberdogJson::Document2String(res, after)) {
        return;
      }
      respond.set_data(after);
      writer->Write(respond);
      break;
  }
}
}  // namespace athena_cyberdog_app
