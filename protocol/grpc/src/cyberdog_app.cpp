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
using namespace std::chrono_literals;

#define CON_TO_CHAR(a) (reinterpret_cast<const char *>(a))
namespace athena_cyberdog_app
{
static int64_t requestNumber;
Cyberdog_app::Cyberdog_app()
: Node("motion_test_server"), ticks_(0),  can_process_messages(false),
  heartbeat_err_cnt(0), heart_beat_thread_(nullptr), app_server_thread_(nullptr),  server_(nullptr),
  app_stub(nullptr), app_disconnected(true), destory_grpc_server_thread_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Cyberdog_app Configuring");
  server_ip = std::make_shared<std::string>("0.0.0.0");

  ip_subscriber = this->create_subscription<std_msgs::msg::String>(
    "ip_notify", rclcpp::SystemDefaultsQoS(), std::bind(&Cyberdog_app::subscribeIp, this, _1));

  heart_beat_thread_ = std::make_shared<std::thread>(&Cyberdog_app::HeartBeat, this);
  timer_interval.init();

  RCLCPP_INFO(get_logger(), "Create server");
  if (server_ == nullptr) {
    app_server_thread_ = std::make_shared<std::thread>(&Cyberdog_app::RunServer, this);
  }
}

std::string Cyberdog_app::getServiceIp()
{
  return *server_ip;
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
  if (path.empty())
  {
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

void Cyberdog_app::pingDog(const ::cyberdogapp::Result* request, ::cyberdogapp::DogInfo * respond)
{
  string sub_name;
  RCLCPP_INFO(get_logger(), "ping dog");
  //1.read psn
  system("echo '123' | sudo -S cat /dev/mmcblk0p11 | grep -a PSN | cut -c 6-23 > /tmp/name.txt");
  string psn = GetFileConecxt("/tmp/name.txt");
  sub_name = psn.substr(0, psn.length()-1);
  RCLCPP_INFO(get_logger(), "psn name:%s", sub_name.c_str());
  //2.no psn, read serial number
  if (sub_name.empty())
  {
    string psn = GetFileConecxt("/sys/firmware/devicetree/base/serial-number");
    if(psn.length()>4)
    {
      sub_name = psn.substr(psn.length()-4, -1);
    }
     RCLCPP_INFO(get_logger(), "serinal name:%s", sub_name.c_str());
  }
  respond->set_connected(!app_disconnected);
  respond->set_dog_name(string("铁蛋") + sub_name);
}
#define LED_BLINK_TIME 5000000000
#define FIND_DOG_AUDIO_ID 7
void Cyberdog_app::findDog()
{
  RCLCPP_INFO(get_logger(), "find dog");
  /*
  auto rear_request = std::make_shared<ception_msgs::srv::SensorDetectionNode::Request>();
  auto head_request = std::make_shared<ception_msgs::srv::SensorDetectionNode::Request>();
  rear_request->clientid = CYBER_CLIENT_LED;
  rear_request->command = ception_msgs::srv::SensorDetectionNode::Request::REAR_LED_RED_BLINK;
  rear_request->priority = ception_msgs::srv::SensorDetectionNode::Request::TYPE_ALARM;
  rear_request->timeout = LED_BLINK_TIME;

  head_request->clientid = CYBER_CLIENT_LED;
  head_request->command = ception_msgs::srv::SensorDetectionNode::Request::HEAD_LED_RED_BLINK;
  head_request->priority = ception_msgs::srv::SensorDetectionNode::Request::TYPE_ALARM;
  head_request->timeout = LED_BLINK_TIME;
  
  //led_client_->async_send_request(rear_request);
  //led_client_->async_send_request(head_request);
  sendGoal(FIND_DOG_AUDIO_ID);
  */
}
void Cyberdog_app::connectDog(const ::cyberdogapp::ConnectRequest* request, ::cyberdogapp::ConnectResult* respond)
{
  RCLCPP_INFO(get_logger(), "connectDog get ip :%s", request->phone_ip().c_str());
  RCLCPP_INFO(get_logger(), "old phoneip is:%s", (*server_ip).c_str());
  app_disconnected = false;
  local_ip = getDogIp(request->phone_ip(), ":");
  RCLCPP_INFO(get_logger(), "local_ip ip :%s", local_ip.c_str());
  std::string phoneIp = getPhoneIp(request->phone_ip(), ":");
  RCLCPP_INFO(get_logger(), "phoneIp ip :%s", phoneIp.c_str());
  if (*server_ip != phoneIp) {
    server_ip = std::make_shared<std::string>(phoneIp);
    destroyGrpc();
    createGrpc();
  }

  // close BT advertisement.
  auto _thread = std::make_shared<std::thread>(&Cyberdog_app::closeBtAdv, this);
  _thread->detach();
  respond->set_result(true);
}

void Cyberdog_app::closeBtAdv()
{
  //setBtCmd(ception_msgs::srv::BtRemoteCommand::Request::SET_BT_ADV_ATT_DISABLE, "");
}
}  // namespace athena_cyberdog_app
