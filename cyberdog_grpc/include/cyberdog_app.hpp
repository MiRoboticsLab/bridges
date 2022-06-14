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
#include <utility>
#include <vector>

// Interfaces
#include "cyberdog_app_client.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "msgdispatcher.hpp"
#include "net_avalible.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/audio_auth_id.hpp"
#include "protocol/srv/audio_auth_token.hpp"
#include "protocol/srv/ota_server_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "threadsafe_queue.hpp"
#include "time_interval.hpp"
using string = std::string;
using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;
namespace carpo_cyberdog_app
{
class Cyberdog_app : public rclcpp::Node
{
public:
  Cyberdog_app();
  std::string getServiceIp();
  void ProcessMsg(
    const ::grpcapi::SendRequest * request,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

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
  void destroyGrpc();
  void createGrpc();
  string GetFileConecxt(string path);
  NetChecker net_checker;
  uint32_t heartbeat_err_cnt;
  bool app_disconnected;
  std::string local_ip;
  TimeInterval timer_interval;
  void HeartBeat();
  void sendMsg(
    const ::grpcapi::SendRequest * request,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  // ros interaction codes
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<protocol::msg::MotionServoResponse>::SharedPtr
    motion_servo_response_sub_;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr
    motion_servo_request_pub_;
  void motion_servo_rsp_callback(
    const protocol::msg::MotionServoResponse::SharedPtr msg);
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr
    motion_ressult_client_;
  void callMotionServoCmd(
    const std::shared_ptr<protocol::srv::MotionResultCmd::Request> req,
    protocol::srv::MotionResultCmd::Response & rep);
  void retrunErrorGrpc(::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  // visual program
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visual_response_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visual_request_pub_;
  void backend_message_callback(const std_msgs::msg::String::SharedPtr msg);

  // audio program
  rclcpp::Client<protocol::srv::AudioAuthId>::SharedPtr audio_auth_request;
  rclcpp::Client<protocol::srv::AudioAuthToken>::SharedPtr audio_auth_response;

  // commcon code
  void send_grpc_msg(int code, const std::string & msg);
  void send_grpc_msg(int code, const Document & doc);

  // image_transmission
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr image_trans_activation_;

  // ota 
  rclcpp::Client<protocol::srv::OtaServerCmd>::SharedPtr ota_client_;
};
}  // namespace carpo_cyberdog_app

#endif  // CYBERDOG_APP_HPP_
