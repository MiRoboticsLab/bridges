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
#define APP_CONNECTED_FAIL_CNT 3
#define CYBER_CLIENT_LED 1
using std::placeholders::_1;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wliteral-suffix"
using std::literals::chrono_literals::operator""ms;
#pragma GCC diagnostic pop
using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;
#define CON_TO_CHAR(a) (reinterpret_cast<const char *>(a))
namespace carpo_cyberdog_app
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

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // ros interaction codes
  motion_servo_response_sub_ = this->create_subscription<protocol::msg::MotionServoResponse>(
    "motion_servo_response",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::motion_servo_rsp_callback, this, _1));

  motion_servo_request_pub_ = this->create_publisher<protocol::msg::MotionServoCmd>(
    "motion_servo_cmd", rclcpp::SystemDefaultsQoS());

  motion_ressult_client_ = this->create_client<protocol::srv::MotionResultCmd>(
    "motion_result_cmd",
    rmw_qos_profile_services_default,
    callback_group_);

  //  code for visual program
  visual_response_sub_ = this->create_subscription<std_msgs::msg::String>(
    "backend_message",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::backend_message_callback, this, _1));

  visual_request_pub_ = this->create_publisher<std_msgs::msg::String>(
    "frontend_message", rclcpp::SystemDefaultsQoS());
}

void Cyberdog_app::HeartBeat()
{
  rclcpp::WallRate r(500ms);
  std::string ipv4;
  while (true) {
    if (can_process_messages && app_stub) {
      if (!app_stub->sendHeartBeat(local_ip)) {
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
  RCLCPP_INFO(get_logger(), "run_server thread id is %ld", gettid());
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
  RCLCPP_INFO(get_logger(), "Server listening on %s", server_address.c_str());
  RCLCPP_INFO(get_logger(), "server thread id is %ld", gettid());
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

//  for motion
void Cyberdog_app::motion_servo_rsp_callback(
  const protocol::msg::MotionServoResponse::SharedPtr msg)
{
  Document json_response(kObjectType);
  CyberdogJson::Add(json_response, "motion_id", msg->motion_id);
  CyberdogJson::Add(json_response, "cmd_id", msg->cmd_id);
  CyberdogJson::Add(json_response, "order_process_bar", msg->order_process_bar);
  CyberdogJson::Add(json_response, "status", msg->status);
  CyberdogJson::Add(json_response, "result", msg->result);
  CyberdogJson::Add(json_response, "code", msg->code);
  send_grpc_msg(::grpcapi::SendRequest::MOTION_SERVO_RESPONSE, json_response);
}


void Cyberdog_app::callMotionServoCmd(
  const std::shared_ptr<protocol::srv::MotionResultCmd::Request> req,
  protocol::srv::MotionResultCmd::Response & rsp)
{
  RCLCPP_INFO(get_logger(), "callMotionServoCmd.");
  std::chrono::seconds timeout(5);

  if (!motion_ressult_client_->wait_for_service()) {
    RCLCPP_INFO(get_logger(), "callCameraService server not avalible");
    return;
  }

  RCLCPP_INFO(get_logger(), "motion_id: %d.", req->motion_id);
  auto future_result = motion_ressult_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);

  if (status == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "success to call camera services.");
  } else {
    RCLCPP_INFO(get_logger(), "Failed to call camera services.");
  }

  rsp.motion_id = future_result.get()->motion_id;
  rsp.result = future_result.get()->result;
  rsp.code = future_result.get()->code;
}


//  for visual
void Cyberdog_app::backend_message_callback(const std_msgs::msg::String::SharedPtr msg)
{
  send_grpc_msg(::grpcapi::SendRequest::MOTION_SERVO_RESPONSE, msg->data);
}

//  commcon code

void Cyberdog_app::send_grpc_msg(int code, const Document & doc)
{
  std::string rsp_string;
  if (!CyberdogJson::Document2String(doc, rsp_string)) {
    RCLCPP_ERROR(get_logger(), "error while encoding to json");
    return;
  }
  send_grpc_msg(code, rsp_string);
}

void Cyberdog_app::send_grpc_msg(int code, const std::string & msg)
{
  ::grpcapi::SendRequest grpc_respons;
  grpc_respons.set_namecode(code);
  grpc_respons.set_params(msg);

  if (can_process_messages && app_stub) {
    app_stub->sendRequest(grpc_respons);
  }
}

//  message pump
void Cyberdog_app::retrunErrorGrpc(::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  ::grpcapi::RecResponse grpc_respond;
  grpc_respond.set_data("ERROR");
  writer->Write(grpc_respond);
}

void Cyberdog_app::ProcessMsg(
  const ::grpcapi::SendRequest * grpc_request,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  RCLCPP_INFO(get_logger(), "ProcessMsg %d", grpc_request->namecode());
  ::grpcapi::RecResponse grpc_respond;
  RCLCPP_INFO(get_logger(), "ProcessMsg %s", grpc_request->params().c_str());
  Document json_resquest(kObjectType);
  Document json_response(kObjectType);
  std::string rsp_string;
  json_resquest.Parse<0>(grpc_request->params().c_str());
  if (json_resquest.HasParseError()) {
    RCLCPP_ERROR(get_logger(), "Parse Error");
    retrunErrorGrpc(writer);
    return;
  }
  switch (grpc_request->namecode()) {
    case ::grpcapi::SendRequest::GET_DEVICE_INFO:

      grpc_respond.set_namecode(grpc_request->namecode());
      CyberdogJson::Add(json_response, "ip", "192.168.55.1");
      if (!CyberdogJson::Document2String(json_response, rsp_string)) {
        RCLCPP_ERROR(get_logger(), "error while encoding to json");
        retrunErrorGrpc(writer);
        return;
      }
      grpc_respond.set_data(rsp_string);
      writer->Write(grpc_respond);
      break;

    case ::grpcapi::SendRequest::MOTION_SERVO_REQUEST:
      {
        protocol::msg::MotionServoCmd motion_servo_cmd;
        CyberdogJson::Get(json_resquest, "motion_id", motion_servo_cmd.motion_id);
        CyberdogJson::Get(json_resquest, "cmd_id", motion_servo_cmd.cmd_id);
        CyberdogJson::Get(json_resquest, "vel_des", motion_servo_cmd.vel_des);
        CyberdogJson::Get(json_resquest, "rpy_des", motion_servo_cmd.rpy_des);
        CyberdogJson::Get(json_resquest, "pos_des", motion_servo_cmd.pos_des);
        CyberdogJson::Get(json_resquest, "acc_des", motion_servo_cmd.acc_des);
        CyberdogJson::Get(json_resquest, "ctrl_point", motion_servo_cmd.ctrl_point);
        CyberdogJson::Get(json_resquest, "foot_pose", motion_servo_cmd.foot_pose);
        CyberdogJson::Get(json_resquest, "step_height", motion_servo_cmd.step_height);
        motion_servo_request_pub_->publish(motion_servo_cmd);
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data("");
        writer->Write(grpc_respond);
      }
      break;

    case ::grpcapi::SendRequest::MOTION_CMD_REQUEST:
      {
        RCLCPP_ERROR(get_logger(), "MOTION_CMD_REQUEST");
        auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
        protocol::srv::MotionResultCmd::Response rsp;

        // get ros service request
        CyberdogJson::Get(json_resquest, "motion_id", req->motion_id);
        CyberdogJson::Get(json_resquest, "vel_des", req->vel_des);
        CyberdogJson::Get(json_resquest, "rpy_des", req->rpy_des);
        CyberdogJson::Get(json_resquest, "pos_des", req->pos_des);
        CyberdogJson::Get(json_resquest, "acc_des", req->acc_des);
        CyberdogJson::Get(json_resquest, "ctrl_point", req->ctrl_point);
        CyberdogJson::Get(json_resquest, "foot_pose", req->foot_pose);
        CyberdogJson::Get(json_resquest, "step_height", req->step_height);
        CyberdogJson::Get(json_resquest, "duration", req->duration);

        // call ros service
        callMotionServoCmd(req, rsp);

        // send service response
        CyberdogJson::Add(json_response, "motion_id", rsp.motion_id);
        CyberdogJson::Add(json_response, "result", rsp.result);
        CyberdogJson::Add(json_response, "code", rsp.code);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(get_logger(), "error while encoding to json");
          retrunErrorGrpc(writer);
          return;
        }

        // send grpc result
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      }
      break;

    case ::grpcapi::SendRequest::VISUAL_FRONTEND_MSG:
      std_msgs::msg::String msg;
      msg.data = grpc_request->params();
      visual_request_pub_->publish(msg);
      break;
  }
}
}  // namespace carpo_cyberdog_app
