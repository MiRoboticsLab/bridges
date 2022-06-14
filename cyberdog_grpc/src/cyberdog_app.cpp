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

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <sys/types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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
: Node("motion_test_server"),
  ticks_(0),
  can_process_messages(false),
  heartbeat_err_cnt(0),
  heart_beat_thread_(nullptr),
  app_server_thread_(nullptr),
  server_(nullptr),
  app_stub(nullptr),
  app_disconnected(true),
  destory_grpc_server_thread_(nullptr)
{
  INFO("Cyberdog_app Configuring");
  server_ip = std::make_shared<std::string>("0.0.0.0");

  ip_subscriber = this->create_subscription<std_msgs::msg::String>(
    "ip_notify", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::subscribeIp, this, _1));

  timer_interval.init();

  INFO("Create server");
  if (server_ == nullptr) {
    app_server_thread_ =
      std::make_shared<std::thread>(&Cyberdog_app::RunServer, this);
  }
  heart_beat_thread_ =
    std::make_shared<std::thread>(&Cyberdog_app::HeartBeat, this);

  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // ros interaction codes
  motion_servo_response_sub_ =
    this->create_subscription<protocol::msg::MotionServoResponse>(
    "motion_servo_response", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::motion_servo_rsp_callback, this, _1));

  motion_servo_request_pub_ =
    this->create_publisher<protocol::msg::MotionServoCmd>(
    "motion_servo_cmd", rclcpp::SystemDefaultsQoS());

  motion_ressult_client_ = this->create_client<protocol::srv::MotionResultCmd>(
    "motion_result_cmd", rmw_qos_profile_services_default, callback_group_);

  //  code for visual program
  visual_response_sub_ = this->create_subscription<std_msgs::msg::String>(
    "backend_message", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::backend_message_callback, this, _1));

  visual_request_pub_ = this->create_publisher<std_msgs::msg::String>(
    "frontend_message", rclcpp::SystemDefaultsQoS());

  // code for audio program
  audio_voiceprint_result_sub_ =
    this->create_subscription<protocol::msg::AudioVoiceprintResult>(
    "audio_voiceprint_result", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::voiceprint_result_callback, this, _1));
  voiceprints_data_sub_ =
    this->create_subscription<std_msgs::msg::Bool>(
    "voiceprints_data_require", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::voiceprints_data_callback, this, _1));
  audio_auth_request =
    this->create_client<protocol::srv::AudioAuthId>("get_authenticate_didsn");
  audio_auth_response = this->create_client<protocol::srv::AudioAuthToken>(
    "set_authenticate_token");
  audio_voiceprint_train =
    this->create_client<protocol::srv::AudioVoiceprintTrain>(
    "voiceprint_train");
  voiceprints_data_notify =
    this->create_client<protocol::srv::AudioVoiceprintsSet>(
    "voiceprints_data_notify");

  // image_transmission
  image_trans_activation_ =
    create_client<std_srvs::srv::SetBool>("activate_image_transmission");

  // ota
  ota_client_ = this->create_client<protocol::srv::OtaServerCmd>("ota_grpc");
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

std::string Cyberdog_app::getServiceIp() {return *server_ip;}

void Cyberdog_app::RunServer()
{
  INFO("run_server thread id is %ld", gettid());
  std::string server_address("0.0.0.0:50051");
  CyberdogAppImpl service(server_address);
  service.SetRequesProcess(this);
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS, 1000);
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 1000);
  builder.AddChannelArgument(
    GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS, 500);
  builder.AddChannelArgument(
    GRPC_ARG_HTTP2_MIN_SENT_PING_INTERVAL_WITHOUT_DATA_MS, 1000);
  builder.RegisterService(&service);
  server_ = std::move(builder.BuildAndStart());
  INFO("Server listening on %s", server_address.c_str());
  INFO("server thread id is %ld", gettid());
  server_->Wait();
  INFO("after wait");
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
  INFO("get ip :%s", msg->data.c_str());
  INFO("old phoneip is:%s", (*server_ip).c_str());
  app_disconnected = false;
  local_ip = getDogIp(msg->data, ":");
  INFO("local_ip ip :%s", local_ip.c_str());
  std::string phoneIp = getPhoneIp(msg->data, ":");
  INFO("phoneIp ip :%s", phoneIp.c_str());
  if (*server_ip != phoneIp) {
    server_ip = std::make_shared<std::string>(phoneIp);
    destroyGrpc();
    createGrpc();
  }
}

void Cyberdog_app::destroyGrpcServer()
{
  if (server_ != nullptr) {
    INFO("close server");
    server_->Shutdown();
    INFO("join server");
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
  INFO("Create server");
  if (server_ == nullptr) {
    app_server_thread_ =
      std::make_shared<std::thread>(&Cyberdog_app::RunServer, this);
  }
  INFO("Create client");
  grpc::string ip = *server_ip + std::string(":8980");
  can_process_messages = false;
  heartbeat_err_cnt = 0;
  net_checker.set_ip(*server_ip);
  auto channel_ = grpc::CreateChannel(ip, grpc::InsecureChannelCredentials());
  app_stub = std::make_shared<Cyberdog_App_Client>(channel_);
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
    INFO("open failed");
    return "";
  }

  fgets(buffer, bufflen, _file);
  INFO("get file content:%s", buffer);
  fclose(_file);
  return string(buffer);
}

//  for motion
void Cyberdog_app::motion_servo_rsp_callback(
  const protocol::msg::MotionServoResponse::SharedPtr msg)
{
  Document json_response(kObjectType);
  CyberdogJson::Add(json_response, "motion_id", msg->motion_id);
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
  INFO("callMotionServoCmd.");
  std::chrono::seconds timeout(5);

  if (!motion_ressult_client_->wait_for_service()) {
    INFO("callMotionServoCmd server not avalible");
    return;
  }

  INFO("motion_id: %d.", req->motion_id);
  auto future_result = motion_ressult_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);

  if (status == std::future_status::ready) {
    INFO("success to call callMotionServoCmd services.");
  } else {
    INFO("Failed to call callMotionServoCmd services.");
  }

  rsp.motion_id = future_result.get()->motion_id;
  rsp.result = future_result.get()->result;
  rsp.code = future_result.get()->code;
}

//  for visual
void Cyberdog_app::backend_message_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  send_grpc_msg(::grpcapi::SendRequest::VISUAL_BACKEND_MSG, msg->data);
}

//  for audio
void Cyberdog_app::voiceprint_result_callback(
  const protocol::msg::AudioVoiceprintResult::SharedPtr msg)
{
  Document json_response(kObjectType);
  CyberdogJson::Add(json_response, "code", msg->code);
  CyberdogJson::Add(json_response, "voiceprint_id", msg->voice_print.id);
  send_grpc_msg(::grpcapi::SendRequest::AUDIO_VOICEPRINTTRAIN_RESULT, json_response);
}
void Cyberdog_app::voiceprints_data_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  send_grpc_msg(::grpcapi::SendRequest::AUDIO_VOICEPRINTS_DATA, "{}");
}

//  commcon code

void Cyberdog_app::send_grpc_msg(int code, const Document & doc)
{
  std::string rsp_string;
  if (!CyberdogJson::Document2String(doc, rsp_string)) {
    ERROR("error while encoding to json");
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
void Cyberdog_app::retrunErrorGrpc(
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  ::grpcapi::RecResponse grpc_respond;
  grpc_respond.set_data("ERROR");
  writer->Write(grpc_respond);
}

void Cyberdog_app::ProcessMsg(
  const ::grpcapi::SendRequest * grpc_request,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  RCLCPP_INFO(
    get_logger(), "ProcessMsg %d %s", grpc_request->namecode(),
    grpc_request->params().c_str());
  ::grpcapi::RecResponse grpc_respond;
  Document json_resquest(kObjectType);
  Document json_response(kObjectType);
  std::string rsp_string;
  json_resquest.Parse<0>(grpc_request->params().c_str());
  if (json_resquest.HasParseError()) {
    ERROR("Parse Error");
    retrunErrorGrpc(writer);
    return;
  }
  switch (grpc_request->namecode()) {
    case ::grpcapi::SendRequest::GET_DEVICE_INFO:

      grpc_respond.set_namecode(grpc_request->namecode());
      CyberdogJson::Add(json_response, "ip", "192.168.55.1");
      if (!CyberdogJson::Document2String(json_response, rsp_string)) {
        ERROR("error while encoding to json");
        retrunErrorGrpc(writer);
        return;
      }
      grpc_respond.set_data(rsp_string);
      writer->Write(grpc_respond);
      break;

    case ::grpcapi::SendRequest::MOTION_SERVO_REQUEST: {
        protocol::msg::MotionServoCmd motion_servo_cmd;
        CyberdogJson::Get(json_resquest, "motion_id", motion_servo_cmd.motion_id);
        CyberdogJson::Get(json_resquest, "cmd_type", motion_servo_cmd.cmd_type);
        CyberdogJson::Get(json_resquest, "vel_des", motion_servo_cmd.vel_des);
        CyberdogJson::Get(json_resquest, "rpy_des", motion_servo_cmd.rpy_des);
        CyberdogJson::Get(json_resquest, "pos_des", motion_servo_cmd.pos_des);
        CyberdogJson::Get(json_resquest, "acc_des", motion_servo_cmd.acc_des);
        CyberdogJson::Get(
          json_resquest, "ctrl_point",
          motion_servo_cmd.ctrl_point);
        CyberdogJson::Get(json_resquest, "foot_pose", motion_servo_cmd.foot_pose);
        CyberdogJson::Get(
          json_resquest, "step_height",
          motion_servo_cmd.step_height);
        motion_servo_request_pub_->publish(motion_servo_cmd);
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data("");
        writer->Write(grpc_respond);
      } break;

    case ::grpcapi::SendRequest::MOTION_CMD_REQUEST: {
        ERROR("MOTION_CMD_REQUEST");
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
          ERROR("error while encoding to json");
          retrunErrorGrpc(writer);
          return;
        }

        // send grpc result
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      } break;

    case ::grpcapi::SendRequest::VISUAL_FRONTEND_MSG: {
        std_msgs::msg::String msg;
        msg.data = grpc_request->params();
        visual_request_pub_->publish(msg);
      } break;

    case ::grpcapi::SendRequest::AUDIO_AUTHENTICATION_REQUEST: {
        if (!audio_auth_request->wait_for_service()) {
          RCLCPP_INFO(
            get_logger(),
            "callAuthenticateRequest server not avalible");
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioAuthId::Request>();
        protocol::srv::AudioAuthId::Response rsp;
        auto future_result = audio_auth_request->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(
            get_logger(),
            "success to call authenticate request services.");
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Failed to call authenticate request  services.");
        }
        rsp.did = future_result.get()->did;
        rsp.sn = future_result.get()->sn;
        CyberdogJson::Add(json_response, "did", rsp.did);
        CyberdogJson::Add(json_response, "sn", rsp.sn);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(
            get_logger(),
            "error while encoding authenticate request to json");
          retrunErrorGrpc(writer);
          return;
        }
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      } break;

    case ::grpcapi::SendRequest::AUDIO_AUTHENTICATION_RESPONSE: {
        if (!audio_auth_response->wait_for_service()) {
          RCLCPP_INFO(
            get_logger(),
            "callAuthenticateResponse server not avalible");
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioAuthToken::Request>();
        protocol::srv::AudioAuthToken::Response rsp;
        CyberdogJson::Get(json_resquest, "uid", req->uid);
        // CyberdogJson::Get(json_resquest, "title", req->title);
        CyberdogJson::Get(json_resquest, "token_access", req->token_access);
        CyberdogJson::Get(json_resquest, "token_fresh", req->token_fresh);
        std::string tei;
        CyberdogJson::Get(json_resquest, "token_expires_in", tei);
        req->token_expirein = stoul(tei);
        // CyberdogJson::Get(json_resquest, "token_deviceid",
        // req->token_deviceid);
        auto future_result = audio_auth_response->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(
            get_logger(),
            "success to call authenticate response services.");
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Failed to call authenticate response services.");
        }
        rsp.result = future_result.get()->result;
        CyberdogJson::Add(json_response, "result", rsp.result);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(
            get_logger(),
            "error while encoding authenticate response to json");
          retrunErrorGrpc(writer);
          return;
        }
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      } break;
    case ::grpcapi::SendRequest::AUDIO_VOICEPRINTTRAIN_START: {
        if (!audio_voiceprint_train->wait_for_service()) {
          RCLCPP_INFO(
            get_logger(),
            "call voiceprint train start server not avalible");
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioVoiceprintTrain::Request>();
        req->train_id = protocol::srv::AudioVoiceprintTrain::Request::TID_START;
        CyberdogJson::Get(json_resquest, "nick_name", req->voice_print.name);
        CyberdogJson::Get(json_resquest, "voiceprint_id", req->voice_print.id);
        auto future_result = audio_voiceprint_train->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(
            get_logger(),
            "success to call voiceprint train start response services.");
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Failed to call voiceprint train start response services.");
          return;
        }
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data("{}");
        writer->Write(grpc_respond);
      } break;
    case ::grpcapi::SendRequest::AUDIO_VOICEPRINTTRAIN_CANCEL: {
        if (!audio_voiceprint_train->wait_for_service()) {
          RCLCPP_INFO(
            get_logger(),
            "call voiceprint train cancel server not avalible");
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioVoiceprintTrain::Request>();
        req->train_id = protocol::srv::AudioVoiceprintTrain::Request::TID_CANCEL;
        auto future_result = audio_voiceprint_train->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(
            get_logger(),
            "success to call voiceprint train cancel response services.");
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Failed to call voiceprint train cancel response services.");
          return;
        }
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data("{}");
        writer->Write(grpc_respond);
      } break;
    case ::grpcapi::SendRequest::AUDIO_VOICEPRINTS_DATA: {
        if (!voiceprints_data_notify->wait_for_service()) {
          RCLCPP_INFO(
            get_logger(),
            "call voiceprints data server not avalible");
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioVoiceprintsSet::Request>();
        auto future_result = voiceprints_data_notify->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(
            get_logger(),
            "success to call voiceprints data response services.");
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Failed to call voiceprints data response services.");
          return;
        }
      } break;
    case ::grpcapi::SendRequest::IMAGE_TRANSMISSION_REQUEST: {
        std::chrono::seconds timeout(10);
        if (!image_trans_activation_->wait_for_service(timeout)) {
          RCLCPP_INFO(
            get_logger(),
            "activate_image_transmission server not avalible");
          return;
        }
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        CyberdogJson::Get(json_resquest, "enable", req->data);
        auto res = image_trans_activation_->async_send_request(req);
        auto status = res.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(
            get_logger(),
            "success to call activate_image_transmission services.");
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Failed to call activate_image_transmission services.");
        }
        CyberdogJson::Add(json_response, "success", res.get()->success);
        CyberdogJson::Add(json_response, "message", res.get()->message);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(
            get_logger(),
            "error while encoding authenticate response to json");
          retrunErrorGrpc(writer);
          return;
        }
        grpc_respond.set_namecode(
          ::grpcapi::SendRequest::IMAGE_TRANSMISSION_REQUEST);
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      } break;

    case ::grpcapi::SendRequest::OTA_STATUS_REQUEST:
      {
        std::chrono::seconds timeout(10);
        if (!ota_client_->wait_for_service(timeout)) {
          RCLCPP_INFO(get_logger(), "ota server not avalible");
          return;
        }

        std::string json_result;
        CyberdogJson::Document2String(json_resquest, json_result);
        auto req = std::make_shared<protocol::srv::OtaServerCmd::Request>();
        req->request.key = "ota_command_status_query";
        req->request.value = json_result;
        req->request.type = "JSON";

        auto res = ota_client_->async_send_request(req);
        auto status = res.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(get_logger(), "success to call ota services.");
        } else {
          RCLCPP_INFO(get_logger(), "Failed to call ota services.");
        }

        if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate ota response to json");
          retrunErrorGrpc(writer);
          return;
        }

        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate response to json");
          retrunErrorGrpc(writer);
          return;
        }

        grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_STATUS_REQUEST);
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      }
      break;
    case ::grpcapi::SendRequest::OTA_VERSION_QUERY_REQUEST:
      {
        std::chrono::seconds timeout(10);
        if (!ota_client_->wait_for_service(timeout)) {
          RCLCPP_INFO(get_logger(), "ota server not avalible");
          return;
        }

        std::string json_result;
        CyberdogJson::Document2String(json_resquest, json_result);
        auto req = std::make_shared<protocol::srv::OtaServerCmd::Request>();
        req->request.key = "ota_command_version_query";
        req->request.value = json_result;
        req->request.type = "JSON";
        auto res = ota_client_->async_send_request(req);

        auto status = res.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(get_logger(), "success to call ota services.");
        } else {
          RCLCPP_INFO(get_logger(), "Failed to call ota services.");
        }

        if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate ota response to json");
          retrunErrorGrpc(writer);
          return;
        }

        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate response to json");
          retrunErrorGrpc(writer);
          return;
        }

        grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_VERSION_QUERY_REQUEST);
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      }
      break;
    case ::grpcapi::SendRequest::OTA_START_DOWNLOAD_REQUEST:
      {
        std::chrono::seconds timeout(10);
        if (!ota_client_->wait_for_service(timeout)) {
          RCLCPP_INFO(get_logger(), "ota server not avalible");
          return;
        }

        std::string json_result;
        CyberdogJson::Document2String(json_resquest, json_result);

        auto req = std::make_shared<protocol::srv::OtaServerCmd::Request>();
        req->request.key = "ota_command_start_download";
        req->request.value = json_result;
        req->request.type = "JSON";
        auto res = ota_client_->async_send_request(req);

        auto status = res.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(get_logger(), "success to call ota services.");
        } else {
          RCLCPP_INFO(get_logger(), "Failed to call ota services.");
        }

        if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate ota response to json");
          retrunErrorGrpc(writer);
          return;
        }

        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate response to json");
          retrunErrorGrpc(writer);
          return;
        }

        grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_START_DOWNLOAD_REQUEST);
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      }
      break;
    case ::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST:
      {
        std::chrono::seconds timeout(10);
        if (!ota_client_->wait_for_service(timeout)) {
          RCLCPP_INFO(get_logger(), "ota server not avalible");
          return;
        }

        std::string json_result;
        CyberdogJson::Document2String(json_resquest, json_result);
        auto req = std::make_shared<protocol::srv::OtaServerCmd::Request>();
        req->request.key = "ota_command_start_download";
        req->request.value = json_result;
        req->request.type = "JSON";

        auto res = ota_client_->async_send_request(req);
        auto status = res.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(get_logger(), "success to call ota services.");
        } else {
          RCLCPP_INFO(get_logger(), "Failed to call ota services.");
        }

        if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate ota response to json");
          retrunErrorGrpc(writer);
          return;
        }

        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate response to json");
          retrunErrorGrpc(writer);
          return;
        }

        grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST);
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      }
      break;
    case ::grpcapi::SendRequest::OTA_PROCESS_QUERY_REQUEST:
      {
        std::chrono::seconds timeout(10);
        if (!ota_client_->wait_for_service(timeout)) {
          RCLCPP_INFO(get_logger(), "ota server not avalible");
          return;
        }

        std::string json_result;
        CyberdogJson::Document2String(json_resquest, json_result);
        auto req = std::make_shared<protocol::srv::OtaServerCmd::Request>();
        req->request.key = "ota_command_process_query";
        req->request.value = json_result;
        req->request.type = "JSON";
        auto res = ota_client_->async_send_request(req);

        auto status = res.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(get_logger(), "success to call ota services.");
        } else {
          RCLCPP_INFO(get_logger(), "Failed to call ota services.");
        }

        if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate ota response to json");
          retrunErrorGrpc(writer);
          return;
        }

        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate response to json");
          retrunErrorGrpc(writer);
          return;
        }

        grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST);
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      }
      break;
    case ::grpcapi::SendRequest::OTA_ESTIMATE_UPGRADE_TIME_REQUEST:
      {
        std::chrono::seconds timeout(10);
        if (!ota_client_->wait_for_service(timeout)) {
          RCLCPP_INFO(get_logger(), "ota server not avalible");
          return;
        }

        std::string json_result;
        CyberdogJson::Document2String(json_resquest, json_result);
        auto req = std::make_shared<protocol::srv::OtaServerCmd::Request>();
        req->request.key = "ota_command_estimate_upgrade_time_query";
        req->request.value = json_result;
        req->request.type = "JSON";
        auto res = ota_client_->async_send_request(req);

        auto status = res.wait_for(timeout);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(get_logger(), "success to call ota services.");
        } else {
          RCLCPP_INFO(get_logger(), "Failed to call ota services.");
        }

        if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate ota response to json");
          retrunErrorGrpc(writer);
          return;
        }

        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          RCLCPP_ERROR(get_logger(), "error while encoding authenticate response to json");
          retrunErrorGrpc(writer);
          return;
        }

        grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_ESTIMATE_UPGRADE_TIME_REQUEST);
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      }
      break;
    default:
      break;
  }
}
}  // namespace carpo_cyberdog_app
