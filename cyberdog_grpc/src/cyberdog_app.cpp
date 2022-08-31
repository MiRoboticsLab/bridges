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

#include <stdio.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <sys/time.h>
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
#include "transmit_files.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#define gettid() syscall(SYS_gettid)

using namespace std::chrono_literals;
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

using Navigation = protocol::action::Navigation;

namespace carpo_cyberdog_app
{
static int64_t requestNumber;
Cyberdog_app::Cyberdog_app()
: Node("app_server"),
  ticks_(0),
  can_process_messages_(false),
  heartbeat_err_cnt_(0),
  heart_beat_thread_(nullptr),
  app_server_thread_(nullptr),
  server_(nullptr),
  app_stub_(nullptr),
  app_disconnected(true),
  is_internet(false),
  wifi_strength(0),
  sn("")
{
  if (sn == "") {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grpc_get_sn");
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sn_ger_srv_;
    sn_ger_srv_ =
      node->create_client<std_srvs::srv::Trigger>("get_dog_sn");
    if (!sn_ger_srv_->wait_for_service(std::chrono::seconds(20))) {
      ERROR("call sn server not avalible");
      sn = "unaviable";
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_result = sn_ger_srv_->async_send_request(req);
    if (!(rclcpp::spin_until_future_complete(node, future_result) ==
      rclcpp::FutureReturnCode::SUCCESS))
    {
      ERROR("call get sn service failed!");
      sn = "unkown";
    } else {
      sn = future_result.get()->message;
    }
    INFO("sn:%s", sn.c_str());
  }
  INFO("Cyberdog_app Configuring");
  server_ip = std::make_shared<std::string>("0.0.0.0");

  this->declare_parameter("grpc_server_port", "50052");
  this->declare_parameter("grpc_client_port", "8981");

  grpc_server_port_ = get_parameter("grpc_server_port").as_string();
  grpc_client_port_ = get_parameter("grpc_client_port").as_string();

  // ip_subscriber = this->create_subscription<std_msgs::msg::String>(
  // "ip_notify", rclcpp::SystemDefaultsQoS(),
  // std::bind(&Cyberdog_app::subscribeIp, this, _1));
  connect_status_subscriber = this->create_subscription<protocol::msg::ConnectorStatus>(
    "connector_state", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::subscribeConnectStatus, this, _1));

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
    "robotend_message", rclcpp::SystemDefaultsQoS(),
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
  image_trans_pub_ = this->create_publisher<std_msgs::msg::String>(
    "img_trans_signal_in", 100);
  image_trans_sub_ = this->create_subscription<std_msgs::msg::String>(
    "img_trans_signal_out", 100,
    std::bind(&Cyberdog_app::image_transmission_callback, this, _1));

  // photo and video recording
  camera_service_client_ = this->create_client<protocol::srv::CameraService>(
    "camera_service");

  // ota
  download_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
    "ota_download_percentage", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::HandleDownloadPercentageMsgs, this, _1));

  upgrade_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
    "ota_upgrade_percentage", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::HandleUpgradePercentageMsgs, this, _1));

  reboot_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    "ota_upgrade_reboot", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::HandleUpgradeRebootMsgs, this, _1));

  bms_status_sub_ = this->create_subscription<protocol::msg::BmsStatus>(
    "bms_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::subscribeBmsStatus, this, std::placeholders::_1));

  ota_client_ = this->create_client<protocol::srv::OtaServerCmd>("ota_grpc");

  // connection
  app_connection_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "app_connection_state", rclcpp::SystemDefaultsQoS());

  // robot state
  query_dev_info_client_ =
    this->create_client<protocol::srv::DeviceInfo>("query_divice_info");

  // robot nick name switch
  dev_name_enable_client_ =
    this->create_client<std_srvs::srv::SetBool>("nick_name_switch");

  // robot nick name
  dev_name_set_client_ =
    this->create_client<protocol::srv::AudioNickName>("set_nick_name");

  // audio volume set
  audio_volume_set_client_ =
    this->create_client<protocol::srv::AudioVolumeSet>("audio_volume_set");

  // audio mic set
  audio_execute_client_ =
    this->create_client<protocol::srv::AudioExecute>("set_audio_state");

  audio_action_set_client_ =
    this->create_client<std_srvs::srv::SetBool>("audio_action_set");

  // test
  app_disconnect_pub_ = this->create_publisher<std_msgs::msg::Bool>("disconnect_app", 2);

  // map subscribe
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::processMapMsg, this, _1));

  // dog pose
  dog_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "dog_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::processDogPose, this, _1));

  set_label_client_ = this->create_client<protocol::srv::SetMapLabel>(
    "set_label", rmw_qos_profile_services_default, callback_group_);

  get_label_client_ = this->create_client<protocol::srv::GetMapLabel>(
    "get_label", rmw_qos_profile_services_default, callback_group_);

  navigation_client_ =
    rclcpp_action::create_client<Navigation>(this, "CyberdogNavigation");
}

Cyberdog_app::~Cyberdog_app()
{
  destroyGrpc();
  destroyGrpcServer();
  if (heart_beat_thread_ && heart_beat_thread_->joinable()) {
    heart_beat_thread_->join();
  }
}

void Cyberdog_app::send_msgs_(
  const std::shared_ptr<std::shared_ptr<::grpcapi::SendRequest>> msg)
{
  std::shared_lock<std::shared_mutex> read_lock(stub_mutex_);
  if (can_process_messages_ && app_stub_) {
    app_stub_->sendRequest(**msg);
  }
}

void Cyberdog_app::HeartBeat()
{
  rclcpp::WallRate r(500ms);
  std::string ipv4;
  while (rclcpp::ok()) {
    if (can_process_messages_) {
      bool hearbeat_result(false);
      {
        std::shared_lock<std::shared_mutex> read_lock(stub_mutex_);
        if (app_stub_) {
          hearbeat_result =
            app_stub_->sendHeartBeat(
            local_ip, wifi_strength, bms_status.batt_soc, is_internet, sn);
        } else {
          continue;
        }
      }
      if (!hearbeat_result) {
        if (heartbeat_err_cnt_++ >= APP_CONNECTED_FAIL_CNT) {
          std_msgs::msg::Bool msg;
          msg.data = false;
          app_connection_pub_->publish(msg);
          if (!app_disconnected) {
            destroyGrpc();
            createGrpc();
          }
        }
      } else {
        std_msgs::msg::Bool msg;
        msg.data = true;
        app_connection_pub_->publish(msg);
      }
    }
    r.sleep();
  }
}

std::string Cyberdog_app::getServiceIp() {return *server_ip;}

void Cyberdog_app::RunServer()
{
  INFO("run_server thread id is %ld", gettid());
  std::string server_address("0.0.0.0:");
  server_address += grpc_server_port_;
  CyberdogAppImpl service(server_address);
  service.SetRequesProcess(this);
  ServerBuilder builder;
  builder.SetMaxSendMessageSize(CHUNK_SIZE / 4 * 5);
  builder.SetMaxReceiveMessageSize(CHUNK_SIZE / 4 * 5);
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
// void Cyberdog_app::subscribeIp(const std_msgs::msg::String::SharedPtr msg)
// {
// INFO("get ip :%s", msg->data.c_str());
// INFO("old phoneip is:%s", (*server_ip).c_str());
// app_disconnected = false;
// local_ip = getDogIp(msg->data, ":");
// INFO("local_ip ip :%s", local_ip.c_str());
// std::string phoneIp = getPhoneIp(msg->data, ":");
// INFO("phoneIp ip :%s", phoneIp.c_str());
// if (*server_ip != phoneIp) {
// server_ip = std::make_shared<std::string>(phoneIp);
// destroyGrpc();
// createGrpc();
// }
// }

void Cyberdog_app::subscribeConnectStatus(const protocol::msg::ConnectorStatus::SharedPtr msg)
{
  if (msg->is_connected) {
    local_ip = msg->robot_ip;
    std::string phoneIp = msg->provider_ip;
    if (*server_ip != phoneIp) {
      app_disconnected = false;
      INFO("local_ip ip :%s,pheneIp ip :%s", local_ip.c_str(), phoneIp.c_str());
      server_ip = std::make_shared<std::string>(phoneIp);
      is_internet = msg->is_internet;
      wifi_strength = msg->strength;
      destroyGrpc();
      createGrpc();
    }
  }
}

void Cyberdog_app::subscribeBmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
{
  bms_status = *msg;
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
  std::unique_lock<std::shared_mutex> write_lock(stub_mutex_);
  can_process_messages_ = false;
  if (app_stub_ != nullptr) {
    app_stub_ = nullptr;
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
  grpc::string ip = *server_ip + std::string(":") + grpc_client_port_;
  can_process_messages_ = false;
  heartbeat_err_cnt_ = 0;
  net_checker.set_ip(*server_ip);
  auto channel_ = grpc::CreateChannel(ip, grpc::InsecureChannelCredentials());
  std::unique_lock<std::shared_mutex> write_lock(stub_mutex_);
  app_stub_ = std::make_shared<Cyberdog_App_Client>(channel_);
  can_process_messages_ = true;
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

// for image transmission
void Cyberdog_app::image_transmission_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  send_grpc_msg(::grpcapi::SendRequest::IMAGE_TRANSMISSION_REQUEST, msg->data);
}

// for photo and video recording
bool Cyberdog_app::callCameraService(uint8_t command, uint8_t & result, std::string & msg)
{
  if (!camera_service_client_->wait_for_service(std::chrono::seconds(5))) {
    WARN("camera_service is not activate");
    return false;
  }
  auto request = std::make_shared<protocol::srv::CameraService::Request>();
  request->command = command;
  auto future_result = camera_service_client_->async_send_request(request);
  std::future_status status = future_result.wait_for(std::chrono::seconds(5));
  if (status == std::future_status::ready) {
    result = future_result.get()->result;
    msg = future_result.get()->msg;
    INFO("Succeeded to call camera_service services.");
  } else {
    WARN("Failed to call camera_service services.");
    return false;
  }
  return true;
}

void Cyberdog_app::processMapMsg(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  Document json_response(kObjectType);
  CyberdogJson::Add(json_response, "resolution", msg->info.resolution);
  CyberdogJson::Add(
    json_response, "width",
    static_cast<int>(msg->info.width));
  CyberdogJson::Add(
    json_response, "height",
    static_cast<int>(msg->info.height));
  CyberdogJson::Add(json_response, "px", msg->info.origin.position.x);
  CyberdogJson::Add(json_response, "py", msg->info.origin.position.y);
  CyberdogJson::Add(json_response, "pz", msg->info.origin.position.z);
  CyberdogJson::Add(json_response, "qx", msg->info.origin.orientation.x);
  CyberdogJson::Add(json_response, "qy", msg->info.origin.orientation.y);
  CyberdogJson::Add(json_response, "qz", msg->info.origin.orientation.z);
  CyberdogJson::Add(json_response, "qw", msg->info.origin.orientation.w);
  CyberdogJson::Add(json_response, "data", msg->data);
  send_grpc_msg(::grpcapi::SendRequest::MAP_DATA_REQUEST, json_response);
}

// process dog pose
void Cyberdog_app::processDogPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  Document json_response(kObjectType);
  CyberdogJson::Add(json_response, "px", msg->pose.position.x);
  CyberdogJson::Add(json_response, "py", msg->pose.position.y);
  CyberdogJson::Add(json_response, "pz", msg->pose.position.z);
  CyberdogJson::Add(json_response, "qx", msg->pose.orientation.x);
  CyberdogJson::Add(json_response, "qy", msg->pose.orientation.y);
  CyberdogJson::Add(json_response, "qz", msg->pose.orientation.z);
  CyberdogJson::Add(json_response, "qw", msg->pose.orientation.w);
  INFO("sending dog pose");
  send_grpc_msg(::grpcapi::SendRequest::MAP_DOG_POSE_REQUEST, json_response);
}

bool parseCameraServiceResponseString(
  const std::string & str, size_t & file_size,
  std::string & file_name)
{
  std::stringstream ss;
  size_t comma_index = str.find(',', 0);
  if (comma_index == string::npos) {
    ERROR("error while parsing camera_service respose msg");
    return false;
  }
  file_name = str.substr(0, comma_index);
  ss << str.substr(comma_index + 1);
  ss >> file_size;
  return true;
}

bool Cyberdog_app::returnResponse(
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer,
  uint8_t result,
  const std::string & msg,
  uint32_t namecode)
{
  Document json_response(kObjectType);
  std::string rsp_string;
  CyberdogJson::Add(json_response, "result", result);
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("Error while encoding camera_service response to json");
    retrunErrorGrpc(writer);
    return false;
  }
  ::grpcapi::RecResponse grpc_respond;
  grpc_respond.set_namecode(namecode);
  grpc_respond.set_data(rsp_string);
  writer->Write(grpc_respond);
  return true;
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
  ::grpcapi::SendRequest * grpc_respons = new ::grpcapi::SendRequest();
  grpc_respons->set_namecode(code);
  grpc_respons->set_params(msg);
  auto sender = send_thread_map_.find(code);
  if (sender != send_thread_map_.end()) {
    INFO("found sender for %d", code);
    sender->second->push(std::shared_ptr<::grpcapi::SendRequest>(grpc_respons));
  } else {
    INFO("create sender for %d", code);
    auto new_sender = std::shared_ptr<
      LatestMsgDispather<std::shared_ptr<::grpcapi::SendRequest>>>(
      new LatestMsgDispather<std::shared_ptr<::grpcapi::SendRequest>>());
    send_thread_map_[code] = new_sender;
    new_sender->setCallback(std::bind(&Cyberdog_app::send_msgs_, this, _1));
    new_sender->push(std::shared_ptr<::grpcapi::SendRequest>(grpc_respons));
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

bool Cyberdog_app::HandleOTAStatusRequest(
  const Document & json_resquest,
  ::grpcapi::RecResponse & grpc_respond,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    INFO("ota server not avalible");
    return false;
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
    INFO("success to call ota services.");
  } else {
    INFO("Failed to call ota services.");
  }

  if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
    ERROR("error while encoding authenticate ota response to json");
    retrunErrorGrpc(writer);
    return false;
  }

  CyberdogJson::Get(json_response, "status", response_string);
  grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_STATUS_REQUEST);
  grpc_respond.set_data(response_string);
  writer->Write(grpc_respond);

  return true;
}

void Cyberdog_app::HandleDownloadPercentageMsgs(const std_msgs::msg::Int32 msg)
{
  Document progress_response(kObjectType);
  std::string response_string;

  CyberdogJson::Add(progress_response, "upgrade_progress", 0);
  if (msg.data < 0) {
    CyberdogJson::Add(progress_response, "download_progress", -1);
    CyberdogJson::Add(progress_response, "code", msg.data);
  } else {
    CyberdogJson::Add(progress_response, "download_progress", msg.data);
    CyberdogJson::Add(progress_response, "code", 0);
  }
  CyberdogJson::Document2String(progress_response, response_string);

  // INFO("upgrade_progress: %d", 0);
  INFO("download_progress: %d", msg.data);

  send_grpc_msg(::grpcapi::SendRequest::OTA_PROCESS_QUERY_REQUEST, response_string);
}

void Cyberdog_app::HandleUpgradePercentageMsgs(const std_msgs::msg::Int32 msg)
{
  Document progress_response(kObjectType);
  std::string response_string;

  if (msg.data < 0) {
    CyberdogJson::Add(progress_response, "upgrade_progress", -1);
    CyberdogJson::Add(progress_response, "code", msg.data);
  } else {
    CyberdogJson::Add(progress_response, "upgrade_progress", msg.data);
    CyberdogJson::Add(progress_response, "code", 0);
  }
  CyberdogJson::Add(progress_response, "download_progress", 0);
  CyberdogJson::Document2String(progress_response, response_string);

  INFO("upgrade_progress: %d", msg.data);
  // INFO("download_progress: %d", 0);

  send_grpc_msg(::grpcapi::SendRequest::OTA_PROCESS_QUERY_REQUEST, response_string);
}

void Cyberdog_app::HandleUpgradeRebootMsgs(const std_msgs::msg::Bool msg)
{
  Document progress_response(kObjectType);
  std::string response_string = "";
  CyberdogJson::Document2String(progress_response, response_string);

  INFO("reboot data: %d", msg.data);
  send_grpc_msg(::grpcapi::SendRequest::OTA_NX_REBOOT, response_string);
}

bool Cyberdog_app::HandleOTAVersionQueryRequest(
  const Document & json_resquest,
  ::grpcapi::RecResponse & grpc_respond,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  INFO("GRPC version query.");
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    INFO("ota server not avalible");
    return false;
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
    INFO("success to call ota services.");
  } else {
    INFO("Failed to call ota services.");
  }

  if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
    ERROR("error while encoding authenticate ota response to json");
    retrunErrorGrpc(writer);
    return false;
  }

  CyberdogJson::Get(json_response, "version", response_string);
  grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_VERSION_QUERY_REQUEST);
  grpc_respond.set_data(response_string);
  writer->Write(grpc_respond);
  return true;
}

bool Cyberdog_app::HandleOTAStartDownloadRequest(
  const Document & json_resquest,
  ::grpcapi::RecResponse & grpc_respond,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    INFO("ota server not avalible");
    return false;
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
    INFO("success to call ota services.");
  } else {
    INFO("Failed to call ota services.");
  }

  grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_START_DOWNLOAD_REQUEST);
  grpc_respond.set_data(res.get()->response.value);
  writer->Write(grpc_respond);

  return true;
}

bool Cyberdog_app::HandleOTAStartUpgradeRequest(
  const Document & json_resquest,
  ::grpcapi::RecResponse & grpc_respond,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    INFO("ota server not avalible");
    return false;
  }

  std::string json_result;
  CyberdogJson::Document2String(json_resquest, json_result);
  auto req = std::make_shared<protocol::srv::OtaServerCmd::Request>();
  req->request.key = "ota_command_start_upgrade";
  req->request.value = json_result;
  req->request.type = "JSON";

  auto res = ota_client_->async_send_request(req);
  auto status = res.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO("success to call ota services.");
  } else {
    INFO("Failed to call ota services.");
  }

  std::cout << "response_string: " << res.get()->response.value << std::endl;

  grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST);
  grpc_respond.set_data(res.get()->response.value);
  writer->Write(grpc_respond);

  return true;
}

bool Cyberdog_app::HandleOTAProcessQueryRequest(
  const Document & json_resquest,
  ::grpcapi::RecResponse & grpc_respond,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    INFO("ota server not avalible");
    return false;
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
    INFO("success to call ota services.");
  } else {
    INFO("Failed to call ota services.");
  }

  if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
    ERROR("error while encoding authenticate ota response to json");
    retrunErrorGrpc(writer);
    return false;
  }

  CyberdogJson::Get(json_response, "progress", response_string);
  grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST);
  grpc_respond.set_data(response_string);
  writer->Write(grpc_respond);

  return true;
}

bool Cyberdog_app::HandleOTAEstimateUpgradeTimeRequest(
  const Document & json_resquest,
  ::grpcapi::RecResponse & grpc_respond,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    INFO("ota server not avalible");
    return false;
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
    INFO("success to call ota services.");
  } else {
    INFO("Failed to call ota services.");
  }

  if (!CyberdogJson::String2Document(res.get()->response.value, json_response)) {
    ERROR("error while encoding authenticate ota response to json");
    retrunErrorGrpc(writer);
    return false;
  }

  if (!CyberdogJson::Document2String(json_response, response_string)) {
    ERROR("error while encoding authenticate response to json");
    retrunErrorGrpc(writer);
    return false;
  }

  grpc_respond.set_namecode(::grpcapi::SendRequest::OTA_ESTIMATE_UPGRADE_TIME_REQUEST);
  grpc_respond.set_data(response_string);
  writer->Write(grpc_respond);
  return true;
}

void Cyberdog_app::handleMappingRequest(
  const Document & json_resquest, ::grpcapi::RecResponse & grpc_respond,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  int timeout = 60;
  std::string response_string;
  std::string status;
  double goal_x, goal_y;
  CyberdogJson::Get(json_resquest, "status", status);
  INFO("handleMappingRequest");
  auto mode_goal = Navigation::Goal();
  if (status == "START") {
    mode_goal.nav_type = Navigation::Goal::NAVIGATION_TYPE_START_MAPPING;
  } else if (status == "STOP") {
    mode_goal.nav_type = Navigation::Goal::NAVIGATION_TYPE_STOP_MAPPING;
  } else if (status == "NAVIGATION_AB") {
    if (!json_resquest.HasMember("goalX") || !json_resquest.HasMember("goalY")) {
      ERROR("NAVIGATION_AB should have goalX and goalY settings");
      retrunErrorGrpc(writer);
    }
    CyberdogJson::Get(json_resquest, "goalX", goal_x);
    CyberdogJson::Get(json_resquest, "goalY", goal_y);
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    mode_goal.poses.push_back(goal);
    mode_goal.nav_type = Navigation::Goal::NAVIGATION_TYPE_START_AB;
  } else if (status == "STOP_NAVIGATION_AB") {
    mode_goal.nav_type = Navigation::Goal::NAVIGATION_TYPE_STOP_AB;
  } else if (status == "START_NAVIGATION") {
    mode_goal.nav_type = Navigation::Goal::NAVIGATION_TYPE_START_LOCALIZATION;
  } else if (status == "STOP_NAVIGATION") {
    mode_goal.nav_type = Navigation::Goal::NAVIGATION_TYPE_STOP_LOCALIZATION;
  } else if (status == "START_AUTO_DOCKING") {
    mode_goal.nav_type = Navigation::Goal::NAVIGATION_TYPE_START_AUTO_DOCKING;
  } else if (status == "STOP_AUTO_DOCKING") {
    mode_goal.nav_type = Navigation::Goal::NAVIGATION_TYPE_STOP_AUTO_DOCKING;
  } else {
    ERROR("Unavailable navigation type: %s", status.c_str());
    retrunErrorGrpc(writer);
  }
  auto mode_goal_handle = navigation_client_->async_send_goal(mode_goal);
  auto mode_result =
    navigation_client_->async_get_result(mode_goal_handle.get());
  uint8_t result = 2;
  mode_result.wait_for(std::chrono::seconds(60));
  if (mode_goal_handle.get()->is_result_aware()) {
    if (mode_result.get().result->result ==
      protocol::action::Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS)
    {
      INFO("Navigation action success");
    } else {
      WARN("Navigation action fail");
    }
    result = mode_result.get().result->result;
  }
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> json_writer(strBuf);
  json_writer.StartObject();
  json_writer.Key("result");
  json_writer.Int(result);
  json_writer.EndObject();
  response_string = strBuf.GetString();
  grpc_respond.set_namecode(::grpcapi::SendRequest::MAP_MAPPING_RQUEST);
  grpc_respond.set_data(response_string);
  writer->Write(grpc_respond);
}

void Cyberdog_app::handlLableGetRequest(
  const Document & json_resquest, ::grpcapi::RecResponse & grpc_respond,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!get_label_client_->wait_for_service()) {
    INFO("get map label server not avalible");
    return;
  }
  auto request = std::make_shared<protocol::srv::GetMapLabel::Request>();
  CyberdogJson::Get(json_resquest, "mapName", request->map_name);
  std::chrono::seconds timeout(5);

  auto future_result = get_label_client_->async_send_request(request);
  std::future_status status = future_result.wait_for(timeout);

  if (status == std::future_status::ready) {
    // if (future_result.get()->success ==
    //     protocol::srv::GetMapLabel_Response::RESULT_SUCCESS) {
    protocol::msg::MapLabel labels = future_result.get()->label;
    grpc_respond.set_namecode(::grpcapi::SendRequest::MAP_GET_LABLE_REQUEST);
    rapidjson::StringBuffer strBuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
    writer.StartObject();
    writer.Key("mapName");
    writer.String(labels.map_name.c_str());
    writer.Key("locationLabelInfo");
    writer.StartArray();
    for (int i = 0; i < labels.labels.size(); ++i) {
      writer.StartObject();
      writer.Key("labelName");
      writer.String(labels.labels[i].label_name.c_str());
      writer.Key("physicX");
      writer.Double(labels.labels[i].physic_x);
      writer.Key("physicY");
      writer.Double(labels.labels[i].physic_y);
      writer.EndObject();
    }
    writer.EndArray();
    writer.Key("map");
    writer.StartObject();
    writer.Key("resolution");
    writer.Double(labels.map.info.resolution);
    writer.Key("width");
    writer.Int(labels.map.info.width);
    writer.Key("height");
    writer.Int(labels.map.info.height);
    writer.Key("px");
    writer.Double(labels.map.info.origin.position.x);
    writer.Key("py");
    writer.Double(labels.map.info.origin.position.y);
    writer.Key("pz");
    writer.Double(labels.map.info.origin.position.z);
    writer.Key("qx");
    writer.Double(labels.map.info.origin.orientation.x);
    writer.Key("qy");
    writer.Double(labels.map.info.origin.orientation.y);
    writer.Key("qz");
    writer.Double(labels.map.info.origin.orientation.z);
    writer.Key("qw");
    writer.Double(labels.map.info.origin.orientation.w);
    writer.Key("data");
    writer.StartArray();
    for (auto & each_cell : labels.map.data) {
      writer.Int(each_cell);
    }
    writer.EndArray();
    writer.EndObject();
    writer.EndObject();
    string data = strBuf.GetString();
    grpc_respond.set_data(data);

    std::cout << "json data: " << data << std::endl;


    INFO("Succeed call get map_label services.");
    // } else {
    //   INFO("failed call get map_label services.");
    // }
  } else {
    INFO("Failed to call get map_label services.");
  }
  grpc_writer->Write(grpc_respond);
}

void Cyberdog_app::handlLableSetRequest(
  const Document & json_resquest, ::grpcapi::RecResponse & grpc_respond,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  protocol::msg::MapLabel map_label;
  bool has_label = false;
  std::string response_string;
  bool only_delete = false;

  CyberdogJson::Get(json_resquest, "mapName", map_label.map_name);
  CyberdogJson::Get(json_resquest, "only_delete", only_delete);
  INFO("handlLableSetRequest %s, %d", map_label.map_name.c_str(), only_delete);
  if (json_resquest.HasMember("locationLabelInfo") &&
    json_resquest["locationLabelInfo"].IsArray())
  {
    auto labels = json_resquest["locationLabelInfo"].GetArray();
    for (int i = 0; i < labels.Size(); i++) {
      INFO("handlLableSetRequest get a label");
      protocol::msg::Label label;
      if (labels[i].HasMember("labelName") &&
        labels[i]["labelName"].IsString() && labels[i].HasMember("physicX") &&
        labels[i]["physicX"].IsDouble() && labels[i].HasMember("physicY") &&
        labels[i]["physicY"].IsDouble())
      {
        label.label_name = labels[i]["labelName"].GetString();
        label.physic_x = labels[i]["physicX"].GetDouble();
        label.physic_y = labels[i]["physicY"].GetDouble();
        map_label.labels.push_back(label);
        has_label = true;
        INFO(
          "handlLableSetRequest get a label name: %s, x:%f, y:%f",
          label.label_name.c_str(), label.physic_x, label.physic_y);
      }
    }
  }

  INFO("handlLableSetRequest has_label %d", has_label);

  if (has_label) {
    auto request = std::make_shared<protocol::srv::SetMapLabel::Request>();
    request->label = map_label;
    request->only_delete = only_delete;

    if (!set_label_client_->wait_for_service()) {
      INFO("set map label server not avalible");
      return;
    }
    std::chrono::seconds timeout(5);
    auto future_result = set_label_client_->async_send_request(request);
    std::future_status status = future_result.wait_for(timeout);
    if (status == std::future_status::ready) {
      if (future_result.get()->success ==
        protocol::srv::SetMapLabel_Response::RESULT_SUCCESS)
      {
        INFO("Succeed call map_label services.");
      } else {
        INFO("failed call map_label services.");
      }
    } else {
      INFO("Failed to call map_label services.");
    }
  }

  grpc_respond.set_namecode(::grpcapi::SendRequest::MAP_SET_LABLE_REQUEST);
  grpc_respond.set_data(response_string);
  writer->Write(grpc_respond);
}

void Cyberdog_app::ProcessMsg(
  const ::grpcapi::SendRequest * grpc_request,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  INFO(
    "ProcessMsg %d %s", grpc_request->namecode(),
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
      {
        if (!query_dev_info_client_->wait_for_service()) {
          INFO(
            "call querydevinfo server not avaiable"
          );
          return;
        }
        bool is_sn = false;
        bool is_version = false;
        bool is_uid = false;
        bool is_nick_name = false;
        bool is_volume = false;
        bool is_mic_state = false;
        bool is_voice_control = false;
        bool is_wifi = false;
        bool is_bat_info = false;
        bool is_motor_temper = false;
        bool is_audio_state = false;
        bool is_device_model = false;
        bool is_stand_up = false;
        CyberdogJson::Get(json_resquest, "is_sn", is_sn);
        CyberdogJson::Get(json_resquest, "is_version", is_version);
        CyberdogJson::Get(json_resquest, "is_uid", is_version);
        CyberdogJson::Get(json_resquest, "is_nick_name", is_nick_name);
        CyberdogJson::Get(json_resquest, "is_volume", is_volume);
        CyberdogJson::Get(json_resquest, "is_mic_state", is_mic_state);
        CyberdogJson::Get(json_resquest, "is_voice_control", is_voice_control);
        CyberdogJson::Get(json_resquest, "is_wifi", is_wifi);
        CyberdogJson::Get(json_resquest, "is_bat_info", is_bat_info);
        CyberdogJson::Get(json_resquest, "is_motor_temper", is_motor_temper);
        CyberdogJson::Get(json_resquest, "is_audio_state", is_audio_state);
        CyberdogJson::Get(json_resquest, "is_device_model", is_device_model);
        CyberdogJson::Get(json_resquest, "is_stand", is_stand_up);
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::DeviceInfo::Request>();
        req->enables.resize(20);
        req->enables[0] = is_sn;
        req->enables[1] = is_version;
        req->enables[2] = is_uid;
        req->enables[3] = is_nick_name;
        req->enables[4] = is_volume;
        req->enables[5] = is_mic_state;
        req->enables[6] = is_voice_control;
        req->enables[7] = is_wifi;
        req->enables[8] = is_bat_info;
        req->enables[9] = is_motor_temper;
        req->enables[10] = is_audio_state;
        req->enables[11] = is_device_model;
        req->enables[12] = is_stand_up;
        auto future_result = query_dev_info_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call querydevinfo request services.");
        } else {
          INFO(
            "Failed to call querydevinfo request  services.");
        }
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data(future_result.get()->info);
        writer->Write(grpc_respond);
      }
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

    case ::grpcapi::SendRequest::DEVICE_NAME_SWITCH: {
        if (!dev_name_enable_client_->wait_for_service()) {
          INFO(
            "call set nickname switch server not avaiable"
          );
          retrunErrorGrpc(writer);
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        CyberdogJson::Get(json_resquest, "enable", req->data);
        auto future_result = dev_name_enable_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call set nickname switch request services.");
        } else {
          INFO(
            "Failed to call set nickname switch request  services.");
        }
        CyberdogJson::Add(json_response, "success", future_result.get()->success);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          ERROR("error while device name switch response encoding to json");
          retrunErrorGrpc(writer);
          return;
        }
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      } break;

    case ::grpcapi::SendRequest::DEVICE_NAME_SET: {
        if (!dev_name_set_client_->wait_for_service()) {
          INFO(
            "call setnickname server not avaiable"
          );
          retrunErrorGrpc(writer);
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioNickName::Request>();
        CyberdogJson::Get(json_resquest, "nick_name", req->nick_name);
        CyberdogJson::Get(json_resquest, "wake_name", req->wake_name);
        auto future_result = dev_name_set_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call setnickname request services.");
        } else {
          INFO(
            "Failed to call setnickname request  services.");
        }
        CyberdogJson::Add(json_response, "success", future_result.get()->success);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          ERROR("error while device name set response encoding to json");
          retrunErrorGrpc(writer);
          return;
        }
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      } break;

    case ::grpcapi::SendRequest::DEVICE_VOLUME_SET: {
        if (!audio_volume_set_client_->wait_for_service()) {
          INFO(
            "call volume set server not avalible");
          retrunErrorGrpc(writer);
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioVolumeSet::Request>();
        int volume;
        CyberdogJson::Get(json_resquest, "volume", volume);
        req->volume = volume;
        auto future_result = audio_volume_set_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call setvolume request services.");
        } else {
          INFO(
            "Failed to call setvolume request  services.");
        }
        CyberdogJson::Add(json_response, "success", future_result.get()->success);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          ERROR("error while volume set response encoding to json");
          retrunErrorGrpc(writer);
          return;
        }
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      } break;

    case ::grpcapi::SendRequest::DEVICE_MIC_SET: {
        if (!audio_execute_client_->wait_for_service()) {
          INFO(
            "call mic set server not avalible");
          retrunErrorGrpc(writer);
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioExecute::Request>();
        req->client = "app_server";
        bool enable;
        CyberdogJson::Get(json_resquest, "enable", enable);
        req->status.state =
          (enable ==
          true) ? protocol::msg::AudioStatus::AUDIO_STATUS_NORMAL : protocol::msg::AudioStatus::
          AUDIO_STATUS_OFFMIC;
        auto future_result = audio_execute_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call set mic state request services.");
        } else {
          INFO(
            "Failed to call set mic state request  services.");
        }
        CyberdogJson::Add(json_response, "success", future_result.get()->result);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          ERROR("error while set mic state response encoding to json");
          retrunErrorGrpc(writer);
          return;
        }
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      } break;

    case ::grpcapi::SendRequest::DEVICE_AUDIO_SET: {
        if (!audio_action_set_client_->wait_for_service()) {
          INFO(
            "call audio action set server not avalible");
          retrunErrorGrpc(writer);
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        bool enable;
        CyberdogJson::Get(json_resquest, "enable", enable);
        req->data = enable;
        auto future_result = audio_action_set_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call set audio action request services.");
        } else {
          INFO(
            "Failed to call set audio action request  services.");
        }
        CyberdogJson::Add(json_response, "success", future_result.get()->success);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          ERROR("error while set audio action response encoding to json");
          retrunErrorGrpc(writer);
          return;
        }
        grpc_respond.set_data(rsp_string);
        writer->Write(grpc_respond);
      } break;

    case ::grpcapi::SendRequest::AUDIO_AUTHENTICATION_REQUEST: {
        if (!audio_auth_request->wait_for_service()) {
          INFO(
            "callAuthenticateRequest server not avalible");
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioAuthId::Request>();
        protocol::srv::AudioAuthId::Response rsp;
        auto future_result = audio_auth_request->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call authenticate request services.");
        } else {
          INFO(
            "Failed to call authenticate request  services.");
        }
        rsp.did = future_result.get()->did;
        rsp.sn = future_result.get()->sn;
        CyberdogJson::Add(json_response, "did", rsp.did);
        CyberdogJson::Add(json_response, "sn", rsp.sn);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          ERROR(
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
          INFO(
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
          INFO(
            "success to call authenticate response services.");
        } else {
          INFO(
            "Failed to call authenticate response services.");
        }
        rsp.result = future_result.get()->result;
        CyberdogJson::Add(json_response, "result", rsp.result);
        if (!CyberdogJson::Document2String(json_response, rsp_string)) {
          ERROR(
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
          INFO(
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
          INFO(
            "success to call voiceprint train start response services.");
        } else {
          INFO(
            "Failed to call voiceprint train start response services.");
          return;
        }
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data("{}");
        writer->Write(grpc_respond);
      } break;
    case ::grpcapi::SendRequest::AUDIO_VOICEPRINTTRAIN_CANCEL: {
        if (!audio_voiceprint_train->wait_for_service()) {
          INFO(
            "call voiceprint train cancel server not avalible");
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioVoiceprintTrain::Request>();
        req->train_id = protocol::srv::AudioVoiceprintTrain::Request::TID_CANCEL;
        auto future_result = audio_voiceprint_train->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call voiceprint train cancel response services.");
        } else {
          INFO(
            "Failed to call voiceprint train cancel response services.");
          return;
        }
        grpc_respond.set_namecode(grpc_request->namecode());
        grpc_respond.set_data("{}");
        writer->Write(grpc_respond);
      } break;
    case ::grpcapi::SendRequest::AUDIO_VOICEPRINTS_DATA: {
        if (!voiceprints_data_notify->wait_for_service()) {
          INFO(
            "call voiceprints data server not avalible");
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioVoiceprintsSet::Request>();
        auto future_result = voiceprints_data_notify->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call voiceprints data response services.");
        } else {
          INFO(
            "Failed to call voiceprints data response services.");
          return;
        }
      } break;
    case ::grpcapi::SendRequest::IMAGE_TRANSMISSION_REQUEST: {
        std_msgs::msg::String it_msg;
        if (!CyberdogJson::Document2String(json_resquest, it_msg.data)) {
          ERROR("error while parse image transmission data to string");
          retrunErrorGrpc(writer);
          return;
        }
        image_trans_pub_->publish(it_msg);
      } break;
    case ::grpcapi::SendRequest::CAMERA_SERVICE: {
        uint32_t command = 2;
        CyberdogJson::Get(json_resquest, "command", command);
        if (!processCameraMsg(grpc_request->namecode(), command, writer)) {
          return;
        }
      } break;
    case ::grpcapi::SendRequest::OTA_STATUS_REQUEST:
      {
        if (!HandleOTAStatusRequest(json_resquest, grpc_respond, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_VERSION_QUERY_REQUEST:
      {
        if (!HandleOTAVersionQueryRequest(json_resquest, grpc_respond, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_START_DOWNLOAD_REQUEST:
      {
        if (!HandleOTAStartDownloadRequest(json_resquest, grpc_respond, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST:
      {
        if (!HandleOTAStartUpgradeRequest(json_resquest, grpc_respond, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_PROCESS_QUERY_REQUEST:
      {
        if (!HandleOTAProcessQueryRequest(json_resquest, grpc_respond, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_ESTIMATE_UPGRADE_TIME_REQUEST:
      {
        if (!HandleOTAEstimateUpgradeTimeRequest(json_resquest, grpc_respond, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::MAP_SET_LABLE_REQUEST: {
        handlLableSetRequest(json_resquest, grpc_respond, writer);
      } break;

    case ::grpcapi::SendRequest::MAP_GET_LABLE_REQUEST: {
        handlLableGetRequest(json_resquest, grpc_respond, writer);
      } break;
    case ::grpcapi::SendRequest::MAP_MAPPING_RQUEST: {
        handleMappingRequest(json_resquest, grpc_respond, writer);
      } break;
    case 55001: {
        std_msgs::msg::Bool msg;
        msg.data = true;
        grpc_respond.set_namecode(55001);
        grpc_respond.set_data("");
        writer->Write(grpc_respond);
        app_disconnect_pub_->publish(msg);
      } break;
    default:
      break;
  }
}

void Cyberdog_app::ProcessGetFile(
  const ::grpcapi::SendRequest * grpc_request,
  ::grpc::ServerWriter<::grpcapi::FileChunk> * writer)
{
  INFO_STREAM("getFile, namecode: " << grpc_request->namecode());
  Document json_request(kObjectType);
  std::string rsp_string;
  json_request.Parse<0>(grpc_request->params().c_str());
  switch (grpc_request->namecode()) {
    case ::grpcapi::SendRequest::CAMERA_SERVICE: {
        uint32_t command = 255;
        CyberdogJson::Get(json_request, "command", command);
        if (!processCameraMsg(grpc_request->namecode(), command, writer)) {
          return;
        }
      } break;
    default: {
        ERROR("nameCode is wrong for calling getFile service!");
        TransmitFiles::SendErrorCode(255, writer);
      } break;
  }
}

bool Cyberdog_app::processCameraMsg(
  uint32_t namecode,
  uint8_t command,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  uint8_t result;
  std::string msg;
  if (!callCameraService(command, result, msg)) {
    result = 7;
  }
  return returnResponse(writer, result, msg, namecode);
}

bool Cyberdog_app::processCameraMsg(
  uint32_t namecode,
  uint8_t command,
  ::grpc::ServerWriter<::grpcapi::FileChunk> * writer)
{
  uint8_t result;
  std::string msg;
  if (!callCameraService(command, result, msg)) {
    result = 7;
  }
  return TransmitFiles::ReturnCameraFile(writer, result, msg);
}
}  // namespace carpo_cyberdog_app
