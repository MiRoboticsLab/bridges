// Copyright (c) 2023 Xiaomi Corporation
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
#include <set>
#include <atomic>
#include <condition_variable>
#include <list>

#include "cyberdog_app_server.hpp"
#include "transmit_files.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#define gettid() syscall(SYS_gettid)

using namespace std::chrono_literals;
using grpc::ServerWriter;
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

namespace cyberdog
{
namespace bridges
{
std::atomic_int TransmitFiles::thread_counts_ = 0;
static int64_t requestNumber;
Cyberdog_app::Cyberdog_app()
: Node("app_server"),
  ticks_(0),
  can_process_messages_(false),
  heart_beat_thread_(nullptr),
  app_server_thread_(nullptr),
  server_(nullptr),
  app_stub_(nullptr),
  app_disconnected(true),
  is_internet(false),
  wifi_strength(0),
  sn("")
{
  ready_sn_ptr =
    std::make_unique<cyberdog::bridges::ReadySnNode>();
  sn = ready_sn_ptr->WaitSn();
  // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grpc_get_sn");
  // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sn_ger_srv_;
  // sn_ger_srv_ =
  //   node->create_client<std_srvs::srv::Trigger>("get_dog_sn");
  // if (!sn_ger_srv_->wait_for_service(std::chrono::seconds(20))) {
  //   ERROR("call sn server not available");
  //   sn = "unaviable";
  // } else {
  //   auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  //   auto future_result = sn_ger_srv_->async_send_request(req);
  //   if (!(rclcpp::spin_until_future_complete(node, future_result, std::chrono::seconds(3)) ==
  //     rclcpp::FutureReturnCode::SUCCESS))
  //   {
  //     ERROR("call get sn service failed!");
  //     sn = "unkown";
  //   } else {
  //     sn = future_result.get()->message;
  //   }
  // }
  INFO("sn:%s", sn.c_str());
  INFO("Cyberdog_app Configuring");
  server_ip = std::make_shared<std::string>("0.0.0.0");

  this->declare_parameter("grpc_server_port", "50052");
  this->declare_parameter("grpc_client_port", "8981");

  grpc_server_port_ = "50052";
  grpc_client_port_ = "8981";

  INFO("Start creating ROS components.");
  connector_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions connector_sub_option;
  connector_sub_option.callback_group = connector_callback_group_;
  connect_status_subscriber = this->create_subscription<protocol::msg::ConnectorStatus>(
    "connector_state", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::subscribeConnectStatus, this, _1),
    connector_sub_option);

  timer_interval.init();

  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // ros interaction codes
  motion_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions motion_sub_option;
  motion_sub_option.callback_group = motion_callback_group_;
  motion_servo_response_sub_ =
    this->create_subscription<protocol::msg::MotionServoResponse>(
    "motion_servo_response", 1000,
    std::bind(&Cyberdog_app::motion_servo_rsp_callback, this, _1),
    motion_sub_option);
  namecode_queue_size_[::grpcapi::SendRequest::MOTION_SERVO_RESPONSE] = 2;

  motion_servo_request_pub_ =
    this->create_publisher<protocol::msg::MotionServoCmd>(
    "motion_servo_cmd", rclcpp::SystemDefaultsQoS());

  motion_ressult_client_ = this->create_client<protocol::srv::MotionResultCmd>(
    "motion_result_cmd", rmw_qos_profile_services_default, callback_group_);

  //  code for visual program
  visual_response_sub_ = this->create_subscription<std_msgs::msg::String>(
    "robotend_message", rclcpp::ParametersQoS(),
    std::bind(&Cyberdog_app::backend_message_callback, this, _1));

  visual_request_pub_ = this->create_publisher<std_msgs::msg::String>(
    "frontend_message", rclcpp::ParametersQoS());

  // code for ai
  ai_face_entry_client_ =
    this->create_client<protocol::srv::FaceEntry>("cyberdog_face_entry_srv");
  ai_face_recognition_client_ =
    this->create_client<protocol::srv::FaceRec>("cyberdog_face_recognition_srv");
  ai_face_entry_sub_ =
    this->create_subscription<protocol::msg::FaceEntryResult>(
    "face_entry_msg", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::face_entry_result_callback, this, _1));
  ai_face_recognition_sub_ =
    this->create_subscription<protocol::msg::FaceRecognitionResult>(
    "face_rec_msg", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::face_rec_result_callback, this, _1));

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
  namecode_queue_size_[::grpcapi::SendRequest::IMAGE_TRANSMISSION_REQUEST] = 20;

  // photo and video recording
  camera_service_client_ = this->create_client<protocol::srv::CameraService>(
    "camera_service");
  autosaved_file_sub_ = this->create_subscription<std_msgs::msg::String>(
    "autosaved_video_file", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::autoSavedFileCB, this, _1));

  // ota
  download_subscriber_ = this->create_subscription<protocol::msg::OtaUpdate>(
    "ota_download_percentage", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::HandleDownloadPercentageMsgs, this, _1));

  upgrade_subscriber_ = this->create_subscription<protocol::msg::OtaUpdate>(
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

  // account member add
  query_account_add_client_ =
    this->create_client<protocol::srv::AccountAdd>("account_add");

  // account member search
  query_account_search_client_ =
    this->create_client<protocol::srv::AccountSearch>("account_search");

  // account member delete
  query_account_delete_client_ =
    this->create_client<protocol::srv::AccountDelete>("account_delete");

  // account member change
  query_account_change_client_ =
    this->create_client<protocol::srv::AccountChange>("account_change");

  set_work_environment_client_ =
    this->create_client<protocol::srv::Trigger>("set_nx_environment");

  upload_syslog_client_ =
    this->create_client<std_srvs::srv::Trigger>("upload_syslog");

  // test
  app_disconnect_pub_ = this->create_publisher<std_msgs::msg::Bool>("disconnect_app", 2);

  // map subscribe
  path_map_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions map_path_sub_option;
  map_path_sub_option.callback_group = path_map_callback_group_;
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&Cyberdog_app::processMapMsg, this, _1),
    map_path_sub_option);
  namecode_queue_size_[::grpcapi::SendRequest::MAP_DATA_REQUEST] = 2;

  // dog pose
  dog_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "dog_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::processDogPose, this, _1));
  namecode_queue_size_[::grpcapi::SendRequest::MAP_DOG_POSE_REQUEST] = 2;

  // mapping and navigation
  set_label_client_ = this->create_client<protocol::srv::SetMapLabel>(
    "set_label", rmw_qos_profile_services_default, callback_group_);

  get_label_client_ = this->create_client<protocol::srv::GetMapLabel>(
    "get_label", rmw_qos_profile_services_default, callback_group_);

  navigation_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  navigation_client_ =
    rclcpp_action::create_client<Navigation>(this, "start_algo_task", navigation_callback_group_);

  nav_path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "plan", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::uploadNavPath, this, _1),
    map_path_sub_option);
  namecode_queue_size_[::grpcapi::SendRequest::NAV_PLAN_PATH] = 2;

  stop_nav_action_client_ = this->create_client<protocol::srv::StopAlgoTask>(
    "stop_algo_task", rmw_qos_profile_services_default, callback_group_);

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 1, std::bind(&Cyberdog_app::laserScanCB, this, _1), map_path_sub_option);
  namecode_queue_size_[::grpcapi::SendRequest::LASER_SCAN] = 1;

  // tracking
  rclcpp::SensorDataQoS tracking_qos;
  tracking_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  tracking_person_sub_ = create_subscription<protocol::msg::Person>(
    "person", tracking_qos,
    std::bind(&Cyberdog_app::publishTrackingPersonCB, this, _1));
  namecode_queue_size_[::grpcapi::SendRequest::TRACKING_OBJ] = 2;
  select_tracking_human_client_ = this->create_client<protocol::srv::BodyRegion>(
    "tracking_object_srv", rmw_qos_profile_services_default, callback_group_);

  // bluetooth
  scan_bluetooth_devices_client_ =
    this->create_client<protocol::srv::BLEScan>("scan_bluetooth_devices");
  connect_bluetooth_device_client_ =
    this->create_client<protocol::srv::BLEConnect>("connect_bluetooth_device");
  disconnected_unexpected_sub_ = create_subscription<std_msgs::msg::Bool>(
    "bluetooth_disconnected_unexpected", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::disconnectedUnexpectedCB, this, _1));
  current_connected_bluetooth_client_ =
    this->create_client<protocol::srv::BLEScan>("get_connected_bluetooth_info");
  ble_battery_client_ =
    this->create_client<protocol::srv::GetBLEBatteryLevel>("ble_device_battery_level");
  ble_device_firmware_version_client_ =
    this->create_client<std_srvs::srv::Trigger>("ble_device_firmware_version");
  delete_ble_history_client_ =
    this->create_client<nav2_msgs::srv::SaveMap>("delete_ble_devices_history");
  ble_firmware_update_notification_sub_ = this->create_subscription<std_msgs::msg::String>(
    "ble_firmware_update_notification", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::bleFirmwareUpdateNotificationCB, this, _1));
  ble_dfu_progress_sub_ = this->create_subscription<protocol::msg::BLEDFUProgress>(
    "ble_dfu_progress", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::bleDFUProgressCB, this, _1));
  update_ble_firmware_client_ =
    this->create_client<std_srvs::srv::Trigger>("update_ble_firmware");
  set_bt_tread_pub_ =
    this->create_publisher<std_msgs::msg::Int8>(
    "set_bluetooth_tread", rclcpp::SystemDefaultsQoS());
  update_bt_tread_sub_ =
    this->create_subscription<std_msgs::msg::Int8>(
    "update_bluetooth_tread", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::updateBTTreadCB, this, _1));
  get_bt_tread_client_ = this->create_client<std_srvs::srv::Trigger>("get_bluetooth_tread");

  // status reporting
  motion_status_sub_ = this->create_subscription<protocol::msg::MotionStatus>(
    "motion_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::motionStatusCB, this, _1));
  task_status_sub_ = this->create_subscription<protocol::msg::AlgoTaskStatus>(
    "algo_task_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::taskStatusCB, this, _1));
  self_check_status_sub_ = this->create_subscription<protocol::msg::SelfCheckStatus>(
    "self_check_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::selfCheckStatusCB, this, _1));
  state_switch_status_sub_ = this->create_subscription<protocol::msg::StateSwitchStatus>(
    "state_switch_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::stateSwitchStatusCB, this, _1));
  bmd_status_sub_ = this->create_subscription<protocol::msg::BmsStatus>(
    "bms_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&Cyberdog_app::bmsStatusCB, this, _1));

  // low power dissipation
  low_power_exit_client_ = this->create_client<std_srvs::srv::Trigger>("low_power_exit");
  auto_low_power_enable_client_ = this->create_client<std_srvs::srv::SetBool>("low_power_onoff");

  // dog leg calibration
  dog_leg_calibration_client_ = this->create_client<std_srvs::srv::SetBool>(
    "dog_leg_calibration", rmw_qos_profile_services_default, callback_group_);

  // stair demo
  start_stair_align_client_ =
    this->create_client<std_srvs::srv::SetBool>("start_stair_align");
  stop_stair_align_client_ =
    this->create_client<std_srvs::srv::Trigger>("stop_stair_align");

  unlock_develop_access_client_ =
    this->create_client<protocol::srv::Unlock>("unlock_develop_access");

  reboot_machine_client_ =
    this->create_client<protocol::srv::RebootMachine>("reboot_machine");

  INFO("Create server");
  if (server_ == nullptr) {
    app_server_thread_ =
      std::make_shared<std::thread>(&Cyberdog_app::RunServer, this);
  }
  heart_beat_thread_ =
    std::make_shared<std::thread>(&Cyberdog_app::HeartBeat, this);
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
  const std::unique_ptr<::grpcapi::SendRequest> msg)
{
  std::shared_lock<std::shared_mutex> read_lock(stub_mutex_);
  if (can_process_messages_ && app_stub_ && connect_mark_) {
    bool send_result = app_stub_->sendRequest(*msg);
    if (!send_result) {
      ERROR("gRPC Msg sending error.");
    }
  }
}

void Cyberdog_app::HeartBeat()
{
  static std::chrono::system_clock::time_point heart_beat_time_point(
    std::chrono::system_clock::now());
  rclcpp::WallRate r(500ms);
  std::string ipv4;
  while (rclcpp::ok()) {
    if (can_process_messages_) {
      update_time_mutex_.lock();
      bool connector_timeout = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now() - connector_update_time_point_).count() > 2;
      update_time_mutex_.unlock();
      bool hearbeat_result(false);
      {
        std::shared_lock<std::shared_mutex> stub_read_lock(stub_mutex_);
        if (app_stub_ && !connector_timeout) {
          int motion_id, task_sub_status, self_check_code, state_switch_state, state_switch_code;
          uint8_t task_status;
          std::string description;
          bool wired_charging, wireless_charging;
          {
            std::shared_lock<std::shared_mutex> status_lock(status_mutex_);
            motion_id = motion_status_.motion_id;
            task_status = task_status_.task_status;
            task_sub_status = task_status_.task_sub_status;
            self_check_code = self_check_status_.code;
            description = self_check_status_.description;
            state_switch_state = state_switch_status_.state;
            state_switch_code = state_switch_status_.code;
            wired_charging = charging_status_.wired_charging;
            wireless_charging = charging_status_.wireless_charging;
          }
          std::shared_lock<std::shared_mutex> connector_read_lock(connector_mutex_);
          hearbeat_result =
            app_stub_->sendHeartBeat(
            local_ip, wifi_strength, bms_status.batt_soc, is_internet, sn,
            motion_id, task_status, task_sub_status, self_check_code, description,
            state_switch_state, state_switch_code, wired_charging, wireless_charging);
        } else if (connector_timeout) {
          WARN_MILLSECONDS(2000, "connector_state topic timeout");
          hearbeat_result = false;
          app_disconnected = true;
        } else {
          r.sleep();
          continue;
        }
      }
      if (!hearbeat_result) {
        bool hear_beat_timeout = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now() - heart_beat_time_point).count() >= 5;
        if (hear_beat_timeout) {
          heart_beat_time_point = std::chrono::system_clock::now();
          connect_mark_ = false;
          std_msgs::msg::Bool msg;
          msg.data = false;
          app_connection_pub_->publish(msg);
          disconnectTaskRequest();
          if (!app_disconnected) {
            destroyGrpc();
            createGrpc();
          }
        }
      } else {
        if (!connect_mark_) {
          connect_mark_ = true;
          publishNotCompleteSendingFiles();
        }
        heart_beat_time_point = std::chrono::system_clock::now();
        std_msgs::msg::Bool msg;
        msg.data = true;
        app_connection_pub_->publish(msg);
      }
    } else {
      WARN_MILLSECONDS(2000, "Not able to send heatbeat while creating gRPC client.");
    }
    r.sleep();
  }
  INFO("Exiting HeartBeat thread...");
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

void Cyberdog_app::subscribeConnectStatus(const protocol::msg::ConnectorStatus::SharedPtr msg)
{
  if (msg->is_connected) {
    update_time_mutex_.lock();
    connector_update_time_point_ = std::chrono::system_clock::now();
    update_time_mutex_.unlock();
    std::string phoneIp = msg->provider_ip;
    if (!phoneIp.empty() && phoneIp != "0.0.0.0") {
      app_disconnected = false;
    } else {
      app_disconnected = true;
    }
    bool different_ip(false);
    {
      std::unique_lock<std::shared_mutex> write_lock(connector_mutex_);
      local_ip = msg->robot_ip;
      different_ip = *server_ip != phoneIp;
      if (different_ip) {
        server_ip = std::make_shared<std::string>(phoneIp);
      }
      is_internet = msg->is_internet;
      wifi_strength = msg->strength;
    }
    INFO_MILLSECONDS(
      2000, "Wifi ssid: %s. signal strength: %d.", msg->ssid.c_str(), msg->strength);
    if (different_ip) {
      INFO("local_ip ip :%s,pheneIp ip :%s", msg->robot_ip.c_str(), msg->provider_ip.c_str());
      destroyGrpc();
      createGrpc();
    }
  } else {
    INFO_MILLSECONDS(10000, "Wifi is not connected");
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
  INFO("Try to reset app_stub_ if it is NULL");
  if (app_stub_) {
    app_stub_.reset();
  }
  net_checker.pause();
}

void Cyberdog_app::createGrpc()
{
  INFO("Create server");
  if (!server_) {
    app_server_thread_ =
      std::make_shared<std::thread>(&Cyberdog_app::RunServer, this);
  }
  INFO("Create client");
  grpc::string ip_port;
  {
    std::shared_lock<std::shared_mutex> read_lock(connector_mutex_);
    ip_port = *server_ip + std::string(":") + grpc_client_port_;
    INFO("Client ip port: %s", ip_port.c_str());
    net_checker.set_ip(*server_ip);
  }
  auto channel_ = grpc::CreateChannel(ip_port, grpc::InsecureChannelCredentials());
  std::unique_lock<std::shared_mutex> write_lock(stub_mutex_);
  app_stub_ = std::make_shared<Cyberdog_App_Client>(channel_);
  can_process_messages_ = true;
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

bool Cyberdog_app::callMotionServoCmd(
  const std::shared_ptr<protocol::srv::MotionResultCmd::Request> req,
  protocol::srv::MotionResultCmd::Response & rsp)
{
  INFO("callMotionServoCmd.");
  std::chrono::seconds timeout(20);

  if (!motion_ressult_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("callMotionServoCmd server not available");
    return false;
  }

  INFO("motion_id: %d.", req->motion_id);
  auto future_result = motion_ressult_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);

  if (status == std::future_status::ready) {
    INFO("success to call callMotionServoCmd services.");
  } else {
    ERROR("Failed to call callMotionServoCmd services.");
    return false;
  }

  rsp.motion_id = future_result.get()->motion_id;
  rsp.result = future_result.get()->result;
  rsp.code = future_result.get()->code;
  return true;
}

//  for visual
void Cyberdog_app::backend_message_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  send_grpc_msg(::grpcapi::SendRequest::VISUAL_BACKEND_MSG, msg->data);
}

// for ai
void Cyberdog_app::face_entry_result_callback(
  const protocol::msg::FaceEntryResult::SharedPtr msg)
{
  INFO("grpc success to get face entry result.");
  Document json_response(kObjectType);
  CyberdogJson::Add(json_response, "result", msg->result);
  CyberdogJson::Add(json_response, "username", msg->username);
  send_grpc_msg(::grpcapi::SendRequest::FACE_ENTRY_RESULT_PUBLISH, json_response);
  INFO("grpc success to send face entry result.");
}
void Cyberdog_app::face_rec_result_callback(
  const protocol::msg::FaceRecognitionResult::SharedPtr msg)
{
  INFO("grpc success to get face recognition result.");
  Document json_response(kObjectType);
  CyberdogJson::Add(json_response, "result", msg->result);
  CyberdogJson::Add(json_response, "username", msg->username);
  send_grpc_msg(::grpcapi::SendRequest::FACE_RECOGNITION_RESULT_PUBLISH, json_response);
  INFO("grpc success to send face recognition result.");
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
  INFO("prepare to upload map data, size: %ld", msg->data.size());
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
    returnErrorGrpc(writer, 323, namecode);
    return false;
  }
  ::grpcapi::RecResponse grpc_response;
  grpc_response.set_namecode(namecode);
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
  return true;
}

//  commcon code

void Cyberdog_app::send_grpc_msg(uint32_t code, const Document & doc)
{
  std::string rsp_string;
  if (!CyberdogJson::Document2String(doc, rsp_string)) {
    ERROR("error while encoding to json");
    return;
  }
  send_grpc_msg(code, rsp_string);
}

void Cyberdog_app::send_grpc_msg(uint32_t code, const std::string & msg)
{
  ::grpcapi::SendRequest * grpc_respons = new ::grpcapi::SendRequest();
  grpc_respons->set_namecode(code);
  grpc_respons->set_params(msg);
  {
    std::shared_lock<std::shared_mutex> read_lock(send_thread_map_mx_);
    auto sender = send_thread_map_.find(code);
    if (sender != send_thread_map_.end()) {
      INFO("found sender for %d", code);
      sender->second->push(std::move(std::unique_ptr<::grpcapi::SendRequest>(grpc_respons)));
      return;
    }
  }
  std::unique_lock<std::shared_mutex> write_lock(send_thread_map_mx_);
  auto sender = send_thread_map_.find(code);
  if (sender != send_thread_map_.end()) {
    INFO("found sender for %d", code);
    sender->second->push(std::move(std::unique_ptr<::grpcapi::SendRequest>(grpc_respons)));
  } else {
    INFO("create sender for %d", code);
    if (namecode_queue_size_.find(code) != namecode_queue_size_.end()) {
      send_thread_map_[code] =
        std::make_shared<LatestMsgDispather<std::unique_ptr<::grpcapi::SendRequest>>>(
        namecode_queue_size_[code]);
    } else {
      send_thread_map_[code] =
        std::make_shared<LatestMsgDispather<std::unique_ptr<::grpcapi::SendRequest>>>();
    }
    send_thread_map_[code]->setCallback(std::bind(&Cyberdog_app::send_msgs_, this, _1));
    send_thread_map_[code]->push(
      std::move(std::unique_ptr<::grpcapi::SendRequest>(grpc_respons)));
  }
}

//  message pump
void Cyberdog_app::returnErrorGrpc(
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer,
  int error_code, uint32_t namecode)
{
  ::grpcapi::RecResponse grpc_response;
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> json_writer(strBuf);
  json_writer.StartObject();
  json_writer.Key("error_code");
  json_writer.Int(error_code);
  json_writer.EndObject();
  std::string response_string = strBuf.GetString();
  grpc_response.set_data(response_string);
  grpc_response.set_namecode(namecode);
  grpc_writer->Write(grpc_response);
}

bool Cyberdog_app::HandleOTAStatusRequest(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    ERROR("ota server not available");
    return false;
  }

  std::string json_result;
  CyberdogJson::Document2String(json_request, json_result);
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
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return false;
  }

  CyberdogJson::Get(json_response, "status", response_string);
  grpc_response.set_namecode(::grpcapi::SendRequest::OTA_STATUS_REQUEST);
  grpc_response.set_data(response_string);
  writer->Write(grpc_response);

  return true;
}

void Cyberdog_app::HandleDownloadPercentageMsgs(const protocol::msg::OtaUpdate msg)
{
  Document progress_response(kObjectType);
  std::string response_string;

  CyberdogJson::Add(progress_response, "upgrade_progress", 0);
  CyberdogJson::Add(progress_response, "download_progress", msg.progress);
  CyberdogJson::Add(progress_response, "code", msg.code);
  CyberdogJson::Document2String(progress_response, response_string);

  // INFO("upgrade_progress: %d", 0);
  INFO("download response: %s", response_string.c_str());

  send_grpc_msg(::grpcapi::SendRequest::OTA_PROCESS_QUERY_REQUEST, response_string);
}

void Cyberdog_app::HandleUpgradePercentageMsgs(const protocol::msg::OtaUpdate msg)
{
  Document progress_response(kObjectType);
  std::string response_string;

  CyberdogJson::Add(progress_response, "download_progress", 0);
  CyberdogJson::Add(progress_response, "upgrade_progress", msg.progress);
  CyberdogJson::Add(progress_response, "code", msg.code);
  CyberdogJson::Document2String(progress_response, response_string);

  INFO("upgrade response: %s", response_string.c_str());
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
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  INFO("GRPC version query.");
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    ERROR("ota server not available");
    return false;
  }

  std::string json_result;
  CyberdogJson::Document2String(json_request, json_result);
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
    returnErrorGrpc(writer, 323, ::grpcapi::SendRequest::OTA_VERSION_QUERY_REQUEST);
    return false;
  }

  CyberdogJson::Get(json_response, "version", response_string);
  grpc_response.set_namecode(::grpcapi::SendRequest::OTA_VERSION_QUERY_REQUEST);
  grpc_response.set_data(response_string);
  writer->Write(grpc_response);
  return true;
}

bool Cyberdog_app::HandleOTAStartDownloadRequest(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    ERROR("ota server not available");
    return false;
  }

  std::string json_result;
  CyberdogJson::Document2String(json_request, json_result);

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

  grpc_response.set_namecode(::grpcapi::SendRequest::OTA_START_DOWNLOAD_REQUEST);
  grpc_response.set_data(res.get()->response.value);
  writer->Write(grpc_response);

  return true;
}

bool Cyberdog_app::HandleOTAStartUpgradeRequest(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    ERROR("ota server not available");
    return false;
  }

  std::string json_result;
  CyberdogJson::Document2String(json_request, json_result);
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

  grpc_response.set_namecode(::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST);
  grpc_response.set_data(res.get()->response.value);
  writer->Write(grpc_response);

  return true;
}

bool Cyberdog_app::HandleOTAProcessQueryRequest(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    ERROR("ota server not available");
    return false;
  }

  std::string json_result;
  CyberdogJson::Document2String(json_request, json_result);
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
    returnErrorGrpc(writer, 323, ::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST);
    return false;
  }

  CyberdogJson::Get(json_response, "progress", response_string);
  grpc_response.set_namecode(::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST);
  grpc_response.set_data(response_string);
  writer->Write(grpc_response);

  return true;
}

bool Cyberdog_app::HandleOTAEstimateUpgradeTimeRequest(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string response_string;
  std::chrono::seconds timeout(10);
  if (!ota_client_->wait_for_service(timeout)) {
    ERROR("ota server not available");
    return false;
  }

  std::string json_result;
  CyberdogJson::Document2String(json_request, json_result);
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
    returnErrorGrpc(writer, 323, ::grpcapi::SendRequest::OTA_ESTIMATE_UPGRADE_TIME_REQUEST);
    return false;
  }

  if (!CyberdogJson::Document2String(json_response, response_string)) {
    ERROR("error while encoding authenticate response to json");
    returnErrorGrpc(writer, 323, ::grpcapi::SendRequest::OTA_ESTIMATE_UPGRADE_TIME_REQUEST);
    return false;
  }

  grpc_response.set_namecode(::grpcapi::SendRequest::OTA_ESTIMATE_UPGRADE_TIME_REQUEST);
  grpc_response.set_data(response_string);
  writer->Write(grpc_response);
  return true;
}

void Cyberdog_app::handleNavigationAction(
  const Document & json_request, ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer,
  bool create_new_task)
{
  INFO("handleNavigationAction");
  int nav_timeout = 7200;
  std::string response_string;
  uint32_t type;
  geometry_msgs::msg::PoseStamped goal;
  double goal_x, goal_y;
  double yaw;
  bool outdoor(false);
  double keep_distance;
  bool object_tracking(false);
  CyberdogJson::Get(json_request, "type", type);
  CyberdogJson::Get(json_request, "outdoor", outdoor);
  CyberdogJson::Get(json_request, "goalX", goal_x);
  CyberdogJson::Get(json_request, "goalY", goal_y);
  if (json_request.HasMember("theta")) {
    CyberdogJson::Get(json_request, "theta", yaw);
    goal.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
  } else {
    goal.pose.orientation.w = 0;  // if goal without theta, then set quaternion illegal
  }
  uint32_t relative_pos = 0;
  CyberdogJson::Get(json_request, "relative_pos", relative_pos);
  CyberdogJson::Get(json_request, "keep_distance", keep_distance);
  CyberdogJson::Get(json_request, "object_tracking", object_tracking);

  auto mode_goal = Navigation::Goal();
  mode_goal.nav_type = type;
  goal.pose.position.x = goal_x;
  goal.pose.position.y = goal_y;
  mode_goal.poses.push_back(goal);
  mode_goal.outdoor = outdoor;
  mode_goal.relative_pos = relative_pos;
  mode_goal.keep_distance = keep_distance;
  mode_goal.object_tracking = object_tracking;

  auto return_result = [&](int result_code) {
      rapidjson::StringBuffer strBuf;
      rapidjson::Writer<rapidjson::StringBuffer> json_writer(strBuf);
      json_writer.StartObject();
      json_writer.Key("result");
      json_writer.Int(result_code);
      json_writer.EndObject();
      response_string = strBuf.GetString();
      grpc_response.set_data(response_string);
      writer->Write(grpc_response);
      INFO_STREAM("transmit result: " << response_string);
    };

  auto return_feedback = [&](int feedback_code, const std::string & feedback_msg) {
      rapidjson::StringBuffer strBuf;
      rapidjson::Writer<rapidjson::StringBuffer> json_writer(strBuf);
      json_writer.StartObject();
      json_writer.Key("feedback_code");
      json_writer.Int(feedback_code);
      json_writer.Key("feedback_msg");
      json_writer.String(feedback_msg.c_str());
      json_writer.EndObject();
      response_string = strBuf.GetString();
      grpc_response.set_data(response_string);
      writer->Write(grpc_response);
      INFO_STREAM("transmit feedback: " << response_string);
    };

  auto return_accept = [&](int accepted) {
      rapidjson::StringBuffer strBuf;
      rapidjson::Writer<rapidjson::StringBuffer> json_writer(strBuf);
      json_writer.StartObject();
      json_writer.Key("accepted");
      json_writer.Int(accepted);
      json_writer.EndObject();
      response_string = strBuf.GetString();
      grpc_response.set_data(response_string);
      writer->Write(grpc_response);
      INFO_STREAM("transmit acception: " << response_string);
    };

  std::mutex writer_mutex;
  auto feedback_callback =
    [&](rclcpp_action::Client<Navigation>::GoalHandle::SharedPtr goal_handel_ptr,
      const std::shared_ptr<const Navigation::Feedback> feedback) {
      writer_mutex.lock();
      return_feedback(feedback->feedback_code, feedback->feedback_msg);
      writer_mutex.unlock();
    };

  auto fake_feedback_callback =
    [&](int feedback_code) {
      writer_mutex.lock();
      return_feedback(feedback_code, std::string(""));
      writer_mutex.unlock();
    };

  std::shared_ptr<std::condition_variable> result_cv_ptr;
  std::shared_ptr<std::shared_ptr<Navigation::Result>> result_pp;
  std::shared_ptr<std::mutex> result_mx;
  bool uwb_not_from_app = false;  // activated uwb tracking not from app
  size_t goal_hash;
  {
    std::unique_lock<std::mutex> init_lock(task_init_mutex_);
    if (create_new_task) {
      INFO(
        "Sending action goal: type:%d, outdoor:%d, reletive_pos:%d, keep_dis:%f, obj_tracking:%d",
        mode_goal.nav_type, mode_goal.outdoor, mode_goal.relative_pos, mode_goal.keep_distance,
        mode_goal.object_tracking);
      bool acception = action_task_manager_.StartActionTask<Navigation>(
        navigation_client_, mode_goal, feedback_callback,
        result_cv_ptr, result_pp, result_mx, goal_hash);
      if (!acception) {
        WARN("Navigation action request rejected");
        return_accept(2);
        return;
      }
      type_hash_mutex_.lock();
      task_type_hash_map_[mode_goal.nav_type] = goal_hash;
      type_hash_mutex_.unlock();
    } else {  // access a task
      std::shared_lock<std::shared_mutex> read_lock(type_hash_mutex_);
      if (task_type_hash_map_.find(mode_goal.nav_type) == task_type_hash_map_.end()) {
        if (mode_goal.nav_type != Navigation::Goal::NAVIGATION_TYPE_START_UWB_TRACKING) {
          return_accept(3);  // no task of that type recorded
          return;
        }
        uwb_not_from_app = true;
      }
      bool accepted = false;
      if (!uwb_not_from_app) {
        goal_hash = task_type_hash_map_[mode_goal.nav_type];
        accepted = action_task_manager_.AccessTask<Navigation>(
          goal_hash, feedback_callback, result_cv_ptr, result_pp, result_mx);
      } else {
        if (task_status_.task_status == Navigation::Goal::NAVIGATION_TYPE_START_UWB_TRACKING) {
          accepted = true;
        } else {
          accepted = false;
        }
      }
      if (!accepted) {
        return_accept(3);  // the task has already finished
        return;
      }
    }
    return_accept(1);
    if (!create_new_task && uwb_not_from_app) {
      fake_feedback_callback(task_status_.task_sub_status);
    } else if (!create_new_task) {  // access task
      action_task_manager_.CallLatestFeedback(goal_hash);
    } else {  // create task
      action_task_manager_.CallFeedbackBeforeAcception(goal_hash);
    }
  }

  if (uwb_not_from_app) {
    rclcpp::WallRate rate(500ms);
    int latest_sub_status = -1;
    while (rclcpp::ok() &&
      task_status_.task_status != Navigation::Goal::NAVIGATION_TYPE_START_UWB_TRACKING &&
      connect_mark_)
    {
      rate.sleep();
      if (latest_sub_status != task_status_.task_sub_status) {
        fake_feedback_callback(task_status_.task_sub_status);
        latest_sub_status = task_status_.task_sub_status;
      }
    }
    return_result(Navigation::Result::NAVIGATION_RESULT_TYPE_FAILED);
    return;
  }

  uint8_t result = 2;
  bool result_timeout = false;
  auto result_is_ready =
    [&]() {
      return result_pp && *result_pp;
    };
  std::unique_lock<std::mutex> result_lock(*result_mx);
  bool is_not_timeout = result_cv_ptr->wait_for(
    result_lock, std::chrono::seconds(nav_timeout), result_is_ready);
  if (!is_not_timeout) {
    WARN("Navigation action result timeout");
    result_timeout = true;
    writer_mutex.lock();
    return_result(99);  // timeout code
    writer_mutex.unlock();
  } else {
    result = (*result_pp)->result;
    if (result == protocol::action::Navigation::Result::NAVIGATION_RESULT_TYPE_SUCCESS) {
      INFO("Navigation action task succeeded");
    } else {
      WARN("Navigation action task failed");
    }
    writer_mutex.lock();
    return_result(result);
    writer_mutex.unlock();
  }
  if (connect_mark_) {
    std::unique_lock<std::shared_mutex> write_lock(type_hash_mutex_);
    if (task_type_hash_map_.find(mode_goal.nav_type) != task_type_hash_map_.end() &&
      goal_hash == task_type_hash_map_[mode_goal.nav_type])
    {
      task_type_hash_map_.erase(mode_goal.nav_type);
      INFO("Erase finished task type %d", mode_goal.nav_type);
    }
  }
}

void Cyberdog_app::disconnectTaskRequest()
{
  std::unique_lock<std::shared_mutex> write_lock(type_hash_mutex_);
  if (task_type_hash_map_.empty()) {
    return;
  }
  std::list<size_t> remove_hash_list;
  for (auto & type_hash : task_type_hash_map_) {
    if (!action_task_manager_.RemoveRequest(type_hash.second)) {
      remove_hash_list.push_back(type_hash.first);
    }
  }
  if (remove_hash_list.empty()) {
    return;
  }
  for (auto type : remove_hash_list) {
    task_type_hash_map_.erase(type);
    INFO("Erase unused task type %d", type);
  }
}

void Cyberdog_app::handlLableGetRequest(
  const Document & json_request, ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!get_label_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("get map label server not available");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto request = std::make_shared<protocol::srv::GetMapLabel::Request>();
  CyberdogJson::Get(json_request, "mapName", request->map_name);
  CyberdogJson::Get(json_request, "map_id", request->map_id);
  std::chrono::seconds timeout(10);

  auto future_result = get_label_client_->async_send_request(request);
  std::future_status status = future_result.wait_for(timeout);

  if (status == std::future_status::ready) {
    if (future_result.get()->success ==
      protocol::srv::GetMapLabel_Response::RESULT_SUCCESS)
    {
      INFO("get_label services succeeded.");
    } else {
      WARN("get_label services failed.");
    }
    protocol::msg::MapLabel labels = future_result.get()->label;
    grpc_response.set_namecode(::grpcapi::SendRequest::MAP_GET_LABLE_REQUEST);
    rapidjson::StringBuffer strBuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
    writer.StartObject();
    writer.Key("mapName");
    writer.String(labels.map_name.c_str());
    writer.Key("success");
    writer.Int(future_result.get()->success);
    writer.Key("is_outdoor");
    writer.Bool(labels.is_outdoor);
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
    grpc_response.set_data(data);
  } else {
    ERROR("Call get_map_label services timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
  }
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::handlLableSetRequest(
  const Document & json_request, ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  protocol::msg::MapLabel map_label;
  bool has_label = false;
  std::string response_string;
  bool only_delete = false;
  CyberdogJson::Get(json_request, "mapName", map_label.map_name);
  CyberdogJson::Get(json_request, "is_outdoor", map_label.is_outdoor);
  CyberdogJson::Get(json_request, "map_id", map_label.map_id);
  CyberdogJson::Get(json_request, "only_delete", only_delete);
  INFO("handlLableSetRequest %s, %d", map_label.map_name.c_str(), only_delete);
  if (json_request.HasMember("locationLabelInfo") &&
    json_request["locationLabelInfo"].IsArray())
  {
    auto labels = json_request["locationLabelInfo"].GetArray();
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

  auto request = std::make_shared<protocol::srv::SetMapLabel::Request>();
  request->label = map_label;
  request->only_delete = only_delete;

  if (!set_label_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("set map label server not available");
    returnErrorGrpc(writer, 322, grpc_response.namecode());
    return;
  }
  std::chrono::seconds timeout(10);
  auto future_result = set_label_client_->async_send_request(request);
  std::future_status status = future_result.wait_for(timeout);
  Document json_response(kObjectType);
  if (status == std::future_status::ready) {
    if (future_result.get()->success ==
      protocol::srv::SetMapLabel_Response::RESULT_SUCCESS)
    {
      INFO("set_label services succeeded.");
    } else {
      WARN("set_label services failed.");
    }
    CyberdogJson::Add(json_response, "success", static_cast<int>(future_result.get()->success));
  } else {
    ERROR("calling set_label services timeout.");
    returnErrorGrpc(writer, 321, grpc_response.namecode());
    return;
  }
  if (!CyberdogJson::Document2String(json_response, response_string)) {
    ERROR("error while encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_namecode(::grpcapi::SendRequest::MAP_SET_LABLE_REQUEST);
  grpc_response.set_data(response_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::uploadNavPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  writer.StartObject();
  writer.Key("path_point");
  writer.StartArray();
  for (auto & pose_stamp : msg->poses) {
    writer.StartObject();
    writer.Key("px");
    writer.Double(pose_stamp.pose.position.x);
    writer.Key("py");
    writer.Double(pose_stamp.pose.position.y);
    writer.EndObject();
  }
  writer.EndArray();
  writer.EndObject();
  std::string param = strBuf.GetString();
  INFO("prepare to send navigation global plan, size: %ld", msg->poses.size());
  send_grpc_msg(::grpcapi::SendRequest::NAV_PLAN_PATH, param);
}

void Cyberdog_app::laserScanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped T_map_laser_msg;
  try {
    T_map_laser_msg = tf_buffer_->lookupTransform(
      "map", "laser_frame", tf2::TimePointZero);
    if (!tf_buffer_->canTransform(
        "map", "laser_frame", tf2::get_now(), tf2::durationFromSec(1)))
    {
      return;
    }
  } catch (const tf2::LookupException & ex) {
    WARN_MILLSECONDS(10000, "Map frame doesn't exist");
    return;
  } catch (const tf2::TransformException & ex) {
    WARN_MILLSECONDS(10000, "Could not transform from map to laser_frame");
    return;
  }
  tf2::Transform T_map_laser;
  tf2::fromMsg(T_map_laser_msg.transform, T_map_laser);
  int ranges_size = msg->ranges.size();
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  writer.StartObject();
  writer.Key("laser_point");
  writer.StartArray();
  for (int i = 0; i < ranges_size; ++i) {
    if (msg->ranges[i] < 0.05f) {
      continue;
    }
    float angle = msg->angle_min + i * msg->angle_increment;
    tf2::Vector3 map_frame_point = T_map_laser * tf2::Vector3(
      msg->ranges[i] * std::cos(angle), msg->ranges[i] * std::sin(angle), 0.0);
    writer.StartObject();
    writer.Key("px");
    writer.Double(map_frame_point.x());
    writer.Key("py");
    writer.Double(map_frame_point.y());
    writer.EndObject();
  }
  writer.EndArray();
  writer.EndObject();
  std::string param = strBuf.GetString();
  INFO("prepare to send laser points, size: %ld", msg->ranges.size());
  send_grpc_msg(::grpcapi::SendRequest::LASER_SCAN, param);
}

void Cyberdog_app::publishTrackingPersonCB(const protocol::msg::Person::SharedPtr msg)
{
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  writer.StartObject();
  writer.Key("body_info_roi");
  writer.StartArray();
  for (auto & info : msg->body_info.infos) {
    writer.StartObject();
    writer.Key("roi");
    writer.StartObject();
    writer.Key("x_offset");
    writer.Int(info.roi.x_offset);
    writer.Key("y_offset");
    writer.Int(info.roi.y_offset);
    writer.Key("height");
    writer.Int(info.roi.height);
    writer.Key("width");
    writer.Int(info.roi.width);
    writer.EndObject();
    writer.Key("reid");
    writer.String(info.reid.c_str());
    writer.EndObject();
  }
  writer.EndArray();
  writer.Key("track_res_roi");
  writer.StartObject();
  writer.Key("x_offset");
  writer.Int(msg->track_res.roi.x_offset);
  writer.Key("y_offset");
  writer.Int(msg->track_res.roi.y_offset);
  writer.Key("height");
  writer.Int(msg->track_res.roi.height);
  writer.Key("width");
  writer.Int(msg->track_res.roi.width);
  writer.EndObject();
  writer.EndObject();
  std::string param = strBuf.GetString();
  send_grpc_msg(::grpcapi::SendRequest::TRACKING_OBJ, param);
}

void Cyberdog_app::selectTrackingObject(
  Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!select_tracking_human_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("tracking_object_srv server is not available");
    returnErrorGrpc(writer, 322, grpc_response.namecode());
    return;
  }
  rapidjson::Value roi;
  CyberdogJson::Get(json_request, "roi", roi);
  if (!roi.IsObject() || !roi.HasMember("x_offset") || !roi.HasMember("y_offset") ||
    !roi.HasMember("height") || !roi.HasMember("width"))
  {
    ERROR("format of tracking object json is not correct.");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<protocol::srv::BodyRegion::Request>();
  req->roi.x_offset = roi["x_offset"].GetInt();
  req->roi.y_offset = roi["y_offset"].GetInt();
  req->roi.height = roi["height"].GetInt();
  req->roi.width = roi["width"].GetInt();
  std::chrono::seconds timeout(30);
  auto future_result = select_tracking_human_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    bool suc = future_result.get()->success;
    INFO("Got tracking_object_srv result: %d.", suc);
    CyberdogJson::Add(json_response, "success", suc);
  } else {
    ERROR("call tracking_object_srv timeout.");
    returnErrorGrpc(writer, 321, grpc_response.namecode());
    return;
  }
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set tracking_object_srv response encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::handleStopAction(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!stop_nav_action_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("stop_algo_task server is not available");
    returnErrorGrpc(writer, 322, grpc_response.namecode());
    return;
  }
  uint32_t type(0);
  std::string map_name;
  CyberdogJson::Get(json_request, "type", type);
  CyberdogJson::Get(json_request, "map_name", map_name);
  auto request = std::make_shared<protocol::srv::StopAlgoTask::Request>();
  request->task_id = type;
  request->map_name = map_name;
  auto future_result = stop_nav_action_client_->async_send_request(request);
  std::chrono::seconds timeout(60);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    uint8_t stop_task_result = future_result.get()->result;
    INFO("Got stop_algo_task result: %d.", stop_task_result);
    CyberdogJson::Add(json_response, "result", stop_task_result);
  } else {
    ERROR("call stop_algo_task timeout.");
    CyberdogJson::Add(json_response, "result", 10);
  }
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set stop_algo_task response encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::scanBluetoothDevices(
  Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!scan_bluetooth_devices_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("scan_bluetooth_device server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  double scan_seconds;
  CyberdogJson::Get(json_request, "scan_seconds", scan_seconds);
  auto req = std::make_shared<protocol::srv::BLEScan::Request>();
  req->scan_seconds = scan_seconds;
  auto future_result = scan_bluetooth_devices_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(
    scan_seconds < 6.0 ? std::chrono::seconds(
      6) : std::chrono::seconds(int64_t(scan_seconds * 1.5)));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    auto devices = future_result.get()->device_info_list;
    writer.StartObject();
    writer.Key("device_info_list");
    writer.StartArray();
    for (auto & dev : devices) {
      writer.StartObject();
      writer.Key("mac");
      writer.String(dev.mac.c_str());
      writer.Key("name");
      writer.String(dev.name.c_str());
      writer.Key("addr_type");
      writer.String(dev.addr_type.c_str());
      writer.Key("device_type");
      writer.Int(dev.device_type);
      writer.Key("firmware_version");
      writer.String(dev.firmware_version.c_str());
      writer.Key("battery_level");
      writer.Double(static_cast<double>(dev.battery_level));
      writer.EndObject();
    }
    writer.EndArray();
    writer.EndObject();
  } else {
    ERROR("call scan_bluetooth_device timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::connectBluetoothDevice(
  Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!connect_bluetooth_device_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("connect_bluetooth_device server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<protocol::srv::BLEConnect::Request>();
  if (json_request.HasMember("selected_device")) {
    rapidjson::Value selected_device;
    CyberdogJson::Get(json_request, "selected_device", selected_device);
    if (!selected_device.IsObject() || !selected_device.HasMember("mac") ||
      !selected_device.HasMember("addr_type") || !selected_device.HasMember("name"))
    {
      ERROR("format of connect bluetooth json is not correct.");
      returnErrorGrpc(grpc_writer, 323, grpc_response.namecode());
      return;
    }
    CyberdogJson::Get(selected_device, "mac", req->selected_device.mac);
    CyberdogJson::Get(selected_device, "name", req->selected_device.name);
    CyberdogJson::Get(selected_device, "addr_type", req->selected_device.addr_type);
  }
  auto future_result = connect_bluetooth_device_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(18));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    int connection_result = future_result.get()->result;
    INFO("bluetooth connection response: %d", connection_result);
    CyberdogJson::Add(json_response, "result", future_result.get()->result);
  } else {
    ERROR("call connect_bluetooth_device timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
  }
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set connect_bluetooth_device response encoding to json");
    returnErrorGrpc(grpc_writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(rsp_string);
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::disconnectedUnexpectedCB(const std_msgs::msg::Bool::SharedPtr msg)
{
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  writer.StartObject();
  writer.Key("disconnected");
  writer.Bool(msg->data);
  writer.EndObject();
  std::string param = strBuf.GetString();
  send_grpc_msg(::grpcapi::SendRequest::BLUETOOTH_DISCONNECTED_UNEXPECTED, param);
}

void Cyberdog_app::currentConnectedBluetoothDevices(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!current_connected_bluetooth_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("current_connected_bluetooth_devices server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<protocol::srv::BLEScan::Request>();
  auto future_result = current_connected_bluetooth_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    auto devices = future_result.get()->device_info_list;
    writer.StartObject();
    writer.Key("device_info_list");
    writer.StartArray();
    for (auto & dev : devices) {
      writer.StartObject();
      writer.Key("mac");
      writer.String(dev.mac.c_str());
      writer.Key("name");
      writer.String(dev.name.c_str());
      writer.Key("addr_type");
      writer.String(dev.addr_type.c_str());
      writer.Key("device_type");
      writer.Int(dev.device_type);
      writer.Key("firmware_version");
      writer.String(dev.firmware_version.c_str());
      writer.Key("battery_level");
      writer.Double(static_cast<double>(dev.battery_level));
      writer.EndObject();
    }
    writer.EndArray();
    writer.EndObject();
  } else {
    ERROR("call current_connected_bluetooth_devices timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::getBLEBatteryLevelHandle(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!ble_battery_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("ble_device_battery_level server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<protocol::srv::GetBLEBatteryLevel::Request>();
  auto future_result = ble_battery_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    bool connected = future_result.get()->connected;
    float persentage = future_result.get()->persentage;
    writer.StartObject();
    writer.Key("connected");
    writer.Bool(connected);
    writer.Key("persentage");
    writer.Bool(persentage);
    writer.EndObject();
  } else {
    ERROR("call ble_device_battery_level timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::getBLEFirmwareVersionHandle(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!ble_device_firmware_version_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("ble_device_firmware_version server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = ble_device_firmware_version_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    writer.StartObject();
    writer.Key("success");
    writer.Bool(future_result.get()->success);
    writer.Key("message");
    writer.String(future_result.get()->message.c_str());
    writer.EndObject();
  } else {
    ERROR("call ble_device_firmware_version timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::deleteBLEHistoryHandle(
  Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!delete_ble_history_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("delete_ble_devices_history server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  CyberdogJson::Get(json_request, "mac", req->map_url);
  auto future_result = delete_ble_history_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    bool success = future_result.get()->result;
    writer.StartObject();
    writer.Key("success");
    writer.Bool(success);
    writer.EndObject();
  } else {
    ERROR("call delete_ble_devices_history timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::bleFirmwareUpdateNotificationCB(const std_msgs::msg::String::SharedPtr msg)
{
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  writer.StartObject();
  writer.Key("data");
  writer.String(msg->data.c_str());
  writer.EndObject();
  std::string param = strBuf.GetString();
  send_grpc_msg(::grpcapi::SendRequest::BLE_FIRMWARE_UPDATE_NOTIFICATION, param);
}

void Cyberdog_app::updateBLEFirmwareHandle(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!update_ble_firmware_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("update_ble_firmware server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = update_ble_firmware_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(10));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    writer.StartObject();
    writer.Key("success");
    writer.Bool(future_result.get()->success);
    writer.Key("message");
    writer.String(future_result.get()->message.c_str());
    writer.EndObject();
  } else {
    ERROR("call update_ble_firmware timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::bleDFUProgressCB(const protocol::msg::BLEDFUProgress::SharedPtr msg)
{
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  writer.StartObject();
  writer.Key("status");
  writer.Int(msg->status);
  writer.Key("progress");
  writer.Double(msg->progress);
  writer.Key("message");
  writer.String(msg->message.c_str());
  writer.EndObject();
  std::string param = strBuf.GetString();
  send_grpc_msg(::grpcapi::SendRequest::BLE_DFU_PROGRESS, param);
}

void Cyberdog_app::setBTTreadHandle(Document & json_request)
{
  int data;
  CyberdogJson::Get(json_request, "data", data);
  std_msgs::msg::Int8 msg;
  msg.data = data;
  set_bt_tread_pub_->publish(msg);
}

void Cyberdog_app::getBTTreadHandle(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!get_bt_tread_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("get_bluetooth_tread server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = get_bt_tread_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    writer.StartObject();
    writer.Key("data");
    writer.Int(std::stoi(future_result.get()->message));
    writer.EndObject();
  } else {
    ERROR("call get_bluetooth_tread timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::updateBTTreadCB(const std_msgs::msg::Int8::SharedPtr msg)
{
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  writer.StartObject();
  writer.Key("data");
  writer.Int(msg->data);
  writer.EndObject();
  std::string param = strBuf.GetString();
  send_grpc_msg(::grpcapi::SendRequest::UPDATE_BT_TREAD, param);
}

void Cyberdog_app::motionStatusCB(const protocol::msg::MotionStatus::SharedPtr msg)
{
  std::unique_lock<std::shared_mutex> lock(status_mutex_);
  motion_status_.motion_id = msg->motion_id;
}

void Cyberdog_app::taskStatusCB(const protocol::msg::AlgoTaskStatus::SharedPtr msg)
{
  std::unique_lock<std::shared_mutex> lock(status_mutex_);
  task_status_.task_status = msg->task_status;
  task_status_.task_sub_status = msg->task_sub_status;
}

void Cyberdog_app::selfCheckStatusCB(const protocol::msg::SelfCheckStatus::SharedPtr msg)
{
  std::unique_lock<std::shared_mutex> lock(status_mutex_);
  self_check_status_.code = msg->code;
  self_check_status_.description = msg->description;
}

void Cyberdog_app::stateSwitchStatusCB(const protocol::msg::StateSwitchStatus::SharedPtr msg)
{
  std::unique_lock<std::shared_mutex> lock(status_mutex_);
  state_switch_status_.state = msg->state;
  state_switch_status_.code = msg->code;
}

void Cyberdog_app::bmsStatusCB(const protocol::msg::BmsStatus::SharedPtr msg)
{
  std::unique_lock<std::shared_mutex> lock(status_mutex_);
  charging_status_.wired_charging = msg->power_wired_charging;
  charging_status_.wireless_charging = msg->power_wp_charging;
}

void Cyberdog_app::statusRequestHandle(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  {
    std::shared_lock<std::shared_mutex> status_lock(status_mutex_);
    writer.StartObject();
    writer.Key("motion_status");
    writer.StartObject();
    writer.Key("motion_id");
    writer.Int(motion_status_.motion_id);
    writer.EndObject();
    writer.Key("task_status");
    writer.StartObject();
    writer.Key("task_status");
    writer.Int(task_status_.task_status);
    writer.Key("task_sub_status");
    writer.Int(task_status_.task_sub_status);
    writer.EndObject();
    writer.Key("self_check_status");
    writer.StartObject();
    writer.Key("code");
    writer.Int(self_check_status_.code);
    writer.Key("description");
    writer.String(self_check_status_.description.c_str());
    writer.EndObject();
    writer.Key("state_switch_status");
    writer.StartObject();
    writer.Key("state");
    writer.Int(state_switch_status_.state);
    writer.Key("code");
    writer.Int(state_switch_status_.code);
    writer.EndObject();
    writer.Key("charging_status");
    writer.StartObject();
    writer.Key("wired_charging");
    writer.Bool(charging_status_.wired_charging);
    writer.Key("wireless_charging");
    writer.Bool(charging_status_.wireless_charging);
    writer.EndObject();
    writer.EndObject();
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::lowPowerExitHandle(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!low_power_exit_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("low_power_exit service is not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  std::chrono::seconds timeout(13);  // wait for completely exiting
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = low_power_exit_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    writer.StartObject();
    writer.Key("success");
    writer.Bool(future_result.get()->success);
    writer.EndObject();
  } else {
    ERROR("call low_power_exit timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::autoLowPowerEnableHandle(
  Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!auto_low_power_enable_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("low_power_onoff service is not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  bool data;
  CyberdogJson::Get(json_request, "data", data);
  req->data = data;
  auto future_result = auto_low_power_enable_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    writer.StartObject();
    writer.Key("success");
    writer.Bool(future_result.get()->success);
    writer.EndObject();
  } else {
    ERROR("call low_power_onoff timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::setWorkEnvironmentHandle(
  Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!set_work_environment_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("set_nx_environment service is not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<protocol::srv::Trigger::Request>();
  string data;
  if (!CyberdogJson::Get(json_request, "data", data)) {
    ERROR("Please set data");
    returnErrorGrpc(grpc_writer, 323, grpc_response.namecode());
    return;
  }
  req->data = data;
  auto future_result = set_work_environment_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(3));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    writer.StartObject();
    writer.Key("success");
    writer.Bool(future_result.get()->success);
    writer.Key("message");
    writer.String(future_result.get()->message.c_str());
    writer.EndObject();
  } else {
    ERROR("call set_nx_environment timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::uploadSyslogHandle(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!upload_syslog_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("upload_syslog service is not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = upload_syslog_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(59));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    writer.StartObject();
    writer.Key("success");
    writer.Bool(future_result.get()->success);
    writer.EndObject();
  } else {
    ERROR("call upload_syslog_client_ timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

bool Cyberdog_app::HandleGetDeviceInfoRequest(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!query_dev_info_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call querydevinfo server not avaiable"
    );
    return false;
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
  bool is_lowpower_control = false;
  std::string cur_uid("");
  CyberdogJson::Get(json_request, "is_sn", is_sn);
  CyberdogJson::Get(json_request, "is_version", is_version);
  CyberdogJson::Get(json_request, "is_uid", is_version);
  CyberdogJson::Get(json_request, "is_nick_name", is_nick_name);
  CyberdogJson::Get(json_request, "is_volume", is_volume);
  CyberdogJson::Get(json_request, "is_mic_state", is_mic_state);
  CyberdogJson::Get(json_request, "is_voice_control", is_voice_control);
  CyberdogJson::Get(json_request, "is_wifi", is_wifi);
  CyberdogJson::Get(json_request, "is_bat_info", is_bat_info);
  CyberdogJson::Get(json_request, "is_motor_temper", is_motor_temper);
  CyberdogJson::Get(json_request, "is_audio_state", is_audio_state);
  CyberdogJson::Get(json_request, "is_device_model", is_device_model);
  CyberdogJson::Get(json_request, "is_stand", is_stand_up);
  CyberdogJson::Get(json_request, "is_lowpower_control", is_lowpower_control);
  CyberdogJson::Get(json_request, "uid", cur_uid);
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
  req->enables[13] = is_lowpower_control;
  req->uid = cur_uid;
  auto future_result = query_dev_info_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO("success to call querydevinfo request services.");
  } else {
    INFO("Failed to call querydevinfo request  services.");
  }
  grpc_response.set_data(future_result.get()->info);
  INFO(
    "respond namecode:%d, message:%s", grpc_response.namecode(),
    future_result.get()->info.c_str());
  writer->Write(grpc_response);
  return true;
}

bool Cyberdog_app::HandleAccountAdd(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!query_account_add_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call queryaccountadd server not avaiable"
    );
    return false;
  }
  Document json_response(kObjectType);
  std::string rsp_string;
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<protocol::srv::AccountAdd::Request>();
  CyberdogJson::Get(json_request, "member", req->member);
  INFO("account name is: %s", req->member.c_str());
  auto future_result = query_account_add_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO("success to call queryaccountadd request services.");
  } else {
    INFO("Failed to call queryaccountadd request  services.");
  }
  CyberdogJson::Add(json_response, "success", future_result.get()->code);
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set mic state response encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return false;
  }
  INFO("account_add_server grpc_response is: %s", rsp_string.c_str());
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
  return true;
}

bool Cyberdog_app::HandleAccountSearch(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!query_account_search_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call queryaccountsearch server not avaiable"
    );
    return false;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<protocol::srv::AccountSearch::Request>();
  // AccountSeatch.srv中request为string member，但是app发送的键为"account"
  // CyberdogJson::Get(json_request, "member", req->member);
  CyberdogJson::Get(json_request, "account", req->member);
  INFO("request->member: %s", req->member.c_str());
  auto future_result = query_account_search_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO("success to call querysearchadd request services.");
  } else {
    INFO("Failed to call querysearchadd request  services.");
  }

  // grpc_response.set_namecode(grpc_request->namecode());
  INFO("account_search_server grpc_response is : %s", future_result.get()->data.c_str());
  grpc_response.set_data(future_result.get()->data);
  writer->Write(grpc_response);
  return true;
}

bool Cyberdog_app::HandleAccountDelete(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string rsp_string;
  std::chrono::seconds timeout(3);
  if (!query_account_delete_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call account delete serve not avaiable"
    );
    return false;
  }
  auto req = std::make_shared<protocol::srv::AccountDelete::Request>();
  CyberdogJson::Get(json_request, "member", req->member);
  INFO("req->member is: %s", req->member.c_str());
  // call ros service
  auto future_result = query_account_delete_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO(
      "success to call set mic state request services.");
  } else {
    INFO(
      "Failed to call set mic state request  services.");
    return false;
  }
  CyberdogJson::Add(json_response, "success", future_result.get()->code);
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set mic state response encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return false;
  }
  INFO("Account_delete_server grpc_response is: %s", rsp_string.c_str());
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
  return true;
}

bool Cyberdog_app::HandleAccountChange(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string rsp_string;
  std::chrono::seconds timeout(3);
  if (!query_account_change_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call account delete serve not avaiable"
    );
    return false;
  }
  auto req = std::make_shared<protocol::srv::AccountChange::Request>();
  CyberdogJson::Get(json_request, "pre_name", req->pre_name);
  CyberdogJson::Get(json_request, "new_name", req->new_name);
  // call ros service
  auto future_result = query_account_change_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO(
      "success to call set mic state request services.");
  } else {
    INFO(
      "Failed to call set mic state request  services.");
    return false;
  }

  CyberdogJson::Add(json_response, "success", future_result.get()->code);
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set mic state response encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return false;
  }
  INFO("account_add_server grpc_response is: %s", rsp_string.c_str());
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
  return true;
}

void Cyberdog_app::motionServoRequestHandle(
  const Document & json_request, ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  protocol::msg::MotionServoCmd motion_servo_cmd;
  CyberdogJson::Get(json_request, "motion_id", motion_servo_cmd.motion_id);
  CyberdogJson::Get(json_request, "cmd_type", motion_servo_cmd.cmd_type);
  CyberdogJson::Get(json_request, "vel_des", motion_servo_cmd.vel_des);
  CyberdogJson::Get(json_request, "rpy_des", motion_servo_cmd.rpy_des);
  CyberdogJson::Get(json_request, "pos_des", motion_servo_cmd.pos_des);
  CyberdogJson::Get(json_request, "acc_des", motion_servo_cmd.acc_des);
  CyberdogJson::Get(
    json_request, "ctrl_point",
    motion_servo_cmd.ctrl_point);
  CyberdogJson::Get(json_request, "foot_pose", motion_servo_cmd.foot_pose);
  CyberdogJson::Get(
    json_request, "step_height",
    motion_servo_cmd.step_height);
  CyberdogJson::Get(json_request, "value", motion_servo_cmd.value);
  motion_servo_request_pub_->publish(motion_servo_cmd);
  grpc_response.set_data("");
  writer->Write(grpc_response);
}

void Cyberdog_app::motionCMDRequestHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  INFO("MOTION_CMD_REQUEST");
  auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
  protocol::srv::MotionResultCmd::Response rsp;
  // get ros service request
  CyberdogJson::Get(json_request, "motion_id", req->motion_id);
  CyberdogJson::Get(json_request, "vel_des", req->vel_des);
  CyberdogJson::Get(json_request, "rpy_des", req->rpy_des);
  CyberdogJson::Get(json_request, "pos_des", req->pos_des);
  CyberdogJson::Get(json_request, "acc_des", req->acc_des);
  CyberdogJson::Get(json_request, "ctrl_point", req->ctrl_point);
  CyberdogJson::Get(json_request, "foot_pose", req->foot_pose);
  CyberdogJson::Get(json_request, "step_height", req->step_height);
  CyberdogJson::Get(json_request, "contact", req->contact);
  CyberdogJson::Get(json_request, "duration", req->duration);
  CyberdogJson::Get(json_request, "value", req->value);
  // call ros service
  if (!callMotionServoCmd(req, rsp)) {
    returnErrorGrpc(writer, 321, grpc_response.namecode());
    return;
  }
  // send service response
  CyberdogJson::Add(json_response, "motion_id", rsp.motion_id);
  CyberdogJson::Add(json_response, "result", rsp.result);
  CyberdogJson::Add(json_response, "code", rsp.code);
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  } else {
    INFO(
      "motion_result_cmd response: motion_id: %d, result: %d, code: %d",
      rsp.motion_id, rsp.result, rsp.code);
  }
  // send grpc result
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::faceEntryRequestHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  INFO("grpc get face entry request from app.");
  if (!ai_face_entry_client_->wait_for_service(std::chrono::seconds(3))) {
    WARN("face entry server is not available");
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<protocol::srv::FaceEntry::Request>();
  CyberdogJson::Get(json_request, "command", req->command);
  CyberdogJson::Get(json_request, "username", req->username);
  CyberdogJson::Get(json_request, "oriname", req->oriname);
  CyberdogJson::Get(json_request, "ishost", req->ishost);
  protocol::srv::FaceEntry::Response rsp;
  auto future_result = ai_face_entry_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO("success to call face entry response services.");
  } else {
    WARN("Failed to call face entry response services.");
  }
  rsp.result = future_result.get()->result;
  CyberdogJson::Add(json_response, "result", rsp.result);
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while encoding authenticate response to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
  INFO("grpc send face entry response to app.");
}

void Cyberdog_app::faceRecRequestHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  INFO("grpc get face recognition request from app.");
  if (!ai_face_recognition_client_->wait_for_service(std::chrono::seconds(3))) {
    WARN("face recognition server is not available");
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<protocol::srv::FaceRec::Request>();
  CyberdogJson::Get(json_request, "command", req->command);
  CyberdogJson::Get(json_request, "username", req->username);
  protocol::srv::FaceRec::Response rsp;
  auto future_result = ai_face_recognition_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO("success to call face recognition response services.");
  } else {
    WARN("Failed to call face recognition response services.");
  }
  rsp.result = future_result.get()->result;
  CyberdogJson::Add(json_response, "result", rsp.result);
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while encoding authenticate response to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
  INFO("grpc send face recognition response to app.");
}

void Cyberdog_app::deviceNameSwitchHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!dev_name_enable_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call set nickname switch server not avaiable"
    );
    returnErrorGrpc(writer, 322, grpc_response.namecode());
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  CyberdogJson::Get(json_request, "enable", req->data);
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
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while device name switch response encoding to json");
    returnErrorGrpc(writer);
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::deviceNameSetHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!dev_name_set_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call setnickname server not avaiable"
    );
    returnErrorGrpc(writer, 322, grpc_response.namecode());
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<protocol::srv::AudioNickName::Request>();
  CyberdogJson::Get(json_request, "nick_name", req->nick_name);
  CyberdogJson::Get(json_request, "wake_name", req->wake_name);
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
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while device name set response encoding to json");
    returnErrorGrpc(writer);
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::deviceVolumeSetHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!audio_volume_set_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call volume set server not available");
    returnErrorGrpc(writer, 322, grpc_response.namecode());
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<protocol::srv::AudioVolumeSet::Request>();
  int volume;
  CyberdogJson::Get(json_request, "volume", volume);
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
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while volume set response encoding to json");
    returnErrorGrpc(writer);
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::deviceMicSetHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!audio_execute_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call mic set server not available");
    returnErrorGrpc(writer, 322, grpc_response.namecode());
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<protocol::srv::AudioExecute::Request>();
  req->client = "app_server";
  bool enable;
  CyberdogJson::Get(json_request, "enable", enable);
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
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set mic state response encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::deviceAudioSetHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!audio_action_set_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call audio action set server not available");
    returnErrorGrpc(writer, 322, grpc_response.namecode());
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  bool enable;
  CyberdogJson::Get(json_request, "enable", enable);
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
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set audio action response encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::audioAuthenticationRequestHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!audio_auth_request->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "callAuthenticateRequest server not available");
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
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR(
      "error while encoding authenticate request to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::audioAuthenticationResponseHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!audio_auth_response->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "callAuthenticateResponse server not available");
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<protocol::srv::AudioAuthToken::Request>();
  protocol::srv::AudioAuthToken::Response rsp;
  CyberdogJson::Get(json_request, "uid", req->uid);
  // CyberdogJson::Get(json_request, "title", req->title);
  CyberdogJson::Get(json_request, "token_access", req->token_access);
  CyberdogJson::Get(json_request, "token_fresh", req->token_fresh);
  std::string tei;
  CyberdogJson::Get(json_request, "token_expires_in", tei);
  req->token_expirein = stoul(tei);
  // CyberdogJson::Get(json_request, "token_deviceid",
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
  std::string rsp_string;
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR(
      "error while encoding authenticate response to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
}

void Cyberdog_app::audioVoicePrintTrainStartHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!audio_voiceprint_train->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call voiceprint train start server not available");
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<protocol::srv::AudioVoiceprintTrain::Request>();
  req->train_id = protocol::srv::AudioVoiceprintTrain::Request::TID_START;
  CyberdogJson::Get(json_request, "nick_name", req->voice_print.name);
  CyberdogJson::Get(json_request, "voiceprint_id", req->voice_print.id);
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
  grpc_response.set_data("{}");
  writer->Write(grpc_response);
}

void Cyberdog_app::audioVoicePrintTrainCancelHandle(
  const Document & json_request, Document & json_response,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!audio_voiceprint_train->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call voiceprint train cancel server not available");
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
  grpc_response.set_data("{}");
  writer->Write(grpc_response);
}

void Cyberdog_app::audioVoicePrintDataHandle(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (!voiceprints_data_notify->wait_for_service(std::chrono::seconds(3))) {
    ERROR(
      "call voiceprints data server not available");
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
  writer->Write(grpc_response);
}

bool Cyberdog_app::HandleUnlockDevelopAccess(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  int unlock_result_ = 100;
  Document json_response(kObjectType);
  std::string rsp_string;
  std::chrono::seconds timeout(10);
  if (!unlock_develop_access_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("call unlock develop access serve not avaiable");
    return false;
  }
  auto req = std::make_shared<protocol::srv::Unlock::Request>();
  CyberdogJson::Get(json_request, "httplink", req->httplink);
  INFO("req->httplink is: %s", req->httplink.c_str());
  // call ros service
  auto future_result = unlock_develop_access_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO(
      "success to call unlock develop access request services.");
    unlock_result_ = future_result.get()->unlock_result;
  } else {
    INFO(
      "Failed to call unlock develop access request  services.");
  }
  // int unlock_result_ = future_result.get()->unlock_result;
  CyberdogJson::Add(json_response, "unlock_result", unlock_result_);
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set unlock develop access encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return false;
  }
  INFO("unlock develop access grpc_response is: %s", rsp_string.c_str());
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
  return true;
}
bool Cyberdog_app::RebootManchine(
  const Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  Document json_response(kObjectType);
  std::string rsp_string;
  std::chrono::seconds timeout(5);
  if (!reboot_machine_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("call reboot machine server not avaiable");
    return false;
  }
  auto req = std::make_shared<protocol::srv::RebootMachine::Request>();
  req->rebootmachine = 1997;
  // CyberdogJson::Get(json_request, "httplink", req->httplink);
  // INFO("req->httplink is: %s", req->httplink.c_str());
  // call ros service
  auto future_result = reboot_machine_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO(
      "success to call reboot machine request services.");
  } else {
    INFO(
      "Failed to call reboot machine request  services.");
  }
  int reboot_result_ = future_result.get()->rebootresult;
  CyberdogJson::Add(json_response, "reboot_result", reboot_result_);
  if (!CyberdogJson::Document2String(json_response, rsp_string)) {
    ERROR("error while set reboot machine encoding to json");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return false;
  }
  INFO("reboot machine grpc_response is: %s", rsp_string.c_str());
  grpc_response.set_data(rsp_string);
  writer->Write(grpc_response);
  return true;
}
void Cyberdog_app::dogLegCalibrationHandle(
  Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!dog_leg_calibration_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("dog_leg_calibration service is not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  bool data;
  CyberdogJson::Get(json_request, "data", data);
  req->data = data;
  auto future_result = dog_leg_calibration_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(10));
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  if (status == std::future_status::ready) {
    writer.StartObject();
    writer.Key("success");
    writer.Bool(future_result.get()->success);
    writer.Key("message");
    writer.String(future_result.get()->message.c_str());
    writer.EndObject();
  } else {
    ERROR("call dog_leg_calibration timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}
void Cyberdog_app::ProcessMsg(
  const ::grpcapi::SendRequest * grpc_request,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  INFO(
    "ProcessMsg %d %s", grpc_request->namecode(),
    grpc_request->params().c_str());
  ::grpcapi::RecResponse grpc_response;
  Document json_request(kObjectType);
  Document json_response(kObjectType);
  std::string rsp_string;
  json_request.Parse<0>(grpc_request->params().c_str());
  if (json_request.HasParseError()) {
    ERROR("Parse Error");
    returnErrorGrpc(writer, 323, grpc_response.namecode());
    return;
  }
  grpc_response.set_namecode(grpc_request->namecode());
  switch (grpc_request->namecode()) {
    case ::grpcapi::SendRequest::GET_DEVICE_INFO: {
        if (!HandleGetDeviceInfoRequest(json_request, grpc_response, writer)) {
          return;
        }
      } break;
    case ::grpcapi::SendRequest::MOTION_SERVO_REQUEST: {
        motionServoRequestHandle(json_request, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::MOTION_CMD_REQUEST: {
        motionCMDRequestHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::VISUAL_FRONTEND_MSG: {
        std_msgs::msg::String msg;
        msg.data = grpc_request->params();
        visual_request_pub_->publish(msg);
      } break;
    case ::grpcapi::SendRequest::FACE_ENTRY_REQUEST: {
        faceEntryRequestHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::FACE_RECORDING_REQUEST: {
        faceRecRequestHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::DEVICE_NAME_SWITCH: {
        deviceNameSwitchHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::DEVICE_NAME_SET: {
        deviceNameSetHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::DEVICE_VOLUME_SET: {
        deviceVolumeSetHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::DEVICE_MIC_SET: {
        deviceMicSetHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::DEVICE_AUDIO_SET: {
        deviceAudioSetHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::AUDIO_AUTHENTICATION_REQUEST: {
        audioAuthenticationRequestHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::AUDIO_AUTHENTICATION_RESPONSE: {
        audioAuthenticationResponseHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::AUDIO_VOICEPRINTTRAIN_START: {
        audioVoicePrintTrainStartHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::AUDIO_VOICEPRINTTRAIN_CANCEL: {
        audioVoicePrintTrainCancelHandle(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::AUDIO_VOICEPRINTS_DATA: {
        audioVoicePrintDataHandle(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::IMAGE_TRANSMISSION_REQUEST: {
        std_msgs::msg::String it_msg;
        if (!CyberdogJson::Document2String(json_request, it_msg.data)) {
          ERROR("error while parse image transmission data to string");
          returnErrorGrpc(writer, 323, grpc_response.namecode());
          return;
        }
        image_trans_pub_->publish(it_msg);
        grpc_response.set_data("");
        writer->Write(grpc_response);
      } break;
    case ::grpcapi::SendRequest::CAMERA_SERVICE: {
        uint32_t command = 2;
        CyberdogJson::Get(json_request, "command", command);
        if (!processCameraMsg(grpc_request->namecode(), command, writer)) {
          return;
        }
      } break;
    case ::grpcapi::SendRequest::OTA_STATUS_REQUEST:
      {
        if (!HandleOTAStatusRequest(json_request, grpc_response, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_VERSION_QUERY_REQUEST:
      {
        if (!HandleOTAVersionQueryRequest(json_request, grpc_response, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_START_DOWNLOAD_REQUEST:
      {
        if (!HandleOTAStartDownloadRequest(json_request, grpc_response, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_START_UPGRADE_REQUEST:
      {
        if (!HandleOTAStartUpgradeRequest(json_request, grpc_response, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_PROCESS_QUERY_REQUEST:
      {
        if (!HandleOTAProcessQueryRequest(json_request, grpc_response, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::OTA_ESTIMATE_UPGRADE_TIME_REQUEST:
      {
        if (!HandleOTAEstimateUpgradeTimeRequest(json_request, grpc_response, writer)) {
          return;
        }
      }
      break;
    case ::grpcapi::SendRequest::MAP_SET_LABLE_REQUEST: {
        handlLableSetRequest(json_request, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::MAP_GET_LABLE_REQUEST: {
        handlLableGetRequest(json_request, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::NAV_ACTION: {
        handleNavigationAction(json_request, grpc_response, writer, true);
      } break;
    case ::grpcapi::SendRequest::SELECTED_TRACKING_OBJ: {
        selectTrackingObject(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::STOP_NAV_ACTION: {
        handleStopAction(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::ACCESS_NAV_ACTION: {
        handleNavigationAction(json_request, grpc_response, writer, false);
      } break;
    case ::grpcapi::SendRequest::BLUETOOTH_SCAN: {
        scanBluetoothDevices(json_request, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::BLUETOOTH_CONNECT: {
        connectBluetoothDevice(json_request, json_response, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::BLUETOOTH_CONNECTED_DEVICES: {
        currentConnectedBluetoothDevices(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::BLE_DIVICE_FIRMWARE_VERSION: {
        getBLEFirmwareVersionHandle(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::BLE_DEVICE_BATTERY_LEVEL: {
        getBLEBatteryLevelHandle(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::DELETE_BLE_HISTORY: {
        deleteBLEHistoryHandle(json_request, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::UPDATE_BLE_FIRMWARE: {
        updateBLEFirmwareHandle(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::SET_BT_TREAD: {
        setBTTreadHandle(json_request);
        grpc_response.set_data("");
        writer->Write(grpc_response);
      } break;
    case ::grpcapi::SendRequest::GET_BT_TREAD: {
        getBTTreadHandle(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::STATUS_REQUEST: {
        statusRequestHandle(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::LOW_POWER_EXIT: {
        lowPowerExitHandle(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::AUTO_LOW_POWER_ENABLE: {
        autoLowPowerEnableHandle(json_request, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::SET_WORK_ENVIRONMENT: {
        setWorkEnvironmentHandle(json_request, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::UPLOAD_SYSLOG: {
        uploadSyslogHandle(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::DOG_LEG_CALIBRATION: {
        dogLegCalibrationHandle(json_request, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::ACCOUNT_MEMBER_ADD: {
        if (!HandleAccountAdd(json_request, grpc_response, writer)) {
          return;
        }
      } break;
    case ::grpcapi::SendRequest::ACCOUNT_MEMBER_SEARCH: {
        if (!HandleAccountSearch(json_request, grpc_response, writer)) {
          return;
        }
      } break;
    case ::grpcapi::SendRequest::ACCOUNT_MEMBER_CHANGE: {
        if (!HandleAccountChange(json_request, grpc_response, writer)) {
          return;
        }
      } break;
    case ::grpcapi::SendRequest::ACCOUNT_MEMBER_DELETE: {
        if (!HandleAccountDelete(json_request, grpc_response, writer)) {
          return;
        }
      } break;
    case 55001: {  // for testing
        std_msgs::msg::Bool msg;
        msg.data = true;
        app_disconnect_pub_->publish(msg);
        grpc_response.set_data("");
        writer->Write(grpc_response);
      } break;
    case 1100: {
        startStairAlignHandle(json_request, grpc_response, writer);
      } break;
    case 1101: {
        stopStairAlignHandle(grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::UNLOCK_DEVELOP_ACCESS: {
        HandleUnlockDevelopAccess(json_request, grpc_response, writer);
      } break;
    case ::grpcapi::SendRequest::REBOOT_MACHINE: {
        RebootManchine(json_request, grpc_response, writer);
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
          ERROR("Send file error. Please check your connection!");
          return;
        }
      } break;
    case ::grpcapi::SendRequest::DOWNLOAD_FILE: {
        std::string file_name;
        CyberdogJson::Get(json_request, "file_name", file_name);
        TransmitFiles::SendFile(writer, file_name);
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
  if (TransmitFiles::thread_counts_ > 5) {
    result = 10;
  } else if (!callCameraService(command, result, msg)) {
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
  if (TransmitFiles::thread_counts_ > 5) {
    result = 10;
  } else if (!callCameraService(command, result, msg)) {
    result = 7;
  }
  return TransmitFiles::ReturnCameraFile(writer, result, msg);
}

void Cyberdog_app::publishNotCompleteSendingFiles()
{
  std::set<std::string> files_need_to_be_sent;
  TransmitFiles::SendFile(nullptr, std::string(), std::string(), 0, &files_need_to_be_sent);
  if (!files_need_to_be_sent.empty()) {
    INFO("There are some files need to be downloaded to the phone.");
    rapidjson::StringBuffer strBuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
    writer.StartObject();
    writer.Key("file_name");
    writer.StartArray();
    for (auto & name : files_need_to_be_sent) {
      writer.String(name.c_str());
    }
    writer.EndArray();
    writer.EndObject();
    std::string param = strBuf.GetString();
    send_grpc_msg(::grpcapi::SendRequest::FILES_NOT_DOWNLOAD_COMPLETE, param);
  }
}

void Cyberdog_app::autoSavedFileCB(const std_msgs::msg::String::SharedPtr msg)
{
  std::set<std::string> autosaved_file {msg->data};
  TransmitFiles::SendFile(nullptr, std::string(), std::string(), 0, &autosaved_file);
}

void Cyberdog_app::startStairAlignHandle(
  Document & json_request,
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!start_stair_align_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("start_stair_align server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  bool data;
  CyberdogJson::Get(json_request, "data", data);
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = data;
  auto future_result = start_stair_align_client_->async_send_request(req);
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  std::future_status status = future_result.wait_for(
    std::chrono::seconds(5));
  if (status == std::future_status::ready) {
    bool success = future_result.get()->success;
    writer.StartObject();
    writer.Key("success");
    writer.Bool(success);
    writer.EndObject();
  } else {
    ERROR("call start_stair_align timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}

void Cyberdog_app::stopStairAlignHandle(
  ::grpcapi::RecResponse & grpc_response,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer)
{
  if (!stop_stair_align_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("stop_stair_align server not avaiable");
    returnErrorGrpc(grpc_writer, 322, grpc_response.namecode());
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = stop_stair_align_client_->async_send_request(req);
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  std::future_status status = future_result.wait_for(
    std::chrono::seconds(5));
  if (status == std::future_status::ready) {
    bool success = future_result.get()->success;
    writer.StartObject();
    writer.Key("success");
    writer.Bool(success);
    writer.EndObject();
  } else {
    ERROR("call stop_stair_align timeout.");
    returnErrorGrpc(grpc_writer, 321, grpc_response.namecode());
    return;
  }
  grpc_response.set_data(strBuf.GetString());
  grpc_writer->Write(grpc_response);
}
}  // namespace bridges
}  // namespace cyberdog
