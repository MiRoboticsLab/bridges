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

#ifndef CYBERDOG_APP_HPP_
#define CYBERDOG_APP_HPP_

#include <shared_mutex>
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
#include <map>
#include <atomic>

// Interfaces
#include "cyberdog_app_client.hpp"
#include "action_task.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "msg_dispatcher.hpp"
#include "protocol/action/navigation.hpp"
#include "protocol/action/over_the_air.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "net_avalible.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/msg/audio_voiceprint_result.hpp"
#include "protocol/msg/connector_status.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/label.hpp"
#include "protocol/msg/map_label.hpp"
#include "protocol/msg/face_recognition_result.hpp"
#include "protocol/msg/face_entry_result.hpp"
#include "protocol/srv/audio_auth_id.hpp"
#include "protocol/srv/audio_auth_token.hpp"
#include "protocol/srv/ota_server_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/srv/audio_voiceprint_train.hpp"
#include "protocol/srv/audio_voiceprints_set.hpp"
#include "protocol/srv/camera_service.hpp"
#include "protocol/srv/device_info.hpp"
#include "protocol/srv/audio_nick_name.hpp"
#include "protocol/srv/audio_volume_set.hpp"
#include "protocol/srv/audio_execute.hpp"
#include "protocol/srv/get_map_label.hpp"
#include "protocol/srv/set_map_label.hpp"
#include "protocol/srv/account_add.hpp"
#include "protocol/srv/account_search.hpp"
#include "protocol/srv/account_delete.hpp"
#include "protocol/srv/account_change.hpp"
#include "protocol/msg/person.hpp"
#include "protocol/msg/ota_update.hpp"
#include "protocol/srv/body_region.hpp"
#include "protocol/srv/ble_scan.hpp"
#include "protocol/srv/ble_connect.hpp"
#include "protocol/srv/face_entry.hpp"
#include "protocol/srv/face_rec.hpp"
#include "protocol/srv/stop_algo_task.hpp"
#include "protocol/srv/get_ble_battery_level.hpp"
#include "protocol/msg/bledfu_progress.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/msg/algo_task_status.hpp"
#include "protocol/msg/self_check_status.hpp"
#include "protocol/msg/state_switch_status.hpp"
#include "protocol/srv/unlock.hpp"
#include "protocol/srv/reboot_machine.hpp"
#include "protocol/srv/trigger.hpp"
#include "protocol/srv/elec_skin.hpp"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "threadsafe_queue.hpp"
#include "time_interval.hpp"
#include "ready_sn.hpp"
using string = std::string;
using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;
namespace cyberdog
{
namespace bridges
{
/**
 * @brief ROS node for connection between APP and cyberdog
 */
class Cyberdog_app : public rclcpp::Node
{
public:
  Cyberdog_app();
  ~Cyberdog_app();
  std::string getServiceIp()
  {
    return *server_ip;
  }
  /**
   * @brief Distribute gRPC request
   * @param grpc_request Request from APP
   * @param writer Stream response writer
   */
  void ProcessMsg(
    const ::grpcapi::SendRequest * grpc_request,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  /**
   * @brief Process getFile request
   * @param grpc_request getFile request
   * @param writer Stream response writer
   */
  void ProcessGetFile(
    const ::grpcapi::SendRequest * grpc_request,
    ::grpc::ServerWriter<::grpcapi::FileChunk> * writer);

private:
  uint32_t ticks_;
  std::atomic_bool can_process_messages_;
  /**
   * @brief Running gRPC server
   */
  void RunServer();
  std::shared_ptr<std::thread> app_server_thread_;
  std::shared_ptr<std::thread> heart_beat_thread_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ip_subscriber;
  rclcpp::Subscription<protocol::msg::ConnectorStatus>::SharedPtr connect_status_subscriber;
  rclcpp::CallbackGroup::SharedPtr connector_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    dog_pose_sub_;
  void destroyGrpcServer();
  std::string getDogIp(const string str, const string & split);
  std::string getPhoneIp(const string str, const string & split);
  std::shared_ptr<Cyberdog_App_Client> app_stub_;
  mutable std::shared_mutex stub_mutex_;  // mutex for app_stub_
  std::shared_ptr<std::string> server_ip;
  std::unique_ptr<grpc::Server> server_;
  // void subscribeIp(const std_msgs::msg::String::SharedPtr msg);
  void subscribeConnectStatus(const protocol::msg::ConnectorStatus::SharedPtr msg);
  void subscribeBmsStatus(const protocol::msg::BmsStatus::SharedPtr msg);
  void destroyGrpc();
  void createGrpc();
  string GetFileConecxt(string path);
  NetChecker net_checker;
  std::atomic_bool app_disconnected;
  std::string local_ip;
  bool is_internet;
  int wifi_strength;
  mutable std::shared_mutex connector_mutex_;
  // mutex for local_ip, is_internet, wifi_strength, server_ip
  std::chrono::system_clock::time_point connector_update_time_point_;
  mutable std::mutex update_time_mutex_;  // mutex for connector_update_time_point_

  protocol::msg::BmsStatus bms_status;
  std::string sn;
  TimeInterval timer_interval;
  /**
   * @brief Send heatBeat message
   */
  void HeartBeat();

  // ros interaction codes
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<protocol::msg::MotionServoResponse>::SharedPtr
    motion_servo_response_sub_;
  rclcpp::CallbackGroup::SharedPtr motion_callback_group_;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr
    motion_servo_request_pub_;
  void motion_servo_rsp_callback(
    const protocol::msg::MotionServoResponse::SharedPtr msg);
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr
    motion_ressult_client_;
  bool callMotionServoCmd(
    const std::shared_ptr<protocol::srv::MotionResultCmd::Request> req,
    protocol::srv::MotionResultCmd::Response & rep);
  void returnErrorGrpc(
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer,
    int error_code = -1, uint32_t namecode = 0);

  /**
   * @brief handle ota stauts
   *
   * @param json_request
   * @param grpc_response
   * @param writer
   * @return true
   * @return false
   */
  bool HandleOTAStatusRequest(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  /**
   * @brief handle ota version
   *
   * @param json_request
   * @param grpc_response
   * @param writer
   * @return true
   * @return false
   */
  bool HandleOTAVersionQueryRequest(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  /**
   * @brief handle ota download
   *
   * @param json_request
   * @param grpc_response
   * @param writer
   * @return true
   * @return false
   */
  bool HandleOTAStartDownloadRequest(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  /**
   * @brief handle ota upgrade
   *
   * @param json_request
   * @param grpc_response
   * @param writer
   * @return true
   * @return false
   */
  bool HandleOTAStartUpgradeRequest(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  /**
   * @brief handle ota process query
   *
   * @param json_request
   * @param grpc_response
   * @param writer
   * @return true
   * @return false
   */
  bool HandleOTAProcessQueryRequest(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  /**
   * @brief handle ota time upgrade
   *
   * @param json_request
   * @param grpc_response
   * @param writer
   * @return true
   * @return false
   */
  bool HandleOTAEstimateUpgradeTimeRequest(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  bool HandleGetDeviceInfoRequest(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  bool HandleAccountAdd(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  bool HandleAccountSearch(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  bool HandleAccountDelete(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  bool HandleAccountChange(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  bool HandleUnlockDevelopAccess(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  bool RebootManchine(
    const Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  void motionServoRequestHandle(
    const Document & json_request, ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void motionCMDRequestHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void faceEntryRequestHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void faceRecRequestHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void deviceNameSwitchHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void deviceNameSetHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void deviceValuemSetHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void deviceVolumeSetHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void deviceMicSetHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void deviceAudioSetHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void audioAuthenticationRequestHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void audioAuthenticationResponseHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void audioVoicePrintTrainStartHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void audioVoicePrintTrainCancelHandle(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void audioVoicePrintDataHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  // visual program
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visual_response_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visual_request_pub_;
  void backend_message_callback(const std_msgs::msg::String::SharedPtr msg);

  // ai program
  rclcpp::Client<protocol::srv::FaceEntry>::SharedPtr ai_face_entry_client_;
  rclcpp::Client<protocol::srv::FaceRec>::SharedPtr ai_face_recognition_client_;
  rclcpp::Subscription<protocol::msg::FaceEntryResult>::SharedPtr ai_face_entry_sub_;
  rclcpp::Subscription<protocol::msg::FaceRecognitionResult>::SharedPtr ai_face_recognition_sub_;
  void face_entry_result_callback(const protocol::msg::FaceEntryResult::SharedPtr msg);
  void face_rec_result_callback(const protocol::msg::FaceRecognitionResult::SharedPtr msg);

  // unlock develop access
  rclcpp::Client<protocol::srv::Unlock>::SharedPtr unlock_develop_access_client_;

  // reboot machine
  rclcpp::Client<protocol::srv::RebootMachine>::SharedPtr reboot_machine_client_;

  // audio program
  rclcpp::Client<protocol::srv::AudioAuthId>::SharedPtr audio_auth_request;
  rclcpp::Client<protocol::srv::AudioAuthToken>::SharedPtr audio_auth_response;
  rclcpp::Client<protocol::srv::AudioVoiceprintTrain>::SharedPtr audio_voiceprint_train;
  rclcpp::Client<protocol::srv::AudioVoiceprintsSet>::SharedPtr voiceprints_data_notify;
  rclcpp::Subscription<protocol::msg::AudioVoiceprintResult>::SharedPtr
    audio_voiceprint_result_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr voiceprints_data_sub_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  void voiceprint_result_callback(const protocol::msg::AudioVoiceprintResult::SharedPtr msg);
  void voiceprints_data_callback(const std_msgs::msg::Bool::SharedPtr msg);
  // commcon code
  void send_grpc_msg(uint32_t code, const std::string & msg);
  void send_grpc_msg(uint32_t code, const Document & doc);

  // image_transmission
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr image_trans_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr image_trans_sub_;
  void image_transmission_callback(const std_msgs::msg::String::SharedPtr msg);

  // photo and video recording
  rclcpp::Client<protocol::srv::CameraService>::SharedPtr camera_service_client_;
  bool callCameraService(uint8_t command, uint8_t & result, std::string & msg);
  bool processCameraMsg(
    uint32_t namecode,
    uint8_t command,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  bool processCameraMsg(
    uint32_t namecode,
    uint8_t command,
    ::grpc::ServerWriter<::grpcapi::FileChunk> * writer);
  bool returnResponse(
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer,
    uint8_t result,
    const std::string & msg,
    uint32_t namecode);
  void publishNotCompleteSendingFiles();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autosaved_file_sub_;
  void autoSavedFileCB(const std_msgs::msg::String::SharedPtr msg);

  // ota
  void ResetOTAFlags();
  void HandleDownloadPercentageMsgs(const protocol::msg::OtaUpdate msg);
  void HandleUpgradePercentageMsgs(const protocol::msg::OtaUpdate msg);
  void HandleUpgradeRebootMsgs(const std_msgs::msg::Bool msg);
  rclcpp::Subscription<protocol::msg::OtaUpdate>::SharedPtr download_subscriber_ {nullptr};
  rclcpp::Subscription<protocol::msg::OtaUpdate>::SharedPtr upgrade_subscriber_ {nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reboot_subscriber_ {nullptr};
  rclcpp::Client<protocol::srv::OtaServerCmd>::SharedPtr ota_client_;

  rclcpp_action::Client<protocol::action::OverTheAir>::SharedPtr ota_action_client_;
  void handleOTAAction(
    const std::string & request_param, ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer,
    bool create_new_task);
  std::shared_mutex ota_hash_mx_;
  std::map<std::string, size_t> ota_hash_map_;

  // configured ports
  std::string grpc_server_port_;
  std::string grpc_client_port_;

  // app connection state
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr app_connection_pub_;

  // robot state
  rclcpp::Client<protocol::srv::DeviceInfo>::SharedPtr query_dev_info_client_;

  // robot nick name switch
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr dev_name_enable_client_;

  // robot nick name
  rclcpp::Client<protocol::srv::AudioNickName>::SharedPtr dev_name_set_client_;

  // audio volume set
  rclcpp::Client<protocol::srv::AudioVolumeSet>::SharedPtr audio_volume_set_client_;

  // audio mic state
  rclcpp::Client<protocol::srv::AudioExecute>::SharedPtr audio_execute_client_;

  // account member add
  rclcpp::Client<protocol::srv::AccountAdd>::SharedPtr query_account_add_client_;

  // account member search
  rclcpp::Client<protocol::srv::AccountSearch>::SharedPtr query_account_search_client_;

  // account member delete
  rclcpp::Client<protocol::srv::AccountDelete>::SharedPtr query_account_delete_client_;

  // account member change
  rclcpp::Client<protocol::srv::AccountChange>::SharedPtr query_account_change_client_;

  // process map message
  void processMapMsg(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void processDogPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // for message queue
  void send_msgs_(
    const std::unique_ptr<::grpcapi::SendRequest> msg);
  std::map<uint32_t, std::shared_ptr<LatestMsgDispather<
      std::unique_ptr<::grpcapi::SendRequest>>>>
  send_thread_map_;
  std::map<uint32_t, size_t> namecode_queue_size_;
  std::shared_mutex send_thread_map_mx_;

  // mapping and navigation
  void handlLableSetRequest(
    const Document & json_request, ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  rclcpp::Client<protocol::srv::SetMapLabel>::SharedPtr set_label_client_;

  void handlLableGetRequest(
    const Document & json_request, ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  rclcpp::Client<protocol::srv::GetMapLabel>::SharedPtr get_label_client_;

  void handleNavigationAction(
    const Document & json_request, ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer,
    bool create_new_task = true);

  ActionTaskManager action_task_manager_;
  rclcpp_action::Client<protocol::action::Navigation>::SharedPtr
    navigation_client_;
  rclcpp::CallbackGroup::SharedPtr navigation_callback_group_;
  std::map<uint8_t, size_t> task_type_hash_map_;
  std::shared_mutex type_hash_mutex_;
  std::mutex task_init_mutex_;
  void uploadNavPath(const nav_msgs::msg::Path::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr nav_path_sub_;
  rclcpp::CallbackGroup::SharedPtr path_map_callback_group_;
  bool connect_mark_ {false};
  void disconnectTaskRequest();

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  void laserScanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void handleStopAction(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  rclcpp::Client<protocol::srv::StopAlgoTask>::SharedPtr stop_nav_action_client_;

  // for tracking
  rclcpp::Subscription<protocol::msg::Person>::SharedPtr tracking_person_sub_;
  void publishTrackingPersonCB(const protocol::msg::Person::SharedPtr msg);
  rclcpp::Client<protocol::srv::BodyRegion>::SharedPtr select_tracking_human_client_;
  void selectTrackingObject(
    Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_point_cloud_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr point_cloud_state_client_;
  void handleEnablePointCloud(
    const Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);
  void handlePointCloudStateRequest(
    Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * writer);

  // bluetooth
  rclcpp::Client<protocol::srv::BLEScan>::SharedPtr scan_bluetooth_devices_client_;
  rclcpp::Client<protocol::srv::BLEConnect>::SharedPtr connect_bluetooth_device_client_;
  rclcpp::Client<protocol::srv::BLEScan>::SharedPtr current_connected_bluetooth_client_;
  rclcpp::Client<protocol::srv::GetBLEBatteryLevel>::SharedPtr ble_battery_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ble_device_firmware_version_client_;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr delete_ble_history_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ble_firmware_update_notification_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr update_ble_firmware_client_;
  rclcpp::Subscription<protocol::msg::BLEDFUProgress>::SharedPtr ble_dfu_progress_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr set_bt_tread_pub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr update_bt_tread_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_bt_tread_client_;

  void scanBluetoothDevices(
    Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void connectBluetoothDevice(
    Document & json_request, Document & json_response,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disconnected_unexpected_sub_;
  void disconnectedUnexpectedCB(const std_msgs::msg::Bool::SharedPtr msg);
  void currentConnectedBluetoothDevices(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void getBLEBatteryLevelHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void getBLEFirmwareVersionHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void deleteBLEHistoryHandle(
    Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void bleFirmwareUpdateNotificationCB(const std_msgs::msg::String::SharedPtr msg);
  void updateBLEFirmwareHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void bleDFUProgressCB(const protocol::msg::BLEDFUProgress::SharedPtr msg);
  void setBTTreadHandle(Document & json_request);
  void getBTTreadHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void updateBTTreadCB(const std_msgs::msg::Int8::SharedPtr msg);

  // status reporting
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_;
  rclcpp::Subscription<protocol::msg::AlgoTaskStatus>::SharedPtr task_status_sub_;
  rclcpp::Subscription<protocol::msg::SelfCheckStatus>::SharedPtr self_check_status_sub_;
  rclcpp::Subscription<protocol::msg::StateSwitchStatus>::SharedPtr state_switch_status_sub_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bmd_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr audio_playing_sub_;
  void motionStatusCB(const protocol::msg::MotionStatus::SharedPtr msg);
  void taskStatusCB(const protocol::msg::AlgoTaskStatus::SharedPtr msg);
  void selfCheckStatusCB(const protocol::msg::SelfCheckStatus::SharedPtr msg);
  void stateSwitchStatusCB(const protocol::msg::StateSwitchStatus::SharedPtr msg);
  void bmsStatusCB(const protocol::msg::BmsStatus::SharedPtr msg);
  void audioPlayingCB(const std_msgs::msg::Bool::SharedPtr msg);
  struct
  {
    int motion_id {0};
  } motion_status_;
  struct
  {
    uint8_t task_status {101};
    int task_sub_status {0};
  } task_status_;
  struct
  {
    int code {-1};
    std::string description;
  } self_check_status_;
  struct
  {
    int state {-1};
    int code {0};
  } state_switch_status_;
  struct
  {
    bool wired_charging {false};
    bool wireless_charging {false};
  } charging_status_;
  bool audio_playing_ {false};

  std::shared_mutex status_mutex_;
  void statusRequestHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  // low power dissipation
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr low_power_exit_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr auto_low_power_enable_client_;
  void lowPowerExitHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void autoLowPowerEnableHandle(
    Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  // work environment
  rclcpp::Client<protocol::srv::Trigger>::SharedPtr set_work_environment_client_;
  void setWorkEnvironmentHandle(
    Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr upload_syslog_client_;
  void uploadSyslogHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  // audio action state
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr audio_action_set_client_;

  // dog leg calibration
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr dog_leg_calibration_client_;
  void dogLegCalibrationHandle(
    Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  // test
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr app_disconnect_pub_ {nullptr};

  // stair demo
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_stair_align_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_stair_align_client_;
  void startStairAlignHandle(
    Document & json_request,
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void stopStairAlignHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  // elec skin
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_elec_skin_client_;
  rclcpp::Client<protocol::srv::ElecSkin>::SharedPtr set_elec_skin_client_;
  void enableElecSkinHandle(
    Document & json_resquest,
    ::grpcapi::RecResponse & grpc_respond,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);
  void setElecSkinHandle(
    Document & json_resquest,
    ::grpcapi::RecResponse & grpc_respond,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  // stop audio
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_audio_client_;
  void stopAudioHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  // power off
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_client_;
  void powerOffHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lcm_log_upload_client_;
  void lcmLogHandle(
    ::grpcapi::RecResponse & grpc_response,
    ::grpc::ServerWriter<::grpcapi::RecResponse> * grpc_writer);

  std::unique_ptr<ReadySnNode> ready_sn_ptr {nullptr};

  // tls certificate
  std::string pem_root_certs_, pem_server_key_, pem_server_certs_;

  LOGGER_MINOR_INSTANCE("Cyberdog_app");
};
}  // namespace bridges
}  // namespace cyberdog

#endif  // CYBERDOG_APP_HPP_
