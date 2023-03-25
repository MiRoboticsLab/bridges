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

#include "cyberdog_app_client.hpp"

#include <memory>
#include <string>
#include <vector>
using grpcapi::Result;
using grpcapi::Ticks;
using std::placeholders::_1;
Cyberdog_App_Client::Cyberdog_App_Client(std::shared_ptr<grpc::Channel> channel)
: stub_(grpcapi::GrpcApp::NewStub(channel)) {}
Cyberdog_App_Client::~Cyberdog_App_Client() {}

bool Cyberdog_App_Client::sendRequest(const ::grpcapi::SendRequest & msg)
{
  grpc::ClientContext context;
  context.set_deadline(
    std::chrono::system_clock::now() +
    std::chrono::duration_cast<std::chrono::seconds>(std::chrono::seconds(3)));
  ::grpcapi::RecResponse rsp;
  std::unique_ptr<grpc::ClientReader<::grpcapi::RecResponse>> reader(
    stub_->sendMsg(&context, msg));

  if (!reader) {
    ERROR("sendMsg returns empty pointer!");
    return false;
  }
  while (reader->Read(&rsp)) {
  }

  Status status = reader->Finish();

  if (!status.ok()) {
    ERROR("sendMsg error code:%d", status.error_code());
    return false;
  }
  if (msg.namecode() == grpcapi::SendRequest::MAP_DATA_REQUEST) {
    INFO_STREAM(
      "sendMsg rpc success. namecode: " << msg.namecode() << " params: " <<
        std::string("map data ... json string size: ") << msg.params().size());
  } else if (msg.namecode() == grpcapi::SendRequest::NAV_PLAN_PATH) {
    INFO_STREAM(
      "sendMsg rpc success. namecode: " << msg.namecode() << " params: " <<
        std::string("path data ... json string size: ") << msg.params().size());
  } else {
    INFO_STREAM(
      "sendMsg rpc success. namecode: " << msg.namecode() << " params: " << msg.params());
  }
  return true;
}
bool Cyberdog_App_Client::sendHeartBeat(
  const std::string & ip, int32_t wstrength,
  int32_t battery, bool internet, const std::string & sn,
  int motion_id, uint8_t task_status, int task_sub_status,
  int self_check_code, const std::string & description,
  int state_switch_state, int state_switch_code,
  bool wired_charging, bool wireless_charging)
{
  ClientContext context;
  context.set_deadline(
    std::chrono::system_clock::now() +
    std::chrono::duration_cast<std::chrono::seconds>(std::chrono::seconds(1)));
  Result result;
  Ticks ticks_;
  ticks_.set_ip(ip);
  ticks_.set_wifi_strength(wstrength);
  ticks_.set_battery_power(battery);
  ticks_.set_internet(internet);
  ticks_.set_sn(sn);
  grpcapi::MotionStatus * motion_status = new grpcapi::MotionStatus;
  motion_status->set_motion_id(motion_id);
  ticks_.set_allocated_motion_status(motion_status);
  grpcapi::TaskStatus * alg_task_status = new grpcapi::TaskStatus;
  alg_task_status->set_task_status(task_status);
  alg_task_status->set_task_sub_status(task_sub_status);
  ticks_.set_allocated_task_status(alg_task_status);
  grpcapi::SelfCheckStatus * self_check_status = new grpcapi::SelfCheckStatus;
  self_check_status->set_code(self_check_code);
  self_check_status->set_description(description);
  ticks_.set_allocated_self_check_status(self_check_status);
  grpcapi::StateSwitchStatus * state_switch_status = new grpcapi::StateSwitchStatus;
  state_switch_status->set_state(state_switch_state);
  state_switch_status->set_code(state_switch_code);
  ticks_.set_allocated_state_switch_status(state_switch_status);
  grpcapi::ChargingStatus * charging_status = new grpcapi::ChargingStatus;
  charging_status->set_wired_charging(wired_charging);
  charging_status->set_wireless_charging(wireless_charging);
  ticks_.set_allocated_charging_status(charging_status);
  Status status = stub_->heartbeat(&context, ticks_, &result);
  if (!status.ok()) {
    INFO("SetHeartBeat error code:%d", status.error_code());
    return false;
  }
  INFO_MILLSECONDS(
    2000, "SetHeartBeat rpc success, %s,%d,%d,%d,%s, status: %d,%d,%d,%d,%d,%d,%d,%d",
    ip.c_str(), wstrength, battery, internet, sn.c_str(),
    motion_id, task_status, task_sub_status, self_check_code,
    state_switch_state, state_switch_code, wired_charging,
    wireless_charging);
  return true;
}
