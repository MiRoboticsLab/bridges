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


#ifndef CYBERDOG_APP_CLIENT_HPP_
#define CYBERDOG_APP_CLIENT_HPP_
#include "./cyberdog_app.grpc.pb.h"
#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include "std_msgs/msg/string.hpp"
#include "msg_dispatcher.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;
/**
 * @brief Interface for gRPC client
 */
class Cyberdog_App_Client
{
public:
  /**
   * @brief Construct a new Cyberdog_App_Client
   * @param channel stub channel pointer
   */
  explicit Cyberdog_App_Client(std::shared_ptr<Channel> channel);
  /**
   * @brief Destroy the Cyberdog_App_Client
   */
  ~Cyberdog_App_Client();
  /**
   * @brief Send heartbeat message
   * @param ip Local IP of the robot
   * @param wstrength WiFi signal strength
   * @param battery Battery percentage
   * @param internet If internet connection is available
   * @param sn SN of the robot
   * @param motion_id Current motion ID
   * @param task_status Current task status
   * @param task_sub_status Substatus of task_status
   * @param self_check_code Current self-check status
   * @param description Description of self-check status (json string)
   * @param state_switch_state Current state machine status
   * @param state_switch_code Error code for state machine
   * @param wired_charging If the doge is wired charging
   * @param wireless_charging If the doge is wireless charging
   * @return true Successfully sent heartbeat message
   * @return false Failed to send heartbeat message
   */
  bool sendHeartBeat(
    const std::string & ip, int32_t wstrength,
    int32_t battery, bool internet, const std::string & sn,
    int motion_id, uint8_t task_status, int task_sub_status,
    int self_check_code, const std::string & description,
    int state_switch_state, int state_switch_code,
    bool wired_charging, bool wireless_charging, bool audio_playing);
  /**
   * @brief Call sendMsg service of gRPC
   * @param msg Request of sendMsg
   * @return true Successfully sent sendMsg message
   * @return false Failed to send sendMsg message
   */
  bool sendRequest(const ::grpcapi::SendRequest & msg);

private:
  std::unique_ptr<grpcapi::GrpcApp::Stub> stub_;
  std::shared_ptr<grpc::Channel> channel_;
};

#endif  // CYBERDOG_APP_CLIENT_HPP_
