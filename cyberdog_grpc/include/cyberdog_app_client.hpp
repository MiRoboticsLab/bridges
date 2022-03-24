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
#include "msgdispatcher.hpp"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;
class Cyberdog_App_Client
{
public:
  explicit Cyberdog_App_Client(std::shared_ptr<Channel> channel);
  ~Cyberdog_App_Client();
  bool SetHeartBeat(std::string ip, int wifi_strength, int battery_soc);

private:
  std::unique_ptr<grpcapi::GrpcApp::Stub> stub_;
  std::shared_ptr<grpc::Channel> channel_;
};

#endif  // CYBERDOG_APP_CLIENT_HPP_