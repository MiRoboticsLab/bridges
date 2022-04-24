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

#include "cyberdog_app_client.hpp"
#include <memory>
#include <string>
#include <vector>
using grpcapi::Result;
using grpcapi::Ticks;
using std::placeholders::_1;
Cyberdog_App_Client::Cyberdog_App_Client(std::shared_ptr<grpc::Channel> channel)
: stub_(grpcapi::GrpcApp::NewStub(channel))
{
}
Cyberdog_App_Client::~Cyberdog_App_Client()
{
}

bool Cyberdog_App_Client::sendRequest(const ::grpcapi::SendRequest & msg)
{
  grpc::ClientContext context;
  // gpr_timespec timespec;
  // timespec.tv_sec = 2;
  // timespec.tv_nsec = 0;
  // timespec.clock_type = GPR_TIMESPAN;
  // context.set_deadline(timespec);
  ::grpcapi::RecResponse rsp;
  std::unique_ptr<grpc::ClientReader<::grpcapi::RecResponse>> reader(
    stub_->sendMsg(&context, msg));

  while (reader->Read(&rsp)) {}

  Status status = reader->Finish();

  if (!status.ok()) {
    INFO("sendMsg error code:%d", status.error_code());
    return false;
  }
  INFO("sendMsg rpc success.");
  return true;
}
bool Cyberdog_App_Client::sendHeartBeat(const std::string & ip)
{
  ClientContext context;
  gpr_timespec timespec;
  timespec.tv_sec = 2;
  timespec.tv_nsec = 0;
  timespec.clock_type = GPR_TIMESPAN;
  context.set_deadline(timespec);
  Result result;
  Ticks ticks_;
  ticks_.set_ip(ip);
  Status status = stub_->heartbeat(&context, ticks_, &result);
  if (!status.ok()) {
    INFO("SetHeartBeat error code:%d", status.error_code());
    return false;
  }
  INFO("SetHeartBeat rpc success.");
  return true;
}
