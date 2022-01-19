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


#ifndef CYBERDOG_APP_SERVER_HPP_
#define CYBERDOG_APP_SERVER_HPP_

#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "./cyberdog_app.grpc.pb.h"
#include "cyberdog_app.hpp"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using grpc::Status;
using std::chrono::system_clock;

class CyberdogAppImpl final : public cyberdogapp::CyberdogApp::Service
{
public:
  explicit CyberdogAppImpl(const std::string & db);
  void SetRequesProcess(athena_cyberdog_app::Cyberdog_app * decision)
  {
    decision_ = decision;
  }
 ::grpc::Status pingDog(::grpc::ServerContext* context, const ::cyberdogapp::Result* request, ::grpc::ServerWriter< ::cyberdogapp::DogInfo>* writer)
 override;
 ::grpc::Status findDog(::grpc::ServerContext* context, const ::cyberdogapp::Result* request, ::cyberdogapp::Result* response)
 override;
 ::grpc::Status connectDog(::grpc::ServerContext* context, const ::cyberdogapp::ConnectRequest* request, ::grpc::ServerWriter< ::cyberdogapp::ConnectResult>* writer)
override;
private:
  bool isPeerAvalible(std::string peer);
  athena_cyberdog_app::Cyberdog_app * decision_;
};

#endif  // CYBERDOG_APP_SERVER_HPP_
