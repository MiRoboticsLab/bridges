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

#include "cyberdog_app_server.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

#include <string>
#include <memory>

CyberdogAppImpl::CyberdogAppImpl(const std::string & db)
: decision_(NULL)
{
}

bool CyberdogAppImpl::isPeerAvalible(std::string peer)
{
  if (decision_ == NULL) {
    return false;
  }
  // std::cout << "peer:" << peer << "self_ip:" << decision_->getServiceIp() << std::endl;
  return true;
}

::grpc::Status CyberdogAppImpl::sendMsg(
  ::grpc::ServerContext * context,
  const ::grpcapi::SendRequest * request,
  ::grpc::ServerWriter<::grpcapi::RecResponse> * writer)
{
  if (isPeerAvalible(context->peer())) {
    decision_->ProcessMsg(request, writer);
  }
  INFO("time ------------------------------");
  return ::grpc::Status::OK;
}

::grpc::Status CyberdogAppImpl::getFile(
  ::grpc::ServerContext * context,
  const ::grpcapi::SendRequest * request,
  ::grpc::ServerWriter<::grpcapi::FileChunk> * writer)
{
  if (isPeerAvalible(context->peer())) {
    decision_->ProcessGetFile(request, writer);
  }
  return ::grpc::Status::OK;
}
