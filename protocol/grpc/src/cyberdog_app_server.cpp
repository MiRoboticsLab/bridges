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
  std::cout << "peer:" << peer << "self_ip:" << decision_->getServiceIp() << std::endl;
  return true;
  //  (peer.find(decision_->getServiceIp()) != string::npos);
}
::grpc::Status CyberdogAppImpl::pingDog(::grpc::ServerContext* context, const ::cyberdogapp::Result* request, ::grpc::ServerWriter< ::cyberdogapp::DogInfo>* writer)
{
  ::cyberdogapp::DogInfo respond;
  if (isPeerAvalible(context->peer())) {
    decision_->pingDog(request, &respond);
    writer->Write(respond);
  }
  return Status::OK;
}
::grpc::Status CyberdogAppImpl::findDog(::grpc::ServerContext* context, const ::cyberdogapp::Result* request, ::cyberdogapp::Result* response)
{
  if (isPeerAvalible(context->peer())) {
    decision_->findDog();
  }
  return Status::OK;
}
::grpc::Status CyberdogAppImpl::connectDog(::grpc::ServerContext* context, const ::cyberdogapp::ConnectRequest* request, ::grpc::ServerWriter< ::cyberdogapp::ConnectResult>* writer)
{
  ::cyberdogapp::ConnectResult respond;
  if (isPeerAvalible(context->peer())) {
    decision_->connectDog(request, &respond);
    writer->Write(respond);
  }
  return Status::OK;
}

