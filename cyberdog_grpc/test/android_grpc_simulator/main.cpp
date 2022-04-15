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

#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <thread>
#include <string>
#include <memory>
#include <utility>
#include "cyberdog_grpc/cyberdog_app.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using grpc::Status;
using std::chrono::system_clock;
using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;

class AndroidAppImpl;
class Android_App_Client;

// class Android_app
// {
// public:
//   Android_app()
//   {
//   }
//   void createGrpc()
//   {
//     // printf("Create server");
//     // if (server_ == nullptr) {
//     //   app_server_thread_ = std::make_shared<std::thread>(&Cyberdog_app::RunServer, this);
//     // }
//     // printf("Create client\n");
//     // grpc::string ip = *server_ip + std::string(":8980");
//     // can_process_messages = false;
//     // heartbeat_err_cnt = 0;
//     // net_checker.set_ip(*server_ip);
//     // printf("before channel\n");
//     // auto channel_ = grpc::CreateChannel(ip, grpc::InsecureChannelCredentials());
//     // printf("after channel\n");
//     // app_stub = std::make_shared<Cyberdog_App_Client>(channel_);
//     // printf("end channel\n");
//     // can_process_messages = true;
//     // if (app_disconnected) {
//     //   destroyGrpc();
//     // }
//   }
//   void destroyGrpc()
//   {
//     can_process_messages = false;
//     if (app_stub != nullptr) {
//       app_stub = nullptr;
//     }
//   }
// private:
//   bool can_process_messages;
//   std::shared_ptr<Android_App_Client> app_stub;
//   std::shared_ptr<grpc::Server> server_;
// };

class AndroidAppImpl final : public grpcapi::GrpcApp::Service
{
public:
  explicit AndroidAppImpl(const std::string & db)
  {
  }
  // void SetRequesProcess(Android_app * decision)
  // {
  //   decision_ = decision;
  // }

  ::grpc::Status heartbeat(
    ::grpc::ServerContext * context,
    const ::grpcapi::Ticks * request,
    ::grpcapi::Result * response) override
  {
    // if (isPeerAvalible(context->peer())) {
    printf("Heartbeat %s\n", request->ip().c_str());
    // }
    return ::grpc::Status::OK;
  }

private:
  // bool isPeerAvalible(std::string peer)
  // {
  //   if (decision_ == NULL) {
  //       return false;
  //   }
  //   return true;
  // }
  // Android_app * decision_;
};

class Android_App_Client
{
public:
  explicit Android_App_Client(std::shared_ptr<Channel> channel)
  : stub_(grpcapi::GrpcApp::NewStub(channel))
  {
  }
  bool SendMessage()
  {
    ClientContext context;
    gpr_timespec timespec;
    timespec.tv_sec = 1;
    timespec.tv_nsec = 0;
    timespec.clock_type = GPR_TIMESPAN;
    context.set_deadline(timespec);
    grpcapi::RecResponse respose;
    grpcapi::SendRequest request;
    request.set_namecode(grpcapi::SendRequest::GET_DEVICE_INFO);
    request.set_params("{\"a\": 1}");
    auto uptr_reader = stub_->sendMsg(&context, request);
    while (uptr_reader->Read(&respose)) {
      printf(
        "SendMessage rpc respose(nameCode:%d,data:%s).\n", respose.namecode(),
        respose.data().c_str());
    }
    return true;
  }

private:
  std::unique_ptr<grpcapi::GrpcApp::Stub> stub_;
  std::shared_ptr<grpc::Channel> channel_;
};


int main(int argc, char ** argv)
{
  std::thread sever_thread([]() {
      std::string server_address("0.0.0.0:8980");
      // Android_app ap;
      AndroidAppImpl service(server_address);
      // service.SetRequesProcess(&ap);
      ServerBuilder builder;
      printf("before listening\n");
      builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
      builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS, 1000);
      builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 1000);
      builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS, 500);
      builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_SENT_PING_INTERVAL_WITHOUT_DATA_MS, 1000);
      builder.RegisterService(&service);
      std::shared_ptr<grpc::Server> server_ = std::move(builder.BuildAndStart());
      server_->Wait();
      printf("after wait\n");
    });
  std::thread client_thread([]() {
      std::string ip("0.0.0.0:50051");
      auto channel_ = grpc::CreateChannel(ip, grpc::InsecureChannelCredentials());
      Android_App_Client client(channel_);
      while (true) {
        client.SendMessage();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });
  sever_thread.join();
  client_thread.join();
  return 0;
}
