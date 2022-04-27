#include "cyberdog_ota/ota_node.hpp"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "cyberdog_common/cyberdog_json.hpp"

#include <functional>
#include <utility>

namespace cyberdog
{

const std::string kNodeName = "cyberdog_ota_derver";

using rapidjson::Document;
using rapidjson::kObjectType;

OTANode::OTANode()
: rclcpp::Node(kNodeName)
{
  grpc_server_ = this->create_service<protocol::srv::OtaServerCmd>(
    kOTAGrpcServerName, std::bind(&OTANode::HandleOTAGrpcCommand, this,
      std::placeholders::_1, std::placeholders::_2));

  ota_ready_publisher_ =  this->create_publisher<std_msgs::msg::Bool>(kOTAReadyRequestTopic, 10);;
  ota_ready_sub_ = this->create_subscription<protocol::msg::OtaReady>(kOTAReadyResponseTopic ,
    rclcpp::SystemDefaultsQoS(),
      std::bind(&OTANode::HandleAllWaitOtaReadyModuleMessages, this, std::placeholders::_1));

  manger_ = std::make_shared<Manager>();
  initialized_ = true;
}

OTANode::~OTANode()
{

}

void OTANode::HandleOTAGrpcCommand(
    const std::shared_ptr<protocol::srv::OtaServerCmd::Request> request,
    std::shared_ptr<protocol::srv::OtaServerCmd::Response> response)
{
  
  if (request->request.key == kOTACommandStatusQuery) {
    std::string status;
    Document json_response(kObjectType);
    bool ok = manger_->RunStatusQueryCommand(status);
    if (ok) {
      response->response.key = kOTACommandStatusQuery;
      response->response.type = "JSON";
      response->response.value = status;
    }
  } else if (request->request.key == kOTACommandVersionQuery) { 
      std::string version;
      Document json_response(kObjectType);
      bool ok = manger_->RunVersionQueryCommand(version);
      if (ok) {
        response->response.key = kOTACommandStatusQuery;
        response->response.type = "JSON";
        response->response.value = version;
      }
  } else if (request->request.key == kOTACommandProcessQuery) {
    std::string status;
    Document json_response(kObjectType);
    bool ok = manger_->RunProcessQueryCommand(status);
    if (ok) {
      response->response.key = kOTACommandProcessQuery;
      response->response.type = "JSON";
      response->response.value = status;
    }
  } else if (request->request.key == kOTACommandStartUpgrade) {
    std::string status;
    Document json_response(kObjectType);
    bool ok = manger_->RunStartUpgradeCommand(status);
    if (ok) {
      response->response.key = kOTACommandStartUpgrade;
      response->response.type = "JSON";
      response->response.value = status;
    }
  } else if (request->request.key == kOTACommandStartDownload) {
    std::string status;
    Document json_response(kObjectType);
    bool ok = manger_->RunStartDownloadCommand(status);
    if (ok) {
      response->response.key = kOTACommandStartDownload;
      response->response.type = "JSON";
      response->response.value = status;
    }
  } else if (request->request.key == kOTACommandEstimateUpgradeTimeQuery) {
    std::string status;
    Document json_response(kObjectType);
    bool ok = manger_->RunEstimateUpgradeTimeQueryCommand(status);
    if (ok) {
      response->response.key = kOTACommandEstimateUpgradeTimeQuery;
      response->response.type = "JSON";
      response->response.value = status;
    }
  }
}

void OTANode::HandleAllWaitOtaReadyModuleMessages(const protocol::msg::OtaReady::SharedPtr msg)
{
  if (msg->name == "motion" && msg->ready) {
    module_table_[msg->name] = true;
  } else if (msg->name == "devices") {
    module_table_[msg->name] = true;
  } else if (msg->name == "sensors") {
    module_table_[msg->name] = true;
  } else if (msg->name == "manager") {
    module_table_[msg->name] = true;
  }
}


void OTANode::InitializeAllWaitOtaReadyModule()
{
  module_table_.emplace(std::make_pair("motion", false));
  module_table_.emplace(std::make_pair("devices", false));
  module_table_.emplace(std::make_pair("sensors", false));
  module_table_.emplace(std::make_pair("manager", false));
}

bool OTANode::CheckAllReady()
{
  for (auto module : module_table_) {
    if (!module.second) {
      return false;
    }
  }
  return true;
}

bool OTANode::Finished()
{
  return true;
}

void OTANode::PublishState()
{

}

}  // namespace cyberdog