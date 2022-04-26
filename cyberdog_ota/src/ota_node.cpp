#include "cyberdog_ota/ota_node.hpp"

#include <functional>

namespace cyberdog
{

const std::string kNodeName = "cyberdog_ota_derver";

OTANode::OTANode()
: rclcpp::Node(kNodeName)
{

  grpc_server_ = this->create_service<protocol::srv::OtaServerCmd>(
    kOTAGrpcServerName, std::bind(&OTANode::HandleOTAGrpcCommand, this,
      std::placeholders::_1, std::placeholders::_2));

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
  // # request.
  // OtaCmd request
          // string key
          // string value
          // string type

  // ---
  // # response
  // OtaCmd response
  if (request->request.key == kOTACommandStatusQuery) {
    std::string status;
    bool ok = manger_->RunStatusQueryCommand(status);
    if (ok) {
      response->response.key = kOTACommandStatusQuery;
      response->response.type = "STRING";
      response->response.value = status;
    }
  } else if (request->request.key == kOTACommandVersionQuery) { 
      std::string version;
      bool ok = manger_->RunVersionQueryCommand(version);
      if (ok) {
        response->response.key = kOTACommandStatusQuery;
        response->response.type = "STRING";
        response->response.value = version;
      }

  } else if (request->request.key == kOTACommandProcessQuery) {
    std::string status;
    bool ok = manger_->RunProcessQueryCommand(status);
    if (ok) {
      response->response.key = kOTACommandProcessQuery;
      response->response.type = "JSON";
      response->response.value = status;
    }
  } else if (request->request.key == kOTACommandStartUpgrade) {
    std::string status;
    bool ok = manger_->RunStartUpgradeCommand(status);
    if (ok) {
      response->response.key = kOTACommandStartUpgrade;
      response->response.type = "string";
      response->response.value = status;
    }
  } else if (request->request.key == kOTACommandStartDownload) {
    std::string status;
    bool ok = manger_->RunStartDownloadCommand(status);
    if (ok) {
      response->response.key = kOTACommandStartDownload;
      response->response.type = "STRING";
      response->response.value = status;
    }
  } else if (request->request.key == kOTACommandEstimateUpgradeTimeQuery) {
    std::string status;
    bool ok = manger_->RunEstimateUpgradeTimeQueryCommand(status);
    if (ok) {
      response->response.key = kOTACommandEstimateUpgradeTimeQuery;
      response->response.type = "STRING";
      response->response.value = status;
    }
  }
}

bool OTANode::Finished()
{
  return true;
}

void OTANode::PublishState()
{

}

}  // namespace cyberdog