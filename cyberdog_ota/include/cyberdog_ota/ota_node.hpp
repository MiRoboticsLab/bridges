#ifndef CYBERDOG_OTA_OTA_NODE_HPP_
#define CYBERDOG_OTA_OTA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cyberdog_ota/ota_options.hpp"
#include "cyberdog_ota/manager.hpp"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "protocol/srv/ota_server_cmd.hpp"
#include "cyberdog_common/cyberdog_json.hpp"


#include <memory>
#include <string>
#include <vector>

namespace cyberdog {

class OTANode : public rclcpp::Node
{
public:
  OTANode();
  ~OTANode();

  OTANode(const OTANode&) = delete;
  OTANode& operator=(const OTANode&) = delete;

  bool Finished();

private:
  void HandleOTAGrpcCommand(
    const std::shared_ptr<protocol::srv::OtaServerCmd::Request> request,
    std::shared_ptr<protocol::srv::OtaServerCmd::Response>  response);

  void PublishState();
  std::shared_ptr<Manager> manger_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Service<protocol::srv::OtaServerCmd>::SharedPtr grpc_server_ {nullptr};   

  bool initialized_ {false};

};
 
}  // cyberdog

#endif  // CYBERDOG_OTA_OTA_NODE_HPP_
