#ifndef CYBERDOG_OTA_OTA_NODE_HPP_
#define CYBERDOG_OTA_OTA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cyberdog_ota/ota_options.hpp"
#include "cyberdog_ota/manager.hpp"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "protocol/srv/ota_server_cmd.hpp"
#include "protocol/msg/ota_ready.hpp"
#include "protocol/msg/bms_status.hpp"
#include "cyberdog_common/cyberdog_json.hpp"


#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>

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
  // Accept commands from the app
  void HandleOTAGrpcCommand(
    const std::shared_ptr<protocol::srv::OtaServerCmd::Request> request,
    std::shared_ptr<protocol::srv::OtaServerCmd::Response>  response);

  // Handle the signal that each module is ready for OTA
  void HandleAllWaitOtaReadyModuleMessages(const protocol::msg::OtaReady::SharedPtr msg);

  // Accept battery level information
  void HandleBmsStatusMessages(const protocol::msg::BmsStatus::SharedPtr msg);

  // Initialize all modules etc. in OTA
  void InitializeAllWaitOtaReadyModule();

  // Check that the battery level meets the upgrade conditions
  bool CheckBatteryPowerConditions();

  // Check if all modules are ready
  bool CheckAllReady();

  std::shared_ptr<Manager> manger_;

  // signal for ota
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ota_ready_publisher_;
  rclcpp::Subscription<protocol::msg::OtaReady>::SharedPtr ota_ready_sub_;

  // bms status
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Service<protocol::srv::OtaServerCmd>::SharedPtr grpc_server_ {nullptr};  

  std::mutex mutex_;
  protocol::msg::BmsStatus bms_status_;
  std::map<std::string, bool> module_table_; 
  bool initialized_ {false};
};
 

}  // cyberdog

#endif  // CYBERDOG_OTA_OTA_NODE_HPP_
