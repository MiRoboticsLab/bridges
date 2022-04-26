#ifndef CYBERDOG_OTA_OTA_NODE_HPP_
#define CYBERDOG_OTA_OTA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cyberdog_ota/manager.hpp"

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
  void PublishState();

  std::shared_ptr<Manager> manger_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;

  bool initialized_ {false};

};
 
}  // cyberdog

#endif  // CYBERDOG_OTA_OTA_NODE_HPP_
