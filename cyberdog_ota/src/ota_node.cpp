#include "cyberdog_ota/ota_node.hpp"

namespace cyberdog
{

const std::string kNodeName = "cyberdog_ota_derver";

OTANode::OTANode()
: rclcpp::Node(kNodeName)
{
  manger_ = std::make_shared<Manager>();
  initialized_ = true;
}

OTANode::~OTANode()
{

}

bool OTANode::Finished()
{
  return true;
}

void OTANode::PublishState()
{

}

}  // namespace cyberdog