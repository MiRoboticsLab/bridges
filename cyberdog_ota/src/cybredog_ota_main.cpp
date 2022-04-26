#include "cyberdog_ota/ota_node.hpp"

#include <memory>
#include <string>
#include <vector>

namespace cyberdog
{

void Run()
{
  auto node = std::make_shared<OTANode>();
  rclcpp::spin(node);
  node->Finished();
}

} // namespace cyberdog

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  cyberdog::Run();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
