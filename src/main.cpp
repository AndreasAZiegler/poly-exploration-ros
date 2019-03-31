/* 
 */

#include "PolygonExplorerNode.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executer;

  auto polygon_exploration_node = std::make_shared<PolygonExplorerNode>();
  executer.add_node(polygon_exploration_node);
  executer.spin();
  rclcpp::shutdown();
  return 0;
}
