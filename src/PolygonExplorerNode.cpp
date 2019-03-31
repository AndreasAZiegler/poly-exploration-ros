/* Polygon Explorer ROS class
 */

#include "PolygonExplorerNode.h"
#include <functional>

PolygonExplorerNode::PolygonExplorerNode() : Node("polygon_explorer_node"), timeSynchronizer_(1)
{
  polygonExplorer_.setCallBack(this);
  timeSynchronizer_.registerCallback(&PolygonExplorerNode::subscriberCallback, this);
} /* -----  end of method PolygonExplorerNode::PolygonExplorerNode  (constructor)  ----- */

void PolygonExplorerNode::subscriberCallback(
  const nav_msgs::msg::Odometry::ConstPtr& odometry,
  const sensor_msgs::msg::LaserScan::ConstPtr& laser_scan)
{
  // Solve all of perception here...
}

void PolygonExplorerNode::updateVisualizationCallback(const PoseGraph) { }

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(PolygonExplorerNode, rclcpp::Node)
