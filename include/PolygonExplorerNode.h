/* Polygon Explorer ROS class
 */

#pragma once

#include "message_filters/time_synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "poly_exploration/PolygonExplorer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "poly_exploration/PolygonExplorer.h"
#include "poly_exploration/PolygonExplorerInterface.h"

class PolygonExplorerNode : public rclcpp::Node, public PolygonExplorerInterface
{
public:
  PolygonExplorerNode();

  virtual void updateVisualizationCallback(const PoseGraph) override;

private:
  void subscriberCallback(const nav_msgs::msg::Odometry::ConstPtr& odometry,
                          const sensor_msgs::msg::LaserScan::ConstPtr& laser_scan);

  PolygonExplorer polygonExplorer_;

  message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan> timeSynchronizer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSubscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr posesVisualizationPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr polygonVisualizationPublisher_;
};
