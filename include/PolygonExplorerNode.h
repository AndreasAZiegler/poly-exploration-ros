/* Polygon Explorer ROS class
 */

#pragma once

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "kindr/Core"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "poly_exploration/PolygonExplorer.h"
#include "poly_exploration/PolygonExplorerInterface.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class PolygonExplorerNode : public rclcpp::Node,
                            public PolygonExplorerInterface {
 public:
  PolygonExplorerNode();

  virtual void updateVisualizationCallback(const PoseGraph pose_graph,
                                           const TimeStamp time_stamp) override;

 private:
  void subscriberCallback(
      const std::shared_ptr<nav_msgs::msg::Odometry>& odometry,
      const std::shared_ptr<sensor_msgs::msg::LaserScan>& laser_scan);

  void convertFromRosGeometryMsg(
      const geometry_msgs::msg::PoseWithCovariance& geometryPoseMsg,
      Pose& pose);

  void convertFromRosGeometryMsg(
      const geometry_msgs::msg::Quaternion& geometryQuaternionMsg,
      Rotation& rotationQuaternion);

  void convertFromRosGeometryMsg(
      const geometry_msgs::msg::Point& geometryPointMsg, Position& position);

  PolygonExplorer polygonExplorer_;

  Pose previousPose_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometrySubscription_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laserSubscription_;

  message_filters::TimeSynchronizer<nav_msgs::msg::Odometry,
                                    sensor_msgs::msg::LaserScan>
      timeSynchronizer_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      poseGraphVisualizationPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      polygonVisualizationPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      polygonPointsVisualizationPublisher_;
};
