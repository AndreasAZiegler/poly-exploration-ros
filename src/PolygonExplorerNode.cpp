/* Polygon Explorer ROS class
 */

#include "PolygonExplorerNode.h"
#include <glog/logging.h>
#include <functional>

PolygonExplorerNode::PolygonExplorerNode()
    : Node("polygon_explorer_node"),
      odometrySubscription_(
          message_filters::Subscriber<nav_msgs::msg::Odometry>(this,
                                                               "odom_11_odom")),
      laserSubscription_(
          message_filters::Subscriber<sensor_msgs::msg::LaserScan>(
              this, "scan_11_laser_front")),
      timeSynchronizer_(odometrySubscription_, laserSubscription_, 1) {
  polygonExplorer_.setCallBack(this);
  timeSynchronizer_.registerCallback(&PolygonExplorerNode::subscriberCallback,
                                     this);
} /* -----  end of method PolygonExplorerNode::PolygonExplorerNode
     (constructor)  ----- */

void PolygonExplorerNode::subscriberCallback(
    const std::shared_ptr<nav_msgs::msg::Odometry>& odometry,
    const std::shared_ptr<sensor_msgs::msg::LaserScan>& laser_scan) {
  std::vector<PolygonPoint> polygon_points;
  for (unsigned int i = 0; i < laser_scan->ranges.size(); ++i) {
    double theta = laser_scan->angle_min + i * laser_scan->angle_increment;
    double x = laser_scan->ranges.at(i) * cos(theta);
    double y = laser_scan->ranges.at(i) * sin(theta);

    PointType point_type;
    if (laser_scan->ranges.at(i) >= laser_scan->range_max) {
      point_type = PointType::MAX_RANGE;
    } else {
      point_type = PointType::OBSTACLE;
    }

    polygon_points.emplace_back(x, y, point_type);
  }

  Polygon polygon(polygon_points);

  Pose current_pose;
  convertFromRosGeometryMsg(odometry->pose, current_pose);
  auto position_diff = current_pose.getPosition() - previousPose_.getPosition();
  // Orientation difference of quaternions
  // (http://www.boris-belousov.net/2016/12/01/quat-dist/)
  auto orientation_diff =
      current_pose.getRotation() * previousPose_.getRotation().conjugated();
  // Transform transformation into frame of previous pose
  auto transformation_previous_pose_current_pose =
      Pose(previousPose_.transform(position_diff),
           previousPose_.getRotation() * orientation_diff);

  polygonExplorer_.addPose(transformation_previous_pose_current_pose, polygon);

  previousPose_ = current_pose;
}

void PolygonExplorerNode::convertFromRosGeometryMsg(
    const geometry_msgs::msg::PoseWithCovariance& geometryPoseMsg, Pose& pose) {
  convertFromRosGeometryMsg(geometryPoseMsg.pose.position, pose.getPosition());

  // This is the definition of ROS Geometry pose.
  typedef kindr::RotationQuaternion<double> RotationQuaternionGeometryPoseLike;

  RotationQuaternionGeometryPoseLike rotation;
  convertFromRosGeometryMsg(geometryPoseMsg.pose.orientation, rotation);
  pose.getRotation() = rotation;
}

void PolygonExplorerNode::convertFromRosGeometryMsg(
    const geometry_msgs::msg::Quaternion& geometryQuaternionMsg,
    Rotation& rotationQuaternion) {
  rotationQuaternion.setValues(geometryQuaternionMsg.w, geometryQuaternionMsg.x,
                               geometryQuaternionMsg.y,
                               geometryQuaternionMsg.z);
}

void PolygonExplorerNode::convertFromRosGeometryMsg(
    const geometry_msgs::msg::Point& geometryPointMsg, Position& position) {
  position.x() = geometryPointMsg.x;
  position.y() = geometryPointMsg.y;
  position.z() = geometryPointMsg.z;
}

void PolygonExplorerNode::updateVisualizationCallback(
    const PoseGraph pose_graph) {
  CHECK(!pose_graph.getPoseGraphPoses().empty())
      << "There should be at least one pose in the pose graph!";

  visualization_msgs::msg::Marker pose_graph_marker;
  pose_graph_marker.header.frame_id = "world";
  pose_graph_marker.header.stamp = now();
  pose_graph_marker.ns = "pose_graph";
  pose_graph_marker.frame_locked = true;
  pose_graph_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  pose_graph_marker.action = visualization_msgs::msg::Marker::ADD;
  pose_graph_marker.color.r = 0;
  pose_graph_marker.color.g = 1;
  pose_graph_marker.color.b = 0;
  pose_graph_marker.color.a = 1;

  pose_graph_marker.pose.position.x = 0;
  pose_graph_marker.pose.position.y = 0;
  pose_graph_marker.pose.position.z = 0;
  pose_graph_marker.pose.orientation.x = 0;
  pose_graph_marker.pose.orientation.y = 0;
  pose_graph_marker.pose.orientation.z = 0;
  pose_graph_marker.pose.orientation.w = 1;
  pose_graph_marker.scale.x = 0.1;

  pose_graph_marker.points.clear();

  std::map<unsigned int, Pose> pose_graph_transformations;

  std::queue<unsigned int> pose_ids_to_check;

  pose_ids_to_check.emplace(0);
  pose_graph_transformations[0] = Pose();

  while (!pose_ids_to_check.empty()) {
    auto id_to_check = pose_ids_to_check.front();

    auto adjacent_ids =
        pose_graph.getPoseGraphPose(id_to_check).getAdjacentPosesId();
    auto adjacnet_ids_pose =
        pose_graph.getPoseGraphPose(id_to_check).getAdjacentPoses();
    for (const auto& adjacent_id : adjacent_ids) {
      auto check_to_adjacent_transformation = adjacnet_ids_pose[adjacent_id];
      auto origin_to_adjacent_transformation =
          pose_graph_transformations[id_to_check] *
          check_to_adjacent_transformation;

      pose_ids_to_check.emplace(adjacent_id);
      pose_graph_transformations[adjacent_id] =
          origin_to_adjacent_transformation;

      geometry_msgs::msg::Point point_1;
      point_1.x = pose_graph_transformations[id_to_check].getPosition().x();
      point_1.y = pose_graph_transformations[id_to_check].getPosition().y();
      point_1.z = 0;
      pose_graph_marker.points.push_back(point_1);

      geometry_msgs::msg::Point point_2;
      point_2.x = origin_to_adjacent_transformation.getPosition().x();
      point_2.y = origin_to_adjacent_transformation.getPosition().y();
      point_2.z = 0;
      pose_graph_marker.points.push_back(point_2);
    }

    pose_ids_to_check.pop();
  }

  poseGraphVisualizationPublisher_->publish(pose_graph_marker);
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(PolygonExplorerNode, rclcpp::Node)
