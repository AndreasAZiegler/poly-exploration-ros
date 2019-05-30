/* Polygon Explorer ROS class
 */

#include "PolygonExplorerNode.h"
#include <glog/logging.h>
#include <cmath>
#include <functional>
#include "std_msgs/msg/header.hpp"

PolygonExplorerNode::PolygonExplorerNode()
    : Node("polygon_explorer_node"),
      previousPose_(Pose()),
      odometrySubscription_(
          message_filters::Subscriber<nav_msgs::msg::Odometry>(this, "odom")),
      laserSubscription_(
          message_filters::Subscriber<sensor_msgs::msg::LaserScan>(this,
                                                                   "scan")),
      timeSynchronizer_(odometrySubscription_, laserSubscription_, 1) {
  poseGraphVisualizationPublisher_ =
      create_publisher<visualization_msgs::msg::Marker>("posegraph");
  polygonVisualizationPublisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("polygons");
  polygonPointsVisualizationPublisher_ =
      create_publisher<visualization_msgs::msg::Marker>("polygon_points");
  polygonExplorer_.setCallBack(this);
  timeSynchronizer_.registerCallback(&PolygonExplorerNode::subscriberCallback,
                                     this);
} /* -----  end of method PolygonExplorerNode::PolygonExplorerNode
     (constructor)  ----- */

void PolygonExplorerNode::subscriberCallback(
    const std::shared_ptr<nav_msgs::msg::Odometry>& odometry,
    const std::shared_ptr<sensor_msgs::msg::LaserScan>& laser_scan) {

  Pose current_pose;
  convertFromRosGeometryMsg(odometry->pose, current_pose);
  auto position_diff = current_pose.getPosition() - previousPose_.getPosition();
  // std::cout << "position_diff: " << position_diff << std::endl;

  // Return (and do not update pose graph) if the robot did not move enough
  if (position_diff.norm() < 0.1) {
    return;
  }

  // Orientation difference of quaternions
  // (http://www.boris-belousov.net/2016/12/01/quat-dist/)
  auto orientation_diff =
      current_pose.getRotation() * previousPose_.getRotation().conjugated();
  // Transform transformation into frame of previous pose
  auto transformation_previous_pose_current_pose =
      Pose(previousPose_.getRotation().rotate(position_diff),
           previousPose_.getRotation() * orientation_diff);

  previousPose_ = current_pose;

  // Create polygon
  std::vector<PolygonPoint> polygon_points;
  for (unsigned int i = 0; i < 5; ++i) {
    double theta = laser_scan->angle_min;
    double range = *(laser_scan->ranges.begin()) * i / 5;
    double x = range * cos(theta);
    double y = range * sin(theta);

    PointType point_type = PointType::MAX_RANGE;
    polygon_points.emplace_back(x, y, point_type);
  }

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

    // Check if there is a possible occlusion
    if (fabs(polygon_points.back().getX() - x) > 1.0 ||
        fabs(polygon_points.back().getY() - y) > 1.0) {
      // Add MAX_RANGE point in the middle to get a frontier
      double theta = laser_scan->angle_min + i * laser_scan->angle_increment;
      double x = (laser_scan->ranges.at(i) / 2) * cos(theta);
      double y = (laser_scan->ranges.at(i) / 2) * sin(theta);

      PointType point_type = PointType::MAX_RANGE;

      polygon_points.emplace_back(x, y, point_type);
    }

    polygon_points.emplace_back(x, y, point_type);
  }

  for (unsigned int i = 0; i < 5; ++i) {
    double theta = laser_scan->angle_max;
    double range = *(laser_scan->ranges.end()) * (5 - 1 - i) / 5;
    double x = range * cos(theta);
    double y = range * sin(theta);

    PointType point_type = PointType::MAX_RANGE;
    polygon_points.emplace_back(x, y, point_type);
  }
  polygon_points.emplace_back(*polygon_points.begin());

  Polygon polygon(polygon_points);

  // std::cout << "transformation_previous_pose_current_pose: " << std::endl
  //          << transformation_previous_pose_current_pose << std::endl;
  TimeStamp time_stamp = {laser_scan->header.stamp.sec,
                          laser_scan->header.stamp.nanosec};
  polygonExplorer_.addPose(transformation_previous_pose_current_pose, polygon,
                           time_stamp);
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
    const PoseGraph pose_graph, const TimeStamp time_stamp) {
  CHECK(!pose_graph.getPoseGraphPoses().empty())
      << "There should be at least one pose in the pose graph!";

  std::cout << "Update visualization." << std::endl;

  std_msgs::msg::Header header;
  header.frame_id = "world";
  header.stamp.sec = time_stamp.sec;
  header.stamp.nanosec = time_stamp.nanosec;

  auto[pose_graph_marker, pose_graph_transformations] =
      createPoseGraphMarker(pose_graph, header);

  poseGraphVisualizationPublisher_->publish(pose_graph_marker);

  auto[polygon_markers, polygon_points_marker] =
      createPolygonMarkers(pose_graph, pose_graph_transformations, header);

  polygonVisualizationPublisher_->publish(polygon_markers);
  polygonPointsVisualizationPublisher_->publish(polygon_points_marker);

  std::cout << "Updated visualization." << std::endl;
}

std::tuple<visualization_msgs::msg::Marker, std::map<unsigned int, Pose>>
PolygonExplorerNode::createPoseGraphMarker(
    const PoseGraph& pose_graph, const std_msgs::msg::Header& header) {
  visualization_msgs::msg::Marker pose_graph_marker;
  pose_graph_marker.header = header;
  // pose_graph_marker.ns = "pose_graph";
  // Due to some reasons rviz2 crashes if frames are locked
  // pose_graph_marker.frame_locked = true;
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
  pose_graph_marker.scale.x = 0.025;
  pose_graph_marker.scale.y = 0.0;
  pose_graph_marker.scale.z = 0.0;

  pose_graph_marker.points.clear();

  std::map<unsigned int, Pose> pose_graph_transformations;

  std::queue<unsigned int> pose_ids_to_check;
  std::set<unsigned int> pose_ids_checked;

  pose_ids_to_check.emplace(0);
  pose_graph_transformations[0] = Pose();

  while (!pose_ids_to_check.empty()) {
    auto id_to_check = pose_ids_to_check.front();
    std::cout << "id_to_check: " << id_to_check << std::endl;
    pose_ids_to_check.pop();
    pose_ids_checked.emplace(id_to_check);

    auto adjacent_ids =
        pose_graph.getPoseGraphPose(id_to_check).getAdjacentPosesId();
    auto adjacent_ids_pose =
        pose_graph.getPoseGraphPose(id_to_check).getAdjacentPoses();

    for (const auto& adjacent_id : adjacent_ids) {
      // Skip origin
      if (0 == adjacent_id) {
        continue;
      }

      auto check_to_adjacent_transformation = adjacent_ids_pose[adjacent_id];
      auto origin_to_adjacent_transformation =
          pose_graph_transformations[id_to_check] *
          check_to_adjacent_transformation;

      if (0 == pose_ids_checked.count(adjacent_id)) {
        pose_ids_to_check.emplace(adjacent_id);
      }
      pose_graph_transformations[adjacent_id] =
          origin_to_adjacent_transformation;

      // std::cout << "Origin to " << id_to_check
      //          << " transformation: " << std::endl
      //          << pose_graph_transformations[id_to_check] << std::endl;
      // std::cout << "pose " << id_to_check << " to pose " << adjacent_id
      //          << " transformation: " << std::endl
      //          << check_to_adjacent_transformation << std::endl;
      // std::cout << "Origin to " << adjacent_id
      //          << " transformation: " << std::endl
      //          << origin_to_adjacent_transformation << std::endl;

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
  }

  return std::make_tuple(pose_graph_marker, pose_graph_transformations);
}

std::tuple<visualization_msgs::msg::MarkerArray,
           visualization_msgs::msg::Marker>
PolygonExplorerNode::createPolygonMarkers(
    const PoseGraph& pose_graph,
    std::map<unsigned int, Pose> pose_graph_transformations,
    const std_msgs::msg::Header& header) {
  visualization_msgs::msg::MarkerArray polygon_markers;
  polygon_markers.markers.clear();

  visualization_msgs::msg::Marker polygon_marker_template;
  polygon_marker_template.header = header;
  polygon_marker_template.header.frame_id = "world";
  // polygon_marker.ns = "pose_graph";
  // Due to some reasons rviz2 crashes if frames are locked
  // polygon_marker.frame_locked = true;
  polygon_marker_template.type = visualization_msgs::msg::Marker::LINE_STRIP;
  polygon_marker_template.action = visualization_msgs::msg::Marker::MODIFY;
  polygon_marker_template.color.r = 0.0;
  polygon_marker_template.color.g = 0.0;
  polygon_marker_template.color.b = 0.0;
  polygon_marker_template.color.a = 1.0;

  polygon_marker_template.pose.position.x = 0;
  polygon_marker_template.pose.position.y = 0;
  polygon_marker_template.pose.position.z = 0;
  polygon_marker_template.pose.orientation.x = 0;
  polygon_marker_template.pose.orientation.y = 0;
  polygon_marker_template.pose.orientation.z = 0;
  polygon_marker_template.pose.orientation.w = 1;
  polygon_marker_template.scale.x = 0.013;
  polygon_marker_template.scale.y = 0.0;
  polygon_marker_template.scale.z = 0.0;

  polygon_marker_template.points.clear();
  polygon_marker_template.colors.clear();

  visualization_msgs::msg::Marker polygon_points_marker;
  polygon_points_marker.header = header;
  // polygon_points_marker.ns = "pose_graph";
  // Due to some reasons rviz2 crashes if frames are locked
  // polygon_points_marker.frame_locked = true;
  polygon_points_marker.type = visualization_msgs::msg::Marker::POINTS;
  polygon_points_marker.action = visualization_msgs::msg::Marker::MODIFY;
  polygon_points_marker.color.r = 0;
  polygon_points_marker.color.g = 0;
  polygon_points_marker.color.b = 0;
  polygon_points_marker.color.a = 1;

  polygon_points_marker.pose.position.x = 0;
  polygon_points_marker.pose.position.y = 0;
  polygon_points_marker.pose.position.z = 0;
  polygon_points_marker.pose.orientation.x = 0;
  polygon_points_marker.pose.orientation.y = 0;
  polygon_points_marker.pose.orientation.z = 0;
  polygon_points_marker.pose.orientation.w = 1;
  polygon_points_marker.scale.x = 0.035;
  polygon_points_marker.scale.y = 0.035;
  polygon_points_marker.scale.z = 0.0;

  polygon_points_marker.points.clear();

  unsigned int id = 0;
  for (const auto& pose : pose_graph.getPoseGraphPoses()) {
    std::cout << "Pose id: " << pose.getId() << std::endl;
    visualization_msgs::msg::Marker polygon_marker = polygon_marker_template;
    polygon_marker.id = id++;
    polygon_marker.points.clear();
    polygon_marker.colors.clear();

    auto polygon_pose = pose.getPolygon();
    auto pose_world = pose_graph_transformations[pose.getId()];
    auto polygon_world = polygon_pose.transformPolygon(pose_world);

    auto points = polygon_world.getPoints();
    auto edge_types = polygon_world.getEdgeTypes();
    for (unsigned int i = 0; i < polygon_world.getPoints().size(); ++i) {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = points.at(i).getX();
      marker_point.y = points.at(i).getY();
      marker_point.z = 0;

      std_msgs::msg::ColorRGBA edge_color;
      edge_color.r = 0.0;
      edge_color.g = 0.0;
      edge_color.b = 0.0;
      edge_color.a = 1.0;

      std::cout << "Point " << i << ": edge type: " << edge_types[i]
                << std::endl;
      if (edge_types[i] == EdgeType::OBSTACLE) {
        edge_color.r = 1.0;
        edge_color.g = 0.0;
        edge_color.b = 0.0;
      } else if (edge_types[i] == EdgeType::FRONTIER) {
        edge_color.r = 0.0;
        edge_color.g = 0.0;
        edge_color.b = 1.0;
      }

      std_msgs::msg::ColorRGBA point_color;
      point_color.r = 0.0;
      point_color.g = 0.0;
      point_color.b = 0.0;
      point_color.a = 1.0;

      if (points.at(i).getPointType() == PointType::MAX_RANGE) {
        point_color.b = 1.0;
      } else if (points.at(i).getPointType() == PointType::OBSTACLE) {
        point_color.r = 1.0;
      }

      if (i > 0) {
        polygon_marker.points.push_back(marker_point);
        polygon_marker.colors.push_back(polygon_marker.colors.back());
      }
      polygon_marker.points.push_back(marker_point);
      polygon_marker.colors.push_back(edge_color);

      polygon_points_marker.points.push_back(marker_point);
      polygon_points_marker.colors.push_back(point_color);
    }

    polygon_markers.markers.push_back(polygon_marker);
  }

  return std::make_tuple(polygon_markers, polygon_points_marker);
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(PolygonExplorerNode, rclcpp::Node)
