// Copyright (c) 2022 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Headers in this package
#include "icp_matching/icp_matching_component.hpp"
// Components
#include <rclcpp_components/register_node_macro.hpp>

// Headers needed in this component

namespace icp_matching
{
IcpMatchingComponent::IcpMatchingComponent(const rclcpp::NodeOptions & options)
: Node("icp_matching_node", options)
{
  probability_map_data.resize(map_height * map_width);
  std::fill(probability_map_data.begin(), probability_map_data.end(), unknown);
  Odomsubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&IcpMatchingComponent::Odom_topic_callback, this, std::placeholders::_1));
  Scansubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::QoS(10).best_effort().durability_volatile(),
    std::bind(&IcpMatchingComponent::Scan_topic_callback, this, std::placeholders::_1));
  OccupancyGridpublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
  MarkerPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker", 10);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  using namespace std::chrono_literals;
  // 1秒ごとにOccupancyGridMapをpublishする
  timer_ = this->create_wall_timer(1s, std::bind(&IcpMatchingComponent::publishMap, this));
}

void IcpMatchingComponent::Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  try {
    std::vector<geometry_msgs::msg::Point> point_vec;
    geometry_msgs::msg::TransformStamped laserToMap =
      tf_buffer_->lookupTransform(map_frame, laser_frame, tf2::TimePointZero);
    tf2::Quaternion q(
      laserToMap.transform.rotation.x, laserToMap.transform.rotation.y,
      laserToMap.transform.rotation.z, laserToMap.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    float current_angle = msg->angle_min;
    for (float & scan : msg->ranges) {
      if (msg->range_min <= scan && scan <= msg->range_max) {
        geometry_msgs::msg::Point point;
        // 極座標系をlaserのx,yに変換
        point.x =
          scan * cos(current_angle + yaw) + laserToMap.transform.translation.x + world_width * 0.5f;
        point.y = scan * sin(current_angle + yaw) + laserToMap.transform.translation.y +
                  world_height * 0.5f;
        point_vec.emplace_back(point);
        current_angle += msg->angle_increment;
      }
    }
    geometry_msgs::msg::TransformStamped mapToBaseLink =
      tf_buffer_->lookupTransform("base_link", map_frame, tf2::TimePointZero);
    const int cell_robot_x =
      floor((mapToBaseLink.transform.translation.x + world_width * 0.5f) / map_resolution);
    const int cell_robot_y =
      floor((mapToBaseLink.transform.translation.y + world_height * 0.5f) / map_resolution);
    for (geometry_msgs::msg::Point & point : point_vec) {
      const int cell_point_x = floor(point.x / map_resolution);
      const int cell_point_y = floor(point.y / map_resolution);
      plotBresenhamLine(cell_point_x, cell_robot_x, cell_point_y, cell_robot_y);
      probability_map_data[getRasterScanIndex(map_width, cell_point_x, cell_point_y)] = occupied;
    }
    // publishMarker(point_vec);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      get_logger(), "Could not transform %s to %s: %s", laser_frame, map_frame, ex.what());
    return;
  }
}

void IcpMatchingComponent::Odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // RCLCPP_INFO(get_logger(), "x : %f y : %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void IcpMatchingComponent::resamplePoints(std::vector<geometry_msgs::msg::Point> & vec)
{
  const float interpolate_threthold_min_m = 0.015f;  // [m]
  const float interpolate_threthold_max_m = 0.02f;   // [m]
  if (vec.size() == 0) {
    return;
  }
  int index = 0;
  float distance_sum = 0.f;
  geometry_msgs::msg::Point last_point = vec.at(index);
  std::vector<geometry_msgs::msg::Point> interpolated_points;
  interpolated_points.emplace_back(last_point);
  while (index < vec.size()) {
    const geometry_msgs::msg::Point current_point = vec.at(index);
    const float dx = current_point.x - last_point.x;
    const float dy = current_point.y - last_point.y;
    const float distance_between_neighbor_points = sqrt(pow(dx, 2) + pow(dy, 2));
    // RCLCPP_INFO(get_logger(), "%f", distance_between_neighbor_points);
    distance_sum += distance_between_neighbor_points;
    if (distance_sum < interpolate_threthold_min_m) {
      last_point = current_point;
      index++;
    } else if (distance_sum >= interpolate_threthold_max_m) {
      interpolated_points.emplace_back(current_point);
      last_point = current_point;
      distance_sum = 0.f;
      index++;
    } else {
      const float ratio =
        (distance_between_neighbor_points - (distance_sum - interpolate_threthold_min_m)) /
        distance_between_neighbor_points;
      geometry_msgs::msg::Point newPoint;
      newPoint.x = last_point.x + dx * ratio;
      newPoint.y = last_point.y + dy * ratio;
      interpolated_points.emplace_back(newPoint);
    }
  }
}

void IcpMatchingComponent::publishMap()
{
  nav_msgs::msg::OccupancyGrid map_;
  // OccupancyGridの座標系はmap
  map_.header.frame_id = map_frame;
  // 時間は現在時刻
  map_.header.stamp = get_clock()->now();
  map_.info.width = map_width;
  map_.info.height = map_height;
  map_.info.resolution = map_resolution;
  // originではマップの左下のセル（0,0）の位置を指定する
  // このように指定するとマップの中心がセルの中心となる
  map_.info.origin.position.x = -world_width * 0.5f;
  map_.info.origin.position.y = -world_height * 0.5f;
  map_.data = probability_map_data;
  OccupancyGridpublisher_->publish(map_);
}

void IcpMatchingComponent::publishMarker(std::vector<geometry_msgs::msg::Point> & vec)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame;
  marker.header.stamp = get_clock()->now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.points = vec;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.color.g = 1.0f;
  marker.color.a = 1.0;
  MarkerPublisher_->publish(marker);
}

void IcpMatchingComponent::plotBresenhamLine(int x1, int x2, int y1, int y2)
{
  const bool steep = abs(y2 - y1) > abs(x2 - x1);
  if (steep) {
    using std::swap;
    swap(x1, y1);
    swap(x2, y2);
  }
  if (x1 > x2) {
    using std::swap;
    swap(x1, x2);
    swap(y1, y2);
  }
  const int deltax = x2 - x1;
  const int deltay = abs(y2 - y1);
  int error = deltax / 2;
  int y = y1;
  const int ystep = y1 < y2 ? 1 : -1;
  for (int x = x1; x < x2; x++) {
    int index = 0;

    if (steep) {
      index = getRasterScanIndex(map_width, y, x);
    } else {
      index = getRasterScanIndex(map_width, x, y);
    }
    if (0 <= index && index < probability_map_data.size()) {
      probability_map_data.at(index) = unOccupied;
    }
    error -= deltay;
    if (error < 0) {
      y += ystep;
      error += deltax;
    }
  }
}
}  // namespace icp_matching

RCLCPP_COMPONENTS_REGISTER_NODE(icp_matching::IcpMatchingComponent)