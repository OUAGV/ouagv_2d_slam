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
#include "ouagv_2d_slam/twod_slam_component.hpp"
// Components
#include <rclcpp_components/register_node_macro.hpp>

// Headers needed in this component

namespace twod_slam
{
  TwodSlamComponent::TwodSlamComponent(const rclcpp::NodeOptions &options)
      : Node("twod_slam_node", options)
  {
    MarkerPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker", 10);
    OccupancyGridpublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    Scansubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).best_effort().durability_volatile(),
        std::bind(&TwodSlamComponent::Scan_topic_callback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    using namespace std::chrono_literals;
    // 1秒ごとにOccupancyGridMapをpublishする
    timer_ = this->create_wall_timer(500ms, std::bind(&TwodSlamComponent::publishMap, this));
  }

  void TwodSlamComponent::publishMarker(std::vector<geometry_msgs::msg::Point> &vec)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
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

  void TwodSlamComponent::publishMap()
  {
    OccupancyGridpublisher_->publish(mapManager.getMapData(get_clock()->now()));
  }
  void TwodSlamComponent::Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped laserToMap;
    try
    {
      laserToMap = tf_buffer_->lookupTransform("map", "lidar_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", "laser", "map", ex.what());
      return;
    }
    std::vector<pointcloud_manager::PointWithNormal> point_vec;
    pointCloudManager.scanToPoints(
        msg, point_vec);

    mapManager.updateMap(laserToMap, point_vec);

    if (publish_marker)
    {
      std::vector<geometry_msgs::msg::Point> pub_vec;
      pub_vec.resize(point_vec.size());
      for (size_t i = 0; i < point_vec.size(); i++)
      {
        // if (vec.at(i).type != pointcloud_manager::ptype::ISOLATE)
        //   RCLCPP_INFO(get_logger(), "i : %d nx : %lf ny : %lf\n", i, vec.at(i).normal(0), vec.at(i).normal(1));
        pub_vec.at(i) = point_vec.at(i).point;
      }
      publishMarker(pub_vec);
    }
  }

} // namespace twod_slam

RCLCPP_COMPONENTS_REGISTER_NODE(twod_slam::TwodSlamComponent)