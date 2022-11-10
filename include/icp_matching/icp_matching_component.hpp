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

#pragma once

#include "icp_matching/visibility_control.h"

// Headers in ROS2
#include <math.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
// Headers needed in pub/sub, exposed types
#include <memory> // shared_ptr in pub_

namespace icp_matching
{
  class IcpMatchingComponent : public rclcpp::Node
  {
  public:
    ICP_MATCHING_ICP_MATCHING_COMPONENT_PUBLIC
    explicit IcpMatchingComponent(const rclcpp::NodeOptions &options);

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Scansubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odomsubscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Posepublisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr OccupancyGridpublisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr MarkerPublisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    std::vector<int8_t> probability_map_data;

    const float world_width = 100.f;                      // [m]
    const float world_height = 100.f;                     // [m]
    const float map_resolution = 0.05;                    // [m/cell]
    const int map_width = world_width / map_resolution;   // [cell]
    const int map_height = world_height / map_resolution; // [cell]
    const int unOccupied = 0;
    const int occupied = 100;
    const int unknown = -1;
    std::string map_frame = "map";
    std::string laser_frame = "lidar_link";

    void Odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief 点群の点の間隔を一定に揃える
     * 実装についてはゼロから始めるSLAM入門のP41を参照
     * @param vec
     */
    void resamplePoints(std::vector<geometry_msgs::msg::Point> &vec);

    /**
     * @brief 点群の位置とロボットの位置の間を結んだ直線上のPixelをすべて
     * UnOccupiedで塗りつぶす
     * @param x1
     * @param x2
     * @param y1
     * @param y2
     */
    void plotBresenhamLine(int x1, int x2, int y1, int y2);
    void publishMap();
    void publishMarker(std::vector<geometry_msgs::msg::Point> &vec);
    int getRasterScanIndex(int width, int x, int y) { return y * width + x; }
  };
} // namespace icp_matching