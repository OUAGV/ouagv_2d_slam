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

#include "ouagv_2d_slam/visibility_control_twod_slam.h"

// Headers in ROS2
#include <math.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory> // shared_ptr in pub_
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "ouagv_2d_slam/pointcloud_manager.hpp"
#include "ouagv_2d_slam/map_manager.hpp"

namespace twod_slam
{

  class TwodSlamComponent : public rclcpp::Node
  {
  public:
    TWOD_SLAM_TWOD_SLAM_COMPONENT_PUBLIC
    explicit TwodSlamComponent(const rclcpp::NodeOptions &options);

  private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Posesubscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr MarkerPublisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Scansubscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr OccupancyGridpublisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    pointcloud_manager::PointCloudManager pointCloudManager;
    map_manager::MapManager mapManager;
    std::shared_ptr<tf2_ros::TransformListener>
        tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    const bool publish_marker = false;

    void publishMarker(std::vector<geometry_msgs::msg::Point> &vec);
    void Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publishMap();
  };
} // namespace twod_slam