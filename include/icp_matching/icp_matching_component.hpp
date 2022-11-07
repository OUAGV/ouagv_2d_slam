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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

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
        void Odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    };
}