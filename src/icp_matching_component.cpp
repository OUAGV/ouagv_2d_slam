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
    IcpMatchingComponent::IcpMatchingComponent(const rclcpp::NodeOptions &options)
        : Node("icp_matching_node", options)
    {
        Odomsubscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&IcpMatchingComponent::Odom_topic_callback, this, std::placeholders::_1));
        Scansubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&IcpMatchingComponent::Scan_topic_callback, this, std::placeholders::_1));
        Posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);
    }

    void IcpMatchingComponent::Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "max_angle : %f", msg->angle_max);
    }

    void IcpMatchingComponent::Odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "x : %f y : %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(icp_matching::IcpMatchingComponent)