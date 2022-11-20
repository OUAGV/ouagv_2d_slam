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
        Scansubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::QoS(10).best_effort().durability_volatile(),
            std::bind(&TwodSlamComponent::Scan_topic_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

    void TwodSlamComponent::Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped laserToMap;
        try
        {
            laserToMap =
                tf_buffer_->lookupTransform("map", "lidar_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(
                get_logger(), "Could not transform %s to %s: %s", "laser", "map",
                ex.what());
            return;
        }
        std::vector<geometry_msgs::msg::Point> point_vec;
        const float laser_yaw = tf2::getYaw(laserToMap.transform.rotation);
        float current_angle = msg->angle_min;
        for (float &scan : msg->ranges)
        {
            if (msg->range_min <= scan && scan <= msg->range_max)
            {
                geometry_msgs::msg::Point point;
                point.x =
                    scan * cos(current_angle + laser_yaw) + laserToMap.transform.translation.x;
                point.y = scan * sin(current_angle + laser_yaw) + laserToMap.transform.translation.y;
                point_vec.emplace_back(point);
                current_angle += msg->angle_increment;
            }
        }
        resamplePoints(point_vec);
    }

    void TwodSlamComponent::resamplePoints(std::vector<geometry_msgs::msg::Point> &vec)
    {
        const float point_interval_m = 0.03f;            // [m]
        const float interpolate_threthold_max_m = 0.25f; // [m]
        if (vec.size() == 0)
        {
            return;
        }
        float distance_sum = 0.f;
        geometry_msgs::msg::Point last_point = vec.at(0);
        geometry_msgs::msg::Point new_point = vec.at(0);
        std::vector<geometry_msgs::msg::Point> interpolated_points;
        interpolated_points.emplace_back(last_point);

        for (int i = 1; i < static_cast<int>(vec.size()); i++)
        {
            const geometry_msgs::msg::Point current_point = vec.at(i);
            const float dx = current_point.x - last_point.x;
            const float dy = current_point.y - last_point.y;
            const float distance_between_neighbor_points = sqrt(pow(dx, 2) + pow(dy, 2));

            bool exists = false;
            bool isInserted = false;
            if (distance_sum + distance_between_neighbor_points < point_interval_m)
            {
                distance_sum += distance_between_neighbor_points;
            }
            else if (distance_sum + distance_between_neighbor_points >= interpolate_threthold_max_m)
            {
                new_point = current_point;
                exists = true;
            }
            else
            {
                const float ratio =
                    (point_interval_m - distance_sum) /
                    distance_between_neighbor_points;

                new_point.x = dx * ratio + last_point.x;
                new_point.y = dy * ratio + last_point.y;
                exists = true;
                isInserted = true;
            }
            if (exists)
            {
                interpolated_points.emplace_back(new_point);
                last_point = new_point;
                distance_sum = 0;
                if (isInserted)
                {
                    i--;
                }
            }
            else
            {
                last_point = current_point;
            }
        }
        publishMarker(interpolated_points);
    }

} // namespace twod_slam

RCLCPP_COMPONENTS_REGISTER_NODE(twod_slam::TwodSlamComponent)