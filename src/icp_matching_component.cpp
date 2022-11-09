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
        probability_map_data.resize(map_height * map_width);
        std::fill(probability_map_data.begin(), probability_map_data.end(), unknown);
        Odomsubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&IcpMatchingComponent::Odom_topic_callback, this, std::placeholders::_1));
        Scansubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::QoS(10).best_effort().durability_volatile(),
            std::bind(&IcpMatchingComponent::Scan_topic_callback, this, std::placeholders::_1));
        OccupancyGridpublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);

        using namespace std::chrono_literals;
        // 1秒ごとにOccupancyGridMapをpublishする
        timer_ = this->create_wall_timer(1s, std::bind(&IcpMatchingComponent::publishMap, this));
    }

    void IcpMatchingComponent::Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (odom_)
        {
            // RCLCPP_INFO(get_logger(), "odom and scan ok");
            std::vector<geometry_msgs::msg::Point> pixels = laserScanToPixels(msg, odom_);
            const float pixel_robot_x = odom_->pose.pose.position.x / map_resolution + map_height * 0.5f;
            const float pixel_robot_y = odom_->pose.pose.position.y / map_resolution + map_width * 0.5f;
            geometry_msgs::msg::Point pixel_robot_pose;
            pixel_robot_pose.x = pixel_robot_x;
            pixel_robot_pose.y = pixel_robot_y;
            updateMap(pixels, pixel_robot_pose);
        }
    }

    void IcpMatchingComponent::Odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = msg;
        // RCLCPP_INFO(get_logger(), "x : %f y : %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    std::vector<geometry_msgs::msg::Point> IcpMatchingComponent::laserScanToPixels(
        const sensor_msgs::msg::LaserScan::SharedPtr msg, const nav_msgs::msg::Odometry::SharedPtr odom)
    {

        std::vector<geometry_msgs::msg::Point> ret;
        const float robot_x = odom->pose.pose.position.x;
        const float robot_y = odom->pose.pose.position.y;
        const float robot_yaw = getYawFromOdom(odom);
        // RCLCPP_INFO(get_logger(), "robot pose (odom) : x : %f y : %f yaw : %f", robot_x, robot_y, robot_yaw);
        float current_angle = msg->angle_min;
        for (const float &scan_elem : msg->ranges)
        {
            if (msg->range_min < scan_elem && scan_elem < msg->range_max)
            {
                geometry_msgs::msg::Point point;
                // map座標系に変換
                point.x = scan_elem * cos(current_angle + robot_yaw) + robot_x;
                point.y = scan_elem * sin(current_angle + robot_yaw) + robot_y;
                // pixel座標系に変換
                // マップサイズの半分を足すことでロボットがマップの中心に来るように調整
                point.x = point.x / map_resolution + map_height * 0.5;
                point.y = point.y / map_resolution + map_width * 0.5;
                ret.emplace_back(point);
            }
            current_angle += msg->angle_increment;
        }
        return ret;
    }

    void IcpMatchingComponent::resamplePoints(std::vector<geometry_msgs::msg::Point> &vec)
    {
        const float interpolate_threthold_min_m = 0.015f; // [m]
        const float interpolate_threthold_max_m = 0.02f;  // [m]
        if (vec.size() == 0)
        {
            return;
        }
        int index = 0;
        float distance_sum = 0.f;
        geometry_msgs::msg::Point last_point = vec.at(index);
        std::vector<geometry_msgs::msg::Point> interpolated_points;
        interpolated_points.emplace_back(last_point);
        while (index < vec.size())
        {
            const geometry_msgs::msg::Point current_point = vec.at(index);
            const float dx = current_point.x - last_point.x;
            const float dy = current_point.y - last_point.y;
            const float distance_between_neighbor_points = sqrt(pow(dx, 2) + pow(dy, 2));
            // RCLCPP_INFO(get_logger(), "%f", distance_between_neighbor_points);
            distance_sum += distance_between_neighbor_points;
            if (distance_sum < interpolate_threthold_min_m)
            {
                last_point = current_point;
                index++;
            }
            else if (distance_sum >= interpolate_threthold_max_m)
            {
                interpolated_points.emplace_back(current_point);
                last_point = current_point;
                distance_sum = 0.f;
                index++;
            }
            else
            {
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

    void IcpMatchingComponent::updateMap(
        std::vector<geometry_msgs::msg::Point> &vec, geometry_msgs::msg::Point &robot_pose)
    {
        const int floored_pixel_robot_x = floor(robot_pose.x);
        const int floored_pixel_robot_y = floor(robot_pose.y);
        for (const geometry_msgs::msg::Point &point : vec)
        {
            const int floored_pixel_point_x = floor(point.x);
            const int floored_pixel_point_y = floor(point.y);
            const int index = getRasterScanIndex(map_width, floored_pixel_point_x, floored_pixel_point_y);
            if (0 <= index && index <= probability_map_data.size())
            {
                plotBresenhamLine(floored_pixel_robot_x, floored_pixel_point_x, floored_pixel_robot_y, floored_pixel_point_y);
                probability_map_data.at(index) = occupied;
                RCLCPP_INFO(get_logger(), "index : %d data %d", index, probability_map_data.at(index));
            }
        }
    }

    void IcpMatchingComponent::publishMap()
    {
        nav_msgs::msg::OccupancyGrid map_;
        // OccupancyGridの座標系はmap
        map_.header.frame_id = "odom";
        // 時間は現在時刻
        map_.header.stamp = get_clock()->now();
        map_.info.width = map_width;
        map_.info.height = map_height;
        map_.info.resolution = map_resolution;
        // originではマップの左下のセル（0,0）の位置を指定する
        // このように指定するとマップの中心がセルの中心となる
        map_.info.origin.position.x = -world_width / 2.f;
        map_.info.origin.position.y = -world_height / 2.f;
        map_.data = probability_map_data;
        OccupancyGridpublisher_->publish(map_);
    }

    int IcpMatchingComponent::getRasterScanIndex(int width, int x, int y) { return y * width + x; }

    double IcpMatchingComponent::getYawFromOdom(const nav_msgs::msg::Odometry::SharedPtr &odom)
    {
        tf2::Quaternion q(
            odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
            odom->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void IcpMatchingComponent::plotBresenhamLine(int x1, int x2, int y1, int y2)
    {
        if (x1 == x2)
        {
        }
        int dx = abs(x1 - x2), dy = abs(y1 - y2);
        int p = 2 * dy - dx;
        int twoDy = 2 * dy, twoDyDx = 2 * (dy - dx);
        int x, y, xEnd;
        /*Determine which points to start and End */
        if (x1 > x2)
        {
            x = x2;
            y = y2;
            xEnd = x1;
        }
        else
        {
            x = x1;
            y = y1;
            xEnd = x2;
        }
        while (x < xEnd)
        {
            x++;
            if (p < 0)
            {
                p = p + twoDy;
            }
            else
            {
                y++;
                p = p + twoDyDx;
            }
            const int index = getRasterScanIndex(map_width, x, y);
            if (0 <= index && index < probability_map_data.size())
            {
                probability_map_data.at(index) = unOccupied;
            }
        }
    }
} // namespace icp_matching

RCLCPP_COMPONENTS_REGISTER_NODE(icp_matching::IcpMatchingComponent)