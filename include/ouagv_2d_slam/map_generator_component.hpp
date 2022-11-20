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

#include "ouagv_2d_slam/visibility_control_map_generator.h"

// Headers in ROS2
#include <math.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
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

namespace map_generator
{
  class MapGeneratorComponent : public rclcpp::Node
  {
  public:
    MAP_GENERATOR_MAP_GENERATOR_COMPONENT_PUBLIC
    explicit MapGeneratorComponent(const rclcpp::NodeOptions &options);

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Scansubscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Posepublisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr OccupancyGridpublisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr MarkerPublisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    std::vector<float> probability_map_data;
    std::vector<int> array_count_if_obstacle;
    std::vector<int> array_count_all_hit;

    const float world_width = 100.f;                      // [m]
    const float world_height = 100.f;                     // [m]
    const float map_resolution = 0.05f;                   // [m/cell]
    const int map_width = world_width / map_resolution;   // [cell]
    const int map_height = world_height / map_resolution; // [cell]
    const float unOccupied = 0.01f;
    const float occupied = 0.99f;
    const float priorProbability = 0.5f;
    const float l0 = 0.5f;
    const int unknown = -1;
    const float inverse_range_sensor_model_alpha = 0.1f;
    const bool easy_calculate_prob_method = true;
    std::string map_frame = "map";
    std::string laser_frame = "lidar_link";

    void publishMap();
    void publishMarker(std::vector<geometry_msgs::msg::Point> &vec);
    int getRasterScanIndex(int width, int x, int y) { return y * width + x; }

    float log_odd(float prob)
    {
      return log(prob / (1.0f - prob));
    }

    float get_prob_from_log_odd(float log_odd) { return exp(log_odd) / (1.f + exp(log_odd)); }

    /**
     * @brief scan callback
     *
     * @param msg
     */
    void Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void plotProbablilityMap(
        int robot_x, int laser_x, int robot_y, int laser_y);

    float inverse_range_sensor_model(
        int laser_x,
        int laser_y,
        int current_x,
        int current_y);
  };
} // namespace map_generator