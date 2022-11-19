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
#include "map_generator/map_generator_component.hpp"
// Components
#include <rclcpp_components/register_node_macro.hpp>

// Headers needed in this component

namespace map_generator
{
  MapGeneratorComponent::MapGeneratorComponent(const rclcpp::NodeOptions &options)
      : Node("map_generator_node", options)
  {
    probability_map_data.resize(map_height * map_width);
    if (easy_calculate_prob_method)
    {
      array_count_all_hit.resize(map_height * map_width);
      std::fill(array_count_all_hit.begin(), array_count_all_hit.end(), 0);
      array_count_if_obstacle.resize(map_height * map_width);
      std::fill(array_count_if_obstacle.begin(), array_count_if_obstacle.end(), 0);
    }
    std::fill(probability_map_data.begin(), probability_map_data.end(), log_odd(priorProbability));
    Scansubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).best_effort().durability_volatile(),
        std::bind(&MapGeneratorComponent::Scan_topic_callback, this, std::placeholders::_1));
    OccupancyGridpublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    MarkerPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    using namespace std::chrono_literals;
    // 1秒ごとにOccupancyGridMapをpublishする
    timer_ = this->create_wall_timer(500ms, std::bind(&MapGeneratorComponent::publishMap, this));
  }

  void MapGeneratorComponent::Scan_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    try
    {
      // get tf laser -> map
      geometry_msgs::msg::TransformStamped laserToMap =
          tf_buffer_->lookupTransform(map_frame, laser_frame, tf2::TimePointZero);

      // get (x,y) of base_link in cell coordinate
      const int cell_robot_x =
          floor((laserToMap.transform.translation.x + world_width * 0.5f) / map_resolution);
      const int cell_robot_y =
          floor((laserToMap.transform.translation.y + world_height * 0.5f) / map_resolution);

      tf2::Quaternion q(
          laserToMap.transform.rotation.x, laserToMap.transform.rotation.y,
          laserToMap.transform.rotation.z, laserToMap.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      // get yaw angle of laser in map coordinate
      m.getRPY(roll, pitch, yaw);

      float current_angle = msg->angle_min;
      for (float &scan : msg->ranges)
      {
        if (msg->range_min <= scan && scan <= msg->range_max)
        {
          const float point_x =
              scan * cos(current_angle + yaw) + laserToMap.transform.translation.x + world_width * 0.5f;
          const float point_y = scan * sin(current_angle + yaw) + laserToMap.transform.translation.y +
                                world_height * 0.5f;
          const int cell_point_x = floor(point_x / map_resolution);
          const int cell_point_y = floor(point_y / map_resolution);

          plotProbablilityMap(
              cell_point_x, cell_robot_x, cell_point_y, cell_robot_y);
          int index = getRasterScanIndex(map_width, cell_point_x, cell_point_y);
          float current_prob = probability_map_data.at(index);
          if (easy_calculate_prob_method)
          {
            array_count_if_obstacle.at(index) += 1;
            array_count_all_hit.at(index) += 1;
            const float prob = static_cast<float>(array_count_if_obstacle.at(index)) / static_cast<float>(array_count_all_hit.at(index));
            probability_map_data.at(index) = log_odd(prob);
          }
          else
          {
            probability_map_data.at(index) = current_prob + log_odd(occupied) - log_odd(l0);
          }

          current_angle += msg->angle_increment;
        }
      }
      // publishMarker(point_vec);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(
          get_logger(), "Could not transform %s to %s: %s", laser_frame.c_str(), map_frame.c_str(),
          ex.what());
      return;
    }
  }

  void MapGeneratorComponent::publishMap()
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
    map_.data.resize(map_width * map_height);
    std::fill(map_.data.begin(), map_.data.end(), unknown);
    int index = 0;
    for (float &prob : probability_map_data)
    {
      int integer_prob = static_cast<int>(round(get_prob_from_log_odd(prob) * 100.f));

      map_.data.at(index) = integer_prob;
      index++;
    }

    OccupancyGridpublisher_->publish(map_);
  }

  void MapGeneratorComponent::publishMarker(std::vector<geometry_msgs::msg::Point> &vec)
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

  void MapGeneratorComponent::plotProbablilityMap(
      int robot_x, int laser_x, int robot_y, int laser_y)
  {
    const bool steep = abs(laser_y - robot_y) > abs(laser_x - robot_x);
    if (steep)
    {
      using std::swap;
      swap(robot_x, robot_y);
      swap(laser_x, laser_y);
    }
    if (robot_x > laser_x)
    {
      using std::swap;
      swap(robot_x, laser_x);
      swap(robot_y, laser_y);
    }
    const int deltax = laser_x - robot_x;
    const int deltay = abs(laser_y - robot_y);
    int error = deltax / 2;
    int y = robot_y;
    const int ystep = robot_y < laser_y ? 1 : -1;
    for (int x = robot_x; x < laser_x; x++)
    {
      int index = 0;

      if (steep)
      {
        index = getRasterScanIndex(map_width, y, x);
      }
      else
      {
        index = getRasterScanIndex(map_width, x, y);
      }
      if (0 <= index && index < static_cast<int>(probability_map_data.size()))
      {
        if (easy_calculate_prob_method)
        {
          array_count_all_hit.at(index) = array_count_all_hit.at(index) + 1;
          const float prob = static_cast<float>(array_count_if_obstacle.at(index)) / static_cast<float>(array_count_all_hit.at(index));
          if (prob > 0.f)
          {
            probability_map_data.at(index) = log_odd(prob);
          }
          else
          {
            probability_map_data.at(index) = log_odd(unOccupied);
          }
        }
        else
        {
          float current_prob = probability_map_data.at(index);
          const float new_prob = current_prob +
                                 inverse_range_sensor_model(
                                     laser_x, laser_y, x, y) -
                                 log_odd(l0);
          probability_map_data.at(index) = new_prob;
        }
      }
      error -= deltay;
      if (error < 0)
      {
        y += ystep;
        error += deltax;
      }
    }
  }

  float MapGeneratorComponent::inverse_range_sensor_model(
      int laser_x,
      int laser_y,
      int current_x,
      int current_y)
  {
    if (abs(laser_x - current_x) < 2)
    {
      return log_odd(occupied);
    }
    else if (current_x < laser_x)
    {
      return log_odd(unOccupied);
    }
    else
    {
      return log_odd(l0);
    }
  }
} // namespace map_generator

RCLCPP_COMPONENTS_REGISTER_NODE(map_generator::MapGeneratorComponent)