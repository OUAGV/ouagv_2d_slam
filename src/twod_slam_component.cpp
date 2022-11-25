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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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
      laserToMap = tf_buffer_->lookupTransform("odom", "lidar_link", tf2::TimePointZero);
      laserToMap.transform.translation.x += last_diff_x;
      laserToMap.transform.translation.y += last_diff_y;
      tf2::Quaternion quat;
      const double current_yaw = tf2::getYaw(laserToMap.transform.rotation);
      quat.setRPY(0, 0, current_yaw + last_diff_yaw);
      laserToMap.transform.rotation.x = quat.getX();
      laserToMap.transform.rotation.y = quat.getY();
      laserToMap.transform.rotation.z = quat.getZ();
      laserToMap.transform.rotation.w = quat.getW();
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", "laser", "odom", ex.what());
      return;
    }
    std::vector<pointcloud_manager::PointWithNormal> point_vec;
    pointCloudManager.scanToPoints(msg, point_vec);
    if (is_initial_scan_sub)
    {
      is_initial_scan_sub = false;
      mapManager.updateMap(laserToMap, point_vec);
      return;
    }
    /**
     * ここにスキャンマッチングの処理を入れる
     *
     */
    geometry_msgs::msg::PoseWithCovarianceStamped estimated_pose =
        scanMatcher.estimatePose(mapManager.globalCellMap, point_vec, laserToMap, laserToMap.header.stamp);
    geometry_msgs::msg::TransformStamped estimated_laser_to_map = poseToTransformStamped(estimated_pose);
    // last_estimated_pose = estimated_laser_to_map;

    broadcastTf(estimated_laser_to_map, laserToMap);
    // broadcastTf(laserToMap, laserToMap);

    mapManager.updateMap(estimated_laser_to_map, point_vec);
    // mapManager.updateMap(laserToMap, point_vec);
    if (publish_marker)
    {
      geometry_msgs::msg::Point estimated_point;
      estimated_point.x = estimated_laser_to_map.transform.translation.x;
      estimated_point.y = estimated_laser_to_map.transform.translation.y;
      pub_vec.emplace_back(estimated_point);
      if (pub_vec.size() > 100)
      {
        pub_vec.erase(pub_vec.begin());
      }
      publishMarker(pub_vec);
    }
  }
  geometry_msgs::msg::TransformStamped
  TwodSlamComponent::poseToTransformStamped(geometry_msgs::msg::PoseWithCovarianceStamped &pose)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = pose.header.stamp;
    transformStamped.header.frame_id = pose.header.frame_id;
    transformStamped.child_frame_id = "lidar_link";
    transformStamped.transform.translation.x = pose.pose.pose.position.x;
    transformStamped.transform.translation.y = pose.pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation = pose.pose.pose.orientation;
    return transformStamped;
  }
  void TwodSlamComponent::broadcastTf(geometry_msgs::msg::TransformStamped &laserToMap,
                                      geometry_msgs::msg::TransformStamped &laserToOdom)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = laserToOdom.header.stamp;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = laserToMap.transform.translation.x - laserToOdom.transform.translation.x;
    transformStamped.transform.translation.y = laserToMap.transform.translation.y - laserToOdom.transform.translation.y;

    transformStamped.transform.translation.z = 0.0;
    const double map_yaw = tf2::getYaw(laserToMap.transform.rotation);
    const double odom_yaw = tf2::getYaw(laserToOdom.transform.rotation);
    tf2::Quaternion quat;
    quat.setRPY(0, 0, map_yaw - odom_yaw);
    transformStamped.transform.rotation.x = quat.getX();
    transformStamped.transform.rotation.y = quat.getY();
    transformStamped.transform.rotation.z = quat.getZ();
    transformStamped.transform.rotation.w = quat.getW();
    tf_broadcaster_->sendTransform(transformStamped);

    last_diff_x = transformStamped.transform.translation.x;
    last_diff_y = transformStamped.transform.translation.y;
    last_diff_yaw = map_yaw - odom_yaw;
    std::cout << "diff : " << last_diff_x << "m  " << last_diff_y << "m " << last_diff_yaw << "rad " << std::endl;
  }

} // namespace twod_slam

RCLCPP_COMPONENTS_REGISTER_NODE(twod_slam::TwodSlamComponent)