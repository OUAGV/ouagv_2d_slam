#pragma once
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
// Headers needed in pub/sub, exposed types
#include <memory> // shared_ptr in pub_
#include "ouagv_2d_slam/pointcloud_manager.hpp"
#include <mutex>

namespace map_manager
{
    class MapManager
    {
    public:
        MapManager()
        {
            probability_map_data.resize(map_height * map_width);
            array_count_all_hit.resize(map_height * map_width);
            std::fill(array_count_all_hit.begin(), array_count_all_hit.end(), 0);
            array_count_if_obstacle.resize(map_height * map_width);
            std::fill(array_count_if_obstacle.begin(), array_count_if_obstacle.end(), 0);
            std::fill(probability_map_data.begin(), probability_map_data.end(), log_odd(priorProbability));
        };
        ~MapManager(){};

        // estimated_pose map <- laser
        void updateMap(geometry_msgs::msg::TransformStamped &estimated_pose,
                       std::vector<pointcloud_manager::PointWithNormal> &point_vec);
        nav_msgs::msg::OccupancyGrid getMapData(rclcpp::Time stamp);

        std::vector<pointcloud_manager::PointWithNormal>
        getReferenceMap(double robot_x, double robot_y, double r);

    private:
        std::mutex map_mutex;
        std::vector<int> array_count_if_obstacle;
        std::vector<int> array_count_all_hit;
        std::vector<float> probability_map_data;

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

        size_t getRasterScanIndex(int width, int x, int y) { return static_cast<size_t>(y * width + x); }
        float log_odd(float prob) { return log(prob / (1.0f - prob)); }
        float get_prob_from_log_odd(float log_odd) { return exp(log_odd) / (1.f + exp(log_odd)); }

        void plotProbablilityMap(int robot_x, int laser_x, int robot_y, int laser_y);
        void updateProb(size_t index);
    };
} // namespace map_manager