#pragma once
#include "ouagv_2d_slam/map_manager.hpp"
#include "ouagv_2d_slam/pointcloud_manager.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace icp_scan_matching
{
  class IcpScanMatching
  {
  public:
    IcpScanMatching(){};
    ~IcpScanMatching(){};
    geometry_msgs::msg::PoseWithCovarianceStamped estimatePose(
        std::vector<map_manager::CellWithProb> &global_map,
        std::vector<pointcloud_manager::PointWithNormal> &scanned_points,
        geometry_msgs::msg::TransformStamped &predict_pose,
        rclcpp::Time stamp);
    bool loop_limited = false;

  private:
    const float world_width = 100.f;    // [m]
    const float world_height = 100.f;   // [m]
    const float map_resolution = 0.05f; // [m/cell]
    // 参照点と対象点との距離がこの距離より大きければペアとみなさない
    const double threthold = 1.0;                         // [m]
    const int map_width = world_width / map_resolution;   // [cell]
    const int map_height = world_height / map_resolution; // [cell]
    // 点のペア作成の際、このセルの幅で四方を探す
    const int cell_r = 2; // [cell]

        double estimated_x = 0.f;
    double estimated_y = 0.f;
    double estimated_theta = 0.f;

    void optimize(std::vector<pointcloud_manager::PointWithNormal> &reference_points,
                  std::vector<pointcloud_manager::PointWithNormal> &target_points,
                  double initial_x, double initial_y, double initial_theta);
    size_t getRasterScanIndex(int width, int x, int y) { return static_cast<size_t>(y * width + x); }

    double getCost(double x, double y, double theta,
                   std::vector<pointcloud_manager::PointWithNormal> &reference_points,
                   std::vector<pointcloud_manager::PointWithNormal> &target_points);

    void associatePoints(std::vector<map_manager::CellWithProb> &global_map,
                         std::vector<pointcloud_manager::PointWithNormal> &scanned_points,
                         std::vector<pointcloud_manager::PointWithNormal> &reference_points,
                         std::vector<pointcloud_manager::PointWithNormal> &target_points,
                         double x, double y, double yaw);
  };
} // namespace icp_scan_matching