#pragma once
#include "ouagv_2d_slam/pointcloud_manager.hpp"
#include "ouagv_2d_slam/map_manager.hpp"

namespace icp_scan_matching
{
    class IcpScanMatching
    {
    public:
        IcpScanMatching(){};
        ~IcpScanMatching(){};
        void estimatePose(std::vector<map_manager::CellWithProb> &global_map,
                          std::vector<pointcloud_manager::PointWithNormal> &scanned_points,
                          geometry_msgs::msg::TransformStamped &predict_pose);

    private:
        const float world_width = 100.f;                      // [m]
        const float world_height = 100.f;                     // [m]
        const float map_resolution = 0.05f;                   // [m/cell]
        const int map_width = world_width / map_resolution;   // [cell]
        const int map_height = world_height / map_resolution; // [cell]

        size_t getRasterScanIndex(int width, int x, int y) { return static_cast<size_t>(y * width + x); }
    };
}