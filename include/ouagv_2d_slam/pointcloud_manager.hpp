#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <Eigen/LU>

namespace pointcloud_manager
{
    enum ptype
    {
        UNKNOWN = 0,
        LINE = 1,
        CORNER = 2,
        ISOLATE = 3
    };

    struct PointWithNormal
    {
        std::string frame_id;
        geometry_msgs::msg::Point point;
        Eigen::Vector2d normal;
        ptype type;
    };

    class PointCloudManager
    {
    public:
        PointCloudManager() : laser_frame("lidar_link"){};
        PointCloudManager(std::string laser_frame_) : laser_frame(laser_frame_){};
        ~PointCloudManager(){};
        void scanToPoints(sensor_msgs::msg::LaserScan::SharedPtr msg,
                          std::vector<PointWithNormal> &vec);

    private:
        std::string laser_frame;
        void toPoints(sensor_msgs::msg::LaserScan::SharedPtr msg,
                      std::vector<PointWithNormal> &vec);
        void resamplePoints(std::vector<PointWithNormal> &vec);

        // スキャン点の法線ベクトルを求める。また、直線、コーナ、孤立の場合分けをする。
        void analysePoints(std::vector<PointWithNormal> &lps);

        // 注目点cpの両側の点が、cpからdmin以上dmax以下の場合に、法線を計算する。
        bool calNormal(size_t idx, std::vector<PointWithNormal> &lps, bool isForward, Eigen::Vector2d &normal);
    };

}