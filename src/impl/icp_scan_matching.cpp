#include "ouagv_2d_slam/icp_scan_matching.hpp"

namespace icp_scan_matching
{

    void IcpScanMatching::estimatePose(std::vector<map_manager::CellWithProb> &global_map,
                                       std::vector<pointcloud_manager::PointWithNormal> &scanned_points,
                                       geometry_msgs::msg::TransformStamped &predict_pose)
    {
        const double yaw = tf2::getYaw(predict_pose.transform.rotation);
        const double x = predict_pose.transform.translation.x;
        const double y = predict_pose.transform.translation.y;
        const int cell_r = 2;
        double cost = 0;
        for (pointcloud_manager::PointWithNormal &point : scanned_points)
        {
            const double point_x = point.point.x * cos(yaw) - point.point.y * sin(yaw) + x;
            const double point_y = point.point.x * sin(yaw) + point.point.y * cos(yaw) + y;

            // gridに変換 ここでfloorを使うことで点群の数を削除してるとみなせる？
            const int cell_point_x = floor((point_x + world_width * 0.5f) / map_resolution);
            const int cell_point_y = floor((point_y + world_height * 0.5f) / map_resolution);

            double min_length = 0.f;
            bool isFirst = true;
            for (int y = -cell_r; y < cell_r; y++)
            {
                for (int x = -cell_r; x < cell_r; x++)
                {
                    const size_t index = getRasterScanIndex(map_width, cell_point_x + x, cell_point_y + y);
                    if (index > global_map.size())
                    {
                        continue;
                    }
                    const double length = sqrt(pow((global_map.at(index).point.point.x - point_x), 2) +
                                               pow((global_map.at(index).point.point.y - point_y), 2));
                    if (isFirst)
                    {
                        min_length = length;
                    }
                    else
                    {
                        min_length = (min_length > length) ? length : min_length;
                    }
                }
            }
            cost += min_length;
            std::cout << "length : " << min_length << std::endl;
        }
        std::cout << "cost : " << cost << std::endl;
        std::cout << "---------------------------" << std::endl;
    }
} // namespace icp_scan_matching
