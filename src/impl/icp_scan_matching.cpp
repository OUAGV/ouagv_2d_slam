#include "ouagv_2d_slam/icp_scan_matching.hpp"

namespace icp_scan_matching
{
    geometry_msgs::msg::PoseWithCovarianceStamped IcpScanMatching::estimatePose(
        std::vector<map_manager::CellWithProb> &global_map,
        std::vector<pointcloud_manager::PointWithNormal> &scanned_points,
        geometry_msgs::msg::TransformStamped &predict_pose,
        rclcpp::Time stamp)
    {
        const double yaw = tf2::getYaw(predict_pose.transform.rotation);
        const double x = predict_pose.transform.translation.x;
        const double y = predict_pose.transform.translation.y;
        std::vector<pointcloud_manager::PointWithNormal> reference_points;
        std::vector<pointcloud_manager::PointWithNormal> target_points;
        associatePoints(global_map, scanned_points, reference_points, target_points, x, y, yaw);
        optimize(reference_points, target_points, x, y, yaw);
        std::cout << " x : " << estimated_x;
        std::cout << " y : " << estimated_y;
        std::cout << " theta : " << estimated_theta << std::endl;
        std::cout << "---------------------------" << std::endl;
        geometry_msgs::msg::PoseWithCovarianceStamped ret;
        ret.header.frame_id = "map";
        ret.header.stamp = stamp;
        ret.pose.pose.position.x = estimated_x;
        ret.pose.pose.position.y = estimated_y;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, estimated_theta);
        ret.pose.pose.orientation.x = quat.getX();
        ret.pose.pose.orientation.y = quat.getY();
        ret.pose.pose.orientation.z = quat.getZ();
        ret.pose.pose.orientation.w = quat.getW();
        return ret;
    }

    void IcpScanMatching::associatePoints(
        std::vector<map_manager::CellWithProb> &global_map,
        std::vector<pointcloud_manager::PointWithNormal> &scanned_points,
        std::vector<pointcloud_manager::PointWithNormal> &reference_points,
        std::vector<pointcloud_manager::PointWithNormal> &target_points,
        double x, double y, double yaw)
    {

        for (pointcloud_manager::PointWithNormal &point : scanned_points)
        {

            const double point_x = point.point.x * cos(yaw) - point.point.y * sin(yaw) + x;
            const double point_y = point.point.x * sin(yaw) + point.point.y * cos(yaw) + y;

            const int cell_point_x = floor((point_x + world_width * 0.5f) / map_resolution);
            const int cell_point_y = floor((point_y + world_height * 0.5f) / map_resolution);

            double min_length = 0.f;
            size_t min_index = 0;
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
                    const double length = sqrt(
                        pow((global_map.at(index).point.point.x - point_x), 2) +
                        pow((global_map.at(index).point.point.y - point_y), 2));
                    if (isFirst)
                    {
                        min_length = length;
                        min_index = index;
                        isFirst = false;
                    }
                    else
                    {
                        min_length = (min_length > length) ? length : min_length;
                        min_index = (min_length > length) ? index : min_index;
                    }
                }
            }
            if (!isFirst && min_length < threthold)
            {
                reference_points.emplace_back(global_map.at(min_index).point);
                target_points.emplace_back(point);
            }

            // std::cout << "length : " << min_length << std::endl;
        }
    }

    void IcpScanMatching::optimize(std::vector<pointcloud_manager::PointWithNormal> &reference_points,
                                   std::vector<pointcloud_manager::PointWithNormal> &target_points,
                                   double initial_x, double initial_y, double initial_theta)
    {
        double x = initial_x;
        double y = initial_y;
        double theta = initial_theta;
        estimated_x = x;
        estimated_y = y;
        estimated_theta = theta;
        double min_cost = 999999999.99;
        double last_cost = min_cost;
        // 最急降下法のステップ幅
        const double kk = 0.00001;
        // 数値偏微分（並進）用
        const double dd = 0.00001;
        // 数値偏微分（回転）用
        const double da = 0.00001;
        // 前回のコストと比べてどれくらい変化したか
        const double cost_diff_threthold = 0.5;
        const int loop_count_limit = 50;
        int count = 0;
        if (reference_points.size() < 10)
        {
            std::cout << "Number of associated points is too little." << std::endl;
            return;
        }
        double cost = getCost(x, y, theta, reference_points, target_points);
        while (count < loop_count_limit && abs(last_cost - cost) > cost_diff_threthold)
        {
            count++;
            last_cost = cost;
            const double dEtx = (getCost(x + dd, y, theta, reference_points, target_points) - cost) / dd;
            const double dEty = (getCost(x + dd, y, theta, reference_points, target_points) - cost) / dd;
            const double dEtheta = (getCost(x, y, theta + da, reference_points, target_points) - cost) / da;

            const double dx = -kk * dEtx;
            const double dy = -kk * dEty;
            const double dtheta = -kk * dEtheta;

            x += dx;
            y += dy;
            theta += dtheta;

            cost = getCost(x, y, theta, reference_points, target_points);

            if (cost < min_cost)
            {
                min_cost = cost;
                estimated_x = x;
                estimated_y = y;
                estimated_theta = theta;
            }
            // std::cout << "cost: " << cost << "min cost " << min_cost << std::endl;
        }
        if (count == loop_count_limit)
        {
            std::cout << "point optimize loop count is up to limitation." << std::endl;
        }
        else
        {
            std::cout << "optimized." << std::endl;
        }
    }

    double IcpScanMatching::getCost(double x, double y, double theta,
                                    std::vector<pointcloud_manager::PointWithNormal> &reference_points,
                                    std::vector<pointcloud_manager::PointWithNormal> &target_points)
    {
        double cost_sum = 0.0;
        for (size_t i = 0; i < reference_points.size(); i++)
        {
            const double point_x = target_points.at(i).point.x * cos(theta) - target_points.at(i).point.y * sin(theta) + x;
            const double point_y = target_points.at(i).point.x * sin(theta) + target_points.at(i).point.y * cos(theta) + y;

            // TODO ここの計算を変える
            const double cost = sqrt(pow(reference_points.at(i).point.x - point_x, 2) +
                                     pow(reference_points.at(i).point.y - point_y, 2));
            cost_sum += cost;
        }
        return cost_sum;
    }
} // namespace icp_scan_matching
