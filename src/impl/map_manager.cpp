#include "ouagv_2d_slam/map_manager.hpp"

namespace map_manager
{
    void MapManager::updateMap(geometry_msgs::msg::TransformStamped &estimated_pose,
                               std::vector<pointcloud_manager::PointWithNormal> &point_vec)
    {
        const double yaw = tf2::getYaw(estimated_pose.transform.rotation);
        const double x = estimated_pose.transform.translation.x;
        const double y = estimated_pose.transform.translation.y;

        // cell座標系に変換（map座標系ではない）
        const int cell_robot_x =
            floor((x + world_width * 0.5) / map_resolution);
        const int cell_robot_y =
            floor((y + world_height * 0.5) / map_resolution);

        std::lock_guard<std::mutex> lock(map_mutex);
        for (pointcloud_manager::PointWithNormal &elem : point_vec)
        {
            // cell座標系に変換（map座標系ではない）
            const double point_x = elem.point.x * cos(yaw) - elem.point.y * sin(yaw) + x + world_width * 0.5f;
            const double point_y = elem.point.x * sin(yaw) + elem.point.y * cos(yaw) + y + world_height * 0.5f;

            // gridに変換 ここでfloorを使うことで点群の数を削除してるとみなせる？
            const int cell_point_x = floor(point_x / map_resolution);
            const int cell_point_y = floor(point_y / map_resolution);

            plotProbablilityMap(cell_point_x, cell_robot_x, cell_point_y, cell_robot_y);
            const size_t index = getRasterScanIndex(map_width, cell_point_x, cell_point_y);
            float current_prob = probability_map_data.at(index);
            array_count_if_obstacle.at(index) += 1;
            array_count_all_hit.at(index) += 1;
            const float prob = static_cast<float>(array_count_if_obstacle.at(index)) /
                               static_cast<float>(array_count_all_hit.at(index));
            probability_map_data.at(index) = log_odd(prob);
        }
    }

    nav_msgs::msg::OccupancyGrid MapManager::getMapData(rclcpp::Time stamp)
    {
        nav_msgs::msg::OccupancyGrid map_;
        // OccupancyGridの座標系はmap
        map_.header.frame_id = map_frame;
        // 時間は現在時刻
        map_.header.stamp = stamp;
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
        std::lock_guard<std::mutex> lock(map_mutex);
        for (float &prob : probability_map_data)
        {
            const int integer_prob = static_cast<int>(round(get_prob_from_log_odd(prob) * 100.f));
            map_.data.at(index) = integer_prob;
            index++;
        }
        return map_;
    }

    void MapManager::plotProbablilityMap(int robot_x, int laser_x, int robot_y, int laser_y)
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
        const int ystep = robot_y < laser_y ? 1 : -1;
        int error = deltax / 2;
        int y = robot_y;
        for (int x = robot_x; x < laser_x; x++)
        {
            size_t index = 0;

            if (steep)
            {
                index = getRasterScanIndex(map_width, y, x);
            }
            else
            {
                index = getRasterScanIndex(map_width, x, y);
            }
            updateProb(index);
            error -= deltay;
            if (error < 0)
            {
                y += ystep;
                error += deltax;
            }
        }
    }

    void MapManager::updateProb(size_t index)
    {
        assert(0 <= index && index < probability_map_data.size());
        array_count_all_hit.at(index) = array_count_all_hit.at(index) + 1;
        const float prob = static_cast<float>(array_count_if_obstacle.at(index)) /
                           static_cast<float>(array_count_all_hit.at(index));
        if (prob > 0.f)
        {
            probability_map_data.at(index) = log_odd(prob);
        }
        else
        {
            probability_map_data.at(index) = log_odd(unOccupied);
        }
    }
}