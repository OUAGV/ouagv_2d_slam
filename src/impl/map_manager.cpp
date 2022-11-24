#include "ouagv_2d_slam/map_manager.hpp"

namespace map_manager
{
  void MapManager::updateMap(
      geometry_msgs::msg::TransformStamped &estimated_pose,
      std::vector<pointcloud_manager::PointWithNormal> &point_vec)
  {
    const double yaw = tf2::getYaw(estimated_pose.transform.rotation);
    const double x = estimated_pose.transform.translation.x;
    const double y = estimated_pose.transform.translation.y;

    // cell座標系に変換（map座標系ではない）
    const int cell_robot_x = floor((x + world_width * 0.5) / map_resolution);
    const int cell_robot_y = floor((y + world_height * 0.5) / map_resolution);

    Eigen::Matrix2d rot;
    rot << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);

    for (pointcloud_manager::PointWithNormal &elem : point_vec)
    {
      // map座標系に変換
      const double point_x = elem.point.x * cos(yaw) - elem.point.y * sin(yaw) + x;
      const double point_y = elem.point.x * sin(yaw) + elem.point.y * cos(yaw) + y;

      // gridに変換
      const int cell_point_x = floor((point_x + world_width * 0.5f) / map_resolution);
      const int cell_point_y = floor((point_y + world_height * 0.5f) / map_resolution);
      const size_t index = getRasterScanIndex(map_width, cell_point_x, cell_point_y);

      // TODO ここでは最新の点をセルの代表点としているが、平均をとったほうがいいかも
      elem.point.x = point_x;
      elem.point.y = point_y;
      elem.normal = rot * elem.normal;
      elem.frame_id = "map";
      globalCellMap.at(index).point = elem;

      std::lock_guard<std::mutex> lock(map_mutex);
      plotProbablilityMap(cell_point_x, cell_robot_x, cell_point_y, cell_robot_y);
      globalCellMap.at(index).existed_num += 1;
      globalCellMap.at(index).scanned_num += 1;
      const float prob = static_cast<float>(globalCellMap.at(index).existed_num) /
                         static_cast<float>(globalCellMap.at(index).scanned_num);

      globalCellMap.at(index).prob = log_odd(prob);
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
    for (CellWithProb &elem : globalCellMap)
    {
      const int integer_prob = static_cast<int>(round(get_prob_from_log_odd(elem.prob) * 100.f));
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
    assert(index < globalCellMap.size());
    globalCellMap.at(index).scanned_num += 1;
    const float prob = static_cast<float>(globalCellMap.at(index).existed_num) /
                       static_cast<float>(globalCellMap.at(index).scanned_num);
    globalCellMap.at(index).prob = prob > 0.f ? log_odd(prob) : log_odd(unOccupied);
  }
} // namespace map_manager