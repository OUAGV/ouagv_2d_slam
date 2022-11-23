
#include "ouagv_2d_slam/pointcloud_manager.hpp"

namespace pointcloud_manager
{

    void PointCloudManager::scanToPoints(sensor_msgs::msg::LaserScan::SharedPtr msg,
                                         std::vector<PointWithNormal> &vec)
    {
        toPoints(msg, vec);
        resamplePoints(vec);
        analysePoints(vec);
    }

    void PointCloudManager::toPoints(sensor_msgs::msg::LaserScan::SharedPtr msg,
                                     std::vector<PointWithNormal> &vec)
    {
        float current_angle = msg->angle_min;
        for (float &scan : msg->ranges)
        {
            if (msg->range_min <= scan && scan <= msg->range_max)
            {
                PointWithNormal elem;
                elem.frame_id = laser_frame;
                elem.point.x = scan * cos(current_angle);
                elem.point.y = scan * sin(current_angle);
                vec.emplace_back(elem);
                current_angle += msg->angle_increment;
            }
        };
    }

    void PointCloudManager::resamplePoints(std::vector<PointWithNormal> &vec)
    {
        const float point_interval_m = 0.05f;           // [m]
        const float interpolate_threthold_max_m = 0.3f; // [m]
        assert(vec.size() > 0);
        float distance_sum = 0.f;
        geometry_msgs::msg::Point last_point = vec.at(0).point;
        geometry_msgs::msg::Point new_point = vec.at(0).point;
        std::vector<PointWithNormal> interpolated_points;
        interpolated_points.emplace_back(vec.at(0));

        for (int i = 1; i < static_cast<int>(vec.size()); i++)
        {
            const geometry_msgs::msg::Point &current_point = vec.at(i).point;
            const float dx = current_point.x - last_point.x;
            const float dy = current_point.y - last_point.y;
            const float distance_between_neighbor_points = sqrt(pow(dx, 2) + pow(dy, 2));

            bool exists = false;
            bool isInserted = false;
            if (distance_sum + distance_between_neighbor_points < point_interval_m)
            {
                distance_sum += distance_between_neighbor_points;
            }
            else if (distance_sum + distance_between_neighbor_points >= interpolate_threthold_max_m)
            {
                new_point = current_point;
                exists = true;
            }
            else
            {
                const float ratio = (point_interval_m - distance_sum) / distance_between_neighbor_points;

                new_point.x = dx * ratio + last_point.x;
                new_point.y = dy * ratio + last_point.y;
                exists = true;
                isInserted = true;
            }
            if (exists)
            {
                PointWithNormal new_point_with_normal;
                new_point_with_normal.point = new_point;
                interpolated_points.emplace_back(new_point_with_normal);
                last_point = new_point;
                distance_sum = 0;
                if (isInserted)
                {
                    i--;
                }
            }
            else
            {
                last_point = current_point;
            }
        }
        vec = interpolated_points;
    }

    void PointCloudManager::analysePoints(std::vector<PointWithNormal> &vec)
    {
        const double INVALID = -1.0;
        for (size_t index = 0; index < vec.size(); index++)
        {
            Eigen::Vector2d nL, nR;
            PointWithNormal &current_elem = vec.at(index);
            bool flagL = calNormal(index, vec, false, nL); // nLはlpと左側の点で求めた法線ベクトル
            bool flagR = calNormal(index, vec, true, nR);  // nRはlpと右側の点で求めた法線ベクトル
            nR(0) = -nR(0);                                // 符号をnLと合せる
            nR(1) = -nR(1);
            if (flagL)
            {
                if (flagR)
                { // 左右両側で法線ベクトルが計算可能
                    if (fabs(nL(0) * nR(0) + nL(1) * nR(1)) >= cos(M_PI_4))
                    {
                        // 両側の法線が平行に近い
                        current_elem.type = LINE; // 直線とみなす
                    }
                    else
                    { // 平行から遠ければ、コーナ点とみなす
                        current_elem.type = CORNER;
                    }
                    // 左右両側の法線ベクトルの平均
                    const double dx = nL(0) + nR(0);
                    const double dy = nL(1) + nR(1);
                    const double L = sqrt(dx * dx + dy * dy);
                    current_elem.normal(0) = dx / L;
                    current_elem.normal(1) = dy / L;
                }
                else
                { // 左側しか法線ベクトルがとれなかった
                    current_elem.type = LINE;
                    current_elem.normal = nL;
                }
            }
            else
            {
                if (flagR)
                { // 右側しか法線ベクトルがとれなかった
                    current_elem.type = LINE;
                    current_elem.normal = nR;
                }
                else
                {
                    // 両側とも法線ベクトルがとれなかった
                    current_elem.type = ISOLATE; // 孤立点とみなす
                    current_elem.normal(0) = INVALID;
                    current_elem.normal(1) = INVALID;
                }
            }
        }
    }

    // 注目点cpの両側の点が、cpからdmin以上dmax以下の場合に、法線を計算する。
    bool PointCloudManager::calNormal(size_t idx, std::vector<PointWithNormal> &lps, bool isForward, Eigen::Vector2d &normal)
    {
        const double FPDMIN = 0.06; // ScanPointResampler.dthrSとずらす
        const double FPDMAX = 1.0;
        const int dir = isForward ? 1 : -1;
        const PointWithNormal &cp = lps[idx]; // 注目点
        for (int i = idx + dir; i >= 0 && i < static_cast<int>(lps.size()); i += dir)
        {
            const PointWithNormal &lp = lps[i]; // cpのdir（左か右）側の点
            const double dx = lp.point.x - cp.point.x;
            const double dy = lp.point.y - cp.point.y;
            const double d = sqrt(pow(dx, 2) + pow(dy, 2));
            if (d >= FPDMIN && d <= FPDMAX)
            { // cpとlpの距離dが適切なら法線計算
                normal(0) = dy / d;
                normal(1) = -dx / d;
                return true;
            }

            if (d > FPDMAX) // もはやどんどん離れるので、途中でやめる
                break;
        }

        return false;
    }
}
