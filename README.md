# ouagv_2d_slam
ICPマッチングでSLAMするプログラムですが、動きません
## environment
- ROS2 galactic
- Ubuntu 20.04
- Gazebo上でのみテスト

## Subscribe

LiDAR からのデータ

- `/scan` (sensor_msgs::msg::LaserScan)

デッドレコニングで得られた相対位置

- `/odom` (geometry_msgs::msg::PoseStamped)

## Publish

推定後の位置

- `/estimated_pose` (geometry_msgs::msg::PoseStamped)

マップ

- `/map` (nav_msgs::msg::OccupancyGrid)
