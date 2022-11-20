import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

simulation_dir = os.path.join(
    get_package_share_directory("ouagv_robot_description"))

share_dir = os.path.join(get_package_share_directory("ouagv_2d_slam"))


def generate_launch_description():

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(simulation_dir, "launch"),
             "/joy_move_robot.launch.py"]
        ),
        launch_arguments={"show_rviz": "true"}.items()
    )

    map_generator = Node(package="ouagv_2d_slam",
                         executable="map_generator_node", name="map_generator_node",
                         output="screen")

    ekf = Node(package="ouagv_ekf",
               executable="ouagv_ekf_node", name="ouagv_ekf_node",
               output="screen")

    map_to_odom = Node(package="tf2_ros", executable="static_transform_publisher", arguments=[
                       '0', '0', '0', '0', '0', '0', 'odom', 'map'])

    return LaunchDescription([
        simulator,
        map_to_odom,
        ekf,
        map_generator
    ])
