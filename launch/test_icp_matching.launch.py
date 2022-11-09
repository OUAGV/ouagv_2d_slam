import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

simulation_dir = os.path.join(
    get_package_share_directory("ouagv_robot_description"))

share_dir = os.path.join(get_package_share_directory("icp_matching"))


def generate_launch_description():

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(simulation_dir, "launch"),
             "/show_diff_drive_robot.launch.py"]
        ),
        launch_arguments={"show_rviz": "false"}.items()
    )

    icp_matching = Node(package="icp_matching",
                        executable="icp_matching_node", name="icp_matching_node",
                        output="screen")

    return LaunchDescription([
        simulator,
        icp_matching])
