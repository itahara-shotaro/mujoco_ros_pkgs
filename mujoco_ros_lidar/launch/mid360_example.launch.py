from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    mujoco_ros_path = get_package_share_directory('mujoco_ros')
    lidar_path = get_package_share_directory('mujoco_ros_lidar')

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    os.path.join(mujoco_ros_path, 'launch', 'launch_server.launch.xml')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'modelfile': os.path.join(lidar_path, 'assets', 'lidar_world.xml'),
                    'mujoco_plugin_config': os.path.join(
                        lidar_path, 'config', 'mid360_example_config.ros2.yaml'
                    ),
                }.items(),
            ),
        ]
    )
