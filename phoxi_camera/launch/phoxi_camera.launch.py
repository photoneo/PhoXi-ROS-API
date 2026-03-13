import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('phoxi_camera'),
        'config',
        'phoxi_camera.yaml'
    )

    phoxi_camera_node = Node(
        package='phoxi_camera',
        executable='phoxi_camera_node',
        name='phoxi_camera',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        phoxi_camera_node
    ])
