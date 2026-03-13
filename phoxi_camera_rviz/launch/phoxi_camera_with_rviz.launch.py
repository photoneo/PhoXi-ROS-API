import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('phoxi_camera')
    pkg_rviz_dir = get_package_share_directory('phoxi_camera_rviz')
    config_file = os.path.join(pkg_dir, 'config', 'phoxi_camera.yaml')
    rviz_config_file = os.path.join(pkg_rviz_dir, 'rviz', 'visual_test.rviz')

    phoxi_camera_node = Node(
        package='phoxi_camera',
        executable='phoxi_camera_node',
        name='phoxi_camera',
        output='screen',
        parameters=[config_file]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        phoxi_camera_node,
        rviz_node
    ])
