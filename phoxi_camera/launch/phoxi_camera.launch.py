import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('phoxi_camera'),
        'config',
        'phoxi_camera.yaml'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='phoxi_camera',
        description='Node name. Also scopes topics, services, and the '
                     'default TF frame — set uniquely per physical camera '
                     'when running multiple instances.'
    )

    phoxi_camera_node = Node(
        package='phoxi_camera',
        executable='phoxi_camera_node',
        name=LaunchConfiguration('camera_name'),
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        camera_name_arg,
        phoxi_camera_node
    ])
