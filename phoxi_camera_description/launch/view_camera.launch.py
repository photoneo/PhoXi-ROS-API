import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='PhoXi_3D_Scanner-M-gen3',
        description='Type of PhoXi camera model'
    )

    urdf_file_path = os.path.join(
        get_package_share_directory('phoxi_camera_description'),
        'urdf',
        'phoxi_camera.urdf.xacro'
    )

    robot_description_content = Command([
        'xacro ', urdf_file_path,
        ' model:=', LaunchConfiguration('model')
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('phoxi_camera_description'), 'launch', 'view_camera.rviz')]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
