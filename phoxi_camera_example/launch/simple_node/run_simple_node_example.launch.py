import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    phoxi_camera_rviz_pkg = get_package_share_directory('phoxi_camera_rviz')

    sensor_sn_arg = DeclareLaunchArgument(
        'sensor_sn',
        default_value='InstalledExamples-basic-example',
        description='Serial number of the PhoXi device.'
    )

    # The PhoXi Camera Driver Node
    phoxi_camera_node = Node(
        package='phoxi_camera',
        executable='phoxi_camera_node',
        name='phoxi_camera',
        output='screen',
        parameters=[{'sensor_sn': LaunchConfiguration('sensor_sn')}]
    )

    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(phoxi_camera_rviz_pkg, 'rviz', 'visual_test.rviz')]
    )

    # The Service Caller Example Node
    simple_node_service_caller = Node(
        package='phoxi_camera_example',
        executable='simple_node_service_caller',
        output='screen'
    )

    # The Topic Listener Example Node
    simple_node_topic_listener = Node(
        package='phoxi_camera_example',
        executable='simple_node_topic_listener',
        output='screen'
    )

    # Add a static_transform_publisher node to create the link between a 'world' frame
    # and the camera's frame. This is essential for RViz visualization.
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_camera_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'phoxi_camera_sensor']
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        sensor_sn_arg,
        static_tf_publisher,
        phoxi_camera_node,
        rviz_node,
        simple_node_service_caller,
        simple_node_topic_listener,
        shutdown_on_exit
    ])
