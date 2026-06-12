import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    example_pkg = get_package_share_directory('phoxi_camera_example')

    sensor_sn_arg = DeclareLaunchArgument(
        'sensor_sn',
        default_value='InstalledExamples-basic-example',
        description='Serial number of the PhoXi device.'
    )

    # The PhoXi Camera Driver Node.
    #
    # Frame output settings can be configured statically here as parameters.
    # The camera node picks them up in on_configure and sends them to the device
    # before acquisition starts — so only the listed components are transferred.
    #
    # Alternatively, frame settings can be changed at runtime via:
    #   ros2 param set /phoxi_camera frameSettings.NormalMap false
    #
    # Note: parameter names use '/' as the separator, so they must be written as
    # flat keys (not nested dicts) in this parameters dict.
    phoxi_camera_node = Node(
        package='phoxi_camera',
        executable='phoxi_camera_node',
        name='phoxi_camera',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('sensor_sn'),
            # Frame output settings — comment out entries to leave the device
            # default unchanged, or set to false to disable a component.
            'frameSettings.PointCloud':       True,
            'frameSettings.NormalMap':        False,
            'frameSettings.DepthMap':         True,
            'frameSettings.Texture':          True,
            'frameSettings.ConfidenceMap':    False,
            'frameSettings.ColorCameraImage': False,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(example_pkg, 'rviz', 'simple_node.rviz')]
    )

    simple_node_service_caller = Node(
        package='phoxi_camera_example',
        executable='simple_node_service_caller',
        output='screen'
    )

    simple_node_topic_listener = Node(
        package='phoxi_camera_example',
        executable='simple_node_topic_listener',
        output='screen'
    )

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
        shutdown_on_exit,
    ])
