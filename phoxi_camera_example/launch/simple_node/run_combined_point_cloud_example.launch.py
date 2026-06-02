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

    # publish_combined=true: the camera publishes a single dense PointCloud2 on
    # the "point_cloud" topic carrying all enabled fields (XYZ + normals +
    # texture + confidence + depth) in one message.  Individual topics are not
    # published in this mode.
    phoxi_camera_node = Node(
        package='phoxi_camera',
        executable='phoxi_camera_node',
        name='phoxi_camera',
        output='screen',
        parameters=[{
            'device_id':        LaunchConfiguration('sensor_sn'),
            'publish_combined': True,
            'frameSettings/PointCloud':    True,
            'frameSettings/NormalMap':     True,
            'frameSettings/DepthMap':      True,
            'frameSettings/Texture':       True,
            'frameSettings/ConfidenceMap': True,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(example_pkg, 'rviz', 'combined_point_cloud.rviz')]
    )

    combined_service_caller = Node(
        package='phoxi_camera_example',
        executable='combined_point_cloud_service_caller',
        output='screen'
    )

    combined_topic_listener = Node(
        package='phoxi_camera_example',
        executable='combined_point_cloud_topic_listener',
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
        combined_service_caller,
        combined_topic_listener,
        shutdown_on_exit,
    ])
