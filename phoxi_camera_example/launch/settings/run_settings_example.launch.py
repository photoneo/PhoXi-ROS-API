import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    sensor_sn_arg = DeclareLaunchArgument(
        'sensor_sn',
        default_value='InstalledExamples-basic-example',
        description='Serial number of the PhoXi device.'
    )

    # Device and frame settings supplied as inline parameters are applied to the
    # device during on_configure(), equivalent to a YAML params file but specified
    # in code.  frameSettings.* can also be set at runtime via the parameter service
    # (see settings_example_caller).
    phoxi_camera_node = Node(
        package='phoxi_camera',
        executable='phoxi_camera_node',
        name='phoxi_camera',
        output='screen',
        parameters=[{
            'device_id':                                    LaunchConfiguration('sensor_sn'),
            'frameSettings.PointCloud':                     True,
            'frameSettings.NormalMap':                      False,
            'frameSettings.DepthMap':                       True,
            'frameSettings.Texture':                        True,
            'frameSettings.ConfidenceMap':                  False,
            'frameSettings.ColorCameraImage':               False,
            'frameSettings.EventMap':                       False,
            'deviceSettings.CapturingSettings.LaserPower':  2,
            'deviceSettings.ScanMultiplier':                1,
        }]
    )

    settings_caller = Node(
        package='phoxi_camera_example',
        executable='settings_example_caller',
        output='screen'
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=settings_caller,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        sensor_sn_arg,
        phoxi_camera_node,
        settings_caller,
        shutdown_on_exit,
    ])
