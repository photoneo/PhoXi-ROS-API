"""
run_settings_example.launch.py

Demonstrates configuring frame_settings.* and device_settings.* before and
after configure, and changing a live device setting while the camera is active.

The driver node is started with explicit frame and device setting overrides
passed as NodeOptions parameter overrides.  These are applied to the device
during on_configure() — no extra service calls are needed.

The settings_example_caller node then:
  1. Triggers configure — the driver connects and applies all overrides.
  2. Lists all declared device_settings.* and frame_settings.* parameters.
  3. Activates the camera.
  4. Changes LaserPower live via the parameter service while the camera is
     running.

The launch file shuts everything down automatically when settings_example_caller
exits.

Usage
-----
  ros2 launch phoxi_camera_example run_settings_example.launch.py [sensor_sn:=<sn>]

With the default sensor_sn (InstalledExamples-basic-example) the example runs
against a file camera bundled with PhoXi Control — no hardware required.
"""

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
    # in code.  frame_settings.* can also be set at runtime via the parameter service
    # (see settings_example_caller).
    phoxi_camera_node = Node(
        package='phoxi_camera',
        executable='phoxi_camera_node',
        name='phoxi_camera',
        output='screen',
        parameters=[{
            'device_id':                                    LaunchConfiguration('sensor_sn'),
            'trigger_mode':                                 'Software',
            'frame_settings.PointCloud':                     True,
            'frame_settings.NormalMap':                      False,
            'frame_settings.DepthMap':                       True,
            'frame_settings.Texture':                        True,
            'frame_settings.ConfidenceMap':                  False,
            'frame_settings.ColorCameraImage':               False,
            'frame_settings.EventMap':                       False,
            'device_settings.CapturingSettings.LaserPower':  2,
            'device_settings.ScanMultiplier':                1,
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
