"""
run_composition_example.launch.py

Demonstrates running phoxi_camera and phoxi_camera_composition_example as
composable nodes inside a shared multi-threaded component container.

Both nodes benefit from intra-process communication: the PointCloud2 message
published by phoxi_camera is delivered to the example client without any
serialisation/deserialisation overhead.

Usage
-----
  ros2 launch phoxi_camera_example run_composition_example.launch.py

Before starting the acquisition loop you must transition the phoxi_camera
lifecycle node to the active state and connect it to the device:

  # 1. Configure
  ros2 lifecycle set /phoxi_camera configure

  # 2. Activate
  ros2 lifecycle set /phoxi_camera activate

  # 3. Connect to the device
  ros2 service call /phoxi_camera/connect phoxi_camera_msgs/srv/Connect \\
      "{sn: '<device-serial-number>'}"

  # 4. Start the acquisition loop
  ros2 service call /phoxi_camera_composition_example/start std_srvs/srv/Trigger '{}'

  # 5. Stop the loop at any time
  ros2 service call /phoxi_camera_composition_example/stop std_srvs/srv/Trigger '{}'
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # ------------------------------------------------------------------ #
    #  Component container (multi-threaded so intra-process IPC works     #
    #  alongside concurrent service callbacks)                            #
    # ------------------------------------------------------------------ #
    container = ComposableNodeContainer(
        name='phoxi_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[

            # -- phoxi_camera driver (lifecycle node) ------------------- #
            # NOTE: After launch you must manually drive the lifecycle:
            #   ros2 lifecycle set /phoxi_camera configure
            #   ros2 lifecycle set /phoxi_camera activate
            #   ros2 service call /phoxi_camera/connect \
            #       phoxi_camera_msgs/srv/Connect "{sn: '<sn>'}"
            ComposableNode(
                package='phoxi_camera',
                plugin='phoxi_camera::PhoXiCamera',
                name='phoxi_camera',
                parameters=[{
                    'device_id': 'InstalledExamples-basic-example',
                    'frame_id':  'phoxi_camera_sensor',
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # -- example client ---------------------------------------- #
            ComposableNode(
                package='phoxi_camera_example',
                plugin='phoxi_camera::composition_example::CompositionExample',
                name='phoxi_camera_composition_example',
                parameters=[{
                    'trigger_service_name': '/phoxi_camera/trigger_frame',
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        container,
    ])
