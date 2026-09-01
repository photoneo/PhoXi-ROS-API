import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    config_file = os.path.join(
        get_package_share_directory('phoxi_camera'),
        'config',
        'phoxi_camera.yaml'
    )

    parameters = [config_file]
    device_id = LaunchConfiguration('device_id').perform(context)
    if device_id:
        parameters.append({'device_id': device_id})

    phoxi_camera_node = Node(
        package='phoxi_camera',
        executable='phoxi_camera_node',
        name=LaunchConfiguration('camera_name'),
        output='screen',
        parameters=parameters
    )

    return [phoxi_camera_node]

def generate_launch_description():
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='phoxi_camera',
        description='Node name. Also scopes topics, services, and the '
                     'default TF frame — set uniquely per physical camera '
                     'when running multiple instances.'
    )

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='',
        description='Hardware serial number or network ID of the target '
                     'device. Overrides the value in phoxi_camera.yaml when '
                     'set — leave unset to use the value from the config '
                     'file. Set uniquely per physical camera when running '
                     'multiple instances.'
    )

    return LaunchDescription([
        camera_name_arg,
        device_id_arg,
        OpaqueFunction(function=launch_setup)
    ])
