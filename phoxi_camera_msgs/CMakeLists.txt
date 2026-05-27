cmake_minimum_required(VERSION 3.22)
project(phoxi_camera_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/Connect.srv"
        "srv/TriggerFrame.srv"
)

ament_package()
