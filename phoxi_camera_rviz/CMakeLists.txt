cmake_minimum_required(VERSION 3.22)
project(phoxi_camera_rviz)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
