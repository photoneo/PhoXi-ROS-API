cmake_minimum_required(VERSION 3.22)
project(phoxi_camera_example)

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(phoxi_camera_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(phoxi_camera_composition_example SHARED
        src/composition/composition_example.cpp)
target_include_directories(phoxi_camera_composition_example
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(phoxi_camera_composition_example
    PUBLIC
        rclcpp::rclcpp
        rclcpp_components::component
        ${sensor_msgs_TARGETS}
        ${std_srvs_TARGETS}
)

rclcpp_components_register_nodes(phoxi_camera_composition_example
        "phoxi_camera::composition_example::CompositionExample")

add_executable(phoxi_camera_composition_example_node
        src/composition/composition_example_main.cpp)
target_link_libraries(phoxi_camera_composition_example_node
    PUBLIC
        rclcpp::rclcpp
        phoxi_camera_composition_example
)

add_executable(simple_node_service_caller src/simple_node/simple_node_service_caller.cpp)
ament_target_dependencies(simple_node_service_caller
        "lifecycle_msgs"
        "phoxi_camera_msgs"
        "rclcpp"
        "std_srvs"
)

add_executable(simple_node_topic_listener src/simple_node/simple_node_topic_listener.cpp)
ament_target_dependencies(simple_node_topic_listener
        "rclcpp"
        "sensor_msgs"
)

install(
    TARGETS
        phoxi_camera_composition_example
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    TARGETS
        phoxi_camera_composition_example_node
        simple_node_service_caller
        simple_node_topic_listener
    DESTINATION
        lib/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        FILES README.md
        DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
        "lifecycle_msgs"
        "phoxi_camera_msgs"
        "rclcpp"
        "sensor_msgs"
        "std_srvs"
)
ament_package()
