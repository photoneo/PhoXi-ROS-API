cmake_minimum_required(VERSION 3.22)
project(phoxi_camera)

set(CMAKE_PREFIX_PATH $ENV{PHOXI_CONTROL_PATH} ${CMAKE_PREFIX_PATH})

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(phoxi_camera_msgs REQUIRED)
find_package(phoxi_camera_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-w -fpermissive -pthread -O2)

add_library(${PROJECT_NAME}_PhoXi_Interface SHARED src/PhoXiInterface.cpp)
target_link_libraries(${PROJECT_NAME}_PhoXi_Interface
    PUBLIC
        phoxi_camera_vendor::phoxi_camera_vendor
    PRIVATE
        rt
        pthread
)

add_library(${PROJECT_NAME}_Ros_Interface SHARED
        src/PhoXiCamera.cpp
        src/RosConversions.cpp)

target_link_libraries(${PROJECT_NAME}_Ros_Interface
    PUBLIC
        ${PROJECT_NAME}_PhoXi_Interface
        ${phoxi_camera_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_srvs_TARGETS}
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
    PRIVATE
        ${lifecycle_msgs_TARGETS}
        rclcpp_components::component
)

# Register RosInterface as a composable-node plugin.
rclcpp_components_register_nodes(${PROJECT_NAME}_Ros_Interface "phoxi_camera::PhoXiCamera")

add_executable(${PROJECT_NAME}_node src/phoxi_camera_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}_Ros_Interface
)

install(
        TARGETS ${PROJECT_NAME}_PhoXi_Interface ${PROJECT_NAME}_Ros_Interface
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
)

install(
        TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
        lifecycle_msgs
        phoxi_camera_msgs
        phoxi_camera_vendor
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        sensor_msgs
        std_srvs
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_gmock REQUIRED)

    ament_add_gmock(${PROJECT_NAME}_ros_interface_test
            test_mock/test_ros_interface.cpp
            TIMEOUT 200
    )
    target_link_libraries(${PROJECT_NAME}_ros_interface_test
            ${PROJECT_NAME}_Ros_Interface
    )
    target_include_directories(${PROJECT_NAME}_ros_interface_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_mock)

    ament_add_gtest(${PROJECT_NAME}_conversions_test
            test/test_conversions.cpp
            TIMEOUT 200
    )
    target_link_libraries(${PROJECT_NAME}_conversions_test
            ${PROJECT_NAME}_Ros_Interface
    )

    ament_add_gmock(${PROJECT_NAME}_frame_settings_test
            test_mock/test_frame_settings.cpp
            TIMEOUT 200
    )
    target_link_libraries(${PROJECT_NAME}_frame_settings_test
            ${PROJECT_NAME}_Ros_Interface
    )
    target_include_directories(${PROJECT_NAME}_frame_settings_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_mock)

    ament_add_gmock(${PROJECT_NAME}_setting_types_test
            test_mock/test_setting_types.cpp
            TIMEOUT 200
    )
    target_link_libraries(${PROJECT_NAME}_setting_types_test
            ${PROJECT_NAME}_Ros_Interface
    )
    target_include_directories(${PROJECT_NAME}_setting_types_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_mock)

    ament_add_gmock(${PROJECT_NAME}_device_settings_params_test
            test_mock/test_device_settings_params.cpp
            TIMEOUT 200
    )
    target_link_libraries(${PROJECT_NAME}_device_settings_params_test
            ${PROJECT_NAME}_Ros_Interface
    )
    target_include_directories(${PROJECT_NAME}_device_settings_params_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_mock)

    ament_add_gmock(${PROJECT_NAME}_device_info_params_test
            test_mock/test_device_info_params.cpp
            TIMEOUT 200
    )
    target_link_libraries(${PROJECT_NAME}_device_info_params_test
            ${PROJECT_NAME}_Ros_Interface
    )
    target_include_directories(${PROJECT_NAME}_device_info_params_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_mock)

    ament_add_gtest(${PROJECT_NAME}_hardware_integration_test
            test/test_hardware_integration.cpp
            TIMEOUT 200
    )
    target_link_libraries(${PROJECT_NAME}_hardware_integration_test
            ${PROJECT_NAME}_Ros_Interface
    )
    target_include_directories(${PROJECT_NAME}_hardware_integration_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

    ament_add_gtest(${PROJECT_NAME}_get_frame_test
            test/test_get_frame.cpp
            TIMEOUT 300
    )
    target_link_libraries(${PROJECT_NAME}_get_frame_test
            ${PROJECT_NAME}_Ros_Interface
            message_filters::message_filters
    )
    target_include_directories(${PROJECT_NAME}_get_frame_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

    ament_add_gtest(${PROJECT_NAME}_settings_test
            test/test_settings.cpp
            TIMEOUT 120
    )
    target_link_libraries(${PROJECT_NAME}_settings_test
            ${PROJECT_NAME}_Ros_Interface
    )
    target_include_directories(${PROJECT_NAME}_settings_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

    ament_add_gtest(${PROJECT_NAME}_get_frame_individual_test
            test/test_get_frame_individual.cpp
            TIMEOUT 600
    )
    target_link_libraries(${PROJECT_NAME}_get_frame_individual_test
            ${PROJECT_NAME}_Ros_Interface
            message_filters::message_filters
    )
    target_include_directories(${PROJECT_NAME}_get_frame_individual_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

ament_package()
