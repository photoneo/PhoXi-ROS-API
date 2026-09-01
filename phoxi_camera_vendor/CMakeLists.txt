cmake_minimum_required(VERSION 3.22)
project(phoxi_camera_vendor)

find_package(ament_cmake REQUIRED)
find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

get_target_property(PHOXI_LIB phoxi::phoxi IMPORTED_LOCATION)
get_filename_component(PHOXI_LIB_DIR "${PHOXI_LIB}" DIRECTORY)
configure_file(env-hooks/phoxi_camera_vendor.dsv.in env-hooks/phoxi_camera_vendor.dsv @ONLY)
ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/env-hooks/phoxi_camera_vendor.dsv")

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE phoxi::phoxi)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(PhoXi)

ament_package()

