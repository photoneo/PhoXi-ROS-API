#ifndef PHOXI_CAMERA_ROSCONVERSIONS_H
#define PHOXI_CAMERA_ROSCONVERSIONS_H

#include <memory>
#include <string>

#include "PhoXi.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace phoxi_camera
{

void addField(const std::string& name, int datatype, uint32_t size, uint32_t& offset,
              std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg);

// Full point cloud: XYZ + all available optional fields (normals, rgb/intensity, confidence, depth, event)
std::unique_ptr<sensor_msgs::msg::PointCloud2> phoXiFrameToRosMsg(const pho::api::PFrame& pFrame);
// Lean point cloud: XYZ only, plus packed rgb field when TextureRGB or a same-resolution ColorCameraImage is available
std::unique_ptr<sensor_msgs::msg::PointCloud2> pointsToRosMsg(const pho::api::PFrame& pFrame);
// 32FC3 image with normalized [x, y, z] normal vectors per pixel
std::unique_ptr<sensor_msgs::msg::Image> normalMapToRosMsg(const pho::api::PFrame& pFrame);
// 32FC1 image with orthogonal distances from the camera in mm
std::unique_ptr<sensor_msgs::msg::Image> depthMapToRosMsg(const pho::api::PFrame& pFrame);
// 32FC1 image with measurement confidence values
std::unique_ptr<sensor_msgs::msg::Image> confidenceMapToRosMsg(const pho::api::PFrame& pFrame);
// 32FC1 image with time-of-measurement values (Motion cam in Camera mode only)
std::unique_ptr<sensor_msgs::msg::Image> eventMapToRosMsg(const pho::api::PFrame& pFrame);
// 32FC1 image with grayscale intensity (texture) values
std::unique_ptr<sensor_msgs::msg::Image> textureToRosMsg(const pho::api::PFrame& pFrame);
// rgb16 image with RGB texture from the structured-light sensor
std::unique_ptr<sensor_msgs::msg::Image> textureRgbToRosMsg(const pho::api::PFrame& pFrame);
// rgb16 image with RGB data from the color camera
std::unique_ptr<sensor_msgs::msg::Image> colorCameraImageToRosMsg(const pho::api::PFrame& pFrame);

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROSCONVERSIONS_H
