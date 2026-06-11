#ifndef PHOXI_CAMERA_ROSCONVERSIONS_H
#define PHOXI_CAMERA_ROSCONVERSIONS_H

#include <memory>
#include <string>

#include "phoxi_camera/PhoXiFrame.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace phoxi_camera
{

void addField(const std::string& name, int datatype, uint32_t size, uint32_t& offset,
    const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg);

// Full point cloud: XYZ + all available optional fields (normals, rgb/intensity, confidence, depth, event)
std::unique_ptr<sensor_msgs::msg::PointCloud2> phoXiFrameToRosMsg(const PhoXiFrame& frame);
// Lean point cloud: XYZ only, plus packed rgb field when textureRgb or texture is available
std::unique_ptr<sensor_msgs::msg::PointCloud2> pointsToRosMsg(const PhoXiFrame& frame);
// 32FC3 image with normalized [x, y, z] normal vectors per pixel
std::unique_ptr<sensor_msgs::msg::Image> normalMapToRosMsg(const phoxi_frame_record_t& record);
// 32FC1 image with orthogonal distances from the camera in mm
std::unique_ptr<sensor_msgs::msg::Image> depthMapToRosMsg(const phoxi_frame_record_t& record);
// 32FC1 image with measurement confidence values
std::unique_ptr<sensor_msgs::msg::Image> confidenceMapToRosMsg(const phoxi_frame_record_t& record);
// 32FC1 image with time-of-measurement values (Motion cam in Camera mode only)
std::unique_ptr<sensor_msgs::msg::Image> eventMapToRosMsg(const phoxi_frame_record_t& record);
// 32FC1 image with grayscale intensity (texture) values
std::unique_ptr<sensor_msgs::msg::Image> textureToRosMsg(const phoxi_frame_record_t& record);
// rgb16 image with RGB texture from the structured-light sensor (TEXTURE record, RGB_16 format)
std::unique_ptr<sensor_msgs::msg::Image> textureRgbToRosMsg(const phoxi_frame_record_t& record);
// rgb16 image with RGB data from the color camera
std::unique_ptr<sensor_msgs::msg::Image> colorCameraImageToRosMsg(
    const phoxi_frame_record_t& record);

struct FrameCameraInfos {
    std::unique_ptr<sensor_msgs::msg::CameraInfo> currentCamera;
    std::unique_ptr<sensor_msgs::msg::CameraInfo> currentColorCamera;
};

FrameCameraInfos frameInfoToRosMsgs(const phoxi_frame_record_t& record);

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROSCONVERSIONS_H
