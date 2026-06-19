/**
 * @file RosConversions.h
 * @brief Conversion functions from PhoXi frame records to ROS2 message types.
 *
 * All conversion functions return `unique_ptr` to avoid copies.
 * Point cloud coordinates are converted from mm (API) to metres (ROS convention).
 * Texture values are normalised to [0, 1] for both grayscale and RGB outputs.
 */
#ifndef PHOXI_CAMERA_ROSCONVERSIONS_H
#define PHOXI_CAMERA_ROSCONVERSIONS_H

#include <memory>
#include <string>

#include "phoxi_camera/PhoXiFrame.h"
#include "phoxi_camera_msgs/msg/frame_error.hpp"
#include "phoxi_camera_msgs/msg/frame_info.hpp"
#include "phoxi_camera_msgs/msg/frame_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace phoxi_camera {

/**
 * @brief Append a named field to a PointCloud2 message and advance the byte offset.
 *
 * @param name     Field name (e.g. `"x"`, `"normal_x"`, `"rgb"`).
 * @param datatype `sensor_msgs::msg::PointField` datatype constant.
 * @param size     Field size in bytes.
 * @param offset   Current byte offset into a point; incremented by `size` on return.
 * @param msg      Message to which the field descriptor is appended.
 */
void addField(const std::string& name, int datatype, uint32_t size, uint32_t& offset, const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg);

/**
 * @brief Convert a PhoXi frame to a full PointCloud2 message.
 *
 * Includes XYZ (in metres) plus all available optional fields:
 * normals, packed rgb or float intensity, confidence, depth, and event.
 *
 * @param frame Frame data; at minimum `frame.pointCloud` must be non-null.
 * @return PointCloud2 message, or an empty message if `frame.pointCloud` is null.
 */
std::unique_ptr<sensor_msgs::msg::PointCloud2> phoXiFrameToRosMsg(const PhoXiFrame& frame);

/**
 * @brief Convert a PhoXi frame to a lean XYZ-only PointCloud2 message.
 *
 * Uses only the point cloud record; ignores all other frame components.
 * Coordinates are in metres.
 *
 * @param frame Frame data; `frame.pointCloud` must be non-null.
 * @return PointCloud2 message with only x, y, z fields.
 */
std::unique_ptr<sensor_msgs::msg::PointCloud2> pointsToRosMsg(const PhoXiFrame& frame);

/**
 * @brief Convert a normal-map record to a 32FC3 ROS Image (normalised [x, y, z] per pixel).
 * @param record API frame record of type NORMAL_MAP.
 * @return Image message with encoding `"32FC3"`.
 */
std::unique_ptr<sensor_msgs::msg::Image> normalMapToRosMsg(const phoxi_frame_record_t& record);

/**
 * @brief Convert a depth-map record to a 32FC1 ROS Image (orthogonal distance in mm).
 * @param record API frame record of type DEPTH_MAP.
 * @return Image message with encoding `"32FC1"`.
 */
std::unique_ptr<sensor_msgs::msg::Image> depthMapToRosMsg(const phoxi_frame_record_t& record);

/**
 * @brief Convert a confidence-map record to a 32FC1 ROS Image.
 * @param record API frame record of type CONFIDENCE_MAP.
 * @return Image message with encoding `"32FC1"`, values in [0, 1].
 */
std::unique_ptr<sensor_msgs::msg::Image> confidenceMapToRosMsg(const phoxi_frame_record_t& record);

/**
 * @brief Convert an event-map record to a 32FC1 ROS Image (MotionCam Camera mode only).
 * @param record API frame record of type EVENT_MAP.
 * @return Image message with encoding `"32FC1"`.
 */
std::unique_ptr<sensor_msgs::msg::Image> eventMapToRosMsg(const phoxi_frame_record_t& record);

/**
 * @brief Convert a grayscale texture record to a normalised 32FC1 ROS Image.
 *
 * Raw API values are in [0, 2047]; output values are normalised to [0, 1].
 *
 * @param record API frame record of type TEXTURE (FLOAT_32F variant).
 * @return Image message with encoding `"32FC1"`.
 */
std::unique_ptr<sensor_msgs::msg::Image> textureToRosMsg(const phoxi_frame_record_t& record);

/**
 * @brief Convert an RGB texture record to an rgb8 ROS Image.
 *
 * Raw API uint16 values are in [0, 1023]; output values are 8-bit scaled to [0, 255].
 *
 * @param record API frame record of type TEXTURE (RGB_16 variant).
 * @return Image message with encoding `"rgb8"`.
 */
std::unique_ptr<sensor_msgs::msg::Image> textureRgbToRosMsg(const phoxi_frame_record_t& record);

/**
 * @brief Convert a color camera record to an rgb8 ROS Image.
 *
 * Uses the same conversion as `textureRgbToRosMsg`.
 *
 * @param record API frame record from the color camera.
 * @return Image message with encoding `"rgb8"`.
 */
std::unique_ptr<sensor_msgs::msg::Image> colorCameraImageToRosMsg(const phoxi_frame_record_t& record);

/**
 * @brief Result of parsing a FRAME_INFO record.
 *
 * On a failed frame (`successful == false`), only `frame_error` is populated.
 * On success the remaining fields are filled in as available from the JSON payload.
 */
struct ParsedFrameInfo {
    bool successful = true;                                            ///< False if the API reported a scan failure.
    std::unique_ptr<phoxi_camera_msgs::msg::FrameError> frameError;    ///< Non-null only when `successful` is false.
    std::unique_ptr<phoxi_camera_msgs::msg::FrameInfo> frameInfo;      ///< Per-frame metadata (index, timing, pose, …).
    std::unique_ptr<sensor_msgs::msg::CameraInfo> currentCamera;       ///< Primary camera intrinsics, if available.
    std::unique_ptr<sensor_msgs::msg::CameraInfo> currentColorCamera;  ///< Color camera intrinsics, if available.
};

/**
 * @brief Parse a FRAME_INFO record into structured ROS2 messages.
 *
 * The record payload is JSON produced by the API.  Parsing is best-effort:
 * missing optional fields are silently skipped.
 *
 * @param record API FRAME_INFO record.
 * @return Parsed result; check `successful` before using the other fields.
 */
ParsedFrameInfo parseFrameInfo(const phoxi_frame_record_t& record);

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROSCONVERSIONS_H
