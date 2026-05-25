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

std::unique_ptr<sensor_msgs::msg::PointCloud2> phoXiFrameToRosMsg(const pho::api::PFrame& pFrame);

std::unique_ptr<sensor_msgs::msg::Image> colorCameraImageToRosMsg(const pho::api::PFrame& pFrame);

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROSCONVERSIONS_H
