#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

static void printCameraInfo(const rclcpp::Logger& logger, const std::string& topic,
    const sensor_msgs::msg::CameraInfo& msg)
{
    RCLCPP_INFO(logger, "[%s] %ux%u  %s", topic.c_str(), msg.width, msg.height,
        msg.distortion_model.c_str());
    RCLCPP_INFO(logger, "  K: [%9.3f %9.3f %9.3f]", msg.k[0], msg.k[1], msg.k[2]);
    RCLCPP_INFO(logger, "     [%9.3f %9.3f %9.3f]", msg.k[3], msg.k[4], msg.k[5]);
    RCLCPP_INFO(logger, "     [%9.3f %9.3f %9.3f]", msg.k[6], msg.k[7], msg.k[8]);
    std::ostringstream dStream;
    for (const auto& v : msg.d) {
        dStream << " " << std::fixed << std::setprecision(4) << v;
    }
    RCLCPP_INFO(logger, "  D:%s", dStream.str().c_str());
    RCLCPP_INFO(logger, "  R: [%9.3f %9.3f %9.3f]", msg.r[0], msg.r[1], msg.r[2]);
    RCLCPP_INFO(logger, "     [%9.3f %9.3f %9.3f]", msg.r[3], msg.r[4], msg.r[5]);
    RCLCPP_INFO(logger, "     [%9.3f %9.3f %9.3f]", msg.r[6], msg.r[7], msg.r[8]);
    RCLCPP_INFO(logger, "  P: [%9.3f %9.3f %9.3f %9.3f]", msg.p[0], msg.p[1], msg.p[2], msg.p[3]);
    RCLCPP_INFO(logger, "     [%9.3f %9.3f %9.3f %9.3f]", msg.p[4], msg.p[5], msg.p[6], msg.p[7]);
    RCLCPP_INFO(logger, "     [%9.3f %9.3f %9.3f %9.3f]", msg.p[8], msg.p[9], msg.p[10], msg.p[11]);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("combined_point_cloud_listener_node");
    auto logger = node->get_logger();

    // In publish_combined mode the camera publishes a single dense PointCloud2
    // on "point_cloud" carrying all enabled fields. Individual topics are not
    // published in this mode.
    auto pointCloudSub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud", 10,
        [&logger](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(logger, "Received combined PointCloud2: %ux%u, %zu fields:",
                msg->width, msg->height, msg->fields.size());
            for (const auto& field : msg->fields) {
                RCLCPP_INFO(logger, "  %s (offset %u)", field.name.c_str(), field.offset);
            }
        });

    auto frameInfoCameraSub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/frameInfo/currentCamera", 10,
        [&logger](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            printCameraInfo(logger, "frameInfo/currentCamera", *msg);
        });

    auto frameInfoColorCameraSub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/frameInfo/currentColorCamera", 10,
        [&logger](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            printCameraInfo(logger, "frameInfo/currentColorCamera", *msg);
        });

    RCLCPP_INFO(logger, "Listening on /phoxi_camera/point_cloud (combined mode).");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
