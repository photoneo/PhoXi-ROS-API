#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
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
    auto node = rclcpp::Node::make_shared("topic_listener_node");
    auto logger = node->get_logger();

    // Only topics whose frameSettings component is enabled will publish.
    auto pointsSub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points", 10,
        [&logger](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(logger, "[points]             %ux%u, %zu fields",
                msg->width, msg->height, msg->fields.size());
        });

    auto normalsSub = node->create_subscription<sensor_msgs::msg::Image>(
        "/normals", 10,
        [&logger](sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(logger, "[normals]            %ux%u %s",
                msg->width, msg->height, msg->encoding.c_str());
        });

    auto depthSub = node->create_subscription<sensor_msgs::msg::Image>(
        "/depth", 10,
        [&logger](sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(logger, "[depth]              %ux%u %s",
                msg->width, msg->height, msg->encoding.c_str());
        });

    auto confidenceSub = node->create_subscription<sensor_msgs::msg::Image>(
        "/confidence", 10,
        [&logger](sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(logger, "[confidence]         %ux%u %s",
                msg->width, msg->height, msg->encoding.c_str());
        });

    auto intensitySub = node->create_subscription<sensor_msgs::msg::Image>(
        "/intensity", 10,
        [&logger](sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(logger, "[intensity]          %ux%u %s",
                msg->width, msg->height, msg->encoding.c_str());
        });

    auto textureSub = node->create_subscription<sensor_msgs::msg::Image>(
        "/texture", 10,
        [&logger](sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(logger, "[texture]            %ux%u %s",
                msg->width, msg->height, msg->encoding.c_str());
        });

    auto colorCameraImageSub = node->create_subscription<sensor_msgs::msg::Image>(
        "/color_camera_image", 10,
        [&logger](sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(logger, "[color_camera_image] %ux%u %s",
                msg->width, msg->height, msg->encoding.c_str());
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

    RCLCPP_INFO(logger, "Listening on all phoxi_camera topics.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
