#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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

    RCLCPP_INFO(logger, "Listening on all phoxi_camera topics.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
