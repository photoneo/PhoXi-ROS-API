#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("topic_listener_node"), "SUCCESS: Received a PointCloud2 message with %d points.", msg->width * msg->height);
}

void color_camera_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("topic_listener_node"), "SUCCESS: Received a ColorCameraImage message %ux%u encoding=%s.", msg->width, msg->height, msg->encoding.c_str());
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("topic_listener_node");

    auto point_cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud", 10, point_cloud_callback);
    auto color_camera_image_sub = node->create_subscription<sensor_msgs::msg::Image>("/color_camera_image", 10, color_camera_image_callback);

    RCLCPP_INFO(node->get_logger(), "Listening for messages on /point_cloud and /color_camera_image...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
