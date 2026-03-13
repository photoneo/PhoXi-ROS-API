#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("topic_listener_node"),
                "SUCCESS: Received a PointCloud2 message with %d points.",
                msg->width * msg->height);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("topic_listener_node");

    auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud", 10,
                                                                                 topic_callback);

    RCLCPP_INFO(node->get_logger(), "Listening for point cloud messages on /point_cloud...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
