#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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

    RCLCPP_INFO(logger, "Listening on /phoxi_camera/point_cloud (combined mode).");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
