#include <iomanip>
#include <sstream>

#include "phoxi_camera_msgs/msg/frame_error.hpp"
#include "phoxi_camera_msgs/msg/frame_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

static void printFrameError(const rclcpp::Logger& logger, const phoxi_camera_msgs::msg::FrameError& msg) {
    RCLCPP_ERROR(logger, "[frame_error] %zu message(s):", msg.messages.size());
    for (const auto& e : msg.messages) {
        RCLCPP_ERROR(logger, "  [code=%d severity=%d] %s", e.code, e.severity, e.text.c_str());
    }
}

static void printFrameInfo(const rclcpp::Logger& logger, const phoxi_camera_msgs::msg::FrameInfo& msg) {
    RCLCPP_INFO(logger, "[frame_info] #%d  total=%d  hw=%s", msg.index, msg.total_scan_count, msg.hw_id.c_str());
    RCLCPP_INFO(logger, "  duration=%.2f ms  computation=%.2f ms  transfer=%.2f ms", msg.duration, msg.duration_computation, msg.duration_transfer);
    RCLCPP_INFO(logger, "  timestamp=%.3f ms  early_transfer=%s", msg.timestamp, msg.is_early_transfer_frame ? "true" : "false");
    RCLCPP_INFO(logger, "  sensor_pos=[%.2f %.2f %.2f]", msg.sensor_position[0], msg.sensor_position[1], msg.sensor_position[2]);
    RCLCPP_INFO(logger, "  sensor_x=[%.3f %.3f %.3f]  y=[%.3f %.3f %.3f]  z=[%.3f %.3f %.3f]", msg.sensor_x_axis[0], msg.sensor_x_axis[1], msg.sensor_x_axis[2],
            msg.sensor_y_axis[0], msg.sensor_y_axis[1], msg.sensor_y_axis[2], msg.sensor_z_axis[0], msg.sensor_z_axis[1], msg.sensor_z_axis[2]);
    RCLCPP_INFO(logger, "  balance_rgb=[%.3f %.3f %.3f]  binning=[%d %d]  binning_factor=[%.1f %.1f]", msg.balance_rgb[0], msg.balance_rgb[1], msg.balance_rgb[2],
            msg.camera_binning[0], msg.camera_binning[1], msg.camera_binning_factor[0], msg.camera_binning_factor[1]);
    std::ostringstream tempStream;
    for (const auto& t : msg.temperature) {
        tempStream << std::fixed << std::setprecision(1) << t << " ";
    }
    RCLCPP_INFO(logger, "  temperature=[%s]°C", tempStream.str().c_str());
    RCLCPP_INFO(logger, "  ptp: %s  state=%s  ns=%ld", msg.frame_start_grand_master_identity.c_str(), msg.frame_start_port_state.c_str(), msg.frame_start_time_ns);
    RCLCPP_INFO(logger, "  marker_dots: status=%d  %s", msg.marker_dots_status, msg.marker_dots_message.c_str());
}

static void printCameraInfo(const rclcpp::Logger& logger, const std::string& topic, const sensor_msgs::msg::CameraInfo& msg) {
    RCLCPP_INFO(logger, "[%s] %ux%u  %s", topic.c_str(), msg.width, msg.height, msg.distortion_model.c_str());
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

    // Only topics whose frame_settings component is enabled will publish.
    auto pointsSub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/points", 10, [&logger](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(logger, "[points]             %ux%u, %zu fields", msg->width, msg->height, msg->fields.size());
    });

    auto normalsSub = node->create_subscription<sensor_msgs::msg::Image>("/normals", 10,
            [&logger](sensor_msgs::msg::Image::SharedPtr msg) { RCLCPP_INFO(logger, "[normals]            %ux%u %s", msg->width, msg->height, msg->encoding.c_str()); });

    auto depthSub = node->create_subscription<sensor_msgs::msg::Image>("/depth", 10,
            [&logger](sensor_msgs::msg::Image::SharedPtr msg) { RCLCPP_INFO(logger, "[depth]              %ux%u %s", msg->width, msg->height, msg->encoding.c_str()); });

    auto confidenceSub = node->create_subscription<sensor_msgs::msg::Image>("/confidence", 10,
            [&logger](sensor_msgs::msg::Image::SharedPtr msg) { RCLCPP_INFO(logger, "[confidence]         %ux%u %s", msg->width, msg->height, msg->encoding.c_str()); });

    auto intensitySub = node->create_subscription<sensor_msgs::msg::Image>("/intensity", 10,
            [&logger](sensor_msgs::msg::Image::SharedPtr msg) { RCLCPP_INFO(logger, "[intensity]          %ux%u %s", msg->width, msg->height, msg->encoding.c_str()); });

    auto textureSub = node->create_subscription<sensor_msgs::msg::Image>("/texture", 10,
            [&logger](sensor_msgs::msg::Image::SharedPtr msg) { RCLCPP_INFO(logger, "[texture]            %ux%u %s", msg->width, msg->height, msg->encoding.c_str()); });

    auto colorCameraImageSub = node->create_subscription<sensor_msgs::msg::Image>("/color_camera_image", 10,
            [&logger](sensor_msgs::msg::Image::SharedPtr msg) { RCLCPP_INFO(logger, "[color_camera_image] %ux%u %s", msg->width, msg->height, msg->encoding.c_str()); });

    auto frameInfoSub = node->create_subscription<phoxi_camera_msgs::msg::FrameInfo>(
            "/frame_info", 10, [&logger](phoxi_camera_msgs::msg::FrameInfo::SharedPtr msg) { printFrameInfo(logger, *msg); });

    auto frameInfoCameraSub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/frame_info/current_camera", 10, [&logger](sensor_msgs::msg::CameraInfo::SharedPtr msg) { printCameraInfo(logger, "frame_info/current_camera", *msg); });

    auto frameInfoColorCameraSub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/frame_info/current_color_camera", 10, [&logger](sensor_msgs::msg::CameraInfo::SharedPtr msg) { printCameraInfo(logger, "frame_info/current_color_camera", *msg); });

    auto frameErrorSub = node->create_subscription<phoxi_camera_msgs::msg::FrameError>(
            "/frame_error", 10, [&logger](phoxi_camera_msgs::msg::FrameError::SharedPtr msg) { printFrameError(logger, *msg); });

    RCLCPP_INFO(logger, "Listening on all phoxi_camera topics.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
