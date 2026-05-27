#include <chrono>
#include <memory>

#include "phoxi_camera_msgs/srv/get_active_profile.hpp"
#include "phoxi_camera_msgs/srv/get_profile_list.hpp"
#include "phoxi_camera_msgs/srv/get_startup_profile.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

template<typename SrvT>
typename SrvT::Response::SharedPtr call_service(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& service_name,
    typename SrvT::Request::SharedPtr request = std::make_shared<typename SrvT::Request>()) {
    auto client = node->create_client<SrvT>(service_name);
    if (!client->wait_for_service(5s)) {
        RCLCPP_ERROR(node->get_logger(), "Service '%s' not available.", service_name.c_str());
        return nullptr;
    }
    return client->async_send_request(request).get();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("list_profiles");

    auto list_resp =
        call_service<phoxi_camera_msgs::srv::GetProfileList>(node, "/phoxi_camera/profiles/list");
    if (list_resp && list_resp->success) {
        RCLCPP_INFO(node->get_logger(), "%-30s  FACTORY", "NAME");
        for (size_t i = 0; i < list_resp->names.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "  %-28s  %s",
                        list_resp->names[i].c_str(),
                        list_resp->is_factory[i] ? "yes" : "no");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to get profile list: %s",
                     list_resp ? list_resp->message.c_str() : "timeout");
    }

    auto active_resp = call_service<phoxi_camera_msgs::srv::GetActiveProfile>(
        node, "/phoxi_camera/profiles/get_active");
    if (active_resp && active_resp->success) {
        RCLCPP_INFO(node->get_logger(), "Active profile: %s", active_resp->name.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to get active profile: %s",
                     active_resp ? active_resp->message.c_str() : "timeout");
    }

    auto startup_resp = call_service<phoxi_camera_msgs::srv::GetStartupProfile>(
        node, "/phoxi_camera/profiles/get_startup");
    if (startup_resp && startup_resp->success) {
        RCLCPP_INFO(node->get_logger(), "Startup profile: %s", startup_resp->name.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to get startup profile: %s",
                     startup_resp ? startup_resp->message.c_str() : "timeout");
    }

    rclcpp::shutdown();
    return 0;
}
