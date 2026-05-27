#include <chrono>
#include <memory>
#include <string>

#include "phoxi_camera_msgs/srv/create_profile.hpp"
#include "phoxi_camera_msgs/srv/delete_profile.hpp"
#include "phoxi_camera_msgs/srv/set_active_profile.hpp"
#include "phoxi_camera_msgs/srv/update_profile.hpp"
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

bool check(std::shared_ptr<rclcpp::Node> node, const std::string& action,
           const std::string& name, bool success, const std::string& message) {
    if (!success) {
        RCLCPP_ERROR(node->get_logger(), "%s '%s' FAILED: %s",
                     action.c_str(), name.c_str(), message.c_str());
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "%s '%s': OK", action.c_str(), name.c_str());
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("manage_profiles");

    const std::string profile_name = (argc > 1) ? argv[1] : "example_profile";

    auto create_req = std::make_shared<phoxi_camera_msgs::srv::CreateProfile::Request>();
    create_req->name = profile_name;
    auto create_resp =
        call_service<phoxi_camera_msgs::srv::CreateProfile>(node, "/phoxi_camera/profiles/create", create_req);
    if (!create_resp ||
        !check(node, "Create", profile_name, create_resp->success, create_resp->message)) {
        rclcpp::shutdown();
        return 1;
    }

    auto activate_req = std::make_shared<phoxi_camera_msgs::srv::SetActiveProfile::Request>();
    activate_req->name = profile_name;
    auto activate_resp = call_service<phoxi_camera_msgs::srv::SetActiveProfile>(
        node, "/phoxi_camera/profiles/set_active", activate_req);
    if (activate_resp) {
        check(node, "Activate", profile_name, activate_resp->success, activate_resp->message);
    }

    // Adjust device settings here if needed, then save them into the profile.
    auto update_req = std::make_shared<phoxi_camera_msgs::srv::UpdateProfile::Request>();
    update_req->name = profile_name;
    auto update_resp = call_service<phoxi_camera_msgs::srv::UpdateProfile>(
        node, "/phoxi_camera/profiles/update", update_req);
    if (update_resp) {
        check(node, "Update", profile_name, update_resp->success, update_resp->message);
    }

    auto delete_req = std::make_shared<phoxi_camera_msgs::srv::DeleteProfile::Request>();
    delete_req->name = profile_name;
    auto delete_resp = call_service<phoxi_camera_msgs::srv::DeleteProfile>(
        node, "/phoxi_camera/profiles/delete", delete_req);
    if (delete_resp) {
        check(node, "Delete", profile_name, delete_resp->success, delete_resp->message);
    }

    rclcpp::shutdown();
    return 0;
}
