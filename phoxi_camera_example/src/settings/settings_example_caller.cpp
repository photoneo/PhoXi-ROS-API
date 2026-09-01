#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

bool changeLifecycleState(std::shared_ptr<rclcpp::Node> node, uint8_t transition) {
    const auto client = node->create_client<lifecycle_msgs::srv::ChangeState>("/phoxi_camera/change_state");
    if (!client->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Lifecycle service not available.");
        return false;
    }
    const auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    const auto result = client->async_send_request(request).get();
    if (!result || !result->success) {
        RCLCPP_ERROR(node->get_logger(), "Transition %u failed.", static_cast<unsigned int>(transition));
        return false;
    }
    return true;
}

// Set a parameter on the phoxi_camera node via the parameter service.
// device_settings.* are only declared after configure, so call this post-configure.
bool setParameter(std::shared_ptr<rclcpp::Node> node, const rclcpp::Parameter& param) {
    const auto client = node->create_client<rcl_interfaces::srv::SetParameters>("/phoxi_camera/set_parameters");
    if (!client->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Parameter service not available.");
        return false;
    }
    const auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(param.to_parameter_msg());
    const auto result = client->async_send_request(request).get();
    if (!result || result->results.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Set parameter call failed.");
        return false;
    }
    if (!result->results[0].successful) {
        RCLCPP_ERROR(node->get_logger(), "Failed to set parameter: %s", result->results[0].reason.c_str());
        return false;
    }
    return true;
}

// List all declared device_settings.* and frame_settings.* parameters with values.
// device_settings.* are only declared after configure.
void listSettings(std::shared_ptr<rclcpp::Node> node) {
    auto listClient = node->create_client<rcl_interfaces::srv::ListParameters>("/phoxi_camera/list_parameters");
    auto getClient = node->create_client<rcl_interfaces::srv::GetParameters>("/phoxi_camera/get_parameters");
    if (!listClient->wait_for_service(3s) || !getClient->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Parameter service not available.");
        return;
    }
    for (const auto& prefix : {"device_info", "device_settings", "frame_settings"}) {
        auto listReq = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
        listReq->prefixes = {prefix};
        listReq->depth = 100;
        auto listResp = listClient->async_send_request(listReq).get();
        if (!listResp || listResp->result.names.empty()) {
            continue;
        }
        auto getReq = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        getReq->names = listResp->result.names;
        auto getResp = getClient->async_send_request(getReq).get();

        RCLCPP_INFO(node->get_logger(), "%s (%zu):", prefix, listResp->result.names.size());
        for (size_t i = 0; i < listResp->result.names.size(); ++i) {
            const rclcpp::Parameter p(listResp->result.names[i], rclcpp::ParameterValue(getResp->values[i]));
            RCLCPP_INFO(node->get_logger(), "  %s = %s", p.get_name().c_str(), p.value_to_string().c_str());
        }
    }
}

void runWorkflow(std::shared_ptr<rclcpp::Node> node) {
    // frame_settings.* and device_settings.* overrides are supplied via the launch file.

    // 1. Configure — connects to device, loads schema, and applies all launch-file overrides.
    RCLCPP_INFO(node->get_logger(), "Configuring.");
    if (!changeLifecycleState(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
        rclcpp::shutdown();
        return;
    }

    // 2. List all available settings — device_settings.* are device-specific and
    //    only declared after configure.
    listSettings(node);

    // 3. Activate — starts acquisition.
    RCLCPP_INFO(node->get_logger(), "Activating.");
    if (!changeLifecycleState(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
        rclcpp::shutdown();
        return;
    }

    // 4. Live change — update a device setting while the camera is active.
    std::this_thread::sleep_for(2s);
    RCLCPP_INFO(node->get_logger(), "Setting LaserPower to 3.");
    setParameter(node, rclcpp::Parameter("device_settings.CapturingSettings.LaserPower", 3));

    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("settings_example_caller");
    std::thread workflowThread(runWorkflow, node);
    rclcpp::spin(node);
    workflowThread.join();
    return 0;
}
