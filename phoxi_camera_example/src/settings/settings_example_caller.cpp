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
    auto client = node->create_client<lifecycle_msgs::srv::ChangeState>("/phoxi_camera/change_state");
    if (!client->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Lifecycle service not available.");
        return false;
    }
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto result = client->async_send_request(request).get();
    if (!result || !result->success) {
        RCLCPP_ERROR(node->get_logger(), "Transition %u failed.", static_cast<unsigned int>(transition));
        return false;
    }
    return true;
}

// Set parameters on the phoxi_camera node via the parameter service.
// Call before configure to have them applied in on_configure, or after
// configure (while connected) to apply them immediately to the device.
bool setParameters(std::shared_ptr<rclcpp::Node> node,
    const std::vector<rclcpp::Parameter>& params) {
    auto client = node->create_client<rcl_interfaces::srv::SetParameters>(
        "/phoxi_camera/set_parameters");
    if (!client->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Parameter service not available.");
        return false;
    }
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const auto& param : params) {
        request->parameters.push_back(param.to_parameter_msg());
    }
    auto result = client->async_send_request(request).get();
    if (!result) {
        RCLCPP_ERROR(node->get_logger(), "Set parameters call failed.");
        return false;
    }
    for (const auto& r : result->results) {
        if (!r.successful) {
            RCLCPP_ERROR(node->get_logger(), "Failed to set parameter: %s", r.reason.c_str());
            return false;
        }
    }
    return true;
}

// List all declared device_settings.* and frame_settings.* parameters with values.
// device_settings.* are only declared after configure.
void listSettings(std::shared_ptr<rclcpp::Node> node) {
    auto listClient = node->create_client<rcl_interfaces::srv::ListParameters>(
        "/phoxi_camera/list_parameters");
    auto getClient = node->create_client<rcl_interfaces::srv::GetParameters>(
        "/phoxi_camera/get_parameters");
    if (!listClient->wait_for_service(3s) || !getClient->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Parameter service not available.");
        return;
    }
    for (const auto& prefix : {"device_settings", "frame_settings"}) {
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
            const rclcpp::Parameter p(listResp->result.names[i],
                rclcpp::ParameterValue(getResp->values[i]));
            RCLCPP_INFO(node->get_logger(), "  %s = %s",
                p.get_name().c_str(), p.value_to_string().c_str());
        }
    }
}

void runWorkflow(std::shared_ptr<rclcpp::Node> node) {
    // 1. frame_settings.* are declared at node construction so they can be set
    //    from code before configure; on_configure() reads them and forwards to device.
    RCLCPP_INFO(node->get_logger(), "Setting frame output settings.");
    if (!setParameters(node, {
            rclcpp::Parameter("frame_settings.PointCloud",       true),
            rclcpp::Parameter("frame_settings.NormalMap",        false),
            rclcpp::Parameter("frame_settings.DepthMap",         true),
            rclcpp::Parameter("frame_settings.Texture",          true),
            rclcpp::Parameter("frame_settings.ConfidenceMap",    false),
            rclcpp::Parameter("frame_settings.ColorCameraImage", false),
            rclcpp::Parameter("frame_settings.EventMap",         false),
        })) {
        rclcpp::shutdown();
        return;
    }

    // 2. Configure — connects to device, loads schema, then applies both the inline
    //    parameters from the launch description and the frame_settings set above.
    RCLCPP_INFO(node->get_logger(), "Configuring.");
    if (!changeLifecycleState(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
        rclcpp::shutdown();
        return;
    }

    // 3. List all available settings — device_settings.* are device-specific and
    //    only declared after configure.
    listSettings(node);

    // 4. Activate — starts acquisition.
    RCLCPP_INFO(node->get_logger(), "Activating.");
    if (!changeLifecycleState(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
        rclcpp::shutdown();
        return;
    }

    // 5. Live change — set a device setting while the camera is active.
    //    The camera node's parameter callback validates the change and forwards
    //    it to the device in the same call.
    std::this_thread::sleep_for(2s);
    RCLCPP_INFO(node->get_logger(), "Setting LaserPower to 3.");
    setParameters(node, {
        rclcpp::Parameter("device_settings.CapturingSettings.LaserPower", 3),
    });

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
