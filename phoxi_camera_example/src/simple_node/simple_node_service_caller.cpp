#include <chrono>
#include <memory>
#include <thread>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera_msgs/srv/connect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

/**
 * @brief Helper function to call a lifecycle transition service.
 * @param node The node to use for creating the client.
 * @param transition The ID of the transition to request (e.g., configure, activate).
 * @return True on success, false on failure.
 */
bool change_lifecycle_state(std::shared_ptr<rclcpp::Node> node, uint8_t transition) {
    auto client = node->create_client<lifecycle_msgs::srv::ChangeState>("/phoxi_camera/change_state");
    if (!client->wait_for_service(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Lifecycle service '/phoxi_camera/change_state' not available.");
        return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    auto future = client->async_send_request(request);
    auto result = future.get();
    if (!result || !result->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
        return false;
    }
    return true;
}

void run_workflow(std::shared_ptr<rclcpp::Node> node) {
    // --- 1. Configure the phoxi_camera node ---
    RCLCPP_INFO(node->get_logger(), "Configuring the phoxi_camera node...");
    if (!change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Configuration successful.");
    std::this_thread::sleep_for(1s);

    // --- 2. Activate the phoxi_camera node ---
    RCLCPP_INFO(node->get_logger(), "Activating the phoxi_camera node...");
    if (!change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Activation successful.");

    // --- 3. Create clients and wait for services ---
    auto connect_client = node->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    auto trigger_client = node->create_client<std_srvs::srv::Trigger>("/phoxi_camera/trigger_frame");
    auto disconnect_client = node->create_client<std_srvs::srv::Trigger>("/phoxi_camera/disconnect");

    if (!connect_client->wait_for_service(5s)) {
        RCLCPP_ERROR(node->get_logger(), "Connect service did not appear after activation.");
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(node->get_logger(), "phoxi_camera services are ready.");

    // --- 4. Call Connect ---
    RCLCPP_INFO(node->get_logger(), "Calling connect service...");
    auto connect_request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    auto connect_result = connect_client->async_send_request(connect_request).get();
    if (!connect_result || !connect_result->success) {
        RCLCPP_ERROR(node->get_logger(), "Connect service call failed.");
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Connect successful.");
    std::this_thread::sleep_for(1s);

    // --- 5. Call Trigger Frame ---
    RCLCPP_INFO(node->get_logger(), "Calling trigger_frame service...");
    auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto trigger_result = trigger_client->async_send_request(trigger_request).get();
    if (!trigger_result || !trigger_result->success) {
        RCLCPP_ERROR(node->get_logger(), "Trigger frame service call failed.");
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Trigger frame successful. Check the listener node's output.");
    std::this_thread::sleep_for(5s);

    // --- 6. Call Disconnect ---
    RCLCPP_INFO(node->get_logger(), "Calling disconnect service...");
    disconnect_client->async_send_request(trigger_request);
    std::this_thread::sleep_for(1s);

    RCLCPP_INFO(node->get_logger(), "Service caller workflow finished.");
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("service_caller_node");
    std::thread workflow_thread(run_workflow, node);
    rclcpp::spin(node);
    workflow_thread.join();
    return 0;
}
