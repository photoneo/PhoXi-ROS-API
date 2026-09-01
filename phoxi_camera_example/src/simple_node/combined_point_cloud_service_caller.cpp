#include <chrono>
#include <memory>
#include <thread>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
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

void runWorkflow(std::shared_ptr<rclcpp::Node> node) {
    // Settings are supplied via the launch file.

    // Configure (connects to device) and activate (starts acquisition).
    RCLCPP_INFO(node->get_logger(), "Configuring.");
    if (!changeLifecycleState(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Activating.");
    if (!changeLifecycleState(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
        rclcpp::shutdown();
        return;
    }

    // Trigger a frame — listener receives a single combined PointCloud2.
    auto triggerClient = node->create_client<phoxi_camera_msgs::srv::TriggerFrame>("/phoxi_camera/trigger_frame");
    if (!triggerClient->wait_for_service(5s)) {
        RCLCPP_ERROR(node->get_logger(), "Trigger frame service not available.");
        rclcpp::shutdown();
        return;
    }
    auto triggerReq = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
    triggerReq->wait_grabbing_end = true;

    RCLCPP_INFO(node->get_logger(), "Triggering frame.");
    if (auto r = triggerClient->async_send_request(triggerReq).get(); !r || !r->success) {
        RCLCPP_ERROR(node->get_logger(), "Trigger frame failed.");
    }
    std::this_thread::sleep_for(3s);

    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("combined_point_cloud_caller_node");
    std::thread workflowThread(runWorkflow, node);
    rclcpp::spin(node);
    workflowThread.join();
    return 0;
}
