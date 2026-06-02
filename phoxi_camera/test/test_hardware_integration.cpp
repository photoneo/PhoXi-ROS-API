#include <chrono>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera/PhoXiCamera.h"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace phoxi_camera;

class HardwareIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        sensorSn = "InstalledExamples-basic-example";

        rclcpp::NodeOptions options;
        options.append_parameter_override("device_id", sensorSn);
        options.append_parameter_override("publish_combined", true);

        rosInterface = std::make_shared<phoxi_camera::PhoXiCamera>(options);

        testClientNode = std::make_shared<rclcpp::Node>("test_client_node");

        executor_.add_node(rosInterface->get_node_base_interface());
        executor_.add_node(testClientNode);
    }

    void TearDown() override {}

    bool changeLcState(uint8_t transition, const std::chrono::seconds& timeout = std::chrono::seconds(10)) {
        auto client = testClientNode->create_client<lifecycle_msgs::srv::ChangeState>("/phoxi_camera/change_state");
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(testClientNode->get_logger(), "change_state service not available!");
            return false;
        }
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        auto future = client->async_send_request(request);
        return executor_.spin_until_future_complete(future, timeout) == rclcpp::FutureReturnCode::SUCCESS && future.get()->success;
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<phoxi_camera::PhoXiCamera> rosInterface;
    std::shared_ptr<rclcpp::Node> testClientNode;
    std::string sensorSn;
};

TEST_F(HardwareIntegrationTest, FullLifecycleAndData) {
    RCLCPP_INFO(testClientNode->get_logger(), "Configuring node...");
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
    RCLCPP_INFO(testClientNode->get_logger(), "Activating node...");
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    RCLCPP_INFO(testClientNode->get_logger(), "Subscribing to /point_cloud and triggering frame...");
    bool messageReceived = false;
    sensor_msgs::msg::PointCloud2::SharedPtr receivedMsg;
    auto sub = testClientNode->create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud", 1, [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(testClientNode->get_logger(), "SUCCESS: Received a point cloud message!");
        messageReceived = true;
        receivedMsg = msg;
    });

    auto triggerClient = testClientNode->create_client<phoxi_camera_msgs::srv::TriggerFrame>("/phoxi_camera/trigger_frame");
    ASSERT_TRUE(triggerClient->wait_for_service(std::chrono::seconds(5)));

    auto triggerRequest = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
    triggerRequest->wait_grabbing_end = true;
    auto triggerFuture = triggerClient->async_send_request(triggerRequest);

    ASSERT_EQ(executor_.spin_until_future_complete(triggerFuture, std::chrono::seconds(15)), rclcpp::FutureReturnCode::SUCCESS) << "Trigger service call timed out!";
    ASSERT_TRUE(triggerFuture.get()->success);

    auto startTime = std::chrono::steady_clock::now();
    while (!messageReceived && (std::chrono::steady_clock::now() - startTime) < std::chrono::seconds(2)) {
        executor_.spin_some(std::chrono::milliseconds(10));
    }

    ASSERT_TRUE(messageReceived) << "Node reported successful trigger, but no point cloud message was received!";
    EXPECT_GT(receivedMsg->data.size(), 0) << "Received point cloud is empty!";
    EXPECT_NE(receivedMsg->header.frame_id, "") << "Frame ID is empty!";
    EXPECT_TRUE(receivedMsg->header.stamp.sec > 0 || receivedMsg->header.stamp.nanosec > 0) << "Timestamp is not set!";

    RCLCPP_INFO(testClientNode->get_logger(), "Deactivating and cleaning up...");
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
