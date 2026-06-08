#include <chrono>
#include <string>

#include "gtest/gtest.h"
#include "hardware_test_fixture.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class HardwareIntegrationTest : public DeviceRequiredTest {
protected:
    rclcpp::NodeOptions makeNodeOptions() override {
        auto options = DeviceRequiredTest::makeNodeOptions();
        options.append_parameter_override("publish_combined", true);
        return options;
    }
};

TEST_F(HardwareIntegrationTest, FullLifecycleAndData) {
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 30s));

    bool messageReceived = false;
    sensor_msgs::msg::PointCloud2::SharedPtr receivedMsg;
    auto sub = mClientNode->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud", 1,
        [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            messageReceived = true;
            receivedMsg = msg;
        });

    auto req = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
    req->wait_grabbing_end = true;
    auto resp =
        callService<phoxi_camera_msgs::srv::TriggerFrame>("/phoxi_camera/trigger_frame", req, 30s);
    ASSERT_NE(resp, nullptr);
    ASSERT_TRUE(resp->success);

    auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!messageReceived && std::chrono::steady_clock::now() < deadline) {
        mExecutor.spin_some(10ms);
    }

    ASSERT_TRUE(messageReceived);
    EXPECT_GT(receivedMsg->data.size(), 0u);
    EXPECT_NE(receivedMsg->header.frame_id, "");
    EXPECT_TRUE(receivedMsg->header.stamp.sec > 0 || receivedMsg->header.stamp.nanosec > 0u);

    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 10s));
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 10s));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    if (DeviceRequiredTest::deviceId().empty()) {
        std::cerr << "[ERROR] PHO_TEST_DEVICE_ID environment variable is not set.\n";
        rclcpp::shutdown();
        return 1;
    }
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
