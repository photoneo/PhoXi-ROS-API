#include <chrono>
#include <string>

#include "gtest/gtest.h"
#include "hardware_test_fixture.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "phoxi_camera_msgs/msg/frame_error.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class HardwareIntegrationTest : public DeviceRequiredTest {
protected:
    static void SetUpTestSuite() {
        rclcpp::NodeOptions options;
        options.append_parameter_override("device_id", deviceId());
        options.append_parameter_override("publish_combined", true);
        suiteSetUp(options);
    }

    static void TearDownTestSuite() { suiteTearDown(); }
};

TEST_F(HardwareIntegrationTest, FullLifecycleAndData) {
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 30s));

    bool pointCloudReceived = false;
    bool frameErrorReceived = false;
    sensor_msgs::msg::PointCloud2::SharedPtr receivedMsg;

    auto pcSub = mClientNode->create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud", 1, [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pointCloudReceived = true;
        receivedMsg = msg;
    });
    auto errSub = mClientNode->create_subscription<phoxi_camera_msgs::msg::FrameError>(
            "/frameError", 1, [&](phoxi_camera_msgs::msg::FrameError::SharedPtr) { frameErrorReceived = true; });

    auto req = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
    req->wait_grabbing_end = true;
    auto resp = callService<phoxi_camera_msgs::srv::TriggerFrame>("/phoxi_camera/trigger_frame", req, 30s);
    ASSERT_NE(resp, nullptr);
    ASSERT_TRUE(resp->success);

    auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!pointCloudReceived && !frameErrorReceived && std::chrono::steady_clock::now() < deadline) {
        mExecutor.spin_some(10ms);
    }

    ASSERT_TRUE(pointCloudReceived || frameErrorReceived) << "Neither point_cloud nor frameError received within timeout";

    if (pointCloudReceived) {
        EXPECT_GT(receivedMsg->data.size(), 0u);
        EXPECT_NE(receivedMsg->header.frame_id, "");
        EXPECT_TRUE(receivedMsg->header.stamp.sec > 0 || receivedMsg->header.stamp.nanosec > 0u);
    }

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
