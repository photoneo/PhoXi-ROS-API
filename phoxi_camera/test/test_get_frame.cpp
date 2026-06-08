#include <chrono>
#include <string>

#include "gtest/gtest.h"
#include "hardware_test_fixture.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class FrameTest : public DeviceRequiredTest {
protected:
    rclcpp::NodeOptions makeNodeOptions() override {
        auto options = DeviceRequiredTest::makeNodeOptions();
        options.append_parameter_override("publish_combined", true);
        return options;
    }

    void SetUp() override {
        DeviceRequiredTest::SetUp();
        ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 30s))
            << "Failed to activate";
        mTriggerClient = mClientNode->create_client<phoxi_camera_msgs::srv::TriggerFrame>(
            "/phoxi_camera/trigger_frame");
        ASSERT_TRUE(mTriggerClient->wait_for_service(5s));
        mPointCloudSub = mClientNode->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", rclcpp::SystemDefaultsQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { mLatestPointCloud = msg; });
    }

    void TearDown() override {
        mPointCloudSub.reset();
        mTriggerClient.reset();
        changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 10s);
        DeviceRequiredTest::TearDown();
    }

    sensor_msgs::msg::PointCloud2::SharedPtr triggerAndReceivePointCloud() {
        mLatestPointCloud = nullptr;
        mExecutor.spin_some(50ms);
        mLatestPointCloud = nullptr;

        auto req = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
        req->wait_grabbing_end = true;
        auto future = mTriggerClient->async_send_request(req);
        if (mExecutor.spin_until_future_complete(future, 30s) !=
                rclcpp::FutureReturnCode::SUCCESS ||
            !future.get()->success) {
            return nullptr;
        }
        auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!mLatestPointCloud && std::chrono::steady_clock::now() < deadline) {
            mExecutor.spin_some(10ms);
        }
        return mLatestPointCloud;
    }

    rclcpp::Client<phoxi_camera_msgs::srv::TriggerFrame>::SharedPtr mTriggerClient;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudSub;
    sensor_msgs::msg::PointCloud2::SharedPtr mLatestPointCloud;
};

TEST_F(FrameTest, SoftwareTrigger_PointCloudReceived) {
    auto msg = triggerAndReceivePointCloud();
    ASSERT_NE(msg, nullptr);
    EXPECT_GT(msg->width * msg->height, 0u);
    EXPECT_GT(msg->data.size(), 0u);
    EXPECT_NE(msg->header.frame_id, "");
    EXPECT_TRUE(msg->header.stamp.sec > 0 || msg->header.stamp.nanosec > 0u);
}

TEST_F(FrameTest, SoftwareTrigger_PointCloudDimensionsConsistent) {
    auto msg = triggerAndReceivePointCloud();
    ASSERT_NE(msg, nullptr);
    EXPECT_GT(msg->width, 0u);
    EXPECT_GT(msg->height, 0u);
    EXPECT_GT(msg->point_step, 0u);
    EXPECT_EQ(msg->data.size(), static_cast<size_t>(msg->width * msg->height * msg->point_step));
}

TEST_F(FrameTest, SoftwareTrigger_PointCloudHasXYZFields) {
    auto msg = triggerAndReceivePointCloud();
    ASSERT_NE(msg, nullptr);
    auto hasField = [&](const std::string& name) {
        for (const auto& f : msg->fields) {
            if (f.name == name) {
                return true;
            }
        }
        return false;
    };
    EXPECT_TRUE(hasField("x"));
    EXPECT_TRUE(hasField("y"));
    EXPECT_TRUE(hasField("z"));
}

TEST_F(FrameTest, SoftwareTrigger_10Frames_AllReceived) {
    for (int i = 0; i < 10; ++i) {
        auto msg = triggerAndReceivePointCloud();
        ASSERT_NE(msg, nullptr) << "Frame " << i << " not received";
        EXPECT_GT(msg->data.size(), 0u) << "Frame " << i << " has empty data";
    }
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
