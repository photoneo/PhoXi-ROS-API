#include <chrono>
#include <string>

#include "gtest/gtest.h"
#include "hardware_test_fixture.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "phoxi_camera_msgs/msg/frame_info.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class FrameTest : public DeviceRequiredTest {
protected:
    enum class CameraType { SCANNER, MOTIONCAM };

    using PC2 = sensor_msgs::msg::PointCloud2;
    using Img = sensor_msgs::msg::Image;
    using CI = sensor_msgs::msg::CameraInfo;
    using FI = phoxi_camera_msgs::msg::FrameInfo;

    struct ReceivedFrame {
        PC2::ConstSharedPtr pointCloud;
        Img::ConstSharedPtr colorCamera; // /color_camera_image; only set for COLOR devices
        explicit operator bool() const { return pointCloud != nullptr; }
    };

    using Sync2 = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PC2, Img>>;

    static inline CameraType sCameraType = CameraType::SCANNER;
    static inline bool sIsColorDevice = false;

    static void SetUpTestSuite() {
        rclcpp::NodeOptions options;
        options.append_parameter_override("device_id", deviceId());
        options.append_parameter_override("publish_combined", true);
        options.append_parameter_override("frameSettings.PointCloud", true);
        options.append_parameter_override("frameSettings.DepthMap", true);
        options.append_parameter_override("frameSettings.Texture", true);
        options.append_parameter_override("frameSettings.ColorCameraImage", true);
        suiteSetUp(options);

        const std::string opModeParam = "deviceSettings.CapturingSettings.OperationMode";
        if (sLcNode->has_parameter(opModeParam)) {
            sCameraType = CameraType::MOTIONCAM;
            auto result = sLcNode->set_parameter(rclcpp::Parameter(opModeParam, "Camera"));
            ASSERT_TRUE(result.successful)
                << "Failed to set MotionCam Camera mode: " << result.reason;
        }
        // frameSettings.ColorCameraImage is only declared for cameras with a color camera.
        sIsColorDevice = sLcNode->has_parameter("frameSettings.ColorCameraImage");
    }

    static void TearDownTestSuite() {
        suiteTearDown();
    }

    std::string textureSourceParam() const {
        return (sCameraType == CameraType::MOTIONCAM)
            ? "deviceSettings.CameraMode.TextureSource"
            : "deviceSettings.CapturingSettings.TextureSource";
    }

    bool setTextureSource(const std::string& value) {
        const auto paramName = textureSourceParam();
        if (!mLcNode->has_parameter(paramName)) {
            return false;
        }
        return mLcNode->set_parameter(rclcpp::Parameter(paramName, value)).successful;
    }

    void usePointCloudOnly() {
        mSync2.reset();
        mLatestFrame = {};
        mDirectConn = mPointCloudSub.registerCallback(
            [this](const PC2::ConstSharedPtr& msg) {
                mLatestFrame = {msg, nullptr};
            });
    }

    void useColorCameraSync() {
        mDirectConn.disconnect();
        mLatestFrame = {};
        mSync2 = std::make_unique<Sync2>(
            message_filters::sync_policies::ExactTime<PC2, Img>(5),
            mPointCloudSub, mColorCameraSub);
        mSync2->registerCallback(
            [this](PC2::ConstSharedPtr pts, Img::ConstSharedPtr colorCam) {
                mLatestFrame = {std::move(pts), std::move(colorCam)};
            });
    }

    void SetUp() override {
        DeviceRequiredTest::SetUp();
        ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 30s))
            << "Failed to activate";
        mTriggerClient = mClientNode->create_client<phoxi_camera_msgs::srv::TriggerFrame>(
            "/phoxi_camera/trigger_frame");
        ASSERT_TRUE(mTriggerClient->wait_for_service(5s));

        const auto qos = rclcpp::SystemDefaultsQoS();
        mPointCloudSub.subscribe(mClientNode, "/point_cloud", qos);
        mColorCameraSub.subscribe(mClientNode, "/color_camera_image", qos);
        mFrameInfoSub = mClientNode->create_subscription<FI>(
            "/frameInfo", qos,
            [this](FI::ConstSharedPtr msg) { mLatestFrameInfo = msg; });
        mPrimaryCameraInfoSub = mClientNode->create_subscription<CI>(
            "/frameInfo/currentCamera", qos,
            [this](CI::ConstSharedPtr msg) { mLatestPrimaryCameraInfo = msg; });
        mColorCameraInfoSub = mClientNode->create_subscription<CI>(
            "/frameInfo/currentColorCamera", qos,
            [this](CI::ConstSharedPtr msg) { mLatestColorCameraInfo = msg; });
    }

    void TearDown() override {
        mDirectConn.disconnect();
        mSync2.reset();
        mPointCloudSub.unsubscribe();
        mColorCameraSub.unsubscribe();
        mFrameInfoSub.reset();
        mLatestFrameInfo.reset();
        mPrimaryCameraInfoSub.reset();
        mColorCameraInfoSub.reset();
        mLatestPrimaryCameraInfo.reset();
        mLatestColorCameraInfo.reset();
        mTriggerClient.reset();
        changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 10s);
        DeviceRequiredTest::TearDown();
    }

    ReceivedFrame triggerAndReceive() {
        mLatestFrame = {};
        mExecutor.spin_some(50ms);
        mLatestFrame = {};

        auto req = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
        req->wait_grabbing_end = true;
        auto future = mTriggerClient->async_send_request(req);
        if (mExecutor.spin_until_future_complete(future, 40s) != rclcpp::FutureReturnCode::SUCCESS) {
            ADD_FAILURE() << "Trigger service timed out";
            return {};
        }
        const auto response = future.get();
        if (!response->success) {
            ADD_FAILURE() << "Trigger service failed: " << response->message;
            return {};
        }
        auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!mLatestFrame && std::chrono::steady_clock::now() < deadline) {
            mExecutor.spin_some(10ms);
        }
        if (!mLatestFrame) {
            ADD_FAILURE() << "No frame received within timeout";
        }
        return mLatestFrame;
    }

    std::unique_ptr<Sync2> mSync2;
    message_filters::Connection mDirectConn;

    message_filters::Subscriber<PC2> mPointCloudSub;
    message_filters::Subscriber<Img> mColorCameraSub;
    rclcpp::Subscription<FI>::SharedPtr mFrameInfoSub;
    rclcpp::Subscription<CI>::SharedPtr mPrimaryCameraInfoSub;
    rclcpp::Subscription<CI>::SharedPtr mColorCameraInfoSub;

    rclcpp::Client<phoxi_camera_msgs::srv::TriggerFrame>::SharedPtr mTriggerClient;
    ReceivedFrame mLatestFrame;
    FI::ConstSharedPtr mLatestFrameInfo;
    CI::ConstSharedPtr mLatestPrimaryCameraInfo;
    CI::ConstSharedPtr mLatestColorCameraInfo;
};

// Combined point cloud must always be produced with correct geometry and depth field.
TEST_F(FrameTest, SoftwareTrigger_PointCloudReceived) {
    usePointCloudOnly();
    auto frame = triggerAndReceive();
    ASSERT_NE(frame.pointCloud, nullptr);
    EXPECT_GT(frame.pointCloud->width, 0u);
    EXPECT_GT(frame.pointCloud->height, 0u);
    EXPECT_GT(frame.pointCloud->point_step, 0u);
    EXPECT_EQ(frame.pointCloud->data.size(),
              static_cast<size_t>(
                  frame.pointCloud->width * frame.pointCloud->height *
                  frame.pointCloud->point_step));
    EXPECT_NE(frame.pointCloud->header.frame_id, "");
    EXPECT_TRUE(frame.pointCloud->header.stamp.sec > 0 ||
                frame.pointCloud->header.stamp.nanosec > 0u);
    EXPECT_TRUE(hasField(*frame.pointCloud, "x"));
    EXPECT_TRUE(hasField(*frame.pointCloud, "y"));
    EXPECT_TRUE(hasField(*frame.pointCloud, "z"));
    EXPECT_TRUE(hasField(*frame.pointCloud, "depth"));
}

// Laser texture source must embed an "intensity" field in the combined PointCloud2.
// COLOR devices additionally provide /color_camera_image with the same timestamp,
// so they use ExactTime sync and verify both outputs.
TEST_F(FrameTest, LaserTexture_PointCloudHasIntensityField) {
    if (!setTextureSource("Laser")) {
        GTEST_SKIP() << textureSourceParam() << " not available on this device";
    }
    if (sIsColorDevice) {
        useColorCameraSync();
    } else {
        usePointCloudOnly();
    }
    auto frame = triggerAndReceive();
    ASSERT_NE(frame.pointCloud, nullptr);
    EXPECT_TRUE(hasField(*frame.pointCloud, "depth")) << "depth field missing";
    EXPECT_TRUE(hasField(*frame.pointCloud, "intensity"))
        << "intensity field missing for Laser texture source";
    if (sIsColorDevice) {
        ASSERT_NE(frame.colorCamera, nullptr) << "/color_camera_image not received";
        EXPECT_GT(frame.colorCamera->width, 0u);
        EXPECT_GT(frame.colorCamera->height, 0u);
        EXPECT_GT(frame.colorCamera->data.size(), 0u);
        EXPECT_EQ(frame.pointCloud->header.stamp, frame.colorCamera->header.stamp);
    }
}

// Color texture source must embed an "rgb" field in the combined PointCloud2 and also produce
// /color_camera_image with a matching timestamp. Only valid for COLOR devices; skipped otherwise.
TEST_F(FrameTest, ColorTexture_PointCloudHasRgbFieldAndColorCameraImageReceived) {
    if (!sIsColorDevice) {
        GTEST_SKIP() << "Not a COLOR device";
    }
    ASSERT_TRUE(setTextureSource("Color"))
        << "Color texture source not accepted on COLOR device";
    useColorCameraSync();
    auto frame = triggerAndReceive();
    ASSERT_NE(frame.pointCloud, nullptr);
    EXPECT_TRUE(hasField(*frame.pointCloud, "depth")) << "depth field missing";
    EXPECT_TRUE(hasField(*frame.pointCloud, "rgb"))
        << "rgb field missing for Color texture source";
    ASSERT_NE(frame.colorCamera, nullptr) << "/color_camera_image not received";
    EXPECT_GT(frame.colorCamera->width, 0u);
    EXPECT_GT(frame.colorCamera->height, 0u);
    EXPECT_GT(frame.colorCamera->data.size(), 0u);
    EXPECT_EQ(frame.pointCloud->header.stamp, frame.colorCamera->header.stamp);
}

// PHOXI_FRAME_TYPE_FRAMEINFO must produce a populated FrameInfo message every frame.
TEST_F(FrameTest, FrameInfo_FrameInfoMsgReceived) {
    usePointCloudOnly();
    triggerAndReceive();

    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!mLatestFrameInfo && std::chrono::steady_clock::now() < deadline) {
        mExecutor.spin_some(10ms);
    }

    ASSERT_NE(mLatestFrameInfo, nullptr) << "No frameInfo message received";
    EXPECT_FALSE(mLatestFrameInfo->hw_id.empty());
    EXPECT_GT(mLatestFrameInfo->total_scan_count, 0);
    EXPECT_GT(mLatestFrameInfo->duration, 0.0);
    EXPECT_FALSE(mLatestFrameInfo->temperature.empty());
    EXPECT_NE(mLatestFrameInfo->header.frame_id, "");
    EXPECT_TRUE(mLatestFrameInfo->header.stamp.sec > 0 || mLatestFrameInfo->header.stamp.nanosec > 0u);
}

// PHOXI_FRAME_TYPE_FRAMEINFO must produce valid CameraInfo on both topics every frame.
TEST_F(FrameTest, FrameInfo_CameraInfoReceived) {
    usePointCloudOnly();
    triggerAndReceive();

    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!mLatestPrimaryCameraInfo && std::chrono::steady_clock::now() < deadline) {
        mExecutor.spin_some(10ms);
    }

    ASSERT_NE(mLatestPrimaryCameraInfo, nullptr) << "No primary camera info received";
    EXPECT_GT(mLatestPrimaryCameraInfo->width, 0u);
    EXPECT_GT(mLatestPrimaryCameraInfo->height, 0u);
    EXPECT_EQ(mLatestPrimaryCameraInfo->distortion_model, "plumb_bob");
    EXPECT_GT(mLatestPrimaryCameraInfo->k[0], 0.0);  // fx
    EXPECT_GT(mLatestPrimaryCameraInfo->k[4], 0.0);  // fy
    EXPECT_NE(mLatestPrimaryCameraInfo->header.frame_id, "");
    EXPECT_TRUE(mLatestPrimaryCameraInfo->header.stamp.sec > 0 ||
                mLatestPrimaryCameraInfo->header.stamp.nanosec > 0u);

    if (sIsColorDevice) {
        ASSERT_NE(mLatestColorCameraInfo, nullptr) << "No color camera info received";
        EXPECT_GT(mLatestColorCameraInfo->width, 0u);
        EXPECT_GT(mLatestColorCameraInfo->height, 0u);
        EXPECT_EQ(mLatestColorCameraInfo->distortion_model, "plumb_bob");
        EXPECT_GT(mLatestColorCameraInfo->k[0], 0.0);
        EXPECT_GT(mLatestColorCameraInfo->k[4], 0.0);
        EXPECT_EQ(mLatestColorCameraInfo->header.stamp, mLatestPrimaryCameraInfo->header.stamp);
    }
}

// Repeated Laser triggers must all succeed — verifies there are no per-frame resource leaks.
TEST_F(FrameTest, LaserTexture_10Frames_AllReceived) {
    if (!setTextureSource("Laser")) {
        GTEST_SKIP() << textureSourceParam() << " not available on this device";
    }
    if (sIsColorDevice) {
        useColorCameraSync();
    } else {
        usePointCloudOnly();
    }
    for (int i = 0; i < 10; ++i) {
        auto frame = triggerAndReceive();
        ASSERT_NE(frame.pointCloud, nullptr) << "Frame " << i << " not received";
        EXPECT_GT(frame.pointCloud->data.size(), 0u) << "Frame " << i << " has empty data";
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
