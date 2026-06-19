#include <chrono>
#include <string>

#include "gtest/gtest.h"
#include "hardware_test_fixture.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "phoxi_camera_msgs/msg/frame_info.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class IndividualTopicsFrameTest : public DeviceRequiredTest {
protected:
    enum class CameraType { SCANNER, MOTIONCAM };

    using PC2 = sensor_msgs::msg::PointCloud2;
    using Img = sensor_msgs::msg::Image;
    using CI = sensor_msgs::msg::CameraInfo;
    using FI = phoxi_camera_msgs::msg::FrameInfo;

    struct SyncedFrame {
        PC2::ConstSharedPtr points;
        Img::ConstSharedPtr depth;
        Img::ConstSharedPtr intensity;    // /intensity: grayscale laser texture
        Img::ConstSharedPtr texture;      // /texture: color texture
        Img::ConstSharedPtr colorCamera;  // /color_camera_image
        explicit operator bool() const { return points != nullptr && depth != nullptr; }
    };

    struct TriggerResult {
        bool serviceSucceeded = false;
        std::string serviceMessage;
        SyncedFrame frame;
    };

    using Sync2 = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PC2, Img>>;
    using Sync3 = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PC2, Img, Img>>;
    using Sync4 = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PC2, Img, Img, Img>>;

    static inline CameraType sCameraType = CameraType::SCANNER;
    static inline bool sIsColorDevice = false;

    static void SetUpTestSuite() {
        rclcpp::NodeOptions options;
        options.append_parameter_override("device_id", deviceId());
        options.append_parameter_override("frame_settings.PointCloud", true);
        options.append_parameter_override("frame_settings.DepthMap", true);
        options.append_parameter_override("frame_settings.Texture", true);
        options.append_parameter_override("frame_settings.ColorCameraImage", true);
        suiteSetUp(options);

        const std::string opModeParam = "device_settings.CapturingSettings.OperationMode";
        if (sLcNode->has_parameter(opModeParam)) {
            sCameraType = CameraType::MOTIONCAM;
            auto result = sLcNode->set_parameter(rclcpp::Parameter(opModeParam, "Camera"));
            ASSERT_TRUE(result.successful) << "Failed to set MotionCam Camera mode: " << result.reason;
        }
        // frame_settings.ColorCameraImage is only declared for cameras with a color camera.
        sIsColorDevice = sLcNode->has_parameter("frame_settings.ColorCameraImage");
    }

    static void TearDownTestSuite() { suiteTearDown(); }

    std::string textureSourceParam() const {
        return (sCameraType == CameraType::MOTIONCAM) ? "device_settings.CameraMode.TextureSource" : "device_settings.CapturingSettings.TextureSource";
    }

    bool setTextureSource(const std::string& value) {
        const auto paramName = textureSourceParam();
        if (!mLcNode->has_parameter(paramName)) {
            return false;
        }
        return mLcNode->set_parameter(rclcpp::Parameter(paramName, value)).successful;
    }

    void resetSyncs() {
        mSync2.reset();
        mSync3.reset();
        mSync4.reset();
        mLatestFrame = {};
    }

    void setupSync2() {
        resetSyncs();
        mSync2 = std::make_unique<Sync2>(message_filters::sync_policies::ExactTime<PC2, Img>(5), mPointsSub, mDepthSub);
        mSync2->registerCallback(std::bind(&IndividualTopicsFrameTest::onSync2, this, std::placeholders::_1, std::placeholders::_2));
    }

    void setupSync3Laser() {
        resetSyncs();
        mSync3 = std::make_unique<Sync3>(message_filters::sync_policies::ExactTime<PC2, Img, Img>(5), mPointsSub, mDepthSub, mIntensitySub);
        mSync3->registerCallback(std::bind(&IndividualTopicsFrameTest::onSync3Laser, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    void setupSync4Laser() {
        resetSyncs();
        mSync4 = std::make_unique<Sync4>(message_filters::sync_policies::ExactTime<PC2, Img, Img, Img>(5), mPointsSub, mDepthSub, mIntensitySub, mColorCameraSub);
        mSync4->registerCallback(
                std::bind(&IndividualTopicsFrameTest::onSync4Laser, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    }

    void setupSync4Color() {
        resetSyncs();
        mSync4 = std::make_unique<Sync4>(message_filters::sync_policies::ExactTime<PC2, Img, Img, Img>(5), mPointsSub, mDepthSub, mTextureSub, mColorCameraSub);
        mSync4->registerCallback(
                std::bind(&IndividualTopicsFrameTest::onSync4Color, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    }

    void SetUp() override {
        DeviceRequiredTest::SetUp();
        ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 30s)) << "Failed to activate";
        mTriggerClient = mClientNode->create_client<phoxi_camera_msgs::srv::TriggerFrame>("/phoxi_camera/trigger_frame");
        ASSERT_TRUE(mTriggerClient->wait_for_service(5s));

        const auto qos = rclcpp::SystemDefaultsQoS();
        mPointsSub.subscribe(mClientNode, "/points", qos.get_rmw_qos_profile());
        mDepthSub.subscribe(mClientNode, "/depth", qos.get_rmw_qos_profile());
        mIntensitySub.subscribe(mClientNode, "/intensity", qos.get_rmw_qos_profile());
        mTextureSub.subscribe(mClientNode, "/texture", qos.get_rmw_qos_profile());
        mColorCameraSub.subscribe(mClientNode, "/color_camera_image", qos.get_rmw_qos_profile());
        mFrameInfoSub = mClientNode->create_subscription<FI>("/frame_info", qos, [this](FI::ConstSharedPtr msg) { mLatestFrameInfo = msg; });
        mPrimaryCameraInfoSub = mClientNode->create_subscription<CI>("/frame_info/current_camera", qos, [this](CI::ConstSharedPtr msg) { mLatestPrimaryCameraInfo = msg; });
        mColorCameraInfoSub = mClientNode->create_subscription<CI>("/frame_info/current_color_camera", qos, [this](CI::ConstSharedPtr msg) { mLatestColorCameraInfo = msg; });
    }

    void TearDown() override {
        resetSyncs();
        mPointsSub.unsubscribe();
        mDepthSub.unsubscribe();
        mIntensitySub.unsubscribe();
        mTextureSub.unsubscribe();
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

    TriggerResult triggerOnce() {
        mLatestFrame = {};
        mExecutor.spin_some(50ms);
        mLatestFrame = {};

        auto req = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
        req->wait_grabbing_end = true;
        auto future = mTriggerClient->async_send_request(req);
        if (mExecutor.spin_until_future_complete(future, 30s) != rclcpp::FutureReturnCode::SUCCESS) {
            return {false, "service call timed out", {}};
        }
        const auto response = future.get();
        if (!response->success) {
            return {false, response->message, {}};
        }
        auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!mLatestFrame && std::chrono::steady_clock::now() < deadline) {
            mExecutor.spin_some(10ms);
        }
        return {true, response->message, mLatestFrame};
    }

    SyncedFrame triggerAndReceive() {
        auto result = triggerOnce();
        if (!result.serviceSucceeded) {
            ADD_FAILURE() << "Trigger service failed: " << result.serviceMessage;
            return {};
        }
        if (!result.frame) {
            ADD_FAILURE() << "Trigger succeeded but no synchronized frame was received";
        }
        return result.frame;
    }

    std::unique_ptr<Sync2> mSync2;
    std::unique_ptr<Sync3> mSync3;
    std::unique_ptr<Sync4> mSync4;

    message_filters::Subscriber<PC2> mPointsSub;
    message_filters::Subscriber<Img> mDepthSub;
    message_filters::Subscriber<Img> mIntensitySub;
    message_filters::Subscriber<Img> mTextureSub;
    message_filters::Subscriber<Img> mColorCameraSub;
    rclcpp::Subscription<FI>::SharedPtr mFrameInfoSub;
    rclcpp::Subscription<CI>::SharedPtr mPrimaryCameraInfoSub;
    rclcpp::Subscription<CI>::SharedPtr mColorCameraInfoSub;

    rclcpp::Client<phoxi_camera_msgs::srv::TriggerFrame>::SharedPtr mTriggerClient;
    SyncedFrame mLatestFrame;
    FI::ConstSharedPtr mLatestFrameInfo;
    CI::ConstSharedPtr mLatestPrimaryCameraInfo;
    CI::ConstSharedPtr mLatestColorCameraInfo;

private:
    void onSync2(const PC2::ConstSharedPtr& pts, const Img::ConstSharedPtr& depth) { mLatestFrame = {pts, depth, nullptr, nullptr, nullptr}; }

    void onSync3Laser(const PC2::ConstSharedPtr& pts, const Img::ConstSharedPtr& depth, const Img::ConstSharedPtr& intensity) {
        mLatestFrame = {pts, depth, intensity, nullptr, nullptr};
    }

    void onSync4Laser(const PC2::ConstSharedPtr& pts, const Img::ConstSharedPtr& depth, const Img::ConstSharedPtr& intensity, const Img::ConstSharedPtr& colorCam) {
        mLatestFrame = {pts, depth, intensity, nullptr, colorCam};
    }

    void onSync4Color(const PC2::ConstSharedPtr& pts, const Img::ConstSharedPtr& depth, const Img::ConstSharedPtr& texture, const Img::ConstSharedPtr& colorCam) {
        mLatestFrame = {pts, depth, nullptr, texture, colorCam};
    }
};

// /points and /depth are always produced regardless of camera type or texture config.
TEST_F(IndividualTopicsFrameTest, PointCloudAndDepthMap_AlwaysReceived) {
    setupSync2();
    auto frame = triggerAndReceive();
    ASSERT_NE(frame.points, nullptr);
    ASSERT_NE(frame.depth, nullptr);
    EXPECT_GT(frame.points->width, 0u);
    EXPECT_GT(frame.points->height, 0u);
    EXPECT_GT(frame.points->point_step, 0u);
    EXPECT_EQ(frame.points->data.size(), static_cast<size_t>(frame.points->width * frame.points->height * frame.points->point_step));
    EXPECT_TRUE(hasField(*frame.points, "x"));
    EXPECT_TRUE(hasField(*frame.points, "y"));
    EXPECT_TRUE(hasField(*frame.points, "z"));
    EXPECT_GT(frame.depth->width, 0u);
    EXPECT_GT(frame.depth->height, 0u);
    EXPECT_GT(frame.depth->step, 0u);
    EXPECT_EQ(frame.depth->data.size(), static_cast<size_t>(frame.depth->height * frame.depth->step));
    EXPECT_EQ(frame.points->width, frame.depth->width);
    EXPECT_EQ(frame.points->height, frame.depth->height);
}

// Laser texture → /intensity; COLOR devices also require /color_camera_image in the same sync.
TEST_F(IndividualTopicsFrameTest, LaserTexture_IntensityReceived) {
    if (!setTextureSource("Laser")) {
        GTEST_SKIP() << textureSourceParam() << " not available on this device";
    }
    sIsColorDevice ? setupSync4Laser() : setupSync3Laser();
    auto frame = triggerAndReceive();
    ASSERT_NE(frame.intensity, nullptr);
    EXPECT_GT(frame.intensity->width, 0u);
    EXPECT_GT(frame.intensity->height, 0u);
    EXPECT_GT(frame.intensity->data.size(), 0u);
    EXPECT_EQ(frame.points->width, frame.intensity->width);
    EXPECT_EQ(frame.points->height, frame.intensity->height);
    if (sIsColorDevice) {
        ASSERT_NE(frame.colorCamera, nullptr);
        EXPECT_GT(frame.colorCamera->width, 0u);
        EXPECT_GT(frame.colorCamera->height, 0u);
        EXPECT_GT(frame.colorCamera->data.size(), 0u);
    }
}

// Color texture → /texture + /color_camera_image; skipped for non-COLOR devices.
TEST_F(IndividualTopicsFrameTest, ColorTexture_TextureAndColorCameraReceived) {
    if (!sIsColorDevice) {
        GTEST_SKIP() << "Not a COLOR device";
    }
    if (!setTextureSource("Color")) {
        GTEST_SKIP() << "Color texture source not accepted by this device";
    }
    setupSync4Color();
    auto frame = triggerAndReceive();
    ASSERT_NE(frame.texture, nullptr);
    EXPECT_GT(frame.texture->width, 0u);
    EXPECT_GT(frame.texture->height, 0u);
    EXPECT_GT(frame.texture->data.size(), 0u);
    ASSERT_NE(frame.colorCamera, nullptr);
    EXPECT_GT(frame.colorCamera->width, 0u);
    EXPECT_GT(frame.colorCamera->height, 0u);
    EXPECT_GT(frame.colorCamera->data.size(), 0u);
}

// PHOXI_FRAME_TYPE_FRAMEINFO must produce a populated FrameInfo message every frame.
TEST_F(IndividualTopicsFrameTest, FrameInfo_FrameInfoMsgReceived) {
    setupSync2();
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
TEST_F(IndividualTopicsFrameTest, FrameInfo_CameraInfoReceived) {
    setupSync2();
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
    EXPECT_TRUE(mLatestPrimaryCameraInfo->header.stamp.sec > 0 || mLatestPrimaryCameraInfo->header.stamp.nanosec > 0u);

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

// Trigger while inactive must produce an explicit service failure, never a silent empty frame.
TEST_F(IndividualTopicsFrameTest, TriggerWhenNotActive_ServiceReturnsFailure) {
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 10s));
    auto result = triggerOnce();
    EXPECT_FALSE(result.serviceSucceeded);
    EXPECT_FALSE(result.serviceMessage.empty());
    EXPECT_FALSE(result.frame);
}

// Repeated Laser triggers verify there are no per-frame resource leaks.
TEST_F(IndividualTopicsFrameTest, LaserTexture_10Frames_AllReceived) {
    if (!setTextureSource("Laser")) {
        GTEST_SKIP() << textureSourceParam() << " not available on this device";
    }
    sIsColorDevice ? setupSync4Laser() : setupSync3Laser();
    for (int i = 0; i < 10; ++i) {
        auto frame = triggerAndReceive();
        ASSERT_TRUE(frame) << "Frame " << i << " not received";
        ASSERT_NE(frame.intensity, nullptr) << "Frame " << i << " missing intensity";
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
