#include <chrono>
#include <string>

#include "gtest/gtest.h"
#include "hardware_test_fixture.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class IndividualTopicsFrameTest : public DeviceRequiredTest {
protected:
    enum class CameraType { SCANNER, MOTIONCAM };

    using PC2 = sensor_msgs::msg::PointCloud2;
    using Img = sensor_msgs::msg::Image;

    struct SyncedFrame {
        PC2::ConstSharedPtr points;
        Img::ConstSharedPtr depth;
        Img::ConstSharedPtr intensity;   // /intensity: grayscale laser texture
        Img::ConstSharedPtr texture;     // /texture: color texture
        Img::ConstSharedPtr colorCamera; // /color_camera_image
        explicit operator bool() const { return points != nullptr && depth != nullptr; }
    };

    struct TriggerResult {
        bool serviceSucceeded = false;
        std::string serviceMessage;
        SyncedFrame frame;
    };

    using Sync2 = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PC2, Img>>;
    using Sync3 = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PC2, Img, Img>>;
    using Sync4 = message_filters::Synchronizer<
        message_filters::sync_policies::ExactTime<PC2, Img, Img, Img>>;

    static inline CameraType sCameraType = CameraType::SCANNER;
    static inline bool sIsColorDevice = false;

    static void SetUpTestSuite() {
        rclcpp::NodeOptions options;
        options.append_parameter_override("device_id", deviceId());
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

    void resetSyncs() {
        mSync2.reset();
        mSync3.reset();
        mSync4.reset();
        mLatestFrame = {};
    }

    void setupSync2() {
        resetSyncs();
        mSync2 = std::make_unique<Sync2>(
            message_filters::sync_policies::ExactTime<PC2, Img>(5), mPointsSub, mDepthSub);
        mSync2->registerCallback(
            [this](PC2::ConstSharedPtr pts, Img::ConstSharedPtr depth) {
                mLatestFrame = {std::move(pts), std::move(depth), nullptr, nullptr, nullptr};
            });
    }

    void setupSync3Laser() {
        resetSyncs();
        mSync3 = std::make_unique<Sync3>(
            message_filters::sync_policies::ExactTime<PC2, Img, Img>(5),
            mPointsSub, mDepthSub, mIntensitySub);
        mSync3->registerCallback(
            [this](PC2::ConstSharedPtr pts, Img::ConstSharedPtr depth,
                   Img::ConstSharedPtr intensity) {
                mLatestFrame = {std::move(pts), std::move(depth), std::move(intensity),
                                nullptr, nullptr};
            });
    }

    void setupSync4Laser() {
        resetSyncs();
        mSync4 = std::make_unique<Sync4>(
            message_filters::sync_policies::ExactTime<PC2, Img, Img, Img>(5),
            mPointsSub, mDepthSub, mIntensitySub, mColorCameraSub);
        mSync4->registerCallback(
            [this](PC2::ConstSharedPtr pts, Img::ConstSharedPtr depth,
                   Img::ConstSharedPtr intensity, Img::ConstSharedPtr colorCam) {
                mLatestFrame = {std::move(pts), std::move(depth), std::move(intensity),
                                nullptr, std::move(colorCam)};
            });
    }

    void setupSync4Color() {
        resetSyncs();
        mSync4 = std::make_unique<Sync4>(
            message_filters::sync_policies::ExactTime<PC2, Img, Img, Img>(5),
            mPointsSub, mDepthSub, mTextureSub, mColorCameraSub);
        mSync4->registerCallback(
            [this](PC2::ConstSharedPtr pts, Img::ConstSharedPtr depth,
                   Img::ConstSharedPtr texture, Img::ConstSharedPtr colorCam) {
                mLatestFrame = {std::move(pts), std::move(depth), nullptr,
                                std::move(texture), std::move(colorCam)};
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
        mPointsSub.subscribe(mClientNode, "/points", qos);
        mDepthSub.subscribe(mClientNode, "/depth", qos);
        mIntensitySub.subscribe(mClientNode, "/intensity", qos);
        mTextureSub.subscribe(mClientNode, "/texture", qos);
        mColorCameraSub.subscribe(mClientNode, "/color_camera_image", qos);
    }

    void TearDown() override {
        resetSyncs();
        mPointsSub.unsubscribe();
        mDepthSub.unsubscribe();
        mIntensitySub.unsubscribe();
        mTextureSub.unsubscribe();
        mColorCameraSub.unsubscribe();
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

    rclcpp::Client<phoxi_camera_msgs::srv::TriggerFrame>::SharedPtr mTriggerClient;
    SyncedFrame mLatestFrame;
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
    EXPECT_EQ(frame.points->data.size(),
              static_cast<size_t>(
                  frame.points->width * frame.points->height * frame.points->point_step));
    EXPECT_TRUE(hasField(*frame.points, "x"));
    EXPECT_TRUE(hasField(*frame.points, "y"));
    EXPECT_TRUE(hasField(*frame.points, "z"));
    EXPECT_GT(frame.depth->width, 0u);
    EXPECT_GT(frame.depth->height, 0u);
    EXPECT_GT(frame.depth->step, 0u);
    EXPECT_EQ(frame.depth->data.size(),
              static_cast<size_t>(frame.depth->height * frame.depth->step));
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
