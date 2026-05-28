#include <cstdint>
#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/PhoXiFrame.h"
#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera/PhoXiCamera.h"
#include "phoxi_camera_msgs/srv/connect.hpp"
#include "phoxi_camera_msgs/srv/create_profile.hpp"
#include "phoxi_camera_msgs/srv/delete_profile.hpp"
#include "phoxi_camera_msgs/srv/export_profile.hpp"
#include "phoxi_camera_msgs/srv/get_active_profile.hpp"
#include "phoxi_camera_msgs/srv/get_profile_list.hpp"
#include "phoxi_camera_msgs/srv/get_startup_profile.hpp"
#include "phoxi_camera_msgs/srv/import_profile.hpp"
#include "phoxi_camera_msgs/srv/set_active_profile.hpp"
#include "phoxi_camera_msgs/srv/set_startup_profile.hpp"
#include "phoxi_camera_msgs/srv/update_profile.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace phoxi_camera;
using ::testing::_;
using ::testing::Return;
using ::testing::Throw;

static constexpr const char* kTestDeviceId = "test-device-id";

class MockPhoXiInterface : public PhoXiInterface
{
  public:
    MOCK_METHOD(void, connectCamera,
                (const std::string& deviceId, GetFrameCallback&& getFrameCallback),
                (override));
    MOCK_METHOD(void, disconnectCamera, (), (override));
    MOCK_METHOD(void, triggerFrame, (bool wait_grabbing_end), (override));
    MOCK_METHOD(void, startAcquisition, (), (override));
    MOCK_METHOD(void, stopAcquisition, (), (override));
    MOCK_METHOD(bool, isConnected, (), (override));
    MOCK_METHOD(bool, isAcquiring, (), (override));
    MOCK_METHOD(std::vector<pho::api::PhoXiProfileDescriptor>, getProfileList, (), (override));
    MOCK_METHOD(std::string, getActiveProfile, (), (override));
    MOCK_METHOD(void, setActiveProfile, (const std::string& name), (override));
    MOCK_METHOD(std::string, getStartupProfile, (), (override));
    MOCK_METHOD(void, setStartupProfile, (const std::string& name), (override));
    MOCK_METHOD(void, createProfile, (const std::string& name), (override));
    MOCK_METHOD(void, deleteProfile, (const std::string& name), (override));
    MOCK_METHOD(void, updateProfile, (const std::string& name), (override));
    MOCK_METHOD(pho::api::PhoXiProfileContent, exportProfile, (), (override));
    MOCK_METHOD(void, importProfile, (const pho::api::PhoXiProfileContent& content), (override));
    MOCK_METHOD(void, resetActiveProfile, (), (override));
};

class TestableRosInterface : public PhoXiCamera
{
  public:
    TestableRosInterface(const std::string& deviceId, const rclcpp::NodeOptions& options,
                         std::unique_ptr<PhoXiInterface> phoxiInterface)
    : PhoXiCamera(deviceId, options) {
        this->mPhoXiInterface = std::move(phoxiInterface);
    }
};

class RosInterfaceTest : public ::testing::Test
{
  protected:
    void SetUp() override {
        auto mockPtr = std::make_unique<MockPhoXiInterface>();
        mockInterface = mockPtr.get();
        testing::Mock::AllowLeak(mockInterface);

        EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(false));

        rclcpp::NodeOptions options;
        lcNode = std::make_shared<TestableRosInterface>(kTestDeviceId, options,
                                                          std::move(mockPtr));

        testClientNode = std::make_shared<rclcpp::Node>("test_client_node");

        executor_.add_node(lcNode->get_node_base_interface());
        executor_.add_node(testClientNode);
    }

    void TearDown() override {
        executor_.remove_node(lcNode->get_node_base_interface());
        executor_.remove_node(testClientNode);

        lcNode.reset();
        testClientNode.reset();
    }

    bool changeLcState(uint8_t transition) {
        auto lcClient = testClientNode->create_client<lifecycle_msgs::srv::ChangeState>(
            "/phoxi_camera/change_state");
        if (!lcClient->wait_for_service(std::chrono::seconds(2))) {
            return false;
        }
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        auto future = lcClient->async_send_request(request);
        return executor_.spin_until_future_complete(future) == rclcpp::FutureReturnCode::SUCCESS &&
               future.get()->success;
    }

    // Configure-only: set up connectCamera expectation and run CONFIGURE transition.
    bool configureOnly() {
        EXPECT_CALL(*mockInterface, connectCamera(kTestDeviceId, _)).Times(1);
        return changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }

    // Cleanup with disconnect: set up disconnectCamera expectation and run CLEANUP transition.
    bool cleanupWithDisconnect() {
        EXPECT_CALL(*mockInterface, disconnectCamera()).Times(1);
        return changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }

    // Configure + activate, capturing the frame callback from connectCamera.
    PhoXiInterface::GetFrameCallback configureActivateCapture() {
        PhoXiInterface::GetFrameCallback cb;
        EXPECT_CALL(*mockInterface, connectCamera(kTestDeviceId, _))
            .WillOnce([&cb](const std::string&, PhoXiInterface::GetFrameCallback&& captured) {
                cb = std::move(captured);
            });
        if (!changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) return {};
        EXPECT_CALL(*mockInterface, startAcquisition());
        if (!changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) return {};
        return cb;
    }

    // Call a ROS2 service and return its response, or nullptr on timeout/failure.
    template<typename SrvT>
    typename SrvT::Response::SharedPtr callService(
        const std::string& serviceName,
        typename SrvT::Request::SharedPtr request = std::make_shared<typename SrvT::Request>()) {
        auto client = testClientNode->create_client<SrvT>(serviceName);
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            return nullptr;
        }
        auto future = client->async_send_request(request);
        if (executor_.spin_until_future_complete(future) != rclcpp::FutureReturnCode::SUCCESS) {
            return nullptr;
        }
        return future.get();
    }

    // Subscribe to a topic, inject a frame, and return the first received message (or nullptr).
    template<typename MsgT>
    typename MsgT::SharedPtr injectAndReceive(
        const PhoXiInterface::GetFrameCallback& frameCb,
        const std::string& topic,
        const PhoXiFrame& frame) {
        std::promise<typename MsgT::SharedPtr> promise;
        auto future = promise.get_future().share();
        bool received = false;
        auto sub = testClientNode->create_subscription<MsgT>(
            topic, rclcpp::SystemDefaultsQoS(),
            [&promise, &received](typename MsgT::SharedPtr msg) {
                if (!received) { received = true; promise.set_value(msg); }
            });
        executor_.spin_some(std::chrono::milliseconds(50));
        frameCb(frame);
        if (executor_.spin_until_future_complete(future, std::chrono::seconds(2)) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            return nullptr;
        }
        return future.get();
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    MockPhoXiInterface* mockInterface;
    std::shared_ptr<TestableRosInterface> lcNode;
    std::shared_ptr<rclcpp::Node> testClientNode;
};

// ── Lifecycle transitions ────────────────────────────────────────────────────

TEST_F(RosInterfaceTest, LifecycleTransitions) {
    EXPECT_CALL(*mockInterface, connectCamera(kTestDeviceId, _)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    EXPECT_CALL(*mockInterface, startAcquisition()).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mockInterface, stopAcquisition()).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));

    EXPECT_CALL(*mockInterface, disconnectCamera()).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
}

TEST_F(RosInterfaceTest, ConfigureFailsWhenConnectionFails) {
    EXPECT_CALL(*mockInterface, connectCamera(kTestDeviceId, _))
        .WillOnce(Throw(PhoXiInterfaceException("Device not found")));

    ASSERT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
}

TEST_F(RosInterfaceTest, ActivateFailsWhenAcquisitionFails) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, startAcquisition())
        .WillOnce(Throw(UnableToStartAcquisition("Hardware error")));
    ASSERT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    ASSERT_TRUE(cleanupWithDisconnect());
}

// ── Connect service ──────────────────────────────────────────────────────────

TEST_F(RosInterfaceTest, ConnectServiceSuccessWhileActive) {
    EXPECT_CALL(*mockInterface, connectCamera(kTestDeviceId, _)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    // startAcquisition called twice: once in on_activate, once in connectCallback (is_active=true)
    EXPECT_CALL(*mockInterface, startAcquisition()).Times(2);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mockInterface, stopAcquisition()).Times(1);
    EXPECT_CALL(*mockInterface, connectCamera("test-sn-123", _)).Times(1);

    auto client =
        testClientNode->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    request->sn = "test-sn-123";

    auto future = client->async_send_request(request);
    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
}

TEST_F(RosInterfaceTest, ConnectServiceSuccessWhileInactive) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, connectCamera("other-device", _)).Times(1);

    auto client =
        testClientNode->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    request->sn = "other-device";

    auto future = client->async_send_request(request);
    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, ConnectServiceFailure) {
    configureActivateCapture();

    EXPECT_CALL(*mockInterface, stopAcquisition()).Times(1);
    EXPECT_CALL(*mockInterface, connectCamera(_, _))
        .WillOnce(Throw(PhoXiInterfaceException("Device not found")));

    auto client =
        testClientNode->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    request->sn = "any-sn";

    auto future = client->async_send_request(request);
    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);

    auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->message, "Device not found");
}

// ── Trigger frame ────────────────────────────────────────────────────────────

TEST_F(RosInterfaceTest, TriggerFrame) {
    configureActivateCapture();

    EXPECT_CALL(*mockInterface, triggerFrame(false)).Times(1);

    auto client = testClientNode->create_client<phoxi_camera_msgs::srv::TriggerFrame>(
        "/phoxi_camera/trigger_frame");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
    request->wait_grabbing_end = false;
    auto future = client->async_send_request(request);

    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
}

TEST_F(RosInterfaceTest, TriggerFrameWithWaitGrabbingEnd) {
    configureActivateCapture();

    EXPECT_CALL(*mockInterface, triggerFrame(true)).Times(1);

    auto client = testClientNode->create_client<phoxi_camera_msgs::srv::TriggerFrame>(
        "/phoxi_camera/trigger_frame");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
    request->wait_grabbing_end = true;
    auto future = client->async_send_request(request);

    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
}

// ── Topic publication ────────────────────────────────────────────────────────

TEST_F(RosInterfaceTest, ColorCameraImagePublished) {
    EXPECT_CALL(*mockInterface, connectCamera(kTestDeviceId, _)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    // startAcquisition called twice: once in on_activate, once in connectCallback (is_active=true)
    EXPECT_CALL(*mockInterface, startAcquisition()).Times(2);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mockInterface, stopAcquisition()).Times(1);
    PhoXiInterface::GetFrameCallback capturedCb;
    EXPECT_CALL(*mockInterface, connectCamera("test-sn", _))
        .WillOnce([&capturedCb](const std::string&, PhoXiInterface::GetFrameCallback&& cb) {
            capturedCb = std::move(cb);
        });

    auto connectClient =
        testClientNode->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(connectClient->wait_for_service(std::chrono::seconds(2)));
    auto connectReq = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    connectReq->sn = "test-sn";
    auto connectFuture = connectClient->async_send_request(connectReq);
    ASSERT_EQ(executor_.spin_until_future_complete(connectFuture), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(connectFuture.get()->success);
    ASSERT_TRUE(capturedCb);

    uint16_t colorData[2][3] = {{1000, 2000, 3000}, {4000, 5000, 6000}};
    phoxi_frame_record_t colorRec = {PHOXI_FRAME_TYPE_COLORCAMERAIMAGE, PHOXI_FRAME_FORMAT_RGB_16,
                                      2, 1, sizeof(colorData), colorData};
    PhoXiFrame frame;
    frame.colorCamera = &colorRec;

    auto received = injectAndReceive<sensor_msgs::msg::Image>(capturedCb, "color_camera_image", frame);
    ASSERT_NE(received, nullptr);
    EXPECT_EQ(received->encoding, "rgb16");
    EXPECT_EQ(received->width, 2u);
    EXPECT_EQ(received->height, 1u);
    EXPECT_EQ(received->step, 2u * 3u * sizeof(uint16_t));

    uint16_t r0, g0, b0, r1, g1, b1;
    std::memcpy(&r0, received->data.data() + 0 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&g0, received->data.data() + 1 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&b0, received->data.data() + 2 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&r1, received->data.data() + 3 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&g1, received->data.data() + 4 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&b1, received->data.data() + 5 * sizeof(uint16_t), sizeof(uint16_t));
    EXPECT_EQ(r0, 1000u); EXPECT_EQ(g0, 2000u); EXPECT_EQ(b0, 3000u);
    EXPECT_EQ(r1, 4000u); EXPECT_EQ(g1, 5000u); EXPECT_EQ(b1, 6000u);
}

TEST_F(RosInterfaceTest, PointCloudPublished) {
    lcNode->set_parameter(rclcpp::Parameter("publish_combined", true));
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float pts[1][3] = {{1000.0f, 2000.0f, 3000.0f}};
    float nrm[1][3] = {{0.1f, 0.2f, 0.9f}};
    uint16_t rgb[1][3] = {{1023, 511, 0}};
    float confData[1] = {0.75f};
    float depthData[1] = {500.0f};
    phoxi_frame_record_t pcRec   = {PHOXI_FRAME_TYPE_POINTCLOUD,    PHOXI_FRAME_FORMAT_POINT3_32F,
                                     1, 1, sizeof(pts),       pts};
    phoxi_frame_record_t nmRec   = {PHOXI_FRAME_TYPE_NORMALMAP,     PHOXI_FRAME_FORMAT_POINT3_32F,
                                     1, 1, sizeof(nrm),       nrm};
    phoxi_frame_record_t rgbRec  = {PHOXI_FRAME_TYPE_TEXTURE,       PHOXI_FRAME_FORMAT_RGB_16,
                                     1, 1, sizeof(rgb),       rgb};
    phoxi_frame_record_t confRec = {PHOXI_FRAME_TYPE_CONFIDENCEMAP, PHOXI_FRAME_FORMAT_FLOAT_32F,
                                     1, 1, sizeof(confData),  confData};
    phoxi_frame_record_t depRec  = {PHOXI_FRAME_TYPE_DEPTHMAP,      PHOXI_FRAME_FORMAT_FLOAT_32F,
                                     1, 1, sizeof(depthData), depthData};
    PhoXiFrame frame;
    frame.pointCloud    = &pcRec;
    frame.normalMap     = &nmRec;
    frame.textureRgb    = &rgbRec;
    frame.confidenceMap = &confRec;
    frame.depthMap      = &depRec;

    auto msg = injectAndReceive<sensor_msgs::msg::PointCloud2>(frameCb, "point_cloud", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->width, 1u);
    EXPECT_EQ(msg->height, 1u);
    // x y z  +  normal_x/y/z  +  rgb  +  confidence  +  depth
    EXPECT_EQ(msg->point_step, 3 * 4u + 3 * 4u + 4u + 4u + 4u);

    auto fieldOffset = [&](const std::string& name) -> uint32_t {
        for (const auto& f : msg->fields) {
            if (f.name == name) return f.offset;
        }
        ADD_FAILURE() << "Field '" << name << "' not found";
        return 0;
    };

    const uint8_t* d = msg->data.data();
    float x, y, z, nx, ny, nz, pcConf, pcDepth;
    uint32_t rgbPacked;
    std::memcpy(&x,       d + fieldOffset("x"),          sizeof(float));
    std::memcpy(&y,       d + fieldOffset("y"),          sizeof(float));
    std::memcpy(&z,       d + fieldOffset("z"),          sizeof(float));
    std::memcpy(&nx,      d + fieldOffset("normal_x"),   sizeof(float));
    std::memcpy(&ny,      d + fieldOffset("normal_y"),   sizeof(float));
    std::memcpy(&nz,      d + fieldOffset("normal_z"),   sizeof(float));
    std::memcpy(&rgbPacked, d + fieldOffset("rgb"),     sizeof(uint32_t));
    std::memcpy(&pcConf,  d + fieldOffset("confidence"), sizeof(float));
    std::memcpy(&pcDepth, d + fieldOffset("depth"),      sizeof(float));

    EXPECT_FLOAT_EQ(x, 1.0f);   // 1000 mm ÷ 1000
    EXPECT_FLOAT_EQ(y, 2.0f);
    EXPECT_FLOAT_EQ(z, 3.0f);
    EXPECT_FLOAT_EQ(nx, 0.1f);
    EXPECT_FLOAT_EQ(ny, 0.2f);
    EXPECT_FLOAT_EQ(nz, 0.9f);
    EXPECT_EQ((rgbPacked >> 16) & 0xFF, 255u);   // 1023/1023 * 255
    EXPECT_EQ((rgbPacked >>  8) & 0xFF, static_cast<uint8_t>((511.0f / 1023.0f) * 255.0f));
    EXPECT_EQ( rgbPacked        & 0xFF, 0u);
    EXPECT_FLOAT_EQ(pcConf,  0.75f);
    EXPECT_FLOAT_EQ(pcDepth, 500.0f);
}

TEST_F(RosInterfaceTest, PointsPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float pts[1][3] = {{500.0f, 600.0f, 700.0f}};
    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F,
                                   1, 1, sizeof(pts), pts};
    PhoXiFrame frame;
    frame.pointCloud = &pcRec;

    auto msg = injectAndReceive<sensor_msgs::msg::PointCloud2>(frameCb, "points", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->width, 1u);
    EXPECT_EQ(msg->height, 1u);
    EXPECT_EQ(msg->point_step, 12u);  // XYZ only, no color

    float x, y, z;
    std::memcpy(&x, msg->data.data() + 0, sizeof(float));
    std::memcpy(&y, msg->data.data() + 4, sizeof(float));
    std::memcpy(&z, msg->data.data() + 8, sizeof(float));
    EXPECT_FLOAT_EQ(x, 0.5f);  // 500 mm ÷ 1000
    EXPECT_FLOAT_EQ(y, 0.6f);
    EXPECT_FLOAT_EQ(z, 0.7f);
}

TEST_F(RosInterfaceTest, PointsNotPublishedWhenPointCloudEmpty) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    bool received = false;
    auto sub = testClientNode->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SystemDefaultsQoS(),
        [&received](sensor_msgs::msg::PointCloud2::SharedPtr) { received = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(PhoXiFrame{});  // empty frame — no PointCloud
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(received);
}

TEST_F(RosInterfaceTest, PointCloudNotPublishedWhenNotCombined) {
    // publish_combined = false (default): point_cloud topic must not receive any message
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float pts[1][3] = {{1000.0f, 0.0f, 0.0f}};
    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F,
                                   1, 1, sizeof(pts), pts};
    PhoXiFrame frame;
    frame.pointCloud = &pcRec;

    bool received = false;
    auto sub = testClientNode->create_subscription<sensor_msgs::msg::PointCloud2>(
        "point_cloud", rclcpp::SystemDefaultsQoS(),
        [&received](sensor_msgs::msg::PointCloud2::SharedPtr) { received = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(frame);
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(received);
}

TEST_F(RosInterfaceTest, IndividualTopicsNotPublishedWhenCombined) {
    // publish_combined = true: individual topics (e.g. points) must not receive any message
    lcNode->set_parameter(rclcpp::Parameter("publish_combined", true));
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float pts[1][3] = {{1000.0f, 0.0f, 0.0f}};
    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F,
                                   1, 1, sizeof(pts), pts};
    PhoXiFrame frame;
    frame.pointCloud = &pcRec;

    bool received = false;
    auto sub = testClientNode->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SystemDefaultsQoS(),
        [&received](sensor_msgs::msg::PointCloud2::SharedPtr) { received = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(frame);
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(received);
}

TEST_F(RosInterfaceTest, NormalMapPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float normals[1][3] = {{0.1f, 0.2f, 0.9f}};
    phoxi_frame_record_t nmRec = {PHOXI_FRAME_TYPE_NORMALMAP, PHOXI_FRAME_FORMAT_POINT3_32F,
                                   1, 1, sizeof(normals), normals};
    PhoXiFrame frame;
    frame.normalMap = &nmRec;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frameCb, "normals", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "32FC3");
    EXPECT_EQ(msg->width, 1u);
    EXPECT_EQ(msg->height, 1u);
    EXPECT_EQ(msg->step, 3u * sizeof(float));

    float nx, ny, nz;
    std::memcpy(&nx, msg->data.data() + 0 * sizeof(float), sizeof(float));
    std::memcpy(&ny, msg->data.data() + 1 * sizeof(float), sizeof(float));
    std::memcpy(&nz, msg->data.data() + 2 * sizeof(float), sizeof(float));
    EXPECT_FLOAT_EQ(nx, 0.1f);
    EXPECT_FLOAT_EQ(ny, 0.2f);
    EXPECT_FLOAT_EQ(nz, 0.9f);
}

TEST_F(RosInterfaceTest, DepthMapPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float depth[1] = {1234.5f};
    phoxi_frame_record_t depRec = {PHOXI_FRAME_TYPE_DEPTHMAP, PHOXI_FRAME_FORMAT_FLOAT_32F,
                                    1, 1, sizeof(depth), depth};
    PhoXiFrame frame;
    frame.depthMap = &depRec;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frameCb, "depth", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "32FC1");
    EXPECT_EQ(msg->width, 1u);
    EXPECT_EQ(msg->height, 1u);
    EXPECT_EQ(msg->step, sizeof(float));

    float val;
    std::memcpy(&val, msg->data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(val, 1234.5f);
}

TEST_F(RosInterfaceTest, ConfidenceMapPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float conf[1] = {0.85f};
    phoxi_frame_record_t confRec = {PHOXI_FRAME_TYPE_CONFIDENCEMAP, PHOXI_FRAME_FORMAT_FLOAT_32F,
                                     1, 1, sizeof(conf), conf};
    PhoXiFrame frame;
    frame.confidenceMap = &confRec;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frameCb, "confidence", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "32FC1");

    float val;
    std::memcpy(&val, msg->data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(val, 0.85f);
}

TEST_F(RosInterfaceTest, EventMapPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float ev[1] = {0.42f};
    phoxi_frame_record_t evRec = {PHOXI_FRAME_TYPE_EVENTMAP, PHOXI_FRAME_FORMAT_FLOAT_32F,
                                   1, 1, sizeof(ev), ev};
    PhoXiFrame frame;
    frame.eventMap = &evRec;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frameCb, "event", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "32FC1");

    float val;
    std::memcpy(&val, msg->data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(val, 0.42f);
}

TEST_F(RosInterfaceTest, IntensityPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float tex[1] = {0.75f};
    phoxi_frame_record_t texRec = {PHOXI_FRAME_TYPE_TEXTURE, PHOXI_FRAME_FORMAT_FLOAT_32F,
                                    1, 1, sizeof(tex), tex};
    PhoXiFrame frame;
    frame.texture = &texRec;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frameCb, "intensity", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "32FC1");

    float val;
    std::memcpy(&val, msg->data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(val, 0.75f);
}

TEST_F(RosInterfaceTest, TexturePublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    uint16_t rgb[2][3] = {{100, 200, 300}, {400, 500, 600}};
    phoxi_frame_record_t rgbRec = {PHOXI_FRAME_TYPE_TEXTURE, PHOXI_FRAME_FORMAT_RGB_16,
                                    2, 1, sizeof(rgb), rgb};
    PhoXiFrame frame;
    frame.textureRgb = &rgbRec;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frameCb, "texture", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "rgb16");
    EXPECT_EQ(msg->width, 2u);
    EXPECT_EQ(msg->height, 1u);
    EXPECT_EQ(msg->step, 2u * 3u * sizeof(uint16_t));

    uint16_t r0, g0, b0, r1, g1, b1;
    std::memcpy(&r0, msg->data.data() + 0 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&g0, msg->data.data() + 1 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&b0, msg->data.data() + 2 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&r1, msg->data.data() + 3 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&g1, msg->data.data() + 4 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&b1, msg->data.data() + 5 * sizeof(uint16_t), sizeof(uint16_t));
    EXPECT_EQ(r0, 100u); EXPECT_EQ(g0, 200u); EXPECT_EQ(b0, 300u);
    EXPECT_EQ(r1, 400u); EXPECT_EQ(g1, 500u); EXPECT_EQ(b1, 600u);
}

// ── Profile services ────────────────────────────────────────────────────────

TEST_F(RosInterfaceTest, ProfileServicesUnavailableWhenUnconfigured) {
    // Services are only created during on_configure, so they must not be discoverable before that.
    auto client = testClientNode->create_client<phoxi_camera_msgs::srv::GetProfileList>(
        "/phoxi_camera/profiles/list");
    EXPECT_FALSE(client->wait_for_service(std::chrono::milliseconds(500)));
}

TEST_F(RosInterfaceTest, GetProfileListReturnsProfiles) {
    ASSERT_TRUE(configureOnly());

    pho::api::PhoXiProfileDescriptor userProfile;
    userProfile.Name = "MyProfile";
    userProfile.IsFactory = false;
    pho::api::PhoXiProfileDescriptor factoryProfile;
    factoryProfile.Name = "Factory";
    factoryProfile.IsFactory = true;
    EXPECT_CALL(*mockInterface, getProfileList())
        .WillOnce(Return(std::vector<pho::api::PhoXiProfileDescriptor>{userProfile, factoryProfile}));

    auto response = callService<phoxi_camera_msgs::srv::GetProfileList>(
        "/phoxi_camera/profiles/list");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    ASSERT_EQ(response->names.size(), 2u);
    EXPECT_EQ(response->names[0], "MyProfile");
    EXPECT_FALSE(response->is_factory[0]);
    EXPECT_EQ(response->names[1], "Factory");
    EXPECT_TRUE(response->is_factory[1]);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, GetActiveProfileReturnsName) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, getActiveProfile()).WillOnce(Return("MyProfile"));

    auto response = callService<phoxi_camera_msgs::srv::GetActiveProfile>(
        "/phoxi_camera/profiles/get_active");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->name, "MyProfile");

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, SetActiveProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, setActiveProfile("MyProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::SetActiveProfile::Request>();
    request->name = "MyProfile";
    auto response = callService<phoxi_camera_msgs::srv::SetActiveProfile>(
        "/phoxi_camera/profiles/set_active", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, SetActiveProfileReportsSDKError) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, setActiveProfile(_))
        .WillOnce(Throw(PhoXiInterfaceException("Profile not found")));

    auto request = std::make_shared<phoxi_camera_msgs::srv::SetActiveProfile::Request>();
    request->name = "Missing";
    auto response = callService<phoxi_camera_msgs::srv::SetActiveProfile>(
        "/phoxi_camera/profiles/set_active", request);
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->message, "Profile not found");

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, CreateProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, createProfile("NewProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::CreateProfile::Request>();
    request->name = "NewProfile";
    auto response = callService<phoxi_camera_msgs::srv::CreateProfile>(
        "/phoxi_camera/profiles/create", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, DeleteProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, deleteProfile("OldProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::DeleteProfile::Request>();
    request->name = "OldProfile";
    auto response = callService<phoxi_camera_msgs::srv::DeleteProfile>(
        "/phoxi_camera/profiles/delete", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, UpdateProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, updateProfile("MyProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::UpdateProfile::Request>();
    request->name = "MyProfile";
    auto response = callService<phoxi_camera_msgs::srv::UpdateProfile>(
        "/phoxi_camera/profiles/update", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, GetStartupProfileReturnsName) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, getStartupProfile()).WillOnce(Return("StartupProfile"));

    auto response = callService<phoxi_camera_msgs::srv::GetStartupProfile>(
        "/phoxi_camera/profiles/get_startup");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->name, "StartupProfile");

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, SetStartupProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, setStartupProfile("MyProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::SetStartupProfile::Request>();
    request->name = "MyProfile";
    auto response = callService<phoxi_camera_msgs::srv::SetStartupProfile>(
        "/phoxi_camera/profiles/set_startup", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, ExportProfileReturnsContent) {
    ASSERT_TRUE(configureOnly());

    pho::api::PhoXiProfileContent content;
    content.Name = "MyProfile";
    content.Content = {0x01, 0x02, 0x03};
    EXPECT_CALL(*mockInterface, exportProfile()).WillOnce(Return(content));

    auto response = callService<phoxi_camera_msgs::srv::ExportProfile>(
        "/phoxi_camera/profiles/export");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->name, "MyProfile");
    ASSERT_EQ(response->content.size(), 3u);
    EXPECT_EQ(response->content[0], 0x01);
    EXPECT_EQ(response->content[1], 0x02);
    EXPECT_EQ(response->content[2], 0x03);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, ImportProfileSendsContent) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, importProfile(_))
        .WillOnce([](const pho::api::PhoXiProfileContent& c) {
            EXPECT_EQ(c.Name, "ImportedProfile");
            EXPECT_EQ(c.Content.size(), 2u);
            EXPECT_EQ(c.Content[0], 0xAB);
            EXPECT_EQ(c.Content[1], 0xCD);
        });

    auto request = std::make_shared<phoxi_camera_msgs::srv::ImportProfile::Request>();
    request->name = "ImportedProfile";
    request->content = {0xAB, 0xCD};
    auto response = callService<phoxi_camera_msgs::srv::ImportProfile>(
        "/phoxi_camera/profiles/import", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, ResetActiveProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mockInterface, resetActiveProfile()).Times(1);

    auto response = callService<std_srvs::srv::Trigger>("/phoxi_camera/profiles/reset");
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    rclcpp::init(argc, argv);

    int result = RUN_ALL_TESTS();

    rclcpp::shutdown();

    return result;
}
