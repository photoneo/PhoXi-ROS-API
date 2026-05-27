#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera/PhoXiException.h"
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
                (const std::string& deviceId, GetFrameCb&& getFrameCallback),
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
    TestableRosInterface(const std::string& device_id, const rclcpp::NodeOptions& options,
                         std::unique_ptr<PhoXiInterface> phoxi_interface)
    : PhoXiCamera(device_id, options) {
        this->mPhoXiInterface = std::move(phoxi_interface);
    }
};

class RosInterfaceTest : public ::testing::Test
{
  protected:
    void SetUp() override {
        auto mock_ptr = std::make_unique<MockPhoXiInterface>();
        mock_mPhoXiInterface = mock_ptr.get();
        testing::Mock::AllowLeak(mock_mPhoXiInterface);

        EXPECT_CALL(*mock_mPhoXiInterface, isConnected()).WillRepeatedly(Return(false));

        rclcpp::NodeOptions options;
        lc_node_ = std::make_shared<TestableRosInterface>(kTestDeviceId, options,
                                                          std::move(mock_ptr));

        test_client_node_ = std::make_shared<rclcpp::Node>("test_client_node");

        executor_.add_node(lc_node_->get_node_base_interface());
        executor_.add_node(test_client_node_);
    }

    void TearDown() override {
        executor_.remove_node(lc_node_->get_node_base_interface());
        executor_.remove_node(test_client_node_);

        lc_node_.reset();
        test_client_node_.reset();
    }

    bool change_lc_state(uint8_t transition) {
        auto lc_client = test_client_node_->create_client<lifecycle_msgs::srv::ChangeState>(
            "/phoxi_camera/change_state");
        if (!lc_client->wait_for_service(std::chrono::seconds(2))) {
            return false;
        }
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        auto future = lc_client->async_send_request(request);
        return executor_.spin_until_future_complete(future) == rclcpp::FutureReturnCode::SUCCESS &&
               future.get()->success;
    }

    // Configure-only: set up connectCamera expectation and run CONFIGURE transition.
    bool configureOnly() {
        EXPECT_CALL(*mock_mPhoXiInterface, connectCamera(kTestDeviceId, _)).Times(1);
        return change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }

    // Cleanup with disconnect: set up disconnectCamera expectation and run CLEANUP transition.
    bool cleanupWithDisconnect() {
        EXPECT_CALL(*mock_mPhoXiInterface, disconnectCamera()).Times(1);
        return change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }

    // Configure + activate, capturing the frame callback from connectCamera.
    PhoXiInterface::GetFrameCb configureActivateCapture() {
        PhoXiInterface::GetFrameCb cb;
        EXPECT_CALL(*mock_mPhoXiInterface, connectCamera(kTestDeviceId, _))
            .WillOnce([&cb](const std::string&, PhoXiInterface::GetFrameCb&& captured) {
                cb = std::move(captured);
            });
        if (!change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) return {};
        EXPECT_CALL(*mock_mPhoXiInterface, startAcquisition());
        if (!change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) return {};
        return cb;
    }

    // Call a ROS2 service and return its response, or nullptr on timeout/failure.
    template<typename SrvT>
    typename SrvT::Response::SharedPtr call_service(
        const std::string& service_name,
        typename SrvT::Request::SharedPtr request = std::make_shared<typename SrvT::Request>()) {
        auto client = test_client_node_->create_client<SrvT>(service_name);
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
        const PhoXiInterface::GetFrameCb& frame_cb,
        const std::string& topic,
        pho::api::PFrame frame) {
        std::promise<typename MsgT::SharedPtr> promise;
        auto future = promise.get_future().share();
        bool received = false;
        auto sub = test_client_node_->create_subscription<MsgT>(
            topic, rclcpp::SystemDefaultsQoS(),
            [&promise, &received](typename MsgT::SharedPtr msg) {
                if (!received) { received = true; promise.set_value(msg); }
            });
        executor_.spin_some(std::chrono::milliseconds(50));
        frame_cb(frame);
        if (executor_.spin_until_future_complete(future, std::chrono::seconds(2)) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            return nullptr;
        }
        return future.get();
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    MockPhoXiInterface* mock_mPhoXiInterface;
    std::shared_ptr<TestableRosInterface> lc_node_;
    std::shared_ptr<rclcpp::Node> test_client_node_;
};

// ── Lifecycle transitions ────────────────────────────────────────────────────

TEST_F(RosInterfaceTest, LifecycleTransitions) {
    EXPECT_CALL(*mock_mPhoXiInterface, connectCamera(kTestDeviceId, _)).Times(1);
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    EXPECT_CALL(*mock_mPhoXiInterface, startAcquisition()).Times(1);
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mock_mPhoXiInterface, stopAcquisition()).Times(1);
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));

    EXPECT_CALL(*mock_mPhoXiInterface, disconnectCamera()).Times(1);
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
}

TEST_F(RosInterfaceTest, ConfigureFailsWhenConnectionFails) {
    EXPECT_CALL(*mock_mPhoXiInterface, connectCamera(kTestDeviceId, _))
        .WillOnce(Throw(PhoXiInterfaceException("Device not found")));

    ASSERT_FALSE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
}

TEST_F(RosInterfaceTest, ActivateFailsWhenAcquisitionFails) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, startAcquisition())
        .WillOnce(Throw(UnableToStartAcquisition("Hardware error")));
    ASSERT_FALSE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    ASSERT_TRUE(cleanupWithDisconnect());
}

// ── Connect service ──────────────────────────────────────────────────────────

TEST_F(RosInterfaceTest, ConnectServiceSuccessWhileActive) {
    EXPECT_CALL(*mock_mPhoXiInterface, connectCamera(kTestDeviceId, _)).Times(1);
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    // startAcquisition called twice: once in on_activate, once in connect_cb (is_active=true)
    EXPECT_CALL(*mock_mPhoXiInterface, startAcquisition()).Times(2);
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mock_mPhoXiInterface, connectCamera("test-sn-123", _)).Times(1);

    auto client =
        test_client_node_->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    request->sn = "test-sn-123";

    auto future = client->async_send_request(request);
    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
}

TEST_F(RosInterfaceTest, ConnectServiceSuccessWhileInactive) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, connectCamera("other-device", _)).Times(1);

    auto client =
        test_client_node_->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
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

    EXPECT_CALL(*mock_mPhoXiInterface, connectCamera(_, _))
        .WillOnce(Throw(PhoXiInterfaceException("Device not found")));

    auto client =
        test_client_node_->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
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

    EXPECT_CALL(*mock_mPhoXiInterface, triggerFrame(false)).Times(1);

    auto client = test_client_node_->create_client<phoxi_camera_msgs::srv::TriggerFrame>(
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

    EXPECT_CALL(*mock_mPhoXiInterface, triggerFrame(true)).Times(1);

    auto client = test_client_node_->create_client<phoxi_camera_msgs::srv::TriggerFrame>(
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
    EXPECT_CALL(*mock_mPhoXiInterface, connectCamera(kTestDeviceId, _)).Times(1);
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    // startAcquisition called twice: once in on_activate, once in connect_cb (is_active=true)
    EXPECT_CALL(*mock_mPhoXiInterface, startAcquisition()).Times(2);
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    PhoXiInterface::GetFrameCb captured_cb;
    EXPECT_CALL(*mock_mPhoXiInterface, connectCamera("test-sn", _))
        .WillOnce([&captured_cb](const std::string&, PhoXiInterface::GetFrameCb&& cb) {
            captured_cb = std::move(cb);
        });

    auto connect_client =
        test_client_node_->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(connect_client->wait_for_service(std::chrono::seconds(2)));
    auto connect_req = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    connect_req->sn = "test-sn";
    auto connect_future = connect_client->async_send_request(connect_req);
    ASSERT_EQ(executor_.spin_until_future_complete(connect_future), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(connect_future.get()->success);
    ASSERT_TRUE(captured_cb);

    auto pframe = std::make_shared<pho::api::Frame>();
    pframe->ColorCameraImage.Resize(pho::api::PhoXiSize(2, 1));
    pframe->ColorCameraImage[0][0] = {1000, 2000, 3000};
    pframe->ColorCameraImage[0][1] = {4000, 5000, 6000};

    auto received = injectAndReceive<sensor_msgs::msg::Image>(captured_cb, "color_camera_image", pframe);
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
    auto frame_cb = configureActivateCapture();
    ASSERT_TRUE(frame_cb);

    auto pframe = std::make_shared<pho::api::Frame>();
    pframe->PointCloud.Resize(pho::api::PhoXiSize(1, 1));
    pframe->PointCloud[0][0] = {1000.0f, 2000.0f, 3000.0f};
    pframe->NormalMap.Resize(pho::api::PhoXiSize(1, 1));
    pframe->NormalMap[0][0] = {0.1f, 0.2f, 0.9f};
    pframe->TextureRGB.Resize(pho::api::PhoXiSize(1, 1));
    pframe->TextureRGB[0][0] = {1023, 511, 0};
    pframe->ConfidenceMap.Resize(pho::api::PhoXiSize(1, 1));
    pframe->ConfidenceMap[0][0] = 0.75f;
    pframe->DepthMap.Resize(pho::api::PhoXiSize(1, 1));
    pframe->DepthMap[0][0] = 500.0f;

    auto msg = injectAndReceive<sensor_msgs::msg::PointCloud2>(frame_cb, "point_cloud", pframe);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->width, 1u);
    EXPECT_EQ(msg->height, 1u);
    // x y z  +  normal_x/y/z  +  rgb  +  confidence  +  depth
    EXPECT_EQ(msg->point_step, 3 * 4u + 3 * 4u + 4u + 4u + 4u);

    auto field_offset = [&](const std::string& name) -> uint32_t {
        for (const auto& f : msg->fields) {
            if (f.name == name) return f.offset;
        }
        ADD_FAILURE() << "Field '" << name << "' not found";
        return 0;
    };

    const uint8_t* d = msg->data.data();
    float x, y, z, nx, ny, nz, conf, depth;
    uint32_t rgb_packed;
    std::memcpy(&x,         d + field_offset("x"),          sizeof(float));
    std::memcpy(&y,         d + field_offset("y"),          sizeof(float));
    std::memcpy(&z,         d + field_offset("z"),          sizeof(float));
    std::memcpy(&nx,        d + field_offset("normal_x"),   sizeof(float));
    std::memcpy(&ny,        d + field_offset("normal_y"),   sizeof(float));
    std::memcpy(&nz,        d + field_offset("normal_z"),   sizeof(float));
    std::memcpy(&rgb_packed,d + field_offset("rgb"),        sizeof(uint32_t));
    std::memcpy(&conf,      d + field_offset("confidence"), sizeof(float));
    std::memcpy(&depth,     d + field_offset("depth"),      sizeof(float));

    EXPECT_FLOAT_EQ(x, 1.0f);   // 1000 mm ÷ 1000
    EXPECT_FLOAT_EQ(y, 2.0f);
    EXPECT_FLOAT_EQ(z, 3.0f);
    EXPECT_FLOAT_EQ(nx, 0.1f);
    EXPECT_FLOAT_EQ(ny, 0.2f);
    EXPECT_FLOAT_EQ(nz, 0.9f);
    EXPECT_EQ((rgb_packed >> 16) & 0xFF, 255u);   // 1023/1023 * 255
    EXPECT_EQ((rgb_packed >>  8) & 0xFF, static_cast<uint8_t>((511.0f / 1023.0f) * 255.0f));
    EXPECT_EQ( rgb_packed        & 0xFF, 0u);
    EXPECT_FLOAT_EQ(conf,  0.75f);
    EXPECT_FLOAT_EQ(depth, 500.0f);
}

TEST_F(RosInterfaceTest, PointsPublished) {
    auto frame_cb = configureActivateCapture();
    ASSERT_TRUE(frame_cb);

    auto pframe = std::make_shared<pho::api::Frame>();
    pframe->PointCloud.Resize(pho::api::PhoXiSize(1, 1));
    pframe->PointCloud[0][0] = {500.0f, 600.0f, 700.0f};

    auto msg = injectAndReceive<sensor_msgs::msg::PointCloud2>(frame_cb, "points", pframe);
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
    auto frame_cb = configureActivateCapture();
    ASSERT_TRUE(frame_cb);

    bool received = false;
    auto sub = test_client_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SystemDefaultsQoS(),
        [&received](sensor_msgs::msg::PointCloud2::SharedPtr) { received = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frame_cb(std::make_shared<pho::api::Frame>());  // empty frame — no PointCloud
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(received);
}

TEST_F(RosInterfaceTest, NormalMapPublished) {
    auto frame_cb = configureActivateCapture();
    ASSERT_TRUE(frame_cb);

    auto pframe = std::make_shared<pho::api::Frame>();
    pframe->NormalMap.Resize(pho::api::PhoXiSize(1, 1));
    pframe->NormalMap[0][0] = {0.1f, 0.2f, 0.9f};

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frame_cb, "normal_map", pframe);
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
    auto frame_cb = configureActivateCapture();
    ASSERT_TRUE(frame_cb);

    auto pframe = std::make_shared<pho::api::Frame>();
    pframe->DepthMap.Resize(pho::api::PhoXiSize(1, 1));
    pframe->DepthMap[0][0] = 1234.5f;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frame_cb, "depth", pframe);
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
    auto frame_cb = configureActivateCapture();
    ASSERT_TRUE(frame_cb);

    auto pframe = std::make_shared<pho::api::Frame>();
    pframe->ConfidenceMap.Resize(pho::api::PhoXiSize(1, 1));
    pframe->ConfidenceMap[0][0] = 0.85f;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frame_cb, "confidence", pframe);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "32FC1");

    float val;
    std::memcpy(&val, msg->data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(val, 0.85f);
}

TEST_F(RosInterfaceTest, EventMapPublished) {
    auto frame_cb = configureActivateCapture();
    ASSERT_TRUE(frame_cb);

    auto pframe = std::make_shared<pho::api::Frame>();
    pframe->EventMap.Resize(pho::api::PhoXiSize(1, 1));
    pframe->EventMap[0][0] = 0.42f;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frame_cb, "event_map", pframe);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "32FC1");

    float val;
    std::memcpy(&val, msg->data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(val, 0.42f);
}

TEST_F(RosInterfaceTest, TexturePublished) {
    auto frame_cb = configureActivateCapture();
    ASSERT_TRUE(frame_cb);

    auto pframe = std::make_shared<pho::api::Frame>();
    pframe->Texture.Resize(pho::api::PhoXiSize(1, 1));
    pframe->Texture[0][0] = 0.75f;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frame_cb, "texture", pframe);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "32FC1");

    float val;
    std::memcpy(&val, msg->data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(val, 0.75f);
}

TEST_F(RosInterfaceTest, TextureRgbPublished) {
    auto frame_cb = configureActivateCapture();
    ASSERT_TRUE(frame_cb);

    auto pframe = std::make_shared<pho::api::Frame>();
    pframe->TextureRGB.Resize(pho::api::PhoXiSize(2, 1));
    pframe->TextureRGB[0][0] = {100, 200, 300};
    pframe->TextureRGB[0][1] = {400, 500, 600};

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frame_cb, "texture_rgb", pframe);
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
    auto client = test_client_node_->create_client<phoxi_camera_msgs::srv::GetProfileList>(
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
    EXPECT_CALL(*mock_mPhoXiInterface, getProfileList())
        .WillOnce(Return(std::vector<pho::api::PhoXiProfileDescriptor>{userProfile, factoryProfile}));

    auto response = call_service<phoxi_camera_msgs::srv::GetProfileList>(
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

    EXPECT_CALL(*mock_mPhoXiInterface, getActiveProfile()).WillOnce(Return("MyProfile"));

    auto response = call_service<phoxi_camera_msgs::srv::GetActiveProfile>(
        "/phoxi_camera/profiles/get_active");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->name, "MyProfile");

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, SetActiveProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, setActiveProfile("MyProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::SetActiveProfile::Request>();
    request->name = "MyProfile";
    auto response = call_service<phoxi_camera_msgs::srv::SetActiveProfile>(
        "/phoxi_camera/profiles/set_active", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, SetActiveProfileReportsSDKError) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, setActiveProfile(_))
        .WillOnce(Throw(PhoXiInterfaceException("Profile not found")));

    auto request = std::make_shared<phoxi_camera_msgs::srv::SetActiveProfile::Request>();
    request->name = "Missing";
    auto response = call_service<phoxi_camera_msgs::srv::SetActiveProfile>(
        "/phoxi_camera/profiles/set_active", request);
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->message, "Profile not found");

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, CreateProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, createProfile("NewProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::CreateProfile::Request>();
    request->name = "NewProfile";
    auto response = call_service<phoxi_camera_msgs::srv::CreateProfile>(
        "/phoxi_camera/profiles/create", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, DeleteProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, deleteProfile("OldProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::DeleteProfile::Request>();
    request->name = "OldProfile";
    auto response = call_service<phoxi_camera_msgs::srv::DeleteProfile>(
        "/phoxi_camera/profiles/delete", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, UpdateProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, updateProfile("MyProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::UpdateProfile::Request>();
    request->name = "MyProfile";
    auto response = call_service<phoxi_camera_msgs::srv::UpdateProfile>(
        "/phoxi_camera/profiles/update", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, GetStartupProfileReturnsName) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, getStartupProfile()).WillOnce(Return("StartupProfile"));

    auto response = call_service<phoxi_camera_msgs::srv::GetStartupProfile>(
        "/phoxi_camera/profiles/get_startup");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->name, "StartupProfile");

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, SetStartupProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, setStartupProfile("MyProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::SetStartupProfile::Request>();
    request->name = "MyProfile";
    auto response = call_service<phoxi_camera_msgs::srv::SetStartupProfile>(
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
    EXPECT_CALL(*mock_mPhoXiInterface, exportProfile()).WillOnce(Return(content));

    auto response = call_service<phoxi_camera_msgs::srv::ExportProfile>(
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

    EXPECT_CALL(*mock_mPhoXiInterface, importProfile(_))
        .WillOnce([](const pho::api::PhoXiProfileContent& c) {
            EXPECT_EQ(c.Name, "ImportedProfile");
            EXPECT_EQ(c.Content.size(), 2u);
            EXPECT_EQ(c.Content[0], 0xAB);
            EXPECT_EQ(c.Content[1], 0xCD);
        });

    auto request = std::make_shared<phoxi_camera_msgs::srv::ImportProfile::Request>();
    request->name = "ImportedProfile";
    request->content = {0xAB, 0xCD};
    auto response = call_service<phoxi_camera_msgs::srv::ImportProfile>(
        "/phoxi_camera/profiles/import", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanupWithDisconnect());
}

TEST_F(RosInterfaceTest, ResetActiveProfileCallsInterface) {
    ASSERT_TRUE(configureOnly());

    EXPECT_CALL(*mock_mPhoXiInterface, resetActiveProfile()).Times(1);

    auto response = call_service<std_srvs::srv::Trigger>("/phoxi_camera/profiles/reset");
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
