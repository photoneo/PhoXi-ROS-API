#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "camera_test_fixture.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera/PhoXiCamera.h"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/PhoXiFrame.h"
#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera_msgs/msg/frame_error.hpp"
#include "phoxi_camera_msgs/msg/frame_info.hpp"
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
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "phoxi_camera_msgs/srv/update_profile.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace phoxi_camera;
using ::testing::_;
using ::testing::Return;
using ::testing::Throw;

class RosInterfaceTest : public CameraTestFixture {
protected:
    void SetUp() override { SetUpBase("test-device-id", "test_client_node"); }

    PhoXiInterface::GetFrameCallback configureActivateCapture() {
        PhoXiInterface::GetFrameCallback cb;
        EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).WillOnce([&cb](const std::string&, PhoXiInterface::GetFrameCallback&& captured) { cb = std::move(captured); });
        if (!changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
            return {};
        EXPECT_CALL(*mockInterface, startAcquisition());
        if (!changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
            return {};
        return cb;
    }

    template <typename SrvT>
    typename SrvT::Response::SharedPtr callService(const std::string& serviceName, typename SrvT::Request::SharedPtr request = std::make_shared<typename SrvT::Request>()) {
        auto client = clientNode->create_client<SrvT>(serviceName);
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            return nullptr;
        }
        auto future = client->async_send_request(request);
        if (executor_.spin_until_future_complete(future) != rclcpp::FutureReturnCode::SUCCESS) {
            return nullptr;
        }
        return future.get();
    }

    template <typename MsgT> typename MsgT::SharedPtr injectAndReceive(const PhoXiInterface::GetFrameCallback& frameCb, const std::string& topic, const PhoXiFrame& frame) {
        std::promise<typename MsgT::SharedPtr> promise;
        auto future = promise.get_future().share();
        bool received = false;
        auto sub = clientNode->create_subscription<MsgT>(topic, rclcpp::SystemDefaultsQoS(), [&promise, &received](typename MsgT::SharedPtr msg) {
            if (!received) {
                received = true;
                promise.set_value(msg);
            }
        });
        executor_.spin_some(std::chrono::milliseconds(50));
        frameCb(frame);
        if (executor_.spin_until_future_complete(future, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS) {
            return nullptr;
        }
        return future.get();
    }
};


TEST_F(RosInterfaceTest, LifecycleTransitions) {
    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    EXPECT_CALL(*mockInterface, startAcquisition()).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mockInterface, stopAcquisition()).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));

    EXPECT_CALL(*mockInterface, disconnectCamera(testing::_, testing::_)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
}

TEST_F(RosInterfaceTest, ConfigureFailsWhenConnectionFails) {
    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).WillOnce(Throw(PhoXiInterfaceException("Device not found")));

    ASSERT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
}

TEST_F(RosInterfaceTest, ActivateFailsWhenAcquisitionFails) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, startAcquisition()).WillOnce(Throw(UnableToStartAcquisition("Hardware error")));
    ASSERT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    ASSERT_TRUE(cleanup());
}


TEST_F(RosInterfaceTest, ConnectServiceSuccessWhileActive) {
    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    EXPECT_CALL(*mockInterface, startAcquisition()).Times(2);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mockInterface, stopAcquisition()).Times(1);
    EXPECT_CALL(*mockInterface, connectCamera("test-sn-123", _)).Times(1);

    auto client = clientNode->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    request->sn = "test-sn-123";

    auto future = client->async_send_request(request);
    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
}

TEST_F(RosInterfaceTest, ConnectServiceSuccessWhileInactive) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, connectCamera("other-device", _)).Times(1);

    auto client = clientNode->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    request->sn = "other-device";

    auto future = client->async_send_request(request);
    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, ConnectServiceFailure) {
    configureActivateCapture();

    EXPECT_CALL(*mockInterface, stopAcquisition()).Times(1);
    EXPECT_CALL(*mockInterface, connectCamera(_, _)).WillOnce(Throw(PhoXiInterfaceException("Device not found")));

    auto client = clientNode->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    request->sn = "any-sn";

    auto future = client->async_send_request(request);
    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);

    auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->message, "Device not found");
}


TEST_F(RosInterfaceTest, TriggerFrame) {
    configureActivateCapture();

    EXPECT_CALL(*mockInterface, triggerFrame(false)).Times(1);

    auto client = clientNode->create_client<phoxi_camera_msgs::srv::TriggerFrame>("/phoxi_camera/trigger_frame");
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

    auto client = clientNode->create_client<phoxi_camera_msgs::srv::TriggerFrame>("/phoxi_camera/trigger_frame");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::TriggerFrame::Request>();
    request->wait_grabbing_end = true;
    auto future = client->async_send_request(request);

    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
}


TEST_F(RosInterfaceTest, ColorCameraImagePublished) {
    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    EXPECT_CALL(*mockInterface, startAcquisition()).Times(2);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mockInterface, stopAcquisition()).Times(1);
    PhoXiInterface::GetFrameCallback capturedCb;
    EXPECT_CALL(*mockInterface, connectCamera("test-sn", _)).WillOnce([&capturedCb](const std::string&, PhoXiInterface::GetFrameCallback&& cb) { capturedCb = std::move(cb); });

    auto connectClient = clientNode->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(connectClient->wait_for_service(std::chrono::seconds(2)));
    auto connectReq = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    connectReq->sn = "test-sn";
    auto connectFuture = connectClient->async_send_request(connectReq);
    ASSERT_EQ(executor_.spin_until_future_complete(connectFuture), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(connectFuture.get()->success);
    ASSERT_TRUE(capturedCb);

    uint16_t colorData[2][3] = {{100, 200, 512}, {0, 1023, 700}};
    phoxi_frame_record_t colorRec = {PHOXI_FRAME_TYPE_COLORCAMERAIMAGE, PHOXI_FRAME_FORMAT_RGB_16, 2, 1, sizeof(colorData), colorData};
    PhoXiFrame frame;
    frame.colorCamera = &colorRec;

    auto received = injectAndReceive<sensor_msgs::msg::Image>(capturedCb, "color_camera_image", frame);
    ASSERT_NE(received, nullptr);
    EXPECT_EQ(received->encoding, "rgb8");
    EXPECT_EQ(received->width, 2u);
    EXPECT_EQ(received->height, 1u);
    EXPECT_EQ(received->step, 2u * 3u * sizeof(uint8_t));

    constexpr float scale = 255.0f / 1023.0f;
    EXPECT_EQ(received->data[0], static_cast<uint8_t>(100 * scale));
    EXPECT_EQ(received->data[1], static_cast<uint8_t>(200 * scale));
    EXPECT_EQ(received->data[2], static_cast<uint8_t>(512 * scale));
    EXPECT_EQ(received->data[3], static_cast<uint8_t>(0 * scale));
    EXPECT_EQ(received->data[4], static_cast<uint8_t>(1023 * scale));
    EXPECT_EQ(received->data[5], static_cast<uint8_t>(700 * scale));
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
    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(pts), pts};
    phoxi_frame_record_t nmRec = {PHOXI_FRAME_TYPE_NORMALMAP, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(nrm), nrm};
    phoxi_frame_record_t rgbRec = {PHOXI_FRAME_TYPE_TEXTURE, PHOXI_FRAME_FORMAT_RGB_16, 1, 1, sizeof(rgb), rgb};
    phoxi_frame_record_t confRec = {PHOXI_FRAME_TYPE_CONFIDENCEMAP, PHOXI_FRAME_FORMAT_FLOAT_32F, 1, 1, sizeof(confData), confData};
    phoxi_frame_record_t depRec = {PHOXI_FRAME_TYPE_DEPTHMAP, PHOXI_FRAME_FORMAT_FLOAT_32F, 1, 1, sizeof(depthData), depthData};
    PhoXiFrame frame;
    frame.pointCloud = &pcRec;
    frame.normalMap = &nmRec;
    frame.textureRgb = &rgbRec;
    frame.confidenceMap = &confRec;
    frame.depthMap = &depRec;

    auto msg = injectAndReceive<sensor_msgs::msg::PointCloud2>(frameCb, "point_cloud", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->width, 1u);
    EXPECT_EQ(msg->height, 1u);
    EXPECT_EQ(msg->point_step, 3 * 4u + 3 * 4u + 4u + 4u + 4u);

    auto fieldOffset = [&](const std::string& name) -> uint32_t {
        for (const auto& f : msg->fields) {
            if (f.name == name)
                return f.offset;
        }
        ADD_FAILURE() << "Field '" << name << "' not found";
        return 0;
    };

    const uint8_t* d = msg->data.data();
    float x, y, z, nx, ny, nz, pcConf, pcDepth;
    uint32_t rgbPacked;
    std::memcpy(&x, d + fieldOffset("x"), sizeof(float));
    std::memcpy(&y, d + fieldOffset("y"), sizeof(float));
    std::memcpy(&z, d + fieldOffset("z"), sizeof(float));
    std::memcpy(&nx, d + fieldOffset("normal_x"), sizeof(float));
    std::memcpy(&ny, d + fieldOffset("normal_y"), sizeof(float));
    std::memcpy(&nz, d + fieldOffset("normal_z"), sizeof(float));
    std::memcpy(&rgbPacked, d + fieldOffset("rgb"), sizeof(uint32_t));
    std::memcpy(&pcConf, d + fieldOffset("confidence"), sizeof(float));
    std::memcpy(&pcDepth, d + fieldOffset("depth"), sizeof(float));

    EXPECT_FLOAT_EQ(x, 1.0f);  // 1000 mm ÷ 1000
    EXPECT_FLOAT_EQ(y, 2.0f);
    EXPECT_FLOAT_EQ(z, 3.0f);
    EXPECT_FLOAT_EQ(nx, 0.1f);
    EXPECT_FLOAT_EQ(ny, 0.2f);
    EXPECT_FLOAT_EQ(nz, 0.9f);
    EXPECT_EQ((rgbPacked >> 16) & 0xFF, 255u);  // 1023/1023 * 255
    EXPECT_EQ((rgbPacked >> 8) & 0xFF, static_cast<uint8_t>((511.0f / 1023.0f) * 255.0f));
    EXPECT_EQ(rgbPacked & 0xFF, 0u);
    EXPECT_FLOAT_EQ(pcConf, 0.75f);
    EXPECT_FLOAT_EQ(pcDepth, 500.0f);
}

TEST_F(RosInterfaceTest, PointsPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float pts[1][3] = {{500.0f, 600.0f, 700.0f}};
    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(pts), pts};
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
    auto sub = clientNode->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", rclcpp::SystemDefaultsQoS(), [&received](sensor_msgs::msg::PointCloud2::SharedPtr) { received = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(PhoXiFrame{});  // empty frame — no PointCloud
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(received);
}

TEST_F(RosInterfaceTest, PointCloudNotPublishedWhenNotCombined) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float pts[1][3] = {{1000.0f, 0.0f, 0.0f}};
    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(pts), pts};
    PhoXiFrame frame;
    frame.pointCloud = &pcRec;

    bool received = false;
    auto sub = clientNode->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", rclcpp::SystemDefaultsQoS(), [&received](sensor_msgs::msg::PointCloud2::SharedPtr) { received = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(frame);
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(received);
}

TEST_F(RosInterfaceTest, IndividualTopicsNotPublishedWhenCombined) {
    lcNode->set_parameter(rclcpp::Parameter("publish_combined", true));
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float pts[1][3] = {{1000.0f, 0.0f, 0.0f}};
    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(pts), pts};
    PhoXiFrame frame;
    frame.pointCloud = &pcRec;

    bool received = false;
    auto sub = clientNode->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", rclcpp::SystemDefaultsQoS(), [&received](sensor_msgs::msg::PointCloud2::SharedPtr) { received = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(frame);
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(received);
}

TEST_F(RosInterfaceTest, NormalMapPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float normals[1][3] = {{0.1f, 0.2f, 0.9f}};
    phoxi_frame_record_t nmRec = {PHOXI_FRAME_TYPE_NORMALMAP, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(normals), normals};
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
    phoxi_frame_record_t depRec = {PHOXI_FRAME_TYPE_DEPTHMAP, PHOXI_FRAME_FORMAT_FLOAT_32F, 1, 1, sizeof(depth), depth};
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
    phoxi_frame_record_t confRec = {PHOXI_FRAME_TYPE_CONFIDENCEMAP, PHOXI_FRAME_FORMAT_FLOAT_32F, 1, 1, sizeof(conf), conf};
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
    phoxi_frame_record_t evRec = {PHOXI_FRAME_TYPE_EVENTMAP, PHOXI_FRAME_FORMAT_FLOAT_32F, 1, 1, sizeof(ev), ev};
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

    float tex[1] = {1023.5f};
    phoxi_frame_record_t texRec = {PHOXI_FRAME_TYPE_TEXTURE, PHOXI_FRAME_FORMAT_FLOAT_32F, 1, 1, sizeof(tex), tex};
    PhoXiFrame frame;
    frame.texture = &texRec;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frameCb, "intensity", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "32FC1");

    float val;
    std::memcpy(&val, msg->data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(val, 1023.5f / 2047.0f);
}

TEST_F(RosInterfaceTest, TexturePublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    uint16_t rgb[2][3] = {{100, 200, 300}, {400, 500, 600}};
    phoxi_frame_record_t rgbRec = {PHOXI_FRAME_TYPE_TEXTURE, PHOXI_FRAME_FORMAT_RGB_16, 2, 1, sizeof(rgb), rgb};
    PhoXiFrame frame;
    frame.textureRgb = &rgbRec;

    auto msg = injectAndReceive<sensor_msgs::msg::Image>(frameCb, "texture", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, "rgb8");
    EXPECT_EQ(msg->width, 2u);
    EXPECT_EQ(msg->height, 1u);
    EXPECT_EQ(msg->step, 2u * 3u * sizeof(uint8_t));

    constexpr float scale = 255.0f / 1023.0f;
    EXPECT_EQ(msg->data[0], static_cast<uint8_t>(100 * scale));
    EXPECT_EQ(msg->data[1], static_cast<uint8_t>(200 * scale));
    EXPECT_EQ(msg->data[2], static_cast<uint8_t>(300 * scale));
    EXPECT_EQ(msg->data[3], static_cast<uint8_t>(400 * scale));
    EXPECT_EQ(msg->data[4], static_cast<uint8_t>(500 * scale));
    EXPECT_EQ(msg->data[5], static_cast<uint8_t>(600 * scale));
}


TEST_F(RosInterfaceTest, ProfileServicesUnavailableWhenUnconfigured) {
    auto client = clientNode->create_client<phoxi_camera_msgs::srv::GetProfileList>("/phoxi_camera/profiles/list");
    EXPECT_FALSE(client->wait_for_service(std::chrono::milliseconds(500)));
}

TEST_F(RosInterfaceTest, GetProfileListReturnsProfiles) {
    ASSERT_TRUE(configure());

    pho::api::PhoXiProfileDescriptor userProfile;
    userProfile.Name = "MyProfile";
    userProfile.IsFactory = false;
    pho::api::PhoXiProfileDescriptor factoryProfile;
    factoryProfile.Name = "Factory";
    factoryProfile.IsFactory = true;
    EXPECT_CALL(*mockInterface, getProfileList()).WillOnce(Return(std::vector<pho::api::PhoXiProfileDescriptor>{userProfile, factoryProfile}));

    auto response = callService<phoxi_camera_msgs::srv::GetProfileList>("/phoxi_camera/profiles/list");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    ASSERT_EQ(response->names.size(), 2u);
    EXPECT_EQ(response->names[0], "MyProfile");
    EXPECT_FALSE(response->is_factory[0]);
    EXPECT_EQ(response->names[1], "Factory");
    EXPECT_TRUE(response->is_factory[1]);

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, GetActiveProfileReturnsName) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, getActiveProfile()).WillOnce(Return("MyProfile"));

    auto response = callService<phoxi_camera_msgs::srv::GetActiveProfile>("/phoxi_camera/profiles/get_active");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->name, "MyProfile");

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, SetActiveProfileCallsInterface) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, setActiveProfile("MyProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::SetActiveProfile::Request>();
    request->name = "MyProfile";
    auto response = callService<phoxi_camera_msgs::srv::SetActiveProfile>("/phoxi_camera/profiles/set_active", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, SetActiveProfileReportsSDKError) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, setActiveProfile(_)).WillOnce(Throw(PhoXiInterfaceException("Profile not found")));

    auto request = std::make_shared<phoxi_camera_msgs::srv::SetActiveProfile::Request>();
    request->name = "Missing";
    auto response = callService<phoxi_camera_msgs::srv::SetActiveProfile>("/phoxi_camera/profiles/set_active", request);
    ASSERT_NE(response, nullptr);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->message, "Profile not found");

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, CreateProfileCallsInterface) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, createProfile("NewProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::CreateProfile::Request>();
    request->name = "NewProfile";
    auto response = callService<phoxi_camera_msgs::srv::CreateProfile>("/phoxi_camera/profiles/create", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, DeleteProfileCallsInterface) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, deleteProfile("OldProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::DeleteProfile::Request>();
    request->name = "OldProfile";
    auto response = callService<phoxi_camera_msgs::srv::DeleteProfile>("/phoxi_camera/profiles/delete", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, UpdateProfileCallsInterface) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, updateProfile("MyProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::UpdateProfile::Request>();
    request->name = "MyProfile";
    auto response = callService<phoxi_camera_msgs::srv::UpdateProfile>("/phoxi_camera/profiles/update", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, GetStartupProfileReturnsName) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, getStartupProfile()).WillOnce(Return("StartupProfile"));

    auto response = callService<phoxi_camera_msgs::srv::GetStartupProfile>("/phoxi_camera/profiles/get_startup");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->name, "StartupProfile");

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, SetStartupProfileCallsInterface) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, setStartupProfile("MyProfile")).Times(1);

    auto request = std::make_shared<phoxi_camera_msgs::srv::SetStartupProfile::Request>();
    request->name = "MyProfile";
    auto response = callService<phoxi_camera_msgs::srv::SetStartupProfile>("/phoxi_camera/profiles/set_startup", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, ExportProfileReturnsContent) {
    ASSERT_TRUE(configure());

    pho::api::PhoXiProfileContent content;
    content.Name = "MyProfile";
    content.Content = {0x01, 0x02, 0x03};
    EXPECT_CALL(*mockInterface, exportProfile()).WillOnce(Return(content));

    auto response = callService<phoxi_camera_msgs::srv::ExportProfile>("/phoxi_camera/profiles/export");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(response->name, "MyProfile");
    ASSERT_EQ(response->content.size(), 3u);
    EXPECT_EQ(response->content[0], 0x01);
    EXPECT_EQ(response->content[1], 0x02);
    EXPECT_EQ(response->content[2], 0x03);

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, ImportProfileSendsContent) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, importProfile(_)).WillOnce([](const pho::api::PhoXiProfileContent& c) {
        EXPECT_EQ(c.Name, "ImportedProfile");
        EXPECT_EQ(c.Content.size(), 2u);
        EXPECT_EQ(c.Content[0], 0xAB);
        EXPECT_EQ(c.Content[1], 0xCD);
    });

    auto request = std::make_shared<phoxi_camera_msgs::srv::ImportProfile::Request>();
    request->name = "ImportedProfile";
    request->content = {0xAB, 0xCD};
    auto response = callService<phoxi_camera_msgs::srv::ImportProfile>("/phoxi_camera/profiles/import", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanup());
}

static const std::string FRAME_INFO_JSON = R"({
    "info": {
        "current_camera/PerspectiveSettings/CameraMatrix": [800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0],
        "current_camera/PerspectiveSettings/DistortionCoefficients": [-0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "current_camera/Resolution": {"width": 640, "height": 480},
        "current_color_camera/PerspectiveSettings/CameraMatrix": [1000.0, 0.0, 400.0, 0.0, 1000.0, 300.0, 0.0, 0.0, 1.0],
        "current_color_camera/PerspectiveSettings/DistortionCoefficients": [0.05, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "current_color_camera/Resolution": {"width": 1280, "height": 960},
        "hw_id": "TEST-CAM-001",
        "index": 42,
        "total_scan_count": 1000,
        "timestamp": 16028169.085,
        "duration": 95.4,
        "duration_computation": 44.1,
        "duration_transfer": 0.33,
        "is_early_transfer_frame": false,
        "sensor_position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "sensor_x_axis": {"x": 1.0, "y": 0.0, "z": 0.0},
        "sensor_y_axis": {"x": 0.0, "y": 1.0, "z": 0.0},
        "sensor_z_axis": {"x": 0.0, "y": 0.0, "z": 1.0},
        "balance_rgb": {"x": 1.0, "y": 0.5, "z": 1.0},
        "camera_binning": {"h": 1, "w": 1},
        "camera_binning_factor": {"h": 1.0, "w": 1.0},
        "temperature": [59.0, 34.9, 36.5],
        "frame_start_time": {
            "grand_master_identity": "48b02d.fffe.55f29a",
            "port_state": "MASTER",
            "time_since_epoch": 1781179221609790625
        },
        "marker_dots": {"status": 1, "message": "Inactive", "recognized_marker_dots": []}
    },
    "msgs": [
        {"code": 3, "severity": 1, "text": "Low signal quality."}
    ]
})";

TEST_F(RosInterfaceTest, FrameInfo_CurrentCameraPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    phoxi_frame_record_t infoRec{PHOXI_FRAME_TYPE_FRAMEINFO, 0, 0, 0, FRAME_INFO_JSON.size(), const_cast<void*>(static_cast<const void*>(FRAME_INFO_JSON.data()))};
    PhoXiFrame frame;
    frame.frameInfo = &infoRec;

    auto msg = injectAndReceive<sensor_msgs::msg::CameraInfo>(frameCb, "frame_info/current_camera", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->width, 640u);
    EXPECT_EQ(msg->height, 480u);
    EXPECT_EQ(msg->distortion_model, "plumb_bob");
    ASSERT_EQ(msg->d.size(), 14u);
    EXPECT_DOUBLE_EQ(msg->d[0], -0.1);
    EXPECT_DOUBLE_EQ(msg->k[0], 800.0);  // fx
    EXPECT_DOUBLE_EQ(msg->k[4], 800.0);  // fy
    EXPECT_DOUBLE_EQ(msg->k[2], 320.0);  // cx
    EXPECT_DOUBLE_EQ(msg->k[5], 240.0);  // cy
    EXPECT_DOUBLE_EQ(msg->p[0], 800.0);  // P = [K|0]
    EXPECT_DOUBLE_EQ(msg->p[3], 0.0);
    EXPECT_NE(msg->header.frame_id, "");
}

TEST_F(RosInterfaceTest, FrameInfo_CurrentColorCameraPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    phoxi_frame_record_t infoRec{PHOXI_FRAME_TYPE_FRAMEINFO, 0, 0, 0, FRAME_INFO_JSON.size(), const_cast<void*>(static_cast<const void*>(FRAME_INFO_JSON.data()))};
    PhoXiFrame frame;
    frame.frameInfo = &infoRec;

    auto msg = injectAndReceive<sensor_msgs::msg::CameraInfo>(frameCb, "frame_info/current_color_camera", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->width, 1280u);
    EXPECT_EQ(msg->height, 960u);
    EXPECT_EQ(msg->distortion_model, "plumb_bob");
    ASSERT_EQ(msg->d.size(), 14u);
    EXPECT_DOUBLE_EQ(msg->d[0], 0.05);
    EXPECT_DOUBLE_EQ(msg->k[0], 1000.0);
    EXPECT_DOUBLE_EQ(msg->k[4], 1000.0);
    EXPECT_DOUBLE_EQ(msg->k[2], 400.0);
    EXPECT_DOUBLE_EQ(msg->k[5], 300.0);
}

TEST_F(RosInterfaceTest, FrameInfo_NotPublishedWhenFrameInfoNull) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    bool received = false;
    auto sub = clientNode->create_subscription<sensor_msgs::msg::CameraInfo>(
            "frame_info/current_camera", rclcpp::SystemDefaultsQoS(), [&received](sensor_msgs::msg::CameraInfo::SharedPtr) { received = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(PhoXiFrame{});
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(received);
}

TEST_F(RosInterfaceTest, FrameInfo_NotPublishedWhenFieldsMissing) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    const std::string incompleteJson = R"({"info": {"current_camera/Resolution": {"width": 1, "height": 1}}})";
    phoxi_frame_record_t infoRec{PHOXI_FRAME_TYPE_FRAMEINFO, 0, 0, 0, incompleteJson.size(), const_cast<void*>(static_cast<const void*>(incompleteJson.data()))};
    PhoXiFrame frame;
    frame.frameInfo = &infoRec;

    bool received = false;
    auto sub = clientNode->create_subscription<sensor_msgs::msg::CameraInfo>(
            "frame_info/current_camera", rclcpp::SystemDefaultsQoS(), [&received](sensor_msgs::msg::CameraInfo::SharedPtr) { received = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(frame);
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(received);
}

// Non-color device: JSON has CurrentCamera but no CurrentColorCamera key.
// Primary camera info must be published; color camera info must not.
TEST_F(RosInterfaceTest, FrameInfo_ColorCamera_NotPublishedWhenAbsentFromJson) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    const std::string primaryOnlyJson = R"({
        "info": {
            "current_camera/PerspectiveSettings/CameraMatrix": [800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0],
            "current_camera/PerspectiveSettings/DistortionCoefficients": [-0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "current_camera/Resolution": {"width": 640, "height": 480}
        }
    })";
    phoxi_frame_record_t infoRec{PHOXI_FRAME_TYPE_FRAMEINFO, 0, 0, 0, primaryOnlyJson.size(), const_cast<void*>(static_cast<const void*>(primaryOnlyJson.data()))};
    PhoXiFrame frame;
    frame.frameInfo = &infoRec;

    bool colorReceived = false;
    auto colorSub = clientNode->create_subscription<sensor_msgs::msg::CameraInfo>(
            "frame_info/current_color_camera", rclcpp::SystemDefaultsQoS(), [&colorReceived](sensor_msgs::msg::CameraInfo::SharedPtr) { colorReceived = true; });

    auto primaryMsg = injectAndReceive<sensor_msgs::msg::CameraInfo>(frameCb, "frame_info/current_camera", frame);

    executor_.spin_some(std::chrono::milliseconds(100));

    ASSERT_NE(primaryMsg, nullptr);
    EXPECT_EQ(primaryMsg->width, 640u);
    EXPECT_FALSE(colorReceived);
}

TEST_F(RosInterfaceTest, FrameInfo_FrameInfoMsgPublished) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    phoxi_frame_record_t infoRec{PHOXI_FRAME_TYPE_FRAMEINFO, 0, 0, 0, FRAME_INFO_JSON.size(), const_cast<void*>(static_cast<const void*>(FRAME_INFO_JSON.data()))};
    PhoXiFrame frame;
    frame.frameInfo = &infoRec;

    auto msg = injectAndReceive<phoxi_camera_msgs::msg::FrameInfo>(frameCb, "frame_info", frame);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->hw_id, "TEST-CAM-001");
    EXPECT_EQ(msg->index, 42);
    EXPECT_EQ(msg->total_scan_count, 1000);
    EXPECT_DOUBLE_EQ(msg->duration, 95.4);
    EXPECT_FALSE(msg->is_early_transfer_frame);
    EXPECT_DOUBLE_EQ(msg->sensor_position[0], 1.0);
    EXPECT_DOUBLE_EQ(msg->sensor_position[1], 2.0);
    EXPECT_DOUBLE_EQ(msg->sensor_position[2], 3.0);
    EXPECT_DOUBLE_EQ(msg->balance_rgb[0], 1.0);
    EXPECT_DOUBLE_EQ(msg->balance_rgb[1], 0.5);
    EXPECT_EQ(msg->camera_binning[0], 1);
    EXPECT_EQ(msg->camera_binning[1], 1);
    ASSERT_EQ(msg->temperature.size(), 3u);
    EXPECT_DOUBLE_EQ(msg->temperature[0], 59.0);
    EXPECT_EQ(msg->frame_start_grand_master_identity, "48b02d.fffe.55f29a");
    EXPECT_EQ(msg->frame_start_port_state, "MASTER");
    EXPECT_EQ(msg->frame_start_time_ns, 1781179221609790625LL);
    EXPECT_EQ(msg->marker_dots_status, 1);
    EXPECT_EQ(msg->marker_dots_message, "Inactive");
    ASSERT_EQ(msg->messages.size(), 1u);
    EXPECT_EQ(msg->messages[0].code, 3);
    EXPECT_EQ(msg->messages[0].severity, 1);
    EXPECT_EQ(msg->messages[0].text, "Low signal quality.");
    EXPECT_NE(msg->header.frame_id, "");
}

static const std::string FRAME_ERROR_JSON = R"({
    "successful": false,
    "msgs": [
        {"code": 11, "severity": 2, "text": "Marker was not recognized!"},
        {"code": 12, "severity": 1, "text": "Not enough valid circles!"}
    ]
})";

TEST_F(RosInterfaceTest, FrameError_PublishedWhenNotSuccessful) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    phoxi_frame_record_t infoRec{PHOXI_FRAME_TYPE_FRAMEINFO, 0, 0, 0, FRAME_ERROR_JSON.size(), const_cast<void*>(static_cast<const void*>(FRAME_ERROR_JSON.data()))};
    PhoXiFrame frame;
    frame.frameInfo = &infoRec;

    auto msg = injectAndReceive<phoxi_camera_msgs::msg::FrameError>(frameCb, "frame_error", frame);
    ASSERT_NE(msg, nullptr);
    ASSERT_EQ(msg->messages.size(), 2u);
    EXPECT_EQ(msg->messages[0].code, 11);
    EXPECT_EQ(msg->messages[0].severity, 2);
    EXPECT_EQ(msg->messages[0].text, "Marker was not recognized!");
    EXPECT_EQ(msg->messages[1].code, 12);
    EXPECT_EQ(msg->messages[1].severity, 1);
    EXPECT_NE(msg->header.frame_id, "");
}

TEST_F(RosInterfaceTest, FrameError_PointCloudNotPublishedWhenNotSuccessful) {
    lcNode->set_parameter(rclcpp::Parameter("publish_combined", true));
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    float pts[1][3] = {{1000.0f, 0.0f, 0.0f}};
    phoxi_frame_record_t pcRec{PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(pts), pts};
    phoxi_frame_record_t infoRec{PHOXI_FRAME_TYPE_FRAMEINFO, 0, 0, 0, FRAME_ERROR_JSON.size(), const_cast<void*>(static_cast<const void*>(FRAME_ERROR_JSON.data()))};
    PhoXiFrame frame;
    frame.pointCloud = &pcRec;
    frame.frameInfo = &infoRec;

    bool pcReceived = false;
    auto sub = clientNode->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", rclcpp::SystemDefaultsQoS(), [&pcReceived](sensor_msgs::msg::PointCloud2::SharedPtr) { pcReceived = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(frame);
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(pcReceived);
}

TEST_F(RosInterfaceTest, FrameError_NotPublishedWhenSuccessful) {
    auto frameCb = configureActivateCapture();
    ASSERT_TRUE(frameCb);

    phoxi_frame_record_t infoRec{PHOXI_FRAME_TYPE_FRAMEINFO, 0, 0, 0, FRAME_INFO_JSON.size(), const_cast<void*>(static_cast<const void*>(FRAME_INFO_JSON.data()))};
    PhoXiFrame frame;
    frame.frameInfo = &infoRec;

    bool errorReceived = false;
    auto sub = clientNode->create_subscription<phoxi_camera_msgs::msg::FrameError>(
            "frame_error", rclcpp::SystemDefaultsQoS(), [&errorReceived](phoxi_camera_msgs::msg::FrameError::SharedPtr) { errorReceived = true; });

    executor_.spin_some(std::chrono::milliseconds(50));
    frameCb(frame);
    executor_.spin_some(std::chrono::milliseconds(100));

    EXPECT_FALSE(errorReceived);
}

TEST_F(RosInterfaceTest, ResetActiveProfileCallsInterface) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, resetActiveProfile()).Times(1);

    auto response = callService<std_srvs::srv::Trigger>("/phoxi_camera/profiles/reset");
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);

    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, TriggerMode_DefaultIsSoftware_AppliedOnConfigure) {
    const pho::api::PhoXiTriggerMode software = pho::api::PhoXiTriggerMode::Software;
    EXPECT_CALL(*mockInterface, setTriggerMode(software)).Times(1);
    ASSERT_TRUE(configure());
    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, TriggerMode_FreerunOverride_AppliedOnConfigure) {
    SetUpBase("test-device-id", "test_client_node", {rclcpp::Parameter("trigger_mode", "Freerun")});
    const pho::api::PhoXiTriggerMode freerun = pho::api::PhoXiTriggerMode::Freerun;
    EXPECT_CALL(*mockInterface, setTriggerMode(freerun)).Times(1);
    ASSERT_TRUE(configure());
    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, TriggerMode_InvalidValue_ConfigureFails) {
    SetUpBase("test-device-id", "test_client_node", {rclcpp::Parameter("trigger_mode", "Invalid")});
    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setTriggerMode(testing::_)).Times(0);
    EXPECT_CALL(*mockInterface, disconnectCamera(testing::_, testing::_)).Times(testing::AtLeast(1));
    ASSERT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
}

TEST_F(RosInterfaceTest, TriggerMode_LiveChange_ForwardedToDevice) {
    ASSERT_TRUE(configure());
    const pho::api::PhoXiTriggerMode freerun = pho::api::PhoXiTriggerMode::Freerun;
    EXPECT_CALL(*mockInterface, setTriggerMode(freerun)).Times(1);
    auto result = lcNode->set_parameter(rclcpp::Parameter("trigger_mode", "Freerun"));
    EXPECT_TRUE(result.successful);
    ASSERT_TRUE(cleanup());
}

TEST_F(RosInterfaceTest, TriggerMode_LiveChange_InvalidValue_Rejected) {
    ASSERT_TRUE(configure());
    EXPECT_CALL(*mockInterface, setTriggerMode(testing::_)).Times(0);
    auto result = lcNode->set_parameter(rclcpp::Parameter("trigger_mode", "BadValue"));
    EXPECT_FALSE(result.successful);
    ASSERT_TRUE(cleanup());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
