#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include "camera_test_fixture.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "mock_phoxi_interface.h"
#include "phoxi_camera/PhoXiCamera.h"
#include "phoxi_camera/PhoXiFrame.h"
#include "phoxi_camera/PhoXiInterface.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace phoxi_camera;
using ::testing::_;

namespace {

rclcpp::NodeOptions optionsWithName(const std::string& name) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=" + name});
    return options;
}

}  // namespace

// Two independent phoxi_camera instances, renamed via NodeOptions the same way `camera_name` does
// at launch time, proving that topics/services/default TF frames stay isolated per instance.
class MultiCameraTest : public ::testing::Test {
protected:
    struct Instance {
        std::string name;
        std::string deviceId;
        std::shared_ptr<TestableNode> node;
        MockPhoXiInterface* mock = nullptr;
    };

    void SetUp() override {
        setUpInstance(camera1_, "camera_1", "device-1");
        setUpInstance(camera2_, "camera_2", "device-2");
        clientNode_ = std::make_shared<rclcpp::Node>("multi_camera_test_client");
        executor_.add_node(clientNode_);
    }

    void setUpInstance(Instance& inst, const std::string& name, const std::string& deviceId) {
        inst.name = name;
        inst.deviceId = deviceId;
        inst.node = std::make_shared<TestableNode>(deviceId, optionsWithName(name), std::make_unique<MockPhoXiInterface>());
        inst.mock = inst.node->getMock();
        testing::Mock::AllowLeak(inst.mock);
        EXPECT_CALL(*inst.mock, isConnected()).WillRepeatedly(testing::Return(false));
        EXPECT_CALL(*inst.mock, isAcquiring()).WillRepeatedly(testing::Return(false));
        EXPECT_CALL(*inst.mock, setTriggerMode(testing::A<pho::api::PhoXiTriggerMode>())).WillRepeatedly(testing::Return());
        EXPECT_CALL(*inst.mock, getDeviceInfo()).WillRepeatedly(testing::Return(phoxi_camera::PhoXiDeviceInformation{}));
        EXPECT_CALL(*inst.mock, getSettingInfos()).WillRepeatedly(testing::Return(std::vector<phoxi_camera::SettingInfo>{}));
        EXPECT_CALL(*inst.mock, getSettings(testing::_)).WillRepeatedly(testing::Return(SettingValueMap{}));
        executor_.add_node(inst.node->get_node_base_interface());
    }

    void TearDown() override {
        tearDownInstance(camera1_);
        tearDownInstance(camera2_);
        executor_.remove_node(clientNode_);
    }

    void tearDownInstance(Instance& inst) {
        if (!inst.node) {
            return;
        }
        testing::Mock::VerifyAndClearExpectations(inst.mock);
        inst.node->shutdown();
        executor_.remove_node(inst.node->get_node_base_interface());
        inst.node.reset();
    }

    bool changeLcState(const Instance& inst, uint8_t transition) {
        auto client = clientNode_->create_client<lifecycle_msgs::srv::ChangeState>("/" + inst.name + "/change_state");
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            return false;
        }
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition;
        auto future = client->async_send_request(req);
        return executor_.spin_until_future_complete(future) == rclcpp::FutureReturnCode::SUCCESS && future.get()->success;
    }

    PhoXiInterface::GetFrameCallback configureActivate(Instance& inst) {
        PhoXiInterface::GetFrameCallback cb;
        EXPECT_CALL(*inst.mock, connectCamera(inst.deviceId, _)).WillOnce([&cb](const std::string&, PhoXiInterface::GetFrameCallback&& captured) {
            cb = std::move(captured);
        });
        if (!changeLcState(inst, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
            return {};
        }
        EXPECT_CALL(*inst.mock, startAcquisition());
        if (!changeLcState(inst, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
            return {};
        }
        return cb;
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    Instance camera1_;
    Instance camera2_;
    std::shared_ptr<rclcpp::Node> clientNode_;
};

TEST_F(MultiCameraTest, TwoInstancesPublishIsolatedTopicsAndFrameIds) {
    auto cb1 = configureActivate(camera1_);
    ASSERT_TRUE(cb1);
    auto cb2 = configureActivate(camera2_);
    ASSERT_TRUE(cb2);

    int count1 = 0;
    int count2 = 0;
    sensor_msgs::msg::PointCloud2::SharedPtr msg1;
    sensor_msgs::msg::PointCloud2::SharedPtr msg2;
    auto sub1 = clientNode_->create_subscription<sensor_msgs::msg::PointCloud2>("/camera_1/points", rclcpp::SystemDefaultsQoS(), [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        ++count1;
        msg1 = msg;
    });
    auto sub2 = clientNode_->create_subscription<sensor_msgs::msg::PointCloud2>("/camera_2/points", rclcpp::SystemDefaultsQoS(), [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        ++count2;
        msg2 = msg;
    });
    executor_.spin_some(std::chrono::milliseconds(50));

    // Fire camera_1's frame only — camera_2 must stay silent (no topic cross-talk).
    float pts1[1][3] = {{100.0f, 200.0f, 300.0f}};
    phoxi_frame_record_t pcRec1 = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(pts1), pts1};
    PhoXiFrame frame1;
    frame1.pointCloud = &pcRec1;
    cb1(frame1);

    auto deadline1 = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (count1 == 0 && std::chrono::steady_clock::now() < deadline1) {
        executor_.spin_some(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(count1, 1);
    EXPECT_EQ(count2, 0);

    // Now fire camera_2's frame — camera_1's subscriber must not receive a second message.
    float pts2[1][3] = {{400.0f, 500.0f, 600.0f}};
    phoxi_frame_record_t pcRec2 = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(pts2), pts2};
    PhoXiFrame frame2;
    frame2.pointCloud = &pcRec2;
    cb2(frame2);

    auto deadline2 = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (count2 == 0 && std::chrono::steady_clock::now() < deadline2) {
        executor_.spin_some(std::chrono::milliseconds(20));
    }
    ASSERT_EQ(count2, 1);
    EXPECT_EQ(count1, 1);

    ASSERT_NE(msg1, nullptr);
    ASSERT_NE(msg2, nullptr);

    float x1, y1, z1, x2, y2, z2;
    std::memcpy(&x1, msg1->data.data() + 0, sizeof(float));
    std::memcpy(&y1, msg1->data.data() + 4, sizeof(float));
    std::memcpy(&z1, msg1->data.data() + 8, sizeof(float));
    std::memcpy(&x2, msg2->data.data() + 0, sizeof(float));
    std::memcpy(&y2, msg2->data.data() + 4, sizeof(float));
    std::memcpy(&z2, msg2->data.data() + 8, sizeof(float));

    EXPECT_FLOAT_EQ(x1, 0.1f);
    EXPECT_FLOAT_EQ(y1, 0.2f);
    EXPECT_FLOAT_EQ(z1, 0.3f);
    EXPECT_FLOAT_EQ(x2, 0.4f);
    EXPECT_FLOAT_EQ(y2, 0.5f);
    EXPECT_FLOAT_EQ(z2, 0.6f);

    // Default frame_id is derived from each instance's own node name — proves no collision
    // even though neither instance set the `frame_id` parameter explicitly.
    EXPECT_EQ(msg1->header.frame_id, "camera_1_sensor");
    EXPECT_EQ(msg2->header.frame_id, "camera_2_sensor");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
