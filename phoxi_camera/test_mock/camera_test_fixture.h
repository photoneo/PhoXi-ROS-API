#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "mock_phoxi_interface.h"
#include "phoxi_camera/PhoXiCamera.h"
#include "rclcpp/rclcpp.hpp"

class TestableNode : public phoxi_camera::PhoXiCamera {
public:
    TestableNode(const std::string& deviceId, const rclcpp::NodeOptions& options,
                 std::unique_ptr<phoxi_camera::PhoXiInterface> iface)
        : PhoXiCamera(deviceId, options) {
        mPhoXiInterface = std::move(iface);
    }

    MockPhoXiInterface* getMock() const {
        return static_cast<MockPhoXiInterface*>(mPhoXiInterface.get());
    }
};

class CameraTestFixture : public ::testing::Test {
protected:
    void SetUpBase(const std::string& deviceId, const std::string& clientNodeName,
                   std::vector<rclcpp::Parameter> paramOverrides = {}) {
        mDeviceId = deviceId;
        rclcpp::NodeOptions options;
        if (!paramOverrides.empty()) {
            options.parameter_overrides(paramOverrides);
        }
        lcNode = std::make_shared<TestableNode>(mDeviceId, options,
                                                std::make_unique<MockPhoXiInterface>());
        mockInterface = lcNode->getMock();
        testing::Mock::AllowLeak(mockInterface);
        EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(testing::Return(false));
        EXPECT_CALL(*mockInterface, isAcquiring()).WillRepeatedly(testing::Return(false));
        EXPECT_CALL(*mockInterface, setTriggerMode(testing::A<pho::api::PhoXiTriggerMode>())).WillRepeatedly(testing::Return());
        EXPECT_CALL(*mockInterface, getDeviceInfo())
            .WillRepeatedly(testing::Return(phoxi_camera::PhoXiDeviceInformation{}));
        EXPECT_CALL(*mockInterface, getSettingInfos())
            .WillRepeatedly(testing::Return(std::vector<phoxi_camera::SettingInfo>{}));
        EXPECT_CALL(*mockInterface, getSettings(testing::_))
            .WillRepeatedly(testing::Return(SettingValueMap{}));
        clientNode = std::make_shared<rclcpp::Node>(clientNodeName);
        executor_.add_node(lcNode->get_node_base_interface());
        executor_.add_node(clientNode);
    }

    void TearDown() override {
        if (!lcNode) {
            return;
        }
        testing::Mock::VerifyAndClearExpectations(mockInterface);
        lcNode->shutdown();
        executor_.remove_node(lcNode->get_node_base_interface());
        executor_.remove_node(clientNode);
        lcNode.reset();
        clientNode.reset();
    }

    bool changeLcState(uint8_t transition) {
        auto client = clientNode->create_client<lifecycle_msgs::srv::ChangeState>(
            "/phoxi_camera/change_state");
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            return false;
        }
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition;
        auto future = client->async_send_request(req);
        return executor_.spin_until_future_complete(future) == rclcpp::FutureReturnCode::SUCCESS &&
               future.get()->success;
    }

    bool configure() {
        EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, testing::_)).Times(1);
        return changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }

    bool cleanup() {
        EXPECT_CALL(*mockInterface, disconnectCamera()).Times(testing::AtLeast(1));
        return changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }

    void setConnected() {
        EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(testing::Return(true));
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    MockPhoXiInterface* mockInterface = nullptr;
    std::shared_ptr<TestableNode> lcNode;
    std::shared_ptr<rclcpp::Node> clientNode;
    std::string mDeviceId;
};
