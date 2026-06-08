#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera/PhoXiCamera.h"
#include "rclcpp/rclcpp.hpp"

class DeviceRequiredTest : public ::testing::Test {
public:
    static std::string& deviceId() {
        static std::string id;
        return id;
    }

    static void parseDeviceId(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg.rfind("--device_id=", 0) == 0) {
                deviceId() = arg.substr(12);
            } else if (arg == "--device_id" && i + 1 < argc) {
                deviceId() = argv[++i];
            }
        }
    }

protected:
    virtual rclcpp::NodeOptions makeNodeOptions() {
        rclcpp::NodeOptions options;
        options.append_parameter_override("device_id", deviceId());
        return options;
    }

    void SetUp() override {
        mLcNode = std::make_shared<phoxi_camera::PhoXiCamera>(makeNodeOptions());
        mClientNode = std::make_shared<rclcpp::Node>("hw_test_client");
        mExecutor.add_node(mLcNode->get_node_base_interface());
        mExecutor.add_node(mClientNode);
        ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                                  std::chrono::seconds(120)))
            << "Failed to configure device: " << deviceId();
    }

    void TearDown() override {
        changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, std::chrono::seconds(10));
        mExecutor.remove_node(mLcNode->get_node_base_interface());
        mExecutor.remove_node(mClientNode);
        mLcNode->shutdown();
        mLcNode.reset();
        mClientNode.reset();
    }

    bool changeLcState(uint8_t transition,
                       std::chrono::seconds timeout = std::chrono::seconds(10)) {
        auto client = mClientNode->create_client<lifecycle_msgs::srv::ChangeState>(
            "/phoxi_camera/change_state");
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            return false;
        }
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition;
        auto future = client->async_send_request(req);
        return mExecutor.spin_until_future_complete(future, timeout) ==
                   rclcpp::FutureReturnCode::SUCCESS &&
               future.get()->success;
    }

    template <typename SrvT>
    typename SrvT::Response::SharedPtr callService(
        const std::string& name,
        typename SrvT::Request::SharedPtr req = std::make_shared<typename SrvT::Request>(),
        std::chrono::seconds timeout = std::chrono::seconds(30)) {
        auto client = mClientNode->create_client<SrvT>(name);
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            return nullptr;
        }
        auto future = client->async_send_request(req);
        if (mExecutor.spin_until_future_complete(future, timeout) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            return nullptr;
        }
        return future.get();
    }

    rclcpp::executors::SingleThreadedExecutor mExecutor;
    std::shared_ptr<phoxi_camera::PhoXiCamera> mLcNode;
    std::shared_ptr<rclcpp::Node> mClientNode;
};
