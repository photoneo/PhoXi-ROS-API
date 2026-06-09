#pragma once

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera/PhoXiCamera.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

inline const sensor_msgs::msg::PointField* findField(
    const sensor_msgs::msg::PointCloud2& msg, const std::string& name) {
    for (const auto& f : msg.fields) {
        if (f.name == name) {
            return &f;
        }
    }
    return nullptr;
}

inline bool hasField(const sensor_msgs::msg::PointCloud2& msg, const std::string& name) {
    return findField(msg, name) != nullptr;
}

class DeviceRequiredTest : public ::testing::Test {
public:
    static const std::string& deviceId() {
        static const std::string id = [] {
            const char* env = std::getenv("PHO_TEST_DEVICE_ID");
            return env ? std::string(env) : std::string{};
        }();
        return id;
    }

protected:
    // Default suite setup: connects with base options (device_id only).
    // Derived classes override SetUpTestSuite to pass additional parameter overrides.
    static void SetUpTestSuite() {
        rclcpp::NodeOptions options;
        options.append_parameter_override("device_id", deviceId());
        suiteSetUp(options);
    }

    static void TearDownTestSuite() {
        suiteTearDown();
    }

    // Helper for derived-class SetUpTestSuite implementations.
    // Creates the shared executor and node, then runs CONFIGURE (device connect).
    // The node is left in INACTIVE state; per-test activation is the caller's responsibility.
    static void suiteSetUp(const rclcpp::NodeOptions& options) {
        sExecutor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        sLcNode = std::make_shared<phoxi_camera::PhoXiCamera>(options);
        sClientNode = std::make_shared<rclcpp::Node>("hw_test_client");
        sExecutor->add_node(sLcNode->get_node_base_interface());
        sExecutor->add_node(sClientNode);
        auto client = sClientNode->create_client<lifecycle_msgs::srv::ChangeState>(
            "/phoxi_camera/change_state");
        ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        auto future = client->async_send_request(req);
        ASSERT_EQ(sExecutor->spin_until_future_complete(future, std::chrono::seconds(120)),
                  rclcpp::FutureReturnCode::SUCCESS)
            << "Configure timed out for device: " << deviceId();
        ASSERT_TRUE(future.get()->success) << "Failed to configure device: " << deviceId();
    }

    static void suiteTearDown() {
        if (!sExecutor || !sLcNode || !sClientNode) {
            return;
        }
        // Best-effort cleanup; the node may already be unconfigured if a test ran CLEANUP.
        auto client = sClientNode->create_client<lifecycle_msgs::srv::ChangeState>(
            "/phoxi_camera/change_state");
        if (client->wait_for_service(std::chrono::seconds(5))) {
            auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
            auto future = client->async_send_request(req);
            sExecutor->spin_until_future_complete(future, std::chrono::seconds(10));
        }
        sExecutor->remove_node(sLcNode->get_node_base_interface());
        sExecutor->remove_node(sClientNode);
        sLcNode->shutdown();
        sLcNode.reset();
        sClientNode.reset();
        sExecutor.reset();
    }

    DeviceRequiredTest() : mExecutor(*sExecutor) {}

    // Assigns the shared node/client to per-test instance members.
    // Does NOT activate — tests or subclasses that need ACTIVE state call
    // changeLcState(TRANSITION_ACTIVATE) in their own SetUp.
    void SetUp() override {
        mLcNode = sLcNode;
        mClientNode = sClientNode;
    }

    void TearDown() override {
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

    rclcpp::executors::SingleThreadedExecutor& mExecutor;
    std::shared_ptr<phoxi_camera::PhoXiCamera> mLcNode;
    std::shared_ptr<rclcpp::Node> mClientNode;

    static inline std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> sExecutor;
    static inline std::shared_ptr<phoxi_camera::PhoXiCamera> sLcNode;
    static inline std::shared_ptr<rclcpp::Node> sClientNode;
};
