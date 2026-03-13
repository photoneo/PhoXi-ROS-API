#include <chrono>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera/RosInterface.h"
#include "phoxi_camera_msgs/srv/connect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace phoxi_camera;

class HardwareIntegrationTest : public ::testing::Test
{
  protected:
    void SetUp() override {
        sensor_sn_ = "InstalledExamples-basic-example";

        rclcpp::NodeOptions options;
        options.append_parameter_override("sensor_sn", sensor_sn_);

        ros_interface_ = std::make_shared<phoxi_camera::RosInterface>(options);

        test_client_node_ = std::make_shared<rclcpp::Node>("test_client_node");

        executor_.add_node(ros_interface_->get_node_base_interface());
        executor_.add_node(test_client_node_);
    }

    void TearDown() override {}

    bool change_lc_state(uint8_t transition,
                         const std::chrono::seconds& timeout = std::chrono::seconds(10)) {
        auto client = test_client_node_->create_client<lifecycle_msgs::srv::ChangeState>(
            "/phoxi_camera/change_state");
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(test_client_node_->get_logger(), "change_state service not available!");
            return false;
        }
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        auto future = client->async_send_request(request);
        return executor_.spin_until_future_complete(future, timeout) ==
                   rclcpp::FutureReturnCode::SUCCESS &&
               future.get()->success;
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<phoxi_camera::RosInterface> ros_interface_;
    std::shared_ptr<rclcpp::Node> test_client_node_;
    std::string sensor_sn_;
};

TEST_F(HardwareIntegrationTest, FullLifecycleAndData) {
    RCLCPP_INFO(test_client_node_->get_logger(), "Configuring node...");
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
    RCLCPP_INFO(test_client_node_->get_logger(), "Activating node...");
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    RCLCPP_INFO(test_client_node_->get_logger(), "Calling connect service for SN: %s",
                sensor_sn_.c_str());
    auto connect_client =
        test_client_node_->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(connect_client->wait_for_service(std::chrono::seconds(5)));

    auto connect_request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();

    auto connect_future = connect_client->async_send_request(connect_request);
    ASSERT_EQ(executor_.spin_until_future_complete(connect_future, std::chrono::seconds(20)),
              rclcpp::FutureReturnCode::SUCCESS)
        << "Connect service call timed out!";
    ASSERT_TRUE(connect_future.get()->success)
        << "Failed to connect to the real camera! Check PhoXi Control and network. Message: "
        << connect_future.get()->message;

    RCLCPP_INFO(test_client_node_->get_logger(),
                "Subscribing to /point_cloud and triggering frame...");
    bool message_received = false;
    sensor_msgs::msg::PointCloud2::SharedPtr received_msg;
    auto sub = test_client_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud", 1, [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(test_client_node_->get_logger(),
                        "SUCCESS: Received a point cloud message!");
            message_received = true;
            received_msg = msg;
        });

    auto trigger_client =
        test_client_node_->create_client<std_srvs::srv::Trigger>("/phoxi_camera/trigger_frame");
    ASSERT_TRUE(trigger_client->wait_for_service(std::chrono::seconds(5)));

    auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto trigger_future = trigger_client->async_send_request(trigger_request);

    ASSERT_EQ(executor_.spin_until_future_complete(trigger_future, std::chrono::seconds(15)),
              rclcpp::FutureReturnCode::SUCCESS)
        << "Trigger service call timed out!";
    ASSERT_TRUE(trigger_future.get()->success);

    auto start_time = std::chrono::steady_clock::now();
    while (!message_received &&
           (std::chrono::steady_clock::now() - start_time) < std::chrono::seconds(2)) {
        executor_.spin_some(std::chrono::milliseconds(10));
    }

    ASSERT_TRUE(message_received)
        << "Node reported successful trigger, but no point cloud message was received!";
    EXPECT_GT(received_msg->data.size(), 0) << "Received point cloud is empty!";
    EXPECT_FALSE(received_msg->header.frame_id.empty()) << "Frame ID is empty!";

    RCLCPP_INFO(test_client_node_->get_logger(), "Deactivating and cleaning up...");
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
