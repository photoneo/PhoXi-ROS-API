#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera/RosInterface.h"
#include "phoxi_camera_msgs/srv/connect.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace phoxi_camera;
using ::testing::_;
using ::testing::Return;
using ::testing::Throw;

class MockPhoXiInterface : public PhoXiInterface
{
  public:
    MOCK_METHOD(void, connectCamera,
                (std::string HWIdentification, GetFrameCb&& getFrameCallback,
                 pho::api::PhoXiTriggerMode mode, bool startAcquisition),
                (override));
    MOCK_METHOD(void, disconnectCamera, (), (override));
    MOCK_METHOD(void, triggerFrame, (), (override));
};

class TestableRosInterface : public RosInterface
{
  public:
    TestableRosInterface(const rclcpp::NodeOptions& options,
                         std::unique_ptr<PhoXiInterface> phoxi_interface)
    : RosInterface(options) {
        this->phoxi_interface_ = std::move(phoxi_interface);
    }
};

class RosInterfaceTest : public ::testing::Test
{
  protected:
    void SetUp() override {
        auto mock_ptr = std::make_unique<MockPhoXiInterface>();
        mock_phoxi_interface_ = mock_ptr.get();
        testing::Mock::AllowLeak(mock_phoxi_interface_);

        rclcpp::NodeOptions options;
        lc_node_ = std::make_shared<TestableRosInterface>(options, std::move(mock_ptr));

        test_client_node_ = std::make_shared<rclcpp::Node>("test_client_node");

        executor_.add_node(lc_node_->get_node_base_interface());
        executor_.add_node(test_client_node_);

        dummy_pframe_ = std::make_shared<pho::api::Frame>();
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

    rclcpp::executors::SingleThreadedExecutor executor_;
    MockPhoXiInterface* mock_phoxi_interface_;
    std::shared_ptr<TestableRosInterface> lc_node_;
    std::shared_ptr<rclcpp::Node> test_client_node_;
    pho::api::PFrame dummy_pframe_;
};

TEST_F(RosInterfaceTest, LifecycleTransitions) {
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
    EXPECT_CALL(*mock_phoxi_interface_, disconnectCamera()).Times(1);
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
}

TEST_F(RosInterfaceTest, ConnectServiceSuccess) {
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mock_phoxi_interface_, connectCamera("test-sn-123", _, _, _)).Times(1);

    auto client =
        test_client_node_->create_client<phoxi_camera_msgs::srv::Connect>("/phoxi_camera/connect");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<phoxi_camera_msgs::srv::Connect::Request>();
    request->sn = "test-sn-123";

    auto future = client->async_send_request(request);
    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);

    EXPECT_CALL(*mock_phoxi_interface_, disconnectCamera()).Times(1);
}

TEST_F(RosInterfaceTest, ConnectServiceFailure) {
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mock_phoxi_interface_, connectCamera(_, _, _, _))
        .WillOnce(Throw(PhoXiInterfaceException("Device not found")));
    EXPECT_CALL(*mock_phoxi_interface_, disconnectCamera()).Times(1);

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

    EXPECT_CALL(*mock_phoxi_interface_, disconnectCamera()).Times(1);
}

TEST_F(RosInterfaceTest, TriggerFrame) {
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
    ASSERT_TRUE(change_lc_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

    EXPECT_CALL(*mock_phoxi_interface_, triggerFrame()).Times(1);

    auto client =
        test_client_node_->create_client<std_srvs::srv::Trigger>("/phoxi_camera/trigger_frame");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);

    EXPECT_CALL(*mock_phoxi_interface_, disconnectCamera()).Times(1);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    rclcpp::init(argc, argv);

    int result = RUN_ALL_TESTS();

    rclcpp::shutdown();

    return result;
}
