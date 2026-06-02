#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "phoxi_camera/PhoXiCamera.h"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/PhoXiInterface.h"
#include "rclcpp/rclcpp.hpp"

using namespace phoxi_camera;
using ::testing::_;
using ::testing::Pair;
using ::testing::Return;
using ::testing::Throw;
using ::testing::UnorderedElementsAre;

static constexpr const char* kDeviceId = "test-device-fs";

// Alias needed because MOCK_METHOD macros cannot handle commas inside
// template arguments.
using ComponentList = std::vector<std::pair<std::string, bool>>;

// ---------------------------------------------------------------------------
// Mock
// ---------------------------------------------------------------------------

class MockPhoXiInterface : public PhoXiInterface
{
  public:
    MOCK_METHOD(void, connectCamera,
        (const std::string&, GetFrameCallback&&), (override));
    MOCK_METHOD(void, disconnectCamera, (), (override));
    MOCK_METHOD(void, triggerFrame, (bool), (override));
    MOCK_METHOD(void, startAcquisition, (), (override));
    MOCK_METHOD(void, stopAcquisition, (), (override));
    MOCK_METHOD(bool, isConnected, (), (override));
    MOCK_METHOD(bool, isAcquiring, (), (override));
    MOCK_METHOD(void, setFrameOutputSettings, (const ComponentList&), (override));
    MOCK_METHOD(std::vector<pho::api::PhoXiProfileDescriptor>, getProfileList, (), (override));
    MOCK_METHOD(std::string, getActiveProfile, (), (override));
    MOCK_METHOD(void, setActiveProfile, (const std::string&), (override));
    MOCK_METHOD(std::string, getStartupProfile, (), (override));
    MOCK_METHOD(void, setStartupProfile, (const std::string&), (override));
    MOCK_METHOD(void, createProfile, (const std::string&), (override));
    MOCK_METHOD(void, deleteProfile, (const std::string&), (override));
    MOCK_METHOD(void, updateProfile, (const std::string&), (override));
    MOCK_METHOD(pho::api::PhoXiProfileContent, exportProfile, (), (override));
    MOCK_METHOD(void, importProfile, (const pho::api::PhoXiProfileContent&), (override));
    MOCK_METHOD(void, resetActiveProfile, (), (override));
};

class TestableNode : public PhoXiCamera
{
  public:
    TestableNode(const std::string& deviceId, const rclcpp::NodeOptions& options,
        std::unique_ptr<PhoXiInterface> iface)
    : PhoXiCamera(deviceId, options) {
        mPhoXiInterface = std::move(iface);
    }
};

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class FrameSettingsTest : public ::testing::Test
{
  protected:
    void SetUp() override {
        auto mockPtr = std::make_unique<MockPhoXiInterface>();
        mockInterface = mockPtr.get();
        testing::Mock::AllowLeak(mockInterface);

        EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(false));
        EXPECT_CALL(*mockInterface, isAcquiring()).WillRepeatedly(Return(false));

        rclcpp::NodeOptions options;
        lcNode = std::make_shared<TestableNode>(kDeviceId, options, std::move(mockPtr));
        clientNode = std::make_shared<rclcpp::Node>("fs_test_client");

        executor_.add_node(lcNode->get_node_base_interface());
        executor_.add_node(clientNode);
    }

    void TearDown() override {
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
        return executor_.spin_until_future_complete(future) == rclcpp::FutureReturnCode::SUCCESS
            && future.get()->success;
    }

    bool configure() {
        EXPECT_CALL(*mockInterface, connectCamera(kDeviceId, _)).Times(1);
        return changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }

    bool cleanup() {
        EXPECT_CALL(*mockInterface, disconnectCamera()).Times(testing::AtLeast(1));
        return changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }

    // Simulate loading a value from YAML: set the param before configure so
    // on_configure picks it up via get_parameter().
    void setParamBeforeConfigure(const std::string& component, bool enabled) {
        lcNode->set_parameter(
            rclcpp::Parameter("frameSettings/" + component, enabled));
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    MockPhoXiInterface* mockInterface = nullptr;
    std::shared_ptr<TestableNode> lcNode;
    std::shared_ptr<rclcpp::Node> clientNode;
};

// ---------------------------------------------------------------------------
// on_configure behaviour
// ---------------------------------------------------------------------------

TEST_F(FrameSettingsTest, Configure_NoParamsSet_DoesNotCallSetFrameOutputSettings) {
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);
    ASSERT_TRUE(configure());
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_SingleComponentEnabled_CallsWithCorrectComponent) {
    setParamBeforeConfigure("PointCloud", true);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    ASSERT_TRUE(configure());

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "PointCloud");
    EXPECT_TRUE(captured[0].second);

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_ComponentDisabled_PassedAsFalse) {
    setParamBeforeConfigure("NormalMap", false);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    ASSERT_TRUE(configure());

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "NormalMap");
    EXPECT_FALSE(captured[0].second);

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_MultipleComponents_AllPassedTogether) {
    setParamBeforeConfigure("DepthMap", true);
    setParamBeforeConfigure("ConfidenceMap", false);
    setParamBeforeConfigure("EventMap", false);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    ASSERT_TRUE(configure());

    EXPECT_THAT(captured, UnorderedElementsAre(
        Pair("DepthMap", true),
        Pair("ConfidenceMap", false),
        Pair("EventMap", false)));

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_AllComponents_AllSevenPassed) {
    for (const auto* name : {"PointCloud", "NormalMap", "DepthMap", "Texture",
                              "ConfidenceMap", "ColorCameraImage", "EventMap"}) {
        setParamBeforeConfigure(name, true);
    }

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    ASSERT_TRUE(configure());

    EXPECT_EQ(captured.size(), 7u);

    ASSERT_TRUE(cleanup());
}

// ---------------------------------------------------------------------------
// on_configure error paths
// ---------------------------------------------------------------------------

TEST_F(FrameSettingsTest, Configure_SetFrameOutputSettingsThrows_ReturnsFailure) {
    setParamBeforeConfigure("Texture", true);

    EXPECT_CALL(*mockInterface, connectCamera(kDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce(Throw(PhoXiInterfaceException("SDK rejected component")));
    EXPECT_CALL(*mockInterface, disconnectCamera()).Times(1);

    EXPECT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
}

TEST_F(FrameSettingsTest, Configure_SetFrameOutputSettingsThrows_CanReconfigureSuccessfully) {
    setParamBeforeConfigure("Texture", true);

    // First configure: setFrameOutputSettings throws → FAILURE + disconnect
    EXPECT_CALL(*mockInterface, connectCamera(kDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce(Throw(PhoXiInterfaceException("transient error")));
    EXPECT_CALL(*mockInterface, disconnectCamera()).Times(1);
    ASSERT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    // Second configure: succeeds
    EXPECT_CALL(*mockInterface, connectCamera(kDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    ASSERT_TRUE(cleanup());
}

// ---------------------------------------------------------------------------
// Dynamic parameter changes (onParametersChanged)
// ---------------------------------------------------------------------------

TEST_F(FrameSettingsTest, ParameterChange_WhenConnected_CallsSetFrameOutputSettings) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    auto result = lcNode->set_parameter(rclcpp::Parameter("frameSettings/DepthMap", false));
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "DepthMap");
    EXPECT_FALSE(captured[0].second);

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_WhenNotConnected_DoesNotCallSetFrameOutputSettings) {
    ASSERT_TRUE(configure());

    // isConnected stays false (ON_CALL default from SetUp)
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);

    auto result = lcNode->set_parameter(rclcpp::Parameter("frameSettings/NormalMap", false));
    EXPECT_TRUE(result.successful);

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_MultipleComponents_AllPassedInSingleCall) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    lcNode->set_parameters_atomically({
        rclcpp::Parameter("frameSettings/PointCloud", true),
        rclcpp::Parameter("frameSettings/ColorCameraImage", false),
    });

    EXPECT_THAT(captured, UnorderedElementsAre(
        Pair("PointCloud", true),
        Pair("ColorCameraImage", false)));

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_NonFrameSettingsParam_DoesNotCallSetFrameOutputSettings) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);

    lcNode->set_parameter(rclcpp::Parameter("publish_combined", true));

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_MixedParams_OnlyFrameSettingsForwarded) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    lcNode->set_parameters_atomically({
        rclcpp::Parameter("publish_combined", false),
        rclcpp::Parameter("frameSettings/EventMap", false),
    });

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "EventMap");

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_SetFrameOutputSettingsThrows_CallbackReturnsFailure) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce(Throw(PhoXiInterfaceException("device busy")));

    auto result = lcNode->set_parameter(rclcpp::Parameter("frameSettings/Texture", false));
    EXPECT_FALSE(result.successful);

    ASSERT_TRUE(cleanup());
}

// ---------------------------------------------------------------------------
// YAML-style override (param set before configure)
// ---------------------------------------------------------------------------

TEST_F(FrameSettingsTest, YamlOverride_SingleComponentBeforeConfigure_AppliedOnConfigure) {
    setParamBeforeConfigure("ColorCameraImage", false);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    ASSERT_TRUE(configure());

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "ColorCameraImage");
    EXPECT_FALSE(captured[0].second);

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, YamlOverride_MultipleComponents_AllAppliedOnConfigure) {
    setParamBeforeConfigure("PointCloud", true);
    setParamBeforeConfigure("NormalMap", false);
    setParamBeforeConfigure("ConfidenceMap", false);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    ASSERT_TRUE(configure());

    EXPECT_THAT(captured, UnorderedElementsAre(
        Pair("PointCloud", true),
        Pair("NormalMap", false),
        Pair("ConfidenceMap", false)));

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, YamlOverride_UnsetComponentsNotPassed) {
    // Only DepthMap is overridden; the other six should NOT appear.
    setParamBeforeConfigure("DepthMap", true);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) {
            captured = c;
        });

    ASSERT_TRUE(configure());

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "DepthMap");

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, YamlOverride_SetBeforeConfigure_NoSdkCallDuringSet) {
    // Setting a param before configure must not trigger a setFrameOutputSettings
    // call (device not yet connected).
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);
    setParamBeforeConfigure("Texture", false);

    // Now configure: the call happens here (Times(1) on configure).
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(1);
    ASSERT_TRUE(configure());

    ASSERT_TRUE(cleanup());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
