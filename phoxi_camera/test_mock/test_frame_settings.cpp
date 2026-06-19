#include <map>
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
#include "camera_test_fixture.h"

using namespace phoxi_camera;
using ::testing::_;
using ::testing::Pair;
using ::testing::Return;
using ::testing::Throw;
using ::testing::UnorderedElementsAre;

static const std::map<std::string, bool> DEVICE_DEFAULTS = {
    {"PointCloud", true}, {"NormalMap", false}, {"DepthMap", true},
    {"Texture", false}, {"ConfidenceMap", false}, {"ColorCameraImage", false}, {"EventMap", false},
};

class FrameSettingsTest : public CameraTestFixture {
protected:
    // SetUp() intentionally left empty — each test calls InitNode() explicitly
    // so it can supply parameter overrides before node construction.
    void SetUp() override {}

    void InitNode(std::vector<rclcpp::Parameter> paramOverrides = {}) {
        SetUpBase("test-device-fs", "fs_test_client", std::move(paramOverrides));
        EXPECT_CALL(*mockInterface, getFrameOutputSettings(_))
            .WillRepeatedly(Return(DEVICE_DEFAULTS));
    }
};

TEST_F(FrameSettingsTest, Configure_NoOverrides_DeviceNotCalled) {
    InitNode();
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);
    ASSERT_TRUE(configure());
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_WithOverride_WhileConnected_CalledExactlyOnce) {
    // isConnected()=true simulates real post-connect state. Without the mDeclaringDeviceSettings
    // guard, each declare_parameter fires onParametersChanged → setFrameOutputSettings, so with
    // 7 components and one override the call count would be 8 instead of 1.
    // NormalMap device default = false; pre-setting to true is an override.
    InitNode({rclcpp::Parameter("frameSettings.NormalMap", true)});
    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(1);
    ASSERT_TRUE(configure());
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_OverrideSameAsDevice_NotForwarded) {
    InitNode({rclcpp::Parameter("frameSettings.PointCloud", true)});
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);
    ASSERT_TRUE(configure());
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_SingleOverrideEnabled_ForwardedToDevice) {
    InitNode({rclcpp::Parameter("frameSettings.NormalMap", true)});
    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });
    ASSERT_TRUE(configure());
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "NormalMap");
    EXPECT_TRUE(captured[0].second);
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_SingleOverrideDisabled_ForwardedToDevice) {
    InitNode({rclcpp::Parameter("frameSettings.DepthMap", false)});
    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });
    ASSERT_TRUE(configure());
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "DepthMap");
    EXPECT_FALSE(captured[0].second);
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_MultipleOverrides_AllForwarded) {
    InitNode({
        rclcpp::Parameter("frameSettings.NormalMap", true),
        rclcpp::Parameter("frameSettings.DepthMap", false),
        rclcpp::Parameter("frameSettings.Texture", true),
    });
    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });
    ASSERT_TRUE(configure());
    EXPECT_THAT(captured, UnorderedElementsAre(
        Pair("NormalMap", true), Pair("DepthMap", false), Pair("Texture", true)));
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_SetFrameOutputSettingsThrows_ReturnsFailure) {
    InitNode({rclcpp::Parameter("frameSettings.NormalMap", true)});
    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce(Throw(PhoXiInterfaceException("device rejected component")));
    EXPECT_CALL(*mockInterface, disconnectCamera(testing::_, testing::_)).Times(1);
    EXPECT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
}

TEST_F(FrameSettingsTest, Configure_SetFrameOutputSettingsThrows_CanReconfigureSuccessfully) {
    InitNode({rclcpp::Parameter("frameSettings.NormalMap", true)});
    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce(Throw(PhoXiInterfaceException("transient error")));
    EXPECT_CALL(*mockInterface, disconnectCamera(testing::_, testing::_)).Times(1);
    ASSERT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_WhenConnected_ForwardedToDevice) {
    InitNode();
    ASSERT_TRUE(configure());
    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    auto result = lcNode->set_parameter(rclcpp::Parameter("frameSettings.DepthMap", false));
    EXPECT_TRUE(result.successful);
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "DepthMap");
    EXPECT_FALSE(captured[0].second);
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_MultipleComponents_AllPassedInSingleCall) {
    InitNode();
    ASSERT_TRUE(configure());
    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    lcNode->set_parameters_atomically({
        rclcpp::Parameter("frameSettings.PointCloud", true),
        rclcpp::Parameter("frameSettings.ColorCameraImage", false),
    });

    EXPECT_THAT(captured, UnorderedElementsAre(Pair("PointCloud", true), Pair("ColorCameraImage", false)));
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_NonFrameSettingsParam_NotForwarded) {
    InitNode();
    ASSERT_TRUE(configure());
    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);
    lcNode->set_parameter(rclcpp::Parameter("publish_combined", true));
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_MixedParams_OnlyFrameSettingsForwarded) {
    InitNode();
    ASSERT_TRUE(configure());
    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    lcNode->set_parameters_atomically({
        rclcpp::Parameter("publish_combined", false),
        rclcpp::Parameter("frameSettings.EventMap", true),
    });

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "EventMap");
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_SetFrameOutputSettingsThrows_CallbackReturnsFailure) {
    InitNode();
    ASSERT_TRUE(configure());
    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_))
        .WillOnce(Throw(PhoXiInterfaceException("device busy")));

    auto result = lcNode->set_parameter(rclcpp::Parameter("frameSettings.Texture", true));
    EXPECT_FALSE(result.successful);
    ASSERT_TRUE(cleanup());
}

TEST(GetFrameOutputSettingsEmptyTest, EmptyInput_ReturnsAllComponents) {
    MockPhoXiInterface mock;
    const std::map<std::string, bool> allComponents = {
        {"PointCloud",       true},
        {"NormalMap",        false},
        {"DepthMap",         true},
        {"Texture",          false},
        {"ConfidenceMap",    false},
        {"ColorCameraImage", false},
        {"EventMap",         false},
    };
    EXPECT_CALL(mock, getFrameOutputSettings(testing::IsEmpty())).WillOnce(testing::Return(allComponents));

    const auto result = mock.getFrameOutputSettings({});

    EXPECT_EQ(result.size(), allComponents.size());
    for (const auto& [name, expected] : allComponents) {
        ASSERT_TRUE(result.count(name) > 0) << "Missing component: " << name;
        EXPECT_EQ(result.at(name), expected);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
