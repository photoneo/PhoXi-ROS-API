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

class FrameSettingsTest : public CameraTestFixture {
protected:
    void SetUp() override { SetUpBase("test-device-fs", "fs_test_client"); }

    void setParamBeforeConfigure(const std::string& component, bool enabled) {
        lcNode->set_parameter(rclcpp::Parameter("frameSettings." + component, enabled));
    }
};


TEST_F(FrameSettingsTest, Configure_NoParamsSet_DoesNotCallSetFrameOutputSettings) {
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);
    ASSERT_TRUE(configure());
    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_SingleComponentEnabled_CallsWithCorrectComponent) {
    setParamBeforeConfigure("PointCloud", true);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    ASSERT_TRUE(configure());

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "PointCloud");
    EXPECT_TRUE(captured[0].second);

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_ComponentDisabled_PassedAsFalse) {
    setParamBeforeConfigure("NormalMap", false);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

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
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    ASSERT_TRUE(configure());

    EXPECT_THAT(captured, UnorderedElementsAre(Pair("DepthMap", true), Pair("ConfidenceMap", false), Pair("EventMap", false)));

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, Configure_AllComponents_AllSevenPassed) {
    for (const auto* name : {"PointCloud", "NormalMap", "DepthMap", "Texture", "ConfidenceMap", "ColorCameraImage", "EventMap"}) {
        setParamBeforeConfigure(name, true);
    }

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    ASSERT_TRUE(configure());

    EXPECT_EQ(captured.size(), 7u);

    ASSERT_TRUE(cleanup());
}


TEST_F(FrameSettingsTest, Configure_SetFrameOutputSettingsThrows_ReturnsFailure) {
    setParamBeforeConfigure("Texture", true);

    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce(Throw(PhoXiInterfaceException("SDK rejected component")));
    EXPECT_CALL(*mockInterface, disconnectCamera()).Times(1);

    EXPECT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
}

TEST_F(FrameSettingsTest, Configure_SetFrameOutputSettingsThrows_CanReconfigureSuccessfully) {
    setParamBeforeConfigure("Texture", true);

    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce(Throw(PhoXiInterfaceException("transient error")));
    EXPECT_CALL(*mockInterface, disconnectCamera()).Times(1);
    ASSERT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    ASSERT_TRUE(cleanup());
}


TEST_F(FrameSettingsTest, ParameterChange_WhenConnected_CallsSetFrameOutputSettings) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    auto result = lcNode->set_parameter(rclcpp::Parameter("frameSettings.DepthMap", false));
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "DepthMap");
    EXPECT_FALSE(captured[0].second);

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_WhenNotConnected_DoesNotCallSetFrameOutputSettings) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);

    auto result = lcNode->set_parameter(rclcpp::Parameter("frameSettings.NormalMap", false));
    EXPECT_TRUE(result.successful);

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_MultipleComponents_AllPassedInSingleCall) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    lcNode->set_parameters_atomically({
            rclcpp::Parameter("frameSettings.PointCloud", true),
            rclcpp::Parameter("frameSettings.ColorCameraImage", false),
    });

    EXPECT_THAT(captured, UnorderedElementsAre(Pair("PointCloud", true), Pair("ColorCameraImage", false)));

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
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    lcNode->set_parameters_atomically({
            rclcpp::Parameter("publish_combined", false),
            rclcpp::Parameter("frameSettings.EventMap", false),
    });

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "EventMap");

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, ParameterChange_SetFrameOutputSettingsThrows_CallbackReturnsFailure) {
    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce(Throw(PhoXiInterfaceException("device busy")));

    auto result = lcNode->set_parameter(rclcpp::Parameter("frameSettings.Texture", false));
    EXPECT_FALSE(result.successful);

    ASSERT_TRUE(cleanup());
}


TEST_F(FrameSettingsTest, YamlOverride_SingleComponentBeforeConfigure_AppliedOnConfigure) {
    setParamBeforeConfigure("ColorCameraImage", false);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

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
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    ASSERT_TRUE(configure());

    EXPECT_THAT(captured, UnorderedElementsAre(Pair("PointCloud", true), Pair("NormalMap", false), Pair("ConfidenceMap", false)));

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, YamlOverride_UnsetComponentsNotPassed) {
    setParamBeforeConfigure("DepthMap", true);

    std::vector<std::pair<std::string, bool>> captured;
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).WillOnce([&captured](const std::vector<std::pair<std::string, bool>>& c) { captured = c; });

    ASSERT_TRUE(configure());

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "DepthMap");

    ASSERT_TRUE(cleanup());
}

TEST_F(FrameSettingsTest, YamlOverride_SetBeforeConfigure_NoSdkCallDuringSet) {
    EXPECT_CALL(*mockInterface, setFrameOutputSettings(_)).Times(0);
    setParamBeforeConfigure("Texture", false);

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
