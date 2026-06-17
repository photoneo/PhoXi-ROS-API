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
#include "camera_test_fixture.h"

using namespace phoxi_camera;
using ::testing::_;
using ::testing::Return;
using ::testing::Throw;

class DeviceSettingsParamsTest : public CameraTestFixture {
protected:
    void SetUp() override { SetUpBase("test-device-dsp", "dsp_test_client"); }
};

TEST_F(DeviceSettingsParamsTest, EmptySchema_NoDeviceSettingsParams) {
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(std::vector<SettingInfo>{}));
    EXPECT_CALL(*mockInterface, getSettings(_)).Times(0);  // never called if schema empty
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    EXPECT_THROW(lcNode->get_parameter("deviceSettings.anything"),
        rclcpp::exceptions::ParameterNotDeclaredException);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceSettingsParamsTest, SchemaWithMultipleTypes_AllParamsDeclaredCorrectly) {
    std::vector<SettingInfo> schema = {
        {"gain", SettingValueType::DOUBLE, true},
        {"mode", SettingValueType::STRING, true},
        {"enabled", SettingValueType::BOOL, true},
    };
    SettingValueMap deviceVals = {
        {"gain", SettingValue{2.5}},
        {"mode", SettingValue{std::string{"auto"}}},
        {"enabled", SettingValue{false}},
    };
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(deviceVals));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    EXPECT_DOUBLE_EQ(lcNode->get_parameter("deviceSettings.gain").as_double(), 2.5);
    EXPECT_EQ(lcNode->get_parameter("deviceSettings.mode").as_string(), "auto");
    EXPECT_EQ(lcNode->get_parameter("deviceSettings.enabled").as_bool(), false);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceSettingsParamsTest, Configure_WithOverride_WhileConnected_CalledExactlyOnce) {
    // isConnected()=true simulates real post-connect state. Without the mDeclaringDeviceSettings
    // guard, each declare_parameter fires onParametersChanged → setSettings, so with two
    // declared params and one override the call count would be 3 instead of 1.
    // The override must be supplied via NodeOptions because deviceSettings.* are not declared
    // until configure — set_parameter would throw before that.
    std::vector<SettingInfo> schema = {
        {"gain", SettingValueType::DOUBLE, true},
        {"mode", SettingValueType::STRING, true},
    };
    SettingValueMap deviceVals = {
        {"gain", SettingValue{2.5}},
        {"mode", SettingValue{std::string{"auto"}}},
    };

    testing::Mock::VerifyAndClearExpectations(mockInterface);
    executor_.remove_node(lcNode->get_node_base_interface());
    lcNode->shutdown();
    lcNode.reset();

    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("deviceSettings.gain", 9.0)});
    auto node2 = std::make_shared<TestableNode>(mDeviceId, options, std::make_unique<MockPhoXiInterface>());
    MockPhoXiInterface* mock2 = node2->getMock();
    testing::Mock::AllowLeak(mock2);
    executor_.add_node(node2->get_node_base_interface());

    EXPECT_CALL(*mock2, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mock2, isAcquiring()).WillRepeatedly(Return(false));
    EXPECT_CALL(*mock2, getDeviceInfo()).WillRepeatedly(Return(phoxi_camera::PhoXiDeviceInformation{}));
    EXPECT_CALL(*mock2, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mock2, getSettings(_)).WillRepeatedly(Return(deviceVals));
    EXPECT_CALL(*mock2, getFrameComponentInfos()).WillRepeatedly(Return(std::vector<phoxi_camera::FrameComponentInfo>{}));
    EXPECT_CALL(*mock2, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mock2, setSettings(_)).Times(1);

    auto client = clientNode->create_client<lifecycle_msgs::srv::ChangeState>("/phoxi_camera/change_state");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    auto future = client->async_send_request(req);
    ASSERT_EQ(executor_.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);

    executor_.remove_node(node2->get_node_base_interface());
    node2->shutdown();
}

TEST_F(DeviceSettingsParamsTest, YamlOverride_AppliedToDeviceViaSetSettings) {
    const std::string key = "gain";
    std::vector<SettingInfo> schema = {{"gain", SettingValueType::DOUBLE, true}};
    SettingValueMap deviceVals = {{"gain", SettingValue{1.0}}};

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(deviceVals));

    testing::Mock::VerifyAndClearExpectations(mockInterface);
    executor_.remove_node(lcNode->get_node_base_interface());
    lcNode->shutdown();
    lcNode.reset();

    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("deviceSettings.gain", 5.0)});
    auto node2 = std::make_shared<TestableNode>(mDeviceId, options, std::make_unique<MockPhoXiInterface>());
    MockPhoXiInterface* mock2 = node2->getMock();
    testing::Mock::AllowLeak(mock2);
    executor_.add_node(node2->get_node_base_interface());
    EXPECT_CALL(*mock2, isConnected()).WillRepeatedly(Return(false));
    EXPECT_CALL(*mock2, isAcquiring()).WillRepeatedly(Return(false));
    EXPECT_CALL(*mock2, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mock2, getSettings(_)).WillRepeatedly(Return(deviceVals));

    SettingKeyValueList captured;
    EXPECT_CALL(*mock2, setSettings(_)).WillOnce([&captured](const SettingKeyValueList& kv) { captured = kv; });
    EXPECT_CALL(*mock2, connectCamera(mDeviceId, _)).Times(1);
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, "gain");
    EXPECT_DOUBLE_EQ(std::get<double>(captured[0].second), 5.0);

    testing::Mock::VerifyAndClearExpectations(mock2);
    EXPECT_CALL(*mock2, disconnectCamera()).Times(testing::AtLeast(1));
    changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);

    executor_.remove_node(node2->get_node_base_interface());
    node2->shutdown();
    node2.reset();

    rclcpp::NodeOptions opts2;
    lcNode = std::make_shared<TestableNode>(mDeviceId, opts2, std::make_unique<MockPhoXiInterface>());
    mockInterface = lcNode->getMock();
    testing::Mock::AllowLeak(mockInterface);
    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(false));
    EXPECT_CALL(*mockInterface, isAcquiring()).WillRepeatedly(Return(false));
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(std::vector<SettingInfo>{}));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{}));
    executor_.add_node(lcNode->get_node_base_interface());
}

TEST_F(DeviceSettingsParamsTest, NoYamlOverride_SetSettingsNotCalled) {
    std::vector<SettingInfo> schema = {{"gain", SettingValueType::DOUBLE, true}};
    SettingValueMap deviceVals = {{"gain", SettingValue{2.0}}};

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(deviceVals));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    EXPECT_DOUBLE_EQ(lcNode->get_parameter("deviceSettings.gain").as_double(), 2.0);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceSettingsParamsTest, GetSettingsThrows_ConfigureFails_CameraDisconnected) {
    std::vector<SettingInfo> schema = {{"gain", SettingValueType::DOUBLE, true}};

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillOnce(Throw(PhoXiInterfaceException("device error")));
    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, disconnectCamera()).Times(1);
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    EXPECT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
}

TEST_F(DeviceSettingsParamsTest, SetSettingsThrowsDuringYamlOverride_ConfigureFails_CameraDisconnected) {
    std::vector<SettingInfo> schema = {{"gain", SettingValueType::DOUBLE, true}};
    SettingValueMap deviceVals = {{"gain", SettingValue{1.0}}};

    testing::Mock::VerifyAndClearExpectations(mockInterface);
    executor_.remove_node(lcNode->get_node_base_interface());
    lcNode->shutdown();
    lcNode.reset();

    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("deviceSettings.gain", 9.0)});
    auto node2 = std::make_shared<TestableNode>(mDeviceId, options, std::make_unique<MockPhoXiInterface>());
    MockPhoXiInterface* mock2 = node2->getMock();
    testing::Mock::AllowLeak(mock2);
    executor_.add_node(node2->get_node_base_interface());
    EXPECT_CALL(*mock2, isConnected()).WillRepeatedly(Return(false));
    EXPECT_CALL(*mock2, isAcquiring()).WillRepeatedly(Return(false));
    EXPECT_CALL(*mock2, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mock2, getSettings(_)).WillRepeatedly(Return(deviceVals));
    EXPECT_CALL(*mock2, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mock2, disconnectCamera()).Times(1);
    EXPECT_CALL(*mock2, setSettings(_)).WillOnce(Throw(PhoXiInterfaceException("write error")));

    EXPECT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    testing::Mock::VerifyAndClearExpectations(mock2);
    executor_.remove_node(node2->get_node_base_interface());
    node2->shutdown();
    node2.reset();

    rclcpp::NodeOptions opts2;
    lcNode = std::make_shared<TestableNode>(mDeviceId, opts2, std::make_unique<MockPhoXiInterface>());
    mockInterface = lcNode->getMock();
    testing::Mock::AllowLeak(mockInterface);
    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(false));
    EXPECT_CALL(*mockInterface, isAcquiring()).WillRepeatedly(Return(false));
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(std::vector<SettingInfo>{}));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{}));
    executor_.add_node(lcNode->get_node_base_interface());
}

TEST_F(DeviceSettingsParamsTest, RuntimeParamChange_DeviceError_ReturnsFailure) {
    const std::string key = "gain";
    std::vector<SettingInfo> schema = {{"gain", SettingValueType::DOUBLE, true}};
    SettingValueMap deviceVals = {{"gain", SettingValue{1.0}}};

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(deviceVals));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mockInterface, setSettings(_)).WillOnce(Throw(PhoXiInterfaceException("device busy")));

    auto result = lcNode->set_parameter(rclcpp::Parameter("deviceSettings." + key, 3.0));
    EXPECT_FALSE(result.successful);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceSettingsParamsTest, UnknownDeviceSettingsParam_Rejected) {
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(std::vector<SettingInfo>{}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));

    EXPECT_THROW(lcNode->set_parameter(rclcpp::Parameter("deviceSettings.nonExistent", 42.0)),
                 rclcpp::exceptions::ParameterNotDeclaredException);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceSettingsParamsTest, NonDeviceSettingsParamChange_Ignored) {
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(std::vector<SettingInfo>{}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    auto result = lcNode->set_parameter(rclcpp::Parameter("publish_combined", true));
    EXPECT_TRUE(result.successful);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceSettingsParamsTest, KeyMissingFromGetSettingsResponse_ParamNotDeclared) {
    std::vector<SettingInfo> schema = {
        {"gain", SettingValueType::DOUBLE, true},
        {"mode", SettingValueType::STRING, true},
    };
    SettingValueMap deviceVals = {{"gain", SettingValue{3.0}}};

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(deviceVals));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    EXPECT_DOUBLE_EQ(lcNode->get_parameter("deviceSettings.gain").as_double(), 3.0);
    EXPECT_THROW(lcNode->get_parameter("deviceSettings.mode"), rclcpp::exceptions::ParameterNotDeclaredException);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceSettingsParamsTest, AfterFailedConfigure_SuccessfulReconfigurePossible) {
    std::vector<SettingInfo> schema = {{"gain", SettingValueType::DOUBLE, true}};
    SettingValueMap deviceVals = {{"gain", SettingValue{1.0}}};

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, getSettings(_)).WillOnce(Throw(PhoXiInterfaceException("transient")));
    EXPECT_CALL(*mockInterface, disconnectCamera()).Times(1);
    ASSERT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, _)).Times(1);
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(deviceVals));
    ASSERT_TRUE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

    EXPECT_DOUBLE_EQ(lcNode->get_parameter("deviceSettings.gain").as_double(), 1.0);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceSettingsParamsTest, ObjectTypeParamChange_WhenConnected_GetSettingCalled) {
    const std::string key = "res";
    pho::api::PhoXiSize sz(800, 600);
    std::vector<SettingInfo> schema = {{"res", SettingValueType::PHOXI_SIZE, true}};
    SettingValueMap deviceVals = {{"res", SettingValue{sz}}};

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(schema));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(deviceVals));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    EXPECT_CALL(*mockInterface, isConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*mockInterface, getSetting(key)).WillOnce(Return(SettingValue{sz}));

    SettingKeyValueList captured;
    EXPECT_CALL(*mockInterface, setSettings(_)).WillOnce([&captured](const SettingKeyValueList& kv) { captured = kv; });

    auto result = lcNode->set_parameter(rclcpp::Parameter("deviceSettings." + key + ".height", int64_t{1200}));
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, key);
    const auto& sent = std::get<pho::api::PhoXiSize>(captured[0].second);
    EXPECT_EQ(static_cast<int32_t>(sent.Width), 800);
    EXPECT_EQ(static_cast<int32_t>(sent.Height), 1200);

    ASSERT_TRUE(cleanup());
}

TEST(GetSettingsEmptyTest, EmptyInput_ReturnsAllSettings) {
    MockPhoXiInterface mock;
    const SettingValueMap allSettings = {
        {"CapturingSettings/LaserPower", SettingValue{int64_t{1024}}},
        {"CapturingSettings/LEDPower",   SettingValue{int64_t{512}}},
        {"ScanMultiplier",               SettingValue{int64_t{1}}},
        {"Resolution",                   SettingValue{std::string{"High"}}},
    };
    EXPECT_CALL(mock, getSettings(testing::IsEmpty())).WillOnce(testing::Return(allSettings));

    const auto result = mock.getSettings({});

    EXPECT_EQ(result.size(), allSettings.size());
    for (const auto& [key, _] : allSettings) {
        EXPECT_TRUE(result.count(key) > 0) << "Missing key: " << key;
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
