#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "phoxi_camera/PhoXiCamera.h"
#include "phoxi_camera/PhoXiDeviceInformation.h"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/PhoXiInterface.h"
#include "camera_test_fixture.h"

using namespace phoxi_camera;
using ::testing::Return;
using ::testing::Throw;

namespace {
PhoXiDeviceInformation makeDeviceInfo() {
    PhoXiDeviceInformation info;
    info.name = "TestScanner";
    info.type = "PhoXi3DScan";
    info.hwIdentification = "SN-12345";
    info.ipAddress = "192.168.1.100";
    info.status = PhoXiDeviceInformation::Ready;
    info.firmwareVersion = "1.2.3";
    info.variant = "M";
    info.isAlpha = false;
    info.isBlue = true;
    info.isColor = false;
    info.isFileCam = false;
    return info;
}
}  // namespace

class DeviceInfoParamsTest : public CameraTestFixture {
protected:
    void SetUp() override { SetUpBase("test-device-dip", "dip_test_client"); }
};

TEST_F(DeviceInfoParamsTest, AllFields_DeclaredWithCorrectValues) {
    EXPECT_CALL(*mockInterface, getDeviceInfo()).WillRepeatedly(Return(makeDeviceInfo()));

    ASSERT_TRUE(configure());

    EXPECT_EQ(lcNode->get_parameter("deviceInfo.name").as_string(), "TestScanner");
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.type").as_string(), "PhoXi3DScan");
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.deviceId").as_string(), "SN-12345");
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.ipAddress").as_string(), "192.168.1.100");
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.status").as_string(), "Ready");
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.firmwareVersion").as_string(), "1.2.3");
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.variant").as_string(), "M");
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.isAlpha").as_bool(), false);
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.isBlue").as_bool(), true);
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.isColor").as_bool(), false);
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.isFileCam").as_bool(), false);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceInfoParamsTest, AllFields_AreReadOnly) {
    EXPECT_CALL(*mockInterface, getDeviceInfo()).WillRepeatedly(Return(makeDeviceInfo()));

    ASSERT_TRUE(configure());

    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.name", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.type", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.deviceId", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.ipAddress", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.status", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.firmwareVersion", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.variant", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.isAlpha", true)).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.isBlue", false)).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.isColor", true)).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("deviceInfo.isFileCam", true)).successful);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceInfoParamsTest, GetDeviceInfoThrows_ConfigureFails) {
    EXPECT_CALL(*mockInterface, getDeviceInfo())
        .WillOnce(Throw(PhoXiInterfaceException("device error")));
    EXPECT_CALL(*mockInterface, connectCamera(mDeviceId, testing::_)).Times(1);
    EXPECT_CALL(*mockInterface, disconnectCamera(testing::_, testing::_)).Times(1);

    EXPECT_FALSE(changeLcState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
}

TEST_F(DeviceInfoParamsTest, AfterCleanupAndReconfigure_ParamsDeclaredCorrectly) {
    EXPECT_CALL(*mockInterface, getDeviceInfo()).WillRepeatedly(Return(makeDeviceInfo()));

    ASSERT_TRUE(configure());
    ASSERT_TRUE(cleanup());

    EXPECT_CALL(*mockInterface, getDeviceInfo()).WillRepeatedly(Return(makeDeviceInfo()));
    ASSERT_TRUE(configure());

    EXPECT_EQ(lcNode->get_parameter("deviceInfo.name").as_string(), "TestScanner");
    EXPECT_EQ(lcNode->get_parameter("deviceInfo.deviceId").as_string(), "SN-12345");

    ASSERT_TRUE(cleanup());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
