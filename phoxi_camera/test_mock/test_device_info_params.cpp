#include <string>

#include "camera_test_fixture.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "phoxi_camera/PhoXiCamera.h"
#include "phoxi_camera/PhoXiDeviceInformation.h"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/PhoXiInterface.h"

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
    info.ipv6Address = "fe80::1";
    info.hostname = "phoxi-scanner.local";
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

    EXPECT_EQ(lcNode->get_parameter("device_info.name").as_string(), "TestScanner");
    EXPECT_EQ(lcNode->get_parameter("device_info.type").as_string(), "PhoXi3DScan");
    EXPECT_EQ(lcNode->get_parameter("device_info.deviceId").as_string(), "SN-12345");
    EXPECT_EQ(lcNode->get_parameter("device_info.ipAddress").as_string(), "192.168.1.100");
    EXPECT_EQ(lcNode->get_parameter("device_info.ipv6Address").as_string(), "fe80::1");
    EXPECT_EQ(lcNode->get_parameter("device_info.hostname").as_string(), "phoxi-scanner.local");
    EXPECT_EQ(lcNode->get_parameter("device_info.status").as_string(), "Ready");
    EXPECT_EQ(lcNode->get_parameter("device_info.firmwareVersion").as_string(), "1.2.3");
    EXPECT_EQ(lcNode->get_parameter("device_info.variant").as_string(), "M");
    EXPECT_EQ(lcNode->get_parameter("device_info.isAlpha").as_bool(), false);
    EXPECT_EQ(lcNode->get_parameter("device_info.isBlue").as_bool(), true);
    EXPECT_EQ(lcNode->get_parameter("device_info.isColor").as_bool(), false);
    EXPECT_EQ(lcNode->get_parameter("device_info.isFileCam").as_bool(), false);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceInfoParamsTest, AllFields_AreReadOnly) {
    EXPECT_CALL(*mockInterface, getDeviceInfo()).WillRepeatedly(Return(makeDeviceInfo()));

    ASSERT_TRUE(configure());

    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.name", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.type", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.deviceId", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.ipAddress", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.ipv6Address", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.hostname", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.status", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.firmwareVersion", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.variant", "other")).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.isAlpha", true)).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.isBlue", false)).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.isColor", true)).successful);
    EXPECT_FALSE(lcNode->set_parameter(rclcpp::Parameter("device_info.isFileCam", true)).successful);

    ASSERT_TRUE(cleanup());
}

TEST_F(DeviceInfoParamsTest, GetDeviceInfoThrows_ConfigureFails) {
    EXPECT_CALL(*mockInterface, getDeviceInfo()).WillOnce(Throw(PhoXiInterfaceException("device error")));
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

    EXPECT_EQ(lcNode->get_parameter("device_info.name").as_string(), "TestScanner");
    EXPECT_EQ(lcNode->get_parameter("device_info.deviceId").as_string(), "SN-12345");

    ASSERT_TRUE(cleanup());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
