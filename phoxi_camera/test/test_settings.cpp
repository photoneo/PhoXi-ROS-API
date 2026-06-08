#include <algorithm>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "hardware_test_fixture.h"
#include "rclcpp/exceptions.hpp"

namespace {
bool isParamDeclared(rclcpp_lifecycle::LifecycleNode& node, const std::string& key) {
    try {
        node.get_parameter(key);
        return true;
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        return false;
    }
}
}  // namespace

TEST_F(DeviceRequiredTest, DeviceSettings_PopulatedAfterConfigure) {
    auto result = mLcNode->list_parameters({"deviceSettings"}, 100);
    EXPECT_FALSE(result.names.empty()) << "No deviceSettings parameters declared after configure";
}

TEST_F(DeviceRequiredTest, GetSettings_LaserPower_IsPositive) {
    const std::string key = "deviceSettings.CapturingSettings.LaserPower";
    if (!isParamDeclared(*mLcNode, key)) {
        GTEST_SKIP() << key << " not available on this device";
    }
    auto p = mLcNode->get_parameter(key);
    EXPECT_EQ(p.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
    EXPECT_GT(p.as_int(), 0);
}

TEST_F(DeviceRequiredTest, SetSettings_LaserPower_ChangePersists) {
    const std::string key = "deviceSettings.CapturingSettings.LaserPower";
    if (!isParamDeclared(*mLcNode, key)) {
        GTEST_SKIP() << key << " not available on this device";
    }
    const int64_t original = mLcNode->get_parameter(key).as_int();
    const int64_t newValue = original + 1;
    auto setResult = mLcNode->set_parameter(rclcpp::Parameter(key, newValue));
    ASSERT_TRUE(setResult.successful) << setResult.reason;
    EXPECT_EQ(mLcNode->get_parameter(key).as_int(), newValue);
    mLcNode->set_parameter(rclcpp::Parameter(key, original));
}

TEST_F(DeviceRequiredTest, GetSettings_UnknownKey_NotDeclared) {
    EXPECT_THROW(
        mLcNode->get_parameter("deviceSettings.NonExistent.FooBar"),
        rclcpp::exceptions::ParameterNotDeclaredException);
}

TEST_F(DeviceRequiredTest, SetSettings_WrongType_Rejected) {
    const std::string key = "deviceSettings.CapturingSettings.LaserPower";
    if (!isParamDeclared(*mLcNode, key)) {
        GTEST_SKIP() << key << " not available on this device";
    }
    auto setResult = mLcNode->set_parameter(rclcpp::Parameter(key, std::string("invalid")));
    EXPECT_FALSE(setResult.successful);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    if (DeviceRequiredTest::deviceId().empty()) {
        std::cerr << "[ERROR] PHO_TEST_DEVICE_ID environment variable is not set.\n";
        rclcpp::shutdown();
        return 1;
    }
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
