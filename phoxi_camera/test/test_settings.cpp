#include <algorithm>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "hardware_test_fixture.h"
#include "phoxi_camera/PhoXiInterface.h"
#include "rclcpp/exceptions.hpp"

class DirectInterfaceTest : public ::testing::Test {
protected:
    void SetUp() override {
        mInterface.connectCamera(DeviceRequiredTest::deviceId(), [](const phoxi_camera::PhoXiFrame&) {});
    }

    void TearDown() override {
        mInterface.disconnectCamera();
    }

    phoxi_camera::PhoXiInterface mInterface;
};

TEST_F(DirectInterfaceTest, GetSettings_EmptyInput_ReturnsNonEmpty) {
    const auto result = mInterface.getSettings({});
    EXPECT_FALSE(result.empty()) << "getSettings({}) returned no settings";
}

TEST_F(DirectInterfaceTest, GetFrameOutputSettings_EmptyInput_ReturnsNonEmpty) {
    const auto result = mInterface.getFrameOutputSettings({});
    EXPECT_FALSE(result.empty()) << "getFrameOutputSettings({}) returned no components";
}

TEST_F(DirectInterfaceTest, GetSettings_ExplicitKeys_ReturnsOnlyRequestedKeys) {
    const auto all = mInterface.getSettings({});
    if (all.size() < 2) {
        GTEST_SKIP() << "device exposes fewer than 2 settings";
    }
    const std::string targetKey = all.begin()->first;
    const auto result = mInterface.getSettings({targetKey});
    EXPECT_EQ(result.size(), 1u) << "requesting one key should return exactly one entry";
    EXPECT_TRUE(result.count(targetKey) > 0);
}

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
