#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "camera_test_fixture.h"
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
using ::testing::Return;
using ::testing::Throw;

class SettingTypesTest : public CameraTestFixture {
protected:
    void SetUp() override { SetUpBase("test-device-st", "st_test_client"); }
};

static std::vector<SettingInfo> makeSchema(const std::string& key, SettingValueType type, bool isSettable) {
    return {{key, type, isSettable}};
}

struct SimpleScalarCase {
    const char* name;
    SettingValueType type;
    SettingValue initialValue;
    rclcpp::ParameterType expectedParamType;
    std::function<void(const rclcpp::Parameter&)> checkDeclared;
    rclcpp::ParameterValue newValue;
    std::function<void(const SettingValue&)> checkSet;
};

class SimpleScalarTest : public SettingTypesTest, public ::testing::WithParamInterface<SimpleScalarCase> {};

TEST_P(SimpleScalarTest, DeclaredWithCorrectValue) {
    const auto& tc = GetParam();
    const std::string key = "mySetting";
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, tc.type, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, tc.initialValue}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    auto param = lcNode->get_parameter("device_settings." + key);
    ASSERT_EQ(param.get_type(), tc.expectedParamType);
    tc.checkDeclared(param);

    ASSERT_TRUE(cleanup());
}

TEST_P(SimpleScalarTest, ParameterChange_CallsSetSettings) {
    const auto& tc = GetParam();
    const std::string key = "mySetting";
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, tc.type, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, tc.initialValue}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());
    setConnected();

    SettingKeyValueList captured;
    EXPECT_CALL(*mockInterface, setSettings(_)).WillOnce([&captured](const SettingKeyValueList& kv) { captured = kv; });

    auto result = lcNode->set_parameter(rclcpp::Parameter("device_settings." + key, tc.newValue));
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, key);
    tc.checkSet(captured[0].second);

    ASSERT_TRUE(cleanup());
}

static std::string SimpleScalarCaseName(const ::testing::TestParamInfo<SimpleScalarCase>& info) {
    return info.param.name;
}

INSTANTIATE_TEST_SUITE_P(AllScalars, SimpleScalarTest,
        ::testing::Values(SimpleScalarCase{
                                  "Bool",
                                  SettingValueType::BOOL,
                                  SettingValue{true},
                                  rclcpp::PARAMETER_BOOL,
                                  [](const rclcpp::Parameter& p) { EXPECT_EQ(p.as_bool(), true); },
                                  rclcpp::ParameterValue(false),
                                  [](const SettingValue& v) { EXPECT_EQ(std::get<bool>(v), false); },
                          },
                SimpleScalarCase{
                        "Int",
                        SettingValueType::INT,
                        SettingValue{int64_t{42}},
                        rclcpp::PARAMETER_INTEGER,
                        [](const rclcpp::Parameter& p) { EXPECT_EQ(p.as_int(), 42); },
                        rclcpp::ParameterValue(int64_t{99}),
                        [](const SettingValue& v) { EXPECT_EQ(std::get<int64_t>(v), int64_t{99}); },
                },
                SimpleScalarCase{
                        "Double",
                        SettingValueType::DOUBLE,
                        SettingValue{3.14},
                        rclcpp::PARAMETER_DOUBLE,
                        [](const rclcpp::Parameter& p) { EXPECT_DOUBLE_EQ(p.as_double(), 3.14); },
                        rclcpp::ParameterValue(2.71828),
                        [](const SettingValue& v) { EXPECT_DOUBLE_EQ(std::get<double>(v), 2.71828); },
                },
                SimpleScalarCase{
                        "String",
                        SettingValueType::STRING,
                        SettingValue{std::string{"hello"}},
                        rclcpp::PARAMETER_STRING,
                        [](const rclcpp::Parameter& p) { EXPECT_EQ(p.as_string(), "hello"); },
                        rclcpp::ParameterValue(std::string{"new"}),
                        [](const SettingValue& v) { EXPECT_EQ(std::get<std::string>(v), "new"); },
                },
                SimpleScalarCase{
                        "DoubleArray",
                        SettingValueType::DOUBLE_ARRAY,
                        SettingValue{std::vector<double>{1.0, 2.0, 3.0}},
                        rclcpp::PARAMETER_DOUBLE_ARRAY,
                        [](const rclcpp::Parameter& p) { EXPECT_EQ(p.as_double_array(), (std::vector<double>{1.0, 2.0, 3.0})); },
                        rclcpp::ParameterValue(std::vector<double>{7.0, 8.0, 9.0}),
                        [](const SettingValue& v) { EXPECT_EQ(std::get<std::vector<double>>(v), (std::vector<double>{7.0, 8.0, 9.0})); },
                }),
        SimpleScalarCaseName);

struct ReadOnlyCase {
    const char* name;
    SettingValueType type;
    SettingValue deviceValue;
    std::string paramSuffix;
    rclcpp::ParameterValue attemptedValue;
};

class ReadOnlyTest : public SettingTypesTest, public ::testing::WithParamInterface<ReadOnlyCase> {};

TEST_P(ReadOnlyTest, SetAttempt_IsRejected) {
    const auto& tc = GetParam();
    const std::string key = "mySetting";
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, tc.type, false)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, tc.deviceValue}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());
    setConnected();

    auto result = lcNode->set_parameter(rclcpp::Parameter("device_settings." + key + tc.paramSuffix, tc.attemptedValue));
    EXPECT_FALSE(result.successful);

    ASSERT_TRUE(cleanup());
}

static std::string ReadOnlyCaseName(const ::testing::TestParamInfo<ReadOnlyCase>& info) {
    return info.param.name;
}

INSTANTIATE_TEST_SUITE_P(AllReadOnly, ReadOnlyTest,
        ::testing::Values(
                ReadOnlyCase{
                        "CuttingPlanes",
                        SettingValueType::CUTTING_PLANES,
                        SettingValue{std::vector<pho::api::Plane_64f>{pho::api::Plane_64f(pho::api::Point3_64f(1.0, 0.0, 0.0), 5.0)}},
                        "",
                        rclcpp::ParameterValue(std::vector<double>{0.0, 1.0, 0.0, 3.0}),
                },
                ReadOnlyCase{
                        "ScanningVolume",
                        SettingValueType::SCANNING_VOLUME,
                        []() {
                            pho::api::ProjectionGeometry_64f g;
                            g.Origin = pho::api::Point3_64f(0.0, 0.0, 0.0);
                            return SettingValue{g};
                        }(),
                        ".origin",
                        rclcpp::ParameterValue(std::vector<double>{9.0, 9.0, 9.0}),
                },
                ReadOnlyCase{
                        "ScanningVolumeMesh",
                        SettingValueType::SCANNING_VOLUME_MESH,
                        []() {
                            pho::api::PhoXiMesh m;
                            m.PointsPerSection = 3;
                            m.Vertices = {pho::api::Point3_64f(0.0, 0.0, 0.0)};
                            m.Indices = {0u};
                            return SettingValue{m};
                        }(),
                        ".points_per_section",
                        rclcpp::ParameterValue(int64_t{6}),
                },
                ReadOnlyCase{
                        "ReprojectionMap",
                        SettingValueType::REPROJECTION_MAP,
                        []() {
                            pho::api::PhoXiReprojectionMap r;
                            r.Map.Resize(pho::api::PhoXiSize(320, 240));
                            return SettingValue{r};
                        }(),
                        ".width",
                        rclcpp::ParameterValue(int64_t{640}),
                }),
        ReadOnlyCaseName);

TEST_F(SettingTypesTest, PhoxiSize_DeclaresTwoIntSubparams) {
    const std::string key = "mySize";
    pho::api::PhoXiSize sz(1920, 1080);
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::PHOXI_SIZE, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{sz}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    auto wParam = lcNode->get_parameter("device_settings." + key + ".width");
    auto hParam = lcNode->get_parameter("device_settings." + key + ".height");
    ASSERT_EQ(wParam.get_type(), rclcpp::PARAMETER_INTEGER);
    ASSERT_EQ(hParam.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(wParam.as_int(), 1920);
    EXPECT_EQ(hParam.as_int(), 1080);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, PhoxiSize_FieldChange_MergesAndCallsSetSettings) {
    const std::string key = "mySize";
    pho::api::PhoXiSize sz(640, 480);
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::PHOXI_SIZE, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{sz}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());
    setConnected();

    EXPECT_CALL(*mockInterface, getSetting(key)).WillOnce(Return(SettingValue{sz}));

    SettingKeyValueList captured;
    EXPECT_CALL(*mockInterface, setSettings(_)).WillOnce([&captured](const SettingKeyValueList& kv) { captured = kv; });

    auto result = lcNode->set_parameter(rclcpp::Parameter("device_settings." + key + ".width", int64_t{1280}));
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].first, key);
    const auto& sent = std::get<pho::api::PhoXiSize>(captured[0].second);
    EXPECT_EQ(static_cast<int32_t>(sent.Width), 1280);
    EXPECT_EQ(static_cast<int32_t>(sent.Height), 480);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, PhoxiSize64f_DeclaresTwoDoubleSubparams) {
    const std::string key = "mySize64f";
    pho::api::PhoXiSize_64f sz(2.5, 1.5);
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::PHOXI_SIZE_64F, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{sz}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    auto wParam = lcNode->get_parameter("device_settings." + key + ".width");
    auto hParam = lcNode->get_parameter("device_settings." + key + ".height");
    ASSERT_EQ(wParam.get_type(), rclcpp::PARAMETER_DOUBLE);
    ASSERT_EQ(hParam.get_type(), rclcpp::PARAMETER_DOUBLE);
    EXPECT_DOUBLE_EQ(wParam.as_double(), 2.5);
    EXPECT_DOUBLE_EQ(hParam.as_double(), 1.5);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, PhoxiSize64f_FieldChange_MergesAndCallsSetSettings) {
    const std::string key = "mySize64f";
    pho::api::PhoXiSize_64f sz(2.0, 1.0);
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::PHOXI_SIZE_64F, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{sz}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());
    setConnected();

    EXPECT_CALL(*mockInterface, getSetting(key)).WillOnce(Return(SettingValue{sz}));

    SettingKeyValueList captured;
    EXPECT_CALL(*mockInterface, setSettings(_)).WillOnce([&captured](const SettingKeyValueList& kv) { captured = kv; });

    auto result = lcNode->set_parameter(rclcpp::Parameter("device_settings." + key + ".height", 4.0));
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    const auto& sent = std::get<pho::api::PhoXiSize_64f>(captured[0].second);
    EXPECT_DOUBLE_EQ(static_cast<double>(sent.Width), 2.0);
    EXPECT_DOUBLE_EQ(static_cast<double>(sent.Height), 4.0);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, Phoxi2DROI_DeclaresCorrectSubparams) {
    const std::string key = "myROI";
    pho::api::PhoXi2DROI roi(10, 20, 100, 200);
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::PHOXI_2DROI, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{roi}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    const std::string base = "device_settings." + key;
    EXPECT_EQ(lcNode->get_parameter(base + ".x_min").as_int(), 10);
    EXPECT_EQ(lcNode->get_parameter(base + ".y_min").as_int(), 20);
    EXPECT_EQ(lcNode->get_parameter(base + ".x_max").as_int(), 100);
    EXPECT_EQ(lcNode->get_parameter(base + ".y_max").as_int(), 200);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, Phoxi2DROI_FieldChange_MergesAndCallsSetSettings) {
    const std::string key = "myROI";
    pho::api::PhoXi2DROI roi(10, 20, 100, 200);
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::PHOXI_2DROI, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{roi}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());
    setConnected();

    EXPECT_CALL(*mockInterface, getSetting(key)).WillOnce(Return(SettingValue{roi}));

    SettingKeyValueList captured;
    EXPECT_CALL(*mockInterface, setSettings(_)).WillOnce([&captured](const SettingKeyValueList& kv) { captured = kv; });

    auto result = lcNode->set_parameter(rclcpp::Parameter("device_settings." + key + ".x_max", int64_t{300}));
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    const auto& sent = std::get<pho::api::PhoXi2DROI>(captured[0].second);
    EXPECT_EQ(static_cast<int32_t>(sent.Min.x), 10);
    EXPECT_EQ(static_cast<int32_t>(sent.Min.y), 20);
    EXPECT_EQ(static_cast<int32_t>(sent.Max.x), 300);
    EXPECT_EQ(static_cast<int32_t>(sent.Max.y), 200);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, AxisVolume64f_DeclaresCorrectSubparams) {
    const std::string key = "myVol";
    pho::api::AxisVolume_64f vol(pho::api::Point3_64f(-1.0, -2.0, -3.0), pho::api::Point3_64f(1.0, 2.0, 3.0));
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::AXIS_VOLUME_64F, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{vol}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    const std::string base = "device_settings." + key;
    EXPECT_DOUBLE_EQ(lcNode->get_parameter(base + ".x_min").as_double(), -1.0);
    EXPECT_DOUBLE_EQ(lcNode->get_parameter(base + ".y_min").as_double(), -2.0);
    EXPECT_DOUBLE_EQ(lcNode->get_parameter(base + ".z_min").as_double(), -3.0);
    EXPECT_DOUBLE_EQ(lcNode->get_parameter(base + ".x_max").as_double(), 1.0);
    EXPECT_DOUBLE_EQ(lcNode->get_parameter(base + ".y_max").as_double(), 2.0);
    EXPECT_DOUBLE_EQ(lcNode->get_parameter(base + ".z_max").as_double(), 3.0);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, AxisVolume64f_FieldChange_MergesAndCallsSetSettings) {
    const std::string key = "myVol";
    pho::api::AxisVolume_64f vol(pho::api::Point3_64f(-1.0, -2.0, -3.0), pho::api::Point3_64f(1.0, 2.0, 3.0));
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::AXIS_VOLUME_64F, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{vol}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());
    setConnected();

    EXPECT_CALL(*mockInterface, getSetting(key)).WillOnce(Return(SettingValue{vol}));

    SettingKeyValueList captured;
    EXPECT_CALL(*mockInterface, setSettings(_)).WillOnce([&captured](const SettingKeyValueList& kv) { captured = kv; });

    auto result = lcNode->set_parameter(rclcpp::Parameter("device_settings." + key + ".z_max", 99.0));
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    const auto& sent = std::get<pho::api::AxisVolume_64f>(captured[0].second);
    EXPECT_DOUBLE_EQ(static_cast<double>(sent.min.x), -1.0);
    EXPECT_DOUBLE_EQ(static_cast<double>(sent.max.z), 99.0);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, Point3_64f_DeclaresRGBSubparams) {
    const std::string key = "myPoint";
    pho::api::Point3_64f pt(0.1, 0.2, 0.3);
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::POINT3_64F, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{pt}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    const std::string base = "device_settings." + key;
    EXPECT_DOUBLE_EQ(lcNode->get_parameter(base + ".r").as_double(), 0.1);
    EXPECT_DOUBLE_EQ(lcNode->get_parameter(base + ".g").as_double(), 0.2);
    EXPECT_DOUBLE_EQ(lcNode->get_parameter(base + ".b").as_double(), 0.3);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, Point3_64f_FieldChange_MergesAndCallsSetSettings) {
    const std::string key = "myPoint";
    pho::api::Point3_64f pt(0.1, 0.2, 0.3);
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::POINT3_64F, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{pt}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());
    setConnected();

    EXPECT_CALL(*mockInterface, getSetting(key)).WillOnce(Return(SettingValue{pt}));

    SettingKeyValueList captured;
    EXPECT_CALL(*mockInterface, setSettings(_)).WillOnce([&captured](const SettingKeyValueList& kv) { captured = kv; });

    auto result = lcNode->set_parameter(rclcpp::Parameter("device_settings." + key + ".g", 9.9));
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    const auto& sent = std::get<pho::api::Point3_64f>(captured[0].second);
    EXPECT_DOUBLE_EQ(static_cast<double>(sent.x), 0.1);
    EXPECT_DOUBLE_EQ(static_cast<double>(sent.y), 9.9);
    EXPECT_DOUBLE_EQ(static_cast<double>(sent.z), 0.3);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, CuttingPlanes_DeclaredAsDoubleArray) {
    const std::string key = "myPlanes";
    pho::api::Point3_64f normal(1.0, 0.0, 0.0);
    std::vector<pho::api::Plane_64f> planes = {pho::api::Plane_64f(normal, 5.0)};
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::CUTTING_PLANES, false)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{planes}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    auto param = lcNode->get_parameter("device_settings." + key);
    ASSERT_EQ(param.get_type(), rclcpp::PARAMETER_DOUBLE_ARRAY);
    auto arr = param.as_double_array();
    ASSERT_EQ(arr.size(), 4u);
    EXPECT_DOUBLE_EQ(arr[0], 1.0);
    EXPECT_DOUBLE_EQ(arr[1], 0.0);
    EXPECT_DOUBLE_EQ(arr[2], 0.0);
    EXPECT_DOUBLE_EQ(arr[3], 5.0);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, ScanningVolume_DeclaresSevenSubparams) {
    const std::string key = "myScanVol";
    pho::api::ProjectionGeometry_64f geom;
    geom.Origin = pho::api::Point3_64f(1.0, 2.0, 3.0);
    geom.TopLeftTangentialVector = pho::api::Point3_64f(0.1, 0.2, 0.3);
    geom.TopRightTangentialVector = pho::api::Point3_64f(0.4, 0.5, 0.6);
    geom.BottomLeftTangentialVector = pho::api::Point3_64f(0.7, 0.8, 0.9);
    geom.BottomRightTangentialVector = pho::api::Point3_64f(1.1, 1.2, 1.3);
    geom.TopContourPoints = {pho::api::Point3_64f(0.0, 0.0, 1.0)};
    geom.BottomContourPoints = {pho::api::Point3_64f(0.0, 0.0, -1.0)};

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::SCANNING_VOLUME, false)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{geom}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    const std::string base = "device_settings." + key;
    auto origin = lcNode->get_parameter(base + ".origin").as_double_array();
    ASSERT_EQ(origin.size(), 3u);
    EXPECT_DOUBLE_EQ(origin[0], 1.0);
    EXPECT_DOUBLE_EQ(origin[1], 2.0);
    EXPECT_DOUBLE_EQ(origin[2], 3.0);

    auto topContour = lcNode->get_parameter(base + ".top_contour").as_double_array();
    ASSERT_EQ(topContour.size(), 3u);

    auto bottomContour = lcNode->get_parameter(base + ".bottom_contour").as_double_array();
    ASSERT_EQ(bottomContour.size(), 3u);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, ScanningVolumeMesh_DeclaresThreeSubparams) {
    const std::string key = "myMesh";
    pho::api::PhoXiMesh mesh;
    mesh.PointsPerSection = 3;
    mesh.Vertices = {pho::api::Point3_64f(0.0, 0.0, 0.0), pho::api::Point3_64f(1.0, 0.0, 0.0), pho::api::Point3_64f(0.0, 1.0, 0.0)};
    mesh.Indices = {0u, 1u, 2u};

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::SCANNING_VOLUME_MESH, false)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{mesh}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    const std::string base = "device_settings." + key;
    auto pps = lcNode->get_parameter(base + ".points_per_section");
    ASSERT_EQ(pps.get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(pps.as_int(), 3);

    auto verts = lcNode->get_parameter(base + ".vertices").as_double_array();
    ASSERT_EQ(verts.size(), 9u);

    auto idxs = lcNode->get_parameter(base + ".indices").as_integer_array();
    ASSERT_EQ(idxs.size(), 3u);
    EXPECT_EQ(idxs[0], 0);
    EXPECT_EQ(idxs[1], 1);
    EXPECT_EQ(idxs[2], 2);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, ReprojectionMap_DeclaresWidthHeightCvType) {
    const std::string key = "myReproj";
    pho::api::PhoXiReprojectionMap reproj;
    reproj.Map.Resize(pho::api::PhoXiSize(320, 240));

    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::REPROJECTION_MAP, false)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{reproj}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());

    const std::string base = "device_settings." + key;
    ASSERT_EQ(lcNode->get_parameter(base + ".width").get_type(), rclcpp::PARAMETER_INTEGER);
    ASSERT_EQ(lcNode->get_parameter(base + ".height").get_type(), rclcpp::PARAMETER_INTEGER);
    ASSERT_EQ(lcNode->get_parameter(base + ".cv_type").get_type(), rclcpp::PARAMETER_INTEGER);
    EXPECT_EQ(lcNode->get_parameter(base + ".width").as_int(), 320);
    EXPECT_EQ(lcNode->get_parameter(base + ".height").as_int(), 240);

    ASSERT_TRUE(cleanup());
}

TEST_F(SettingTypesTest, Phoxi2DROI_BothMinMaxChanged_SingleSetSettingsCall) {
    const std::string key = "myROI";
    pho::api::PhoXi2DROI roi(10, 20, 100, 200);
    EXPECT_CALL(*mockInterface, getSettingInfos()).WillRepeatedly(Return(makeSchema(key, SettingValueType::PHOXI_2DROI, true)));
    EXPECT_CALL(*mockInterface, getSettings(_)).WillRepeatedly(Return(SettingValueMap{{key, SettingValue{roi}}}));
    EXPECT_CALL(*mockInterface, setSettings(_)).Times(0);

    ASSERT_TRUE(configure());
    setConnected();

    EXPECT_CALL(*mockInterface, getSetting(key)).WillOnce(Return(SettingValue{roi}));

    SettingKeyValueList captured;
    EXPECT_CALL(*mockInterface, setSettings(_)).WillOnce([&captured](const SettingKeyValueList& kv) { captured = kv; });

    const std::string base = "device_settings." + key;
    auto result = lcNode->set_parameters_atomically({
            rclcpp::Parameter(base + ".x_min", int64_t{5}),
            rclcpp::Parameter(base + ".y_min", int64_t{15}),
    });
    EXPECT_TRUE(result.successful);

    ASSERT_EQ(captured.size(), 1u);
    const auto& sent = std::get<pho::api::PhoXi2DROI>(captured[0].second);
    EXPECT_EQ(static_cast<int32_t>(sent.Min.x), 5);
    EXPECT_EQ(static_cast<int32_t>(sent.Min.y), 15);
    EXPECT_EQ(static_cast<int32_t>(sent.Max.x), 100);
    EXPECT_EQ(static_cast<int32_t>(sent.Max.y), 200);

    ASSERT_TRUE(cleanup());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
