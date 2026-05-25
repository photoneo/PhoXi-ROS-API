#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "phoxi_camera/RosConversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

bool fieldExists(const sensor_msgs::msg::PointCloud2& msg, const std::string& field_name) {
    for (const auto& field : msg.fields) {
        if (field.name == field_name) {
            return true;
        }
    }
    return false;
}

uint32_t getFieldOffset(const sensor_msgs::msg::PointCloud2& msg, const std::string& field_name) {
    for (const auto& field : msg.fields) {
        if (field.name == field_name) {
            return field.offset;
        }
    }
    throw std::runtime_error("Field '" + field_name + "' not found in PointCloud2 message");
}

class ConversionTest : public ::testing::Test
{
};

TEST_F(ConversionTest, FullFrameConversion) {
    pho::api::PFrame pframe = std::make_shared<pho::api::Frame>();
    const int width = 1;
    const int height = 2;

    pframe->PointCloud.Resize(pho::api::PhoXiSize(width, height));
    pframe->NormalMap.Resize(pho::api::PhoXiSize(width, height));
    pframe->TextureRGB.Resize(pho::api::PhoXiSize(width, height));
    pframe->ConfidenceMap.Resize(pho::api::PhoXiSize(width, height));
    pframe->DepthMap.Resize(pho::api::PhoXiSize(width, height));

    pframe->PointCloud[0][0] = {1.0f, 2.0f, 3.0f};
    pframe->PointCloud[0][1] = {4.0f, 5.0f, 6.0f};

    pframe->NormalMap[0][0] = {0.1f, 0.2f, 0.3f};
    pframe->NormalMap[0][1] = {0.4f, 0.5f, 0.6f};

    pframe->TextureRGB[0][0] = {1023, 511, 0};
    pframe->TextureRGB[0][1] = {255, 767, 1000};

    pframe->ConfidenceMap[0][0] = 0.9f;
    pframe->ConfidenceMap[0][1] = 0.8f;

    pframe->DepthMap[0][0] = 1.1f;
    pframe->DepthMap[0][1] = 2.2f;

    std::unique_ptr<sensor_msgs::msg::PointCloud2> msg = phoxi_camera::phoXiFrameToRosMsg(pframe);

    ASSERT_NE(msg, nullptr);
    ASSERT_EQ(msg->height, height);
    ASSERT_EQ(msg->width, width);

    ASSERT_TRUE(fieldExists(*msg, "x"));
    ASSERT_TRUE(fieldExists(*msg, "y"));
    ASSERT_TRUE(fieldExists(*msg, "z"));
    ASSERT_TRUE(fieldExists(*msg, "normal_x"));
    ASSERT_TRUE(fieldExists(*msg, "rgb"));
    ASSERT_TRUE(fieldExists(*msg, "confidence"));
    ASSERT_TRUE(fieldExists(*msg, "depth"));
    ASSERT_FALSE(fieldExists(*msg, "intensity"));

    const uint8_t* data_ptr = msg->data.data();
    const uint32_t point_step = msg->point_step;

    uint32_t x_off = getFieldOffset(*msg, "x");
    uint32_t y_off = getFieldOffset(*msg, "y");
    uint32_t z_off = getFieldOffset(*msg, "z");
    uint32_t nx_off = getFieldOffset(*msg, "normal_x");
    uint32_t ny_off = getFieldOffset(*msg, "normal_y");
    uint32_t nz_off = getFieldOffset(*msg, "normal_z");
    uint32_t rgb_off = getFieldOffset(*msg, "rgb");
    uint32_t conf_off = getFieldOffset(*msg, "confidence");
    uint32_t depth_off = getFieldOffset(*msg, "depth");

    float x0, y0, z0, nx0, ny0, nz0, conf0, depth0;
    uint32_t rgb0_packed;

    memcpy(&x0, data_ptr + (0 * point_step) + x_off, sizeof(float));
    memcpy(&y0, data_ptr + (0 * point_step) + y_off, sizeof(float));
    memcpy(&z0, data_ptr + (0 * point_step) + z_off, sizeof(float));
    memcpy(&nx0, data_ptr + (0 * point_step) + nx_off, sizeof(float));
    memcpy(&ny0, data_ptr + (0 * point_step) + ny_off, sizeof(float));
    memcpy(&nz0, data_ptr + (0 * point_step) + nz_off, sizeof(float));
    memcpy(&rgb0_packed, data_ptr + (0 * point_step) + rgb_off, sizeof(uint32_t));
    memcpy(&conf0, data_ptr + (0 * point_step) + conf_off, sizeof(float));
    memcpy(&depth0, data_ptr + (0 * point_step) + depth_off, sizeof(float));

    uint8_t r0 = (rgb0_packed >> 16) & 0xFF;
    uint8_t g0 = (rgb0_packed >> 8) & 0xFF;
    uint8_t b0 = rgb0_packed & 0xFF;

    uint8_t r0_expected = static_cast<uint8_t>((1023.0f / 1023.0f) * 255.0f);
    uint8_t g0_expected = static_cast<uint8_t>((511.0f / 1023.0f) * 255.0f);
    uint8_t b0_expected = static_cast<uint8_t>((0.0f / 1023.0f) * 255.0f);

    EXPECT_FLOAT_EQ(x0, 0.001f);
    EXPECT_FLOAT_EQ(y0, 0.002f);
    EXPECT_FLOAT_EQ(z0, 0.003f);
    EXPECT_FLOAT_EQ(nx0, 0.1f);
    EXPECT_FLOAT_EQ(ny0, 0.2f);
    EXPECT_FLOAT_EQ(nz0, 0.3f);
    EXPECT_EQ(r0, r0_expected);
    EXPECT_EQ(g0, g0_expected);
    EXPECT_EQ(b0, b0_expected);
    EXPECT_FLOAT_EQ(conf0, 0.9f);
    EXPECT_FLOAT_EQ(depth0, 1.1f);

    float x1, y1, z1, nx1, ny1, nz1;
    uint32_t rgb1_packed;

    memcpy(&x1, data_ptr + (1 * point_step) + x_off, sizeof(float));
    memcpy(&y1, data_ptr + (1 * point_step) + y_off, sizeof(float));
    memcpy(&z1, data_ptr + (1 * point_step) + z_off, sizeof(float));
    memcpy(&nx1, data_ptr + (1 * point_step) + nx_off, sizeof(float));
    memcpy(&ny1, data_ptr + (1 * point_step) + ny_off, sizeof(float));
    memcpy(&nz1, data_ptr + (1 * point_step) + nz_off, sizeof(float));
    memcpy(&rgb1_packed, data_ptr + (1 * point_step) + rgb_off, sizeof(uint32_t));

    uint8_t r1 = (rgb1_packed >> 16) & 0xFF;
    uint8_t g1 = (rgb1_packed >> 8) & 0xFF;
    uint8_t b1 = rgb1_packed & 0xFF;

    uint8_t r1_expected = static_cast<uint8_t>((255.0f / 1023.0f) * 255.0f);
    uint8_t g1_expected = static_cast<uint8_t>((767.0f / 1023.0f) * 255.0f);
    uint8_t b1_expected = static_cast<uint8_t>((1000.0f / 1023.0f) * 255.0f);

    EXPECT_FLOAT_EQ(x1, 0.004f);
    EXPECT_FLOAT_EQ(y1, 0.005f);
    EXPECT_FLOAT_EQ(z1, 0.006f);
    EXPECT_FLOAT_EQ(nx1, 0.4f);
    EXPECT_FLOAT_EQ(ny1, 0.5f);
    EXPECT_FLOAT_EQ(nz1, 0.6f);
    EXPECT_EQ(r1, r1_expected);
    EXPECT_EQ(g1, g1_expected);
    EXPECT_EQ(b1, b1_expected);
}

TEST_F(ConversionTest, MinimalFrameXYZOnly) {
    pho::api::PFrame pframe = std::make_shared<pho::api::Frame>();
    pframe->PointCloud.Resize(pho::api::PhoXiSize(1, 1));
    pframe->PointCloud[0][0] = {7.0f, 8.0f, 9.0f};

    std::unique_ptr<sensor_msgs::msg::PointCloud2> msg = phoxi_camera::phoXiFrameToRosMsg(pframe);

    ASSERT_NE(msg, nullptr);
    ASSERT_EQ(msg->data.size(), msg->point_step);

    ASSERT_TRUE(fieldExists(*msg, "x"));
    ASSERT_TRUE(fieldExists(*msg, "y"));
    ASSERT_TRUE(fieldExists(*msg, "z"));
    ASSERT_FALSE(fieldExists(*msg, "normal_x"));
    ASSERT_FALSE(fieldExists(*msg, "rgb"));
    ASSERT_FALSE(fieldExists(*msg, "r"));
    ASSERT_FALSE(fieldExists(*msg, "intensity"));
    ASSERT_FALSE(fieldExists(*msg, "confidence"));

    EXPECT_EQ(msg->point_step, 12);
}

TEST_F(ConversionTest, GrayscaleFrame) {
    pho::api::PFrame pframe = std::make_shared<pho::api::Frame>();
    pframe->PointCloud.Resize(pho::api::PhoXiSize(1, 1));
    pframe->Texture.Resize(pho::api::PhoXiSize(1, 1));
    pframe->Texture[0][0] = 0.75f;

    std::unique_ptr<sensor_msgs::msg::PointCloud2> msg = phoxi_camera::phoXiFrameToRosMsg(pframe);

    ASSERT_NE(msg, nullptr);
    ASSERT_TRUE(fieldExists(*msg, "intensity"));
    ASSERT_FALSE(fieldExists(*msg, "r"));

    EXPECT_EQ(msg->point_step, 16);

    sensor_msgs::PointCloud2Iterator<float> iter_intensity(*msg, "intensity");
    EXPECT_FLOAT_EQ(iter_intensity[0], 0.75f);
}

TEST_F(ConversionTest, ColorCameraImageNullFrame) {
    pho::api::PFrame null_frame;
    EXPECT_THROW(phoxi_camera::colorCameraImageToRosMsg(null_frame), std::invalid_argument);
}

TEST_F(ConversionTest, ColorCameraImageEmptyData) {
    pho::api::PFrame pframe = std::make_shared<pho::api::Frame>();
    EXPECT_THROW(phoxi_camera::colorCameraImageToRosMsg(pframe), std::invalid_argument);
}

TEST_F(ConversionTest, ColorCameraImageMetadata) {
    pho::api::PFrame pframe = std::make_shared<pho::api::Frame>();
    const uint32_t width = 3;
    const uint32_t height = 2;
    pframe->ColorCameraImage.Resize(pho::api::PhoXiSize(width, height));

    auto msg = phoxi_camera::colorCameraImageToRosMsg(pframe);

    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->width, width);
    EXPECT_EQ(msg->height, height);
    EXPECT_EQ(msg->encoding, "rgb16");
    EXPECT_FALSE(msg->is_bigendian);
    EXPECT_EQ(msg->step, width * 3 * sizeof(uint16_t));
    EXPECT_EQ(msg->data.size(), height * msg->step);
}

TEST_F(ConversionTest, ColorCameraImagePixelValues) {
    pho::api::PFrame pframe = std::make_shared<pho::api::Frame>();
    pframe->ColorCameraImage.Resize(pho::api::PhoXiSize(2, 1));
    pframe->ColorCameraImage[0][0] = {1000, 2000, 3000};
    pframe->ColorCameraImage[0][1] = {4000, 5000, 6000};

    auto msg = phoxi_camera::colorCameraImageToRosMsg(pframe);

    ASSERT_NE(msg, nullptr);
    ASSERT_EQ(msg->data.size(), 2 * 3 * sizeof(uint16_t));

    uint16_t r0, g0, b0, r1, g1, b1;
    std::memcpy(&r0, msg->data.data() + 0 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&g0, msg->data.data() + 1 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&b0, msg->data.data() + 2 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&r1, msg->data.data() + 3 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&g1, msg->data.data() + 4 * sizeof(uint16_t), sizeof(uint16_t));
    std::memcpy(&b1, msg->data.data() + 5 * sizeof(uint16_t), sizeof(uint16_t));

    EXPECT_EQ(r0, 1000u);
    EXPECT_EQ(g0, 2000u);
    EXPECT_EQ(b0, 3000u);
    EXPECT_EQ(r1, 4000u);
    EXPECT_EQ(g1, 5000u);
    EXPECT_EQ(b1, 6000u);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
