#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "phoxi_camera/PhoXiFrame.h"
#include "phoxi_camera/RosConversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

bool fieldExists(const sensor_msgs::msg::PointCloud2& msg, const std::string& fieldName) {
    for (const auto& field : msg.fields) {
        if (field.name == fieldName) {
            return true;
        }
    }
    return false;
}

uint32_t getFieldOffset(const sensor_msgs::msg::PointCloud2& msg, const std::string& fieldName) {
    for (const auto& field : msg.fields) {
        if (field.name == fieldName) {
            return field.offset;
        }
    }
    throw std::runtime_error("Field '" + fieldName + "' not found in PointCloud2 message");
}

class ConversionTest : public ::testing::Test {};

TEST_F(ConversionTest, FullFrameConversion) {
    // 2-pixel frame: width=1, height=2 (column vector of 2 points)
    float pts[2][3] = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
    float normals[2][3] = {{0.1f, 0.2f, 0.3f}, {0.4f, 0.5f, 0.6f}};
    uint16_t rgb[2][3] = {{1023, 511, 0}, {255, 767, 1000}};
    float conf[2] = {0.9f, 0.8f};
    float depth[2] = {1.1f, 2.2f};

    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 2, sizeof(pts), pts};
    phoxi_frame_record_t nmRec = {PHOXI_FRAME_TYPE_NORMALMAP, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 2, sizeof(normals), normals};
    phoxi_frame_record_t rgbRec = {PHOXI_FRAME_TYPE_TEXTURE, PHOXI_FRAME_FORMAT_RGB_16, 1, 2, sizeof(rgb), rgb};
    phoxi_frame_record_t confRec = {PHOXI_FRAME_TYPE_CONFIDENCEMAP, PHOXI_FRAME_FORMAT_FLOAT_32F, 1, 2, sizeof(conf), conf};
    phoxi_frame_record_t depRec = {PHOXI_FRAME_TYPE_DEPTHMAP, PHOXI_FRAME_FORMAT_FLOAT_32F, 1, 2, sizeof(depth), depth};

    phoxi_camera::PhoXiFrame frame;
    frame.pointCloud = &pcRec;
    frame.normalMap = &nmRec;
    frame.textureRgb = &rgbRec;
    frame.confidenceMap = &confRec;
    frame.depthMap = &depRec;

    auto msg = phoxi_camera::phoXiFrameToRosMsg(frame);

    ASSERT_NE(msg, nullptr);
    ASSERT_EQ(msg->height, 2u);
    ASSERT_EQ(msg->width, 1u);

    ASSERT_TRUE(fieldExists(*msg, "x"));
    ASSERT_TRUE(fieldExists(*msg, "y"));
    ASSERT_TRUE(fieldExists(*msg, "z"));
    ASSERT_TRUE(fieldExists(*msg, "normal_x"));
    ASSERT_TRUE(fieldExists(*msg, "rgb"));
    ASSERT_TRUE(fieldExists(*msg, "confidence"));
    ASSERT_TRUE(fieldExists(*msg, "depth"));
    ASSERT_FALSE(fieldExists(*msg, "intensity"));

    const uint8_t* dataPtr = msg->data.data();
    const uint32_t pointStep = msg->point_step;

    uint32_t xOff = getFieldOffset(*msg, "x");
    uint32_t yOff = getFieldOffset(*msg, "y");
    uint32_t zOff = getFieldOffset(*msg, "z");
    uint32_t nxOff = getFieldOffset(*msg, "normal_x");
    uint32_t nyOff = getFieldOffset(*msg, "normal_y");
    uint32_t nzOff = getFieldOffset(*msg, "normal_z");
    uint32_t rgbOff = getFieldOffset(*msg, "rgb");
    uint32_t confOff = getFieldOffset(*msg, "confidence");
    uint32_t depOff = getFieldOffset(*msg, "depth");

    float x0, y0, z0, nx0, ny0, nz0, conf0, depth0;
    uint32_t rgb0Packed;
    std::memcpy(&x0, dataPtr + 0 * pointStep + xOff, sizeof(float));
    std::memcpy(&y0, dataPtr + 0 * pointStep + yOff, sizeof(float));
    std::memcpy(&z0, dataPtr + 0 * pointStep + zOff, sizeof(float));
    std::memcpy(&nx0, dataPtr + 0 * pointStep + nxOff, sizeof(float));
    std::memcpy(&ny0, dataPtr + 0 * pointStep + nyOff, sizeof(float));
    std::memcpy(&nz0, dataPtr + 0 * pointStep + nzOff, sizeof(float));
    std::memcpy(&rgb0Packed, dataPtr + 0 * pointStep + rgbOff, sizeof(uint32_t));
    std::memcpy(&conf0, dataPtr + 0 * pointStep + confOff, sizeof(float));
    std::memcpy(&depth0, dataPtr + 0 * pointStep + depOff, sizeof(float));

    uint8_t r0Expected = static_cast<uint8_t>((1023.0f / 1023.0f) * 255.0f);
    uint8_t g0Expected = static_cast<uint8_t>((511.0f / 1023.0f) * 255.0f);
    uint8_t b0Expected = static_cast<uint8_t>((0.0f / 1023.0f) * 255.0f);

    EXPECT_FLOAT_EQ(x0, 0.001f);  // 1.0 mm ÷ 1000
    EXPECT_FLOAT_EQ(y0, 0.002f);
    EXPECT_FLOAT_EQ(z0, 0.003f);
    EXPECT_FLOAT_EQ(nx0, 0.1f);
    EXPECT_FLOAT_EQ(ny0, 0.2f);
    EXPECT_FLOAT_EQ(nz0, 0.3f);
    EXPECT_EQ((rgb0Packed >> 16) & 0xFF, r0Expected);
    EXPECT_EQ((rgb0Packed >> 8) & 0xFF, g0Expected);
    EXPECT_EQ(rgb0Packed & 0xFF, b0Expected);
    EXPECT_FLOAT_EQ(conf0, 0.9f);
    EXPECT_FLOAT_EQ(depth0, 1.1f);

    float x1, y1, z1, nx1, ny1, nz1;
    uint32_t rgb1Packed;
    std::memcpy(&x1, dataPtr + 1 * pointStep + xOff, sizeof(float));
    std::memcpy(&y1, dataPtr + 1 * pointStep + yOff, sizeof(float));
    std::memcpy(&z1, dataPtr + 1 * pointStep + zOff, sizeof(float));
    std::memcpy(&nx1, dataPtr + 1 * pointStep + nxOff, sizeof(float));
    std::memcpy(&ny1, dataPtr + 1 * pointStep + nyOff, sizeof(float));
    std::memcpy(&nz1, dataPtr + 1 * pointStep + nzOff, sizeof(float));
    std::memcpy(&rgb1Packed, dataPtr + 1 * pointStep + rgbOff, sizeof(uint32_t));

    uint8_t r1Expected = static_cast<uint8_t>((255.0f / 1023.0f) * 255.0f);
    uint8_t g1Expected = static_cast<uint8_t>((767.0f / 1023.0f) * 255.0f);
    uint8_t b1Expected = static_cast<uint8_t>((1000.0f / 1023.0f) * 255.0f);

    EXPECT_FLOAT_EQ(x1, 0.004f);
    EXPECT_FLOAT_EQ(y1, 0.005f);
    EXPECT_FLOAT_EQ(z1, 0.006f);
    EXPECT_FLOAT_EQ(nx1, 0.4f);
    EXPECT_FLOAT_EQ(ny1, 0.5f);
    EXPECT_FLOAT_EQ(nz1, 0.6f);
    EXPECT_EQ((rgb1Packed >> 16) & 0xFF, r1Expected);
    EXPECT_EQ((rgb1Packed >> 8) & 0xFF, g1Expected);
    EXPECT_EQ(rgb1Packed & 0xFF, b1Expected);
}

TEST_F(ConversionTest, MinimalFrameXYZOnly) {
    float pts[1][3] = {{7.0f, 8.0f, 9.0f}};
    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(pts), pts};

    phoxi_camera::PhoXiFrame frame;
    frame.pointCloud = &pcRec;

    auto msg = phoxi_camera::phoXiFrameToRosMsg(frame);

    ASSERT_NE(msg, nullptr);
    ASSERT_EQ(msg->data.size(), msg->point_step);

    ASSERT_TRUE(fieldExists(*msg, "x"));
    ASSERT_TRUE(fieldExists(*msg, "y"));
    ASSERT_TRUE(fieldExists(*msg, "z"));
    ASSERT_FALSE(fieldExists(*msg, "normal_x"));
    ASSERT_FALSE(fieldExists(*msg, "rgb"));
    ASSERT_FALSE(fieldExists(*msg, "intensity"));
    ASSERT_FALSE(fieldExists(*msg, "confidence"));

    EXPECT_EQ(msg->point_step, 12u);
}

TEST_F(ConversionTest, GrayscaleFrame) {
    float pts[1][3] = {{0.0f, 0.0f, 0.0f}};
    float tex[1] = {1023.5f};
    phoxi_frame_record_t pcRec = {PHOXI_FRAME_TYPE_POINTCLOUD, PHOXI_FRAME_FORMAT_POINT3_32F, 1, 1, sizeof(pts), pts};
    phoxi_frame_record_t texRec = {PHOXI_FRAME_TYPE_TEXTURE, PHOXI_FRAME_FORMAT_FLOAT_32F, 1, 1, sizeof(tex), tex};

    phoxi_camera::PhoXiFrame frame;
    frame.pointCloud = &pcRec;
    frame.texture = &texRec;

    auto msg = phoxi_camera::phoXiFrameToRosMsg(frame);

    ASSERT_NE(msg, nullptr);
    ASSERT_TRUE(fieldExists(*msg, "intensity"));
    ASSERT_FALSE(fieldExists(*msg, "rgb"));

    EXPECT_EQ(msg->point_step, 16u);

    sensor_msgs::PointCloud2Iterator<float> iterIntensity(*msg, "intensity");
    EXPECT_FLOAT_EQ(iterIntensity[0], 1023.5f / 2047.0f);
}

TEST_F(ConversionTest, ColorCameraImageMetadata) {
    const uint32_t width = 3;
    const uint32_t height = 2;
    std::vector<uint16_t> pixels(width * height * 3, 0);
    phoxi_frame_record_t record = {
            PHOXI_FRAME_TYPE_COLORCAMERAIMAGE, PHOXI_FRAME_FORMAT_RGB_16, static_cast<int>(width), static_cast<int>(height), pixels.size() * sizeof(uint16_t), pixels.data()};

    auto msg = phoxi_camera::colorCameraImageToRosMsg(record);

    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->width, width);
    EXPECT_EQ(msg->height, height);
    EXPECT_EQ(msg->encoding, "rgb8");
    EXPECT_FALSE(msg->is_bigendian);
    EXPECT_EQ(msg->step, width * 3 * sizeof(uint8_t));
    EXPECT_EQ(msg->data.size(), height * msg->step);
}

TEST_F(ConversionTest, ColorCameraImagePixelValues) {
    uint16_t pixels[2][3] = {{100, 200, 512}, {0, 1023, 700}};
    phoxi_frame_record_t record = {PHOXI_FRAME_TYPE_COLORCAMERAIMAGE, PHOXI_FRAME_FORMAT_RGB_16, 2, 1, sizeof(pixels), pixels};

    auto msg = phoxi_camera::colorCameraImageToRosMsg(record);

    ASSERT_NE(msg, nullptr);
    ASSERT_EQ(msg->data.size(), 2u * 3u * sizeof(uint8_t));

    constexpr float scale = 255.0f / 1023.0f;
    EXPECT_EQ(msg->data[0], static_cast<uint8_t>(100 * scale));
    EXPECT_EQ(msg->data[1], static_cast<uint8_t>(200 * scale));
    EXPECT_EQ(msg->data[2], static_cast<uint8_t>(512 * scale));
    EXPECT_EQ(msg->data[3], static_cast<uint8_t>(0 * scale));
    EXPECT_EQ(msg->data[4], static_cast<uint8_t>(1023 * scale));
    EXPECT_EQ(msg->data[5], static_cast<uint8_t>(700 * scale));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
