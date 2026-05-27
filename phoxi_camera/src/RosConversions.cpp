#include "phoxi_camera/RosConversions.h"

#include <algorithm>
#include <cassert>
#include <cstring>

namespace phoxi_camera
{

// Write `val` into `base[offset]` and advance `offset` by sizeof(T).
template<typename T>
void packField(uint8_t* base, uint32_t& offset, const T& val) {
    std::memcpy(base + offset, &val, sizeof(T));
    offset += sizeof(T);
}

void addField(const std::string& name, int datatype, uint32_t size, uint32_t& offset,
    const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    msg->fields.push_back(field);
    offset += size;
}

std::unique_ptr<sensor_msgs::msg::PointCloud2> phoXiFrameToRosMsg(const PhoXiFrame& frame) {
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    if (!frame.pointCloud) {
        return msg;
    }

    const int height = frame.pointCloud->height;
    const int width = frame.pointCloud->width;
    const auto* points = static_cast<const float(*)[3]>(frame.pointCloud->data);
    const auto* normals = frame.normalMap ? static_cast<const float(*)[3]>(frame.normalMap->data) : nullptr;
    const auto* rgbTexture = frame.textureRgb ? static_cast<const uint16_t(*)[3]>(frame.textureRgb->data) : nullptr;
    const auto* texture = (!rgbTexture && frame.texture) ? static_cast<const float*>(frame.texture->data) : nullptr;
    const auto* confidence = frame.confidenceMap ? static_cast<const float*>(frame.confidenceMap->data) : nullptr;
    const auto* depth = frame.depthMap ? static_cast<const float*>(frame.depthMap->data) : nullptr;
    const auto* event = frame.eventMap ? static_cast<const float*>(frame.eventMap->data) : nullptr;

    msg->height = static_cast<uint32_t>(height);
    msg->width = static_cast<uint32_t>(width);
    msg->is_bigendian = false;
    msg->is_dense = false;

    uint32_t currentOffset = 0;
    const uint32_t xOff = currentOffset;
    addField("x", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);

    const uint32_t yOff = currentOffset;
    addField("y", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);

    const uint32_t zOff = currentOffset;
    addField("z", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);

    uint32_t normalXOff = 0, normalYOff = 0, normalZOff = 0;
    if (normals) {
        normalXOff = currentOffset;
        addField("normal_x", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
        normalYOff = currentOffset;
        addField("normal_y", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
        normalZOff = currentOffset;
        addField("normal_z", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }

    uint32_t rgbOff = 0, intensityOff = 0;
    if (rgbTexture) {
        rgbOff = currentOffset;
        addField("rgb", sensor_msgs::msg::PointField::UINT32, 4, currentOffset, msg);
    } else if (texture) {
        intensityOff = currentOffset;
        addField("intensity", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }

    uint32_t confidenceOff = 0;
    if (confidence) {
        confidenceOff = currentOffset;
        addField("confidence", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }

    uint32_t depthOff = 0;
    if (depth) {
        depthOff = currentOffset;
        addField("depth", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }

    uint32_t eventOff = 0;
    if (event) {
        eventOff = currentOffset;
        addField("event", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }

    msg->point_step = currentOffset;
    msg->row_step = msg->width * msg->point_step;
    msg->data.resize(static_cast<size_t>(msg->height) * msg->row_step);

    uint8_t* dataPtr = msg->data.data();
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            const int idx = r * width + c;
            uint8_t* pointStart = dataPtr + r * msg->row_step + c * msg->point_step;

            auto pack = [&](uint32_t off, const auto& val) {
                std::memcpy(pointStart + off, &val, sizeof(val));
            };

            pack(xOff, points[idx][0] / 1000.f);
            pack(yOff, points[idx][1] / 1000.f);
            pack(zOff, points[idx][2] / 1000.f);

            if (normals) {
                pack(normalXOff, normals[idx][0]);
                pack(normalYOff, normals[idx][1]);
                pack(normalZOff, normals[idx][2]);
            }

            if (rgbTexture) {
                constexpr float RGB_SCALE = 255.0f / 1023.0f;
                const auto r8 = static_cast<uint8_t>(rgbTexture[idx][0] * RGB_SCALE);
                const auto g8 = static_cast<uint8_t>(rgbTexture[idx][1] * RGB_SCALE);
                const auto b8 = static_cast<uint8_t>(rgbTexture[idx][2] * RGB_SCALE);
                const uint32_t rgb = (uint32_t{r8} << 16) | (uint32_t{g8} << 8) | uint32_t{b8};
                pack(rgbOff, rgb);
            } else if (texture) {
                pack(intensityOff, texture[idx]);
            }

            if (confidence) {
                pack(confidenceOff, confidence[idx]);
            }
            if (depth) {
                pack(depthOff, depth[idx]);
            }
            if (event) {
                pack(eventOff, event[idx]);
            }
        }
    }

    return msg;
}

std::unique_ptr<sensor_msgs::msg::PointCloud2> pointsToRosMsg(const PhoXiFrame& frame) {
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

    if (!frame.pointCloud) {
        return msg;
    }

    const int height = frame.pointCloud->height;
    const int width = frame.pointCloud->width;

    const auto* points = static_cast<const float(*)[3]>(frame.pointCloud->data);
    const auto* rgbTexture = frame.textureRgb ? static_cast<const uint16_t(*)[3]>(frame.textureRgb->data) : nullptr;
    const auto* texture = (!rgbTexture && frame.texture) ? static_cast<const float*>(frame.texture->data) : nullptr;

    msg->height = static_cast<uint32_t>(height);
    msg->width = static_cast<uint32_t>(width);
    msg->is_bigendian = false;
    msg->is_dense = false;

    uint32_t offset = 0;
    addField("x", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    addField("y", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    addField("z", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    if (rgbTexture) {
        addField("rgb", sensor_msgs::msg::PointField::UINT32, 4, offset, msg);
    } else if (texture) {
        addField("intensity", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    }

    msg->point_step = offset;
    msg->row_step = msg->width * msg->point_step;
    msg->data.resize(msg->height * msg->row_step, 0);

    uint8_t* dataPtr = msg->data.data();
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            const int idx = r * width + c;
            uint8_t* p = dataPtr + r * msg->row_step + c * msg->point_step;
            uint32_t off = 0;
            packField(p, off, points[idx][0] / 1000.f);
            packField(p, off, points[idx][1] / 1000.f);
            packField(p, off, points[idx][2] / 1000.f);

            if (rgbTexture) {
                constexpr float RGB_SCALE = 255.0f / 1023.0f;
                const auto r8 = static_cast<uint8_t>(rgbTexture[idx][0] * RGB_SCALE);
                const auto g8 = static_cast<uint8_t>(rgbTexture[idx][1] * RGB_SCALE);
                const auto b8 = static_cast<uint8_t>(rgbTexture[idx][2] * RGB_SCALE);
                const uint32_t rgb = (uint32_t{r8} << 16) | (uint32_t{g8} << 8) | uint32_t{b8};
                packField(p, off, rgb);
            } else if (texture) {
                packField(p, off, texture[idx]);
            }
        }
    }

    return msg;
}

std::unique_ptr<sensor_msgs::msg::Image> normalMapToRosMsg(const phoxi_frame_record_t& record) {
    assert(record.data);
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->height = static_cast<uint32_t>(record.height);
    msg->width = static_cast<uint32_t>(record.width);
    msg->encoding = "32FC3";
    msg->is_bigendian = false;
    msg->step = msg->width * 3 * static_cast<uint32_t>(sizeof(float));
    msg->data.resize(static_cast<size_t>(msg->height) * msg->step);
    std::memcpy(msg->data.data(), record.data,
                std::min(record.data_size, static_cast<size_t>(msg->height) * msg->step));
    return msg;
}

static std::unique_ptr<sensor_msgs::msg::Image> scalarMapToRosMsg(const phoxi_frame_record_t& record) {
    assert(record.data);
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->height = static_cast<uint32_t>(record.height);
    msg->width = static_cast<uint32_t>(record.width);
    msg->encoding = "32FC1";
    msg->is_bigendian = false;
    msg->step = msg->width * static_cast<uint32_t>(sizeof(float));
    msg->data.resize(static_cast<size_t>(msg->height) * msg->step);
    std::memcpy(msg->data.data(), record.data,
                std::min(record.data_size, static_cast<size_t>(msg->height) * msg->step));
    return msg;
}

static std::unique_ptr<sensor_msgs::msg::Image> rgb16RecordToRosMsg(const phoxi_frame_record_t& record) {
    assert(record.data);
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->height = static_cast<uint32_t>(record.height);
    msg->width = static_cast<uint32_t>(record.width);
    msg->encoding = "rgb16";
    msg->is_bigendian = false;
    msg->step = msg->width * 3 * static_cast<uint32_t>(sizeof(uint16_t));
    msg->data.resize(static_cast<size_t>(msg->height) * msg->step);
    std::memcpy(msg->data.data(), record.data,
                std::min(record.data_size, static_cast<size_t>(msg->height) * msg->step));
    return msg;
}

std::unique_ptr<sensor_msgs::msg::Image> depthMapToRosMsg(const phoxi_frame_record_t& record) {
    return scalarMapToRosMsg(record);
}

std::unique_ptr<sensor_msgs::msg::Image> confidenceMapToRosMsg(const phoxi_frame_record_t& record) {
    return scalarMapToRosMsg(record);
}

std::unique_ptr<sensor_msgs::msg::Image> eventMapToRosMsg(const phoxi_frame_record_t& record) {
    return scalarMapToRosMsg(record);
}

std::unique_ptr<sensor_msgs::msg::Image> textureToRosMsg(const phoxi_frame_record_t& record) {
    return scalarMapToRosMsg(record);
}

std::unique_ptr<sensor_msgs::msg::Image> textureRgbToRosMsg(const phoxi_frame_record_t& record) {
    return rgb16RecordToRosMsg(record);
}

std::unique_ptr<sensor_msgs::msg::Image> colorCameraImageToRosMsg(const phoxi_frame_record_t& record) {
    return rgb16RecordToRosMsg(record);
}

}  // namespace phoxi_camera
