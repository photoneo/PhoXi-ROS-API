#include "phoxi_camera/RosConversions.h"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "phoxi/details/jsoncons/json.hpp"

namespace phoxi_camera
{
static constexpr float MAX_TEXTURE = 2047.0f;                // 11-bit ADC full scale
static constexpr float MAX_RGB_CHANNEL = 1023.0f;            // 10-bit RGB channel full scale
static constexpr float RGB_SCALE = 255.0f / MAX_RGB_CHANNEL; // 10-bit channel to uint8

// Write `val` into `base[offset]` and advance `offset` by sizeof(T).
template <typename T>
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
    const auto* normals =
        frame.normalMap ? static_cast<const float(*)[3]>(frame.normalMap->data) : nullptr;
    const auto* rgbTexture =
        frame.textureRgb ? static_cast<const uint16_t(*)[3]>(frame.textureRgb->data) : nullptr;
    const auto* texture =
        (!rgbTexture && frame.texture) ? static_cast<const float*>(frame.texture->data) : nullptr;
    const auto* confidence =
        frame.confidenceMap ? static_cast<const float*>(frame.confidenceMap->data) : nullptr;
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
                const auto r8 = static_cast<uint8_t>(rgbTexture[idx][0] * RGB_SCALE);
                const auto g8 = static_cast<uint8_t>(rgbTexture[idx][1] * RGB_SCALE);
                const auto b8 = static_cast<uint8_t>(rgbTexture[idx][2] * RGB_SCALE);
                const uint32_t rgb = (uint32_t{r8} << 16) | (uint32_t{g8} << 8) | uint32_t{b8};
                pack(rgbOff, rgb);
            } else if (texture) {
                const float normalizedIntensity = texture[idx] / MAX_TEXTURE;
                pack(intensityOff, normalizedIntensity);
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

    msg->height = static_cast<uint32_t>(height);
    msg->width = static_cast<uint32_t>(width);
    msg->is_bigendian = false;
    msg->is_dense = false;

    uint32_t offset = 0;
    addField("x", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    addField("y", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    addField("z", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);

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
        }
    }

    return msg;
}

template <typename SrcT, typename DstT, typename Transform>
static std::unique_ptr<sensor_msgs::msg::Image> recordToRosMsg(const phoxi_frame_record_t& record,
    const std::string& encoding, uint32_t channels, Transform transform) {
    assert(record.data);
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->height = static_cast<uint32_t>(record.height);
    msg->width = static_cast<uint32_t>(record.width);
    msg->encoding = encoding;
    msg->is_bigendian = false;
    msg->step = msg->width * channels * static_cast<uint32_t>(sizeof(DstT));
    const size_t elementCount = static_cast<size_t>(msg->height) * msg->width * channels;
    msg->data.resize(elementCount * sizeof(DstT));
    const auto* src = static_cast<const SrcT*>(record.data);
    auto* dst = reinterpret_cast<DstT*>(msg->data.data());
    const size_t safeCount = std::min(elementCount, record.data_size / sizeof(SrcT));
    for (size_t i = 0; i < safeCount; ++i) {
        dst[i] = transform(src[i]);
    }
    return msg;
}

template <typename T>
static std::unique_ptr<sensor_msgs::msg::Image> recordToRosMsg(const phoxi_frame_record_t& record,
    const std::string& encoding, uint32_t channels) {
    return recordToRosMsg<T, T>(record, encoding, channels, [](T v) { return v; });
}

std::unique_ptr<sensor_msgs::msg::Image> normalMapToRosMsg(const phoxi_frame_record_t& record) {
    return recordToRosMsg<float>(record, "32FC3", 3);
}

std::unique_ptr<sensor_msgs::msg::Image> depthMapToRosMsg(const phoxi_frame_record_t& record) {
    return recordToRosMsg<float>(record, "32FC1", 1);
}

std::unique_ptr<sensor_msgs::msg::Image> confidenceMapToRosMsg(const phoxi_frame_record_t& record) {
    return recordToRosMsg<float>(record, "32FC1", 1);
}

std::unique_ptr<sensor_msgs::msg::Image> eventMapToRosMsg(const phoxi_frame_record_t& record) {
    return recordToRosMsg<float>(record, "32FC1", 1);
}

std::unique_ptr<sensor_msgs::msg::Image> textureToRosMsg(const phoxi_frame_record_t& record) {
    return recordToRosMsg<float, float>(record, "32FC1", 1,
        [](float v) { return v / MAX_TEXTURE; });
}

std::unique_ptr<sensor_msgs::msg::Image> textureRgbToRosMsg(const phoxi_frame_record_t& record) {
    return recordToRosMsg<uint16_t, uint8_t>(record, "rgb8", 3,
        [](uint16_t v) { return static_cast<uint8_t>(static_cast<float>(v) * RGB_SCALE); });
}

std::unique_ptr<sensor_msgs::msg::Image> colorCameraImageToRosMsg(const phoxi_frame_record_t& record) {
    return recordToRosMsg<uint16_t, uint8_t>(record, "rgb8", 3,
        [](uint16_t v) { return static_cast<uint8_t>(static_cast<float>(v) * RGB_SCALE); });
}

static std::unique_ptr<sensor_msgs::msg::CameraInfo> buildCameraInfo(const std::array<double, 9>& k, std::vector<double> d,
    uint32_t width, uint32_t height) {
    auto msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    msg->width = width;
    msg->height = height;
    msg->distortion_model = "plumb_bob";
    msg->d = std::move(d);
    msg->k = k;
    msg->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    msg->p = {k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0};
    return msg;
}

FrameCameraInfos frameInfoToRosMsgs(const phoxi_frame_record_t& record) {
    const char* dataPtr = static_cast<const char*>(record.data);
    size_t dataSize = record.data_size;
    while (dataSize > 0 && dataPtr[dataSize - 1] == '\0') {
        --dataSize;
    }
    const auto json = pho_jsoncons::json::parse(std::string(dataPtr, dataSize));

    if (!json.contains("info")) {
        return {};
    }
    const auto& info = json["info"];

    auto readArray9 = [](const pho_jsoncons::json& node) -> std::array<double, 9> {
        std::array<double, 9> arr{};
        for (size_t i = 0; i < arr.size() && i < node.size(); ++i) {
            arr[i] = node[i].as<double>();
        }
        return arr;
    };

    auto readD = [](const pho_jsoncons::json& node) -> std::vector<double> {
        std::vector<double> v;
        for (const auto& x : node.array_range()) {
            v.push_back(x.as<double>());
        }
        return v;
    };

    auto tryBuild = [&](const std::string& prefix) -> std::unique_ptr<sensor_msgs::msg::CameraInfo> {
        const std::string MAT  = prefix + "/PerspectiveSettings/CameraMatrix";
        const std::string DIST = prefix + "/PerspectiveSettings/DistortionCoefficients";
        const std::string RES  = prefix + "/Resolution";
        if (!info.contains(MAT) || !info.contains(DIST) || !info.contains(RES)) {
            return nullptr;
        }
        const auto& res = info[RES];
        if (!res.contains("width") || !res.contains("height")) {
            return nullptr;
        }
        return buildCameraInfo(
            readArray9(info[MAT]),
            readD(info[DIST]),
            res["width"].as<uint32_t>(),
            res["height"].as<uint32_t>());
    };

    FrameCameraInfos result;
    result.currentCamera      = tryBuild("current_camera");
    result.currentColorCamera = tryBuild("current_color_camera");
    return result;
}

} // namespace phoxi_camera
