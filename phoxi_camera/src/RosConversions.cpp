#include "phoxi_camera/RosConversions.h"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "phoxi/details/jsoncons/json.hpp"

namespace phoxi_camera {
static constexpr float MAX_TEXTURE = 2047.0f;
static constexpr float MAX_RGB_CHANNEL = 1023.0f;
static constexpr float RGB_SCALE = 255.0f / MAX_RGB_CHANNEL;
static constexpr float INV_MM = 1.f / 1000.f;
static constexpr float INV_MAX_TEXTURE = 1.f / MAX_TEXTURE;


void addField(const std::string& name, int datatype, uint32_t size, uint32_t& offset, const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
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
    const int nPixels = height * width;
    const size_t step = msg->point_step;

    for (int i = 0; i < nPixels; ++i) {
        uint8_t* p = dataPtr + static_cast<size_t>(i) * step;
        float* xyz = reinterpret_cast<float*>(p + xOff);
        xyz[0] = points[i][0] * INV_MM;
        xyz[1] = points[i][1] * INV_MM;
        xyz[2] = points[i][2] * INV_MM;
    }

    if (normals) {
        for (int i = 0; i < nPixels; ++i) {
            float* dst = reinterpret_cast<float*>(dataPtr + static_cast<size_t>(i) * step + normalXOff);
            dst[0] = normals[i][0];
            dst[1] = normals[i][1];
            dst[2] = normals[i][2];
        }
    }

    if (rgbTexture) {
        for (int i = 0; i < nPixels; ++i) {
            const auto r8 = static_cast<uint8_t>(rgbTexture[i][0] * RGB_SCALE);
            const auto g8 = static_cast<uint8_t>(rgbTexture[i][1] * RGB_SCALE);
            const auto b8 = static_cast<uint8_t>(rgbTexture[i][2] * RGB_SCALE);
            const uint32_t rgb = (uint32_t{r8} << 16) | (uint32_t{g8} << 8) | uint32_t{b8};
            *reinterpret_cast<uint32_t*>(dataPtr + static_cast<size_t>(i) * step + rgbOff) = rgb;
        }
    } else if (texture) {
        for (int i = 0; i < nPixels; ++i) {
            *reinterpret_cast<float*>(dataPtr + static_cast<size_t>(i) * step + intensityOff) = texture[i] * INV_MAX_TEXTURE;
        }
    }

    if (confidence) {
        for (int i = 0; i < nPixels; ++i) {
            *reinterpret_cast<float*>(dataPtr + static_cast<size_t>(i) * step + confidenceOff) = confidence[i];
        }
    }
    if (depth) {
        for (int i = 0; i < nPixels; ++i) {
            *reinterpret_cast<float*>(dataPtr + static_cast<size_t>(i) * step + depthOff) = depth[i];
        }
    }
    if (event) {
        for (int i = 0; i < nPixels; ++i) {
            *reinterpret_cast<float*>(dataPtr + static_cast<size_t>(i) * step + eventOff) = event[i];
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

    const int nPixels = height * width;
    const float* src = reinterpret_cast<const float*>(frame.pointCloud->data);
    float* dst = reinterpret_cast<float*>(msg->data.data());
    for (int i = 0; i < nPixels * 3; ++i) {
        dst[i] = src[i] * INV_MM;
    }

    return msg;
}

template <typename SrcT, typename DstT, typename Transform>
static std::unique_ptr<sensor_msgs::msg::Image> recordToRosMsg(const phoxi_frame_record_t& record, const std::string& encoding, uint32_t channels, Transform transform) {
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

template <typename T> static std::unique_ptr<sensor_msgs::msg::Image> recordToRosMsg(const phoxi_frame_record_t& record, const std::string& encoding, uint32_t channels) {
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
    return recordToRosMsg<float, float>(record, "32FC1", 1, [](float v) { return v / MAX_TEXTURE; });
}

std::unique_ptr<sensor_msgs::msg::Image> textureRgbToRosMsg(const phoxi_frame_record_t& record) {
    return recordToRosMsg<uint16_t, uint8_t>(record, "rgb8", 3, [](uint16_t v) { return static_cast<uint8_t>(static_cast<float>(v) * RGB_SCALE); });
}

std::unique_ptr<sensor_msgs::msg::Image> colorCameraImageToRosMsg(const phoxi_frame_record_t& record) {
    return textureRgbToRosMsg(record);
}


static std::array<double, 9> readArray9(const pho_jsoncons::json& node) {
    std::array<double, 9> arr{};
    for (size_t i = 0; i < arr.size() && i < node.size(); ++i) {
        arr[i] = node[i].as<double>();
    }
    return arr;
}

static std::vector<double> readDoubleArray(const pho_jsoncons::json& node) {
    std::vector<double> v;
    v.reserve(node.size());
    for (const auto& x : node.array_range()) {
        v.push_back(x.as<double>());
    }
    return v;
}

static std::array<double, 3> readXyz(const pho_jsoncons::json& info, const std::string& key) {
    if (!info.contains(key)) {
        return {};
    }
    const auto& o = info[key];
    return {o["x"].as<double>(), o["y"].as<double>(), o["z"].as<double>()};
}

static std::unique_ptr<sensor_msgs::msg::CameraInfo> tryBuildCameraInfo(const pho_jsoncons::json& info, const char* matKey, const char* distKey, const char* resKey) {
    if (!info.contains(matKey) || !info.contains(distKey) || !info.contains(resKey)) {
        return nullptr;
    }
    const auto& res = info[resKey];
    if (!res.contains("width") || !res.contains("height")) {
        return nullptr;
    }
    auto k = readArray9(info[matKey]);
    auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>();
    ci->width = res["width"].as<uint32_t>();
    ci->height = res["height"].as<uint32_t>();
    ci->distortion_model = "plumb_bob";
    ci->d = readDoubleArray(info[distKey]);
    ci->k = k;
    ci->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    ci->p = {k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0};
    return ci;
}

static std::vector<phoxi_camera_msgs::msg::FrameMessage> parseMessages(const pho_jsoncons::json& json) {
    std::vector<phoxi_camera_msgs::msg::FrameMessage> result;
    if (!json.contains("msgs")) {
        return result;
    }
    const auto& msgsNode = json["msgs"];
    result.reserve(msgsNode.size());
    for (const auto& m : msgsNode.array_range()) {
        phoxi_camera_msgs::msg::FrameMessage fm;
        if (m.contains("code")) {
            fm.code = m["code"].as<int32_t>();
        }
        if (m.contains("severity")) {
            fm.severity = m["severity"].as<int32_t>();
        }
        if (m.contains("text")) {
            fm.text = m["text"].as<std::string>();
        }
        result.push_back(std::move(fm));
    }
    return result;
}

ParsedFrameInfo parseFrameInfo(const phoxi_frame_record_t& record) {
    const char* dataPtr = static_cast<const char*>(record.data);
    size_t dataSize = record.data_size;
    while (dataSize > 0 && dataPtr[dataSize - 1] == '\0') {
        --dataSize;
    }
    const auto json = pho_jsoncons::json::parse(dataPtr, dataSize);

    ParsedFrameInfo result;
    result.successful = !json.contains("successful") || json["successful"].as<bool>();

    if (!result.successful) {
        auto errorMsg = std::make_unique<phoxi_camera_msgs::msg::FrameError>();
        errorMsg->messages = parseMessages(json);
        result.frameError = std::move(errorMsg);
        return result;
    }

    if (!json.contains("info")) {
        return result;
    }
    const auto& info = json["info"];
    result.currentCamera =
            tryBuildCameraInfo(info, "current_camera/PerspectiveSettings/CameraMatrix", "current_camera/PerspectiveSettings/DistortionCoefficients", "current_camera/Resolution");
    result.currentColorCamera = tryBuildCameraInfo(
            info, "current_color_camera/PerspectiveSettings/CameraMatrix", "current_color_camera/PerspectiveSettings/DistortionCoefficients", "current_color_camera/Resolution");

    auto msg = std::make_unique<phoxi_camera_msgs::msg::FrameInfo>();

    if (info.contains("hw_id")) {
        msg->hw_id = info["hw_id"].as<std::string>();
    }
    if (info.contains("index")) {
        msg->index = info["index"].as<int32_t>();
    }
    if (info.contains("total_scan_count")) {
        msg->total_scan_count = info["total_scan_count"].as<int32_t>();
    }
    if (info.contains("timestamp")) {
        msg->timestamp = info["timestamp"].as<double>();
    }
    if (info.contains("duration")) {
        msg->duration = info["duration"].as<double>();
    }
    if (info.contains("duration_computation")) {
        msg->duration_computation = info["duration_computation"].as<double>();
    }
    if (info.contains("duration_transfer")) {
        msg->duration_transfer = info["duration_transfer"].as<double>();
    }
    if (info.contains("is_early_transfer_frame")) {
        msg->is_early_transfer_frame = info["is_early_transfer_frame"].as<bool>();
    }

    msg->sensor_position = readXyz(info, "sensor_position");
    msg->sensor_x_axis = readXyz(info, "sensor_x_axis");
    msg->sensor_y_axis = readXyz(info, "sensor_y_axis");
    msg->sensor_z_axis = readXyz(info, "sensor_z_axis");
    msg->balance_rgb = readXyz(info, "balance_rgb");

    if (info.contains("camera_binning")) {
        const auto& b = info["camera_binning"];
        msg->camera_binning = {b["h"].as<int32_t>(), b["w"].as<int32_t>()};
    }
    if (info.contains("camera_binning_factor")) {
        const auto& b = info["camera_binning_factor"];
        msg->camera_binning_factor = {b["h"].as<double>(), b["w"].as<double>()};
    }

    if (info.contains("temperature")) {
        const auto& tempNode = info["temperature"];
        msg->temperature.reserve(tempNode.size());
        for (const auto& v : tempNode.array_range()) {
            msg->temperature.push_back(v.as<double>());
        }
    }

    if (info.contains("frame_start_time")) {
        const auto& fst = info["frame_start_time"];
        if (fst.contains("grand_master_identity")) {
            msg->frame_start_grand_master_identity = fst["grand_master_identity"].as<std::string>();
        }
        if (fst.contains("port_state")) {
            msg->frame_start_port_state = fst["port_state"].as<std::string>();
        }
        if (fst.contains("time_since_epoch")) {
            msg->frame_start_time_ns = fst["time_since_epoch"].as<int64_t>();
        }
    }

    if (info.contains("marker_dots")) {
        const auto& md = info["marker_dots"];
        if (md.contains("status")) {
            msg->marker_dots_status = md["status"].as<int32_t>();
        }
        if (md.contains("message")) {
            msg->marker_dots_message = md["message"].as<std::string>();
        }
    }

    msg->messages = parseMessages(json);

    result.frameInfo = std::move(msg);
    return result;
}

}  // namespace phoxi_camera
