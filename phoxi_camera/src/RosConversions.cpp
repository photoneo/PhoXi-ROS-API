#include "phoxi_camera/RosConversions.h"

namespace phoxi_camera
{

void addField(const std::string& name, int datatype, uint32_t size, uint32_t& offset,
              std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    msg->fields.push_back(field);
    offset += size;
}

std::unique_ptr<sensor_msgs::msg::PointCloud2> phoXiFrameToRosMsg(const pho::api::PFrame& pFrame) {
    if (!pFrame) {
        throw std::invalid_argument("pFrame is NULL");
    }

    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

    msg->height = pFrame->GetResolution().Height;
    msg->width = pFrame->GetResolution().Width;
    msg->is_bigendian = false;
    msg->is_dense = false;

    uint32_t currentOffset = 0;

    sensor_msgs::msg::PointField field;
    addField("x", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    addField("y", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    addField("z", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);

    if (!pFrame->NormalMap.Empty()) {
        addField("normal_x", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
        addField("normal_y", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
        addField("normal_z", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }
    if (!pFrame->TextureRGB.Empty()) {
        addField("rgb", sensor_msgs::msg::PointField::UINT32, 4, currentOffset, msg);
    } else if (!pFrame->Texture.Empty()) {
        addField("intensity", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }
    if (!pFrame->ConfidenceMap.Empty()) {
        addField("confidence", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }
    if (!pFrame->DepthMap.Empty()) {
        addField("depth", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }
    if (!pFrame->EventMap.Empty()) {
        addField("event", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }

    msg->point_step = currentOffset;
    msg->row_step = msg->width * msg->point_step;
    msg->data.resize(msg->height * msg->row_step);

    const bool hasPoints = !pFrame->PointCloud.Empty();
    const bool hasNormals = !pFrame->NormalMap.Empty();
    const bool hasRgb = !pFrame->TextureRGB.Empty();
    const bool hasIntensity = !pFrame->Texture.Empty();
    const bool hasConfidence = !pFrame->ConfidenceMap.Empty();
    const bool hasDepth = !pFrame->DepthMap.Empty();
    const bool hasEvent = !pFrame->EventMap.Empty();

    std::map<std::string, uint32_t> offsets;
    for (const auto& field : msg->fields) {
        offsets[field.name] = field.offset;
    }
    const uint32_t xOff = offsets.count("x") ? offsets.at("x") : 0;
    const uint32_t yOff = offsets.count("y") ? offsets.at("y") : 0;
    const uint32_t zOff = offsets.count("z") ? offsets.at("z") : 0;
    const uint32_t normalXOff = offsets.count("normal_x") ? offsets.at("normal_x") : 0;
    const uint32_t normalYOff = offsets.count("normal_y") ? offsets.at("normal_y") : 0;
    const uint32_t normalZOff = offsets.count("normal_z") ? offsets.at("normal_z") : 0;
    const uint32_t rgbOff = offsets.count("rgb") ? offsets.at("rgb") : 0;
    const uint32_t intensityOff = offsets.count("intensity") ? offsets.at("intensity") : 0;
    const uint32_t confidenceOff = offsets.count("confidence") ? offsets.at("confidence") : 0;
    const uint32_t depthOff = offsets.count("depth") ? offsets.at("depth") : 0;
    const uint32_t eventOff = offsets.count("event") ? offsets.at("event") : 0;

    uint8_t* dataPtr = msg->data.data();

    for (size_t r = 0; r < pFrame->GetResolution().Height; ++r) {
        for (size_t c = 0; c < pFrame->GetResolution().Width; ++c) {
            uint8_t* pointStart = dataPtr + (r * msg->row_step) + (c * msg->point_step);

            if (hasPoints) {
                auto point = pFrame->PointCloud.At(r, c);
                point.x /= 1000.;
                point.y /= 1000.;
                point.z /= 1000.;
                std::memcpy(pointStart + xOff, &point.x, sizeof(float));
                std::memcpy(pointStart + yOff, &point.y, sizeof(float));
                std::memcpy(pointStart + zOff, &point.z, sizeof(float));
            }

            if (hasNormals) {
                const auto& normal = pFrame->NormalMap.At(r, c);
                std::memcpy(pointStart + normalXOff, &normal.x, sizeof(float));
                std::memcpy(pointStart + normalYOff, &normal.y, sizeof(float));
                std::memcpy(pointStart + normalZOff, &normal.z, sizeof(float));
            }

            if (hasRgb) {
                const auto& color16 = pFrame->TextureRGB.At(r, c);
                uint8_t r8 =
                    static_cast<uint8_t>((static_cast<float>(color16.r) / 1023.0f) * 255.0f);
                uint8_t g8 =
                    static_cast<uint8_t>((static_cast<float>(color16.g) / 1023.0f) * 255.0f);
                uint8_t b8 =
                    static_cast<uint8_t>((static_cast<float>(color16.b) / 1023.0f) * 255.0f);
                uint32_t rgbPacked = (static_cast<uint32_t>(r8) << 16) |
                                     (static_cast<uint32_t>(g8) << 8) | (static_cast<uint32_t>(b8));
                memcpy(pointStart + rgbOff, &rgbPacked, sizeof(uint32_t));
            } else if (hasIntensity) {
                std::memcpy(pointStart + intensityOff, &pFrame->Texture.At(r, c), sizeof(float));
            }

            if (hasConfidence) {
                std::memcpy(pointStart + confidenceOff, &pFrame->ConfidenceMap.At(r, c),
                            sizeof(float));
            }

            if (hasDepth) {
                std::memcpy(pointStart + depthOff, &pFrame->DepthMap.At(r, c), sizeof(float));
            }

            if (hasEvent) {
                std::memcpy(pointStart + eventOff, &pFrame->EventMap.At(r, c), sizeof(float));
            }
        }
    }

    return msg;
}

}  // namespace phoxi_camera
