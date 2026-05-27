#include "phoxi_camera/RosConversions.h"

namespace phoxi_camera
{

// Write `val` into `base[offset]` and advance `offset` by sizeof(underlying type).
// Plain scalar overload (float, uint32_t, …) — type is already trivially copyable.
template<typename T>
inline void packField(uint8_t* base, uint32_t& offset, const T& val) {
    std::memcpy(base + offset, &val, sizeof(T));
    offset += sizeof(T);
}

// MatType<T> overload — extracts the underlying T via the conversion operator before memcpy.
template<typename T>
inline void packField(uint8_t* base, uint32_t& offset, const pho::api::MatType<T>& val) {
    const T underlying = static_cast<const T&>(val);
    std::memcpy(base + offset, &underlying, sizeof(T));
    offset += sizeof(T);
}

// Scalar<T> overload — Scalar inherits MatType<T> but template deduction doesn't look
// through inheritance, so it would otherwise fall back to the primary template.
template<typename T>
inline void packField(uint8_t* base, uint32_t& offset, const pho::api::Scalar<T>& val) {
    const T underlying = static_cast<const T&>(val);
    std::memcpy(base + offset, &underlying, sizeof(T));
    offset += sizeof(T);
}
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
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    const auto res = pFrame->GetResolution();

    msg->height = res.Height;
    msg->width = res.Width;
    msg->is_bigendian = false;
    msg->is_dense = false;

    const bool hasPoints = !pFrame->PointCloud.Empty();
    const bool hasNormals = !pFrame->NormalMap.Empty();
    const bool hasRgb = !pFrame->TextureRGB.Empty();
    const bool hasIntensity = !hasRgb && !pFrame->Texture.Empty();
    const bool hasConfidence = !pFrame->ConfidenceMap.Empty();
    const bool hasDepth = !pFrame->DepthMap.Empty();
    const bool hasEvent = !pFrame->EventMap.Empty();

    uint32_t currentOffset = 0;

    addField("x", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    addField("y", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    addField("z", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    if (hasNormals) {
        addField("normal_x", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
        addField("normal_y", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
        addField("normal_z", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }
    if (hasRgb) {
        addField("rgb", sensor_msgs::msg::PointField::UINT32, 4, currentOffset, msg);
    } else if (hasIntensity) {
        addField("intensity", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }
    if (hasConfidence) {
        addField("confidence", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }
    if (hasDepth) {
        addField("depth", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }
    if (hasEvent) {
        addField("event", sensor_msgs::msg::PointField::FLOAT32, 4, currentOffset, msg);
    }

    msg->point_step = currentOffset;
    msg->row_step = msg->width * msg->point_step;
    msg->data.resize(msg->height * msg->row_step);

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

    for (int r = 0; r < pFrame->GetResolution().Height; ++r) {
        for (int c = 0; c < pFrame->GetResolution().Width; ++c) {
            uint8_t* pointStart = dataPtr + (r * msg->row_step) + (c * msg->point_step);

            // Adapter: write val at a named (non-sequential) field offset.
            auto pack = [&](uint32_t fieldOff, const auto& val) {
                packField(pointStart, fieldOff, val);
            };

            if (hasPoints) {
                auto point = pFrame->PointCloud.At(r, c);
                point.x /= 1000.;
                point.y /= 1000.;
                point.z /= 1000.;
                pack(xOff, point.x);
                pack(yOff, point.y);
                pack(zOff, point.z);
            }

            if (hasNormals) {
                const auto& normal = pFrame->NormalMap.At(r, c);
                pack(normalXOff, normal.x);
                pack(normalYOff, normal.y);
                pack(normalZOff, normal.z);
            }

            if (hasRgb) {
                const auto& color16 = pFrame->TextureRGB.At(r, c);
                const auto r8 = static_cast<uint8_t>((static_cast<float>(color16.r) / 1023.0f) * 255.0f);
                const auto g8 = static_cast<uint8_t>((static_cast<float>(color16.g) / 1023.0f) * 255.0f);
                const auto b8 = static_cast<uint8_t>((static_cast<float>(color16.b) / 1023.0f) * 255.0f);
                uint32_t rgbPacked = (static_cast<uint32_t>(r8) << 16) |
                                     (static_cast<uint32_t>(g8) << 8) | (static_cast<uint32_t>(b8));
                pack(rgbOff, rgbPacked);
            } else if (hasIntensity) {
                pack(intensityOff, pFrame->Texture.At(r, c));
            }

            if (hasConfidence) {
                pack(confidenceOff, pFrame->ConfidenceMap.At(r, c));
            }

            if (hasDepth) {
                pack(depthOff, pFrame->DepthMap.At(r, c));
            }

            if (hasEvent) {
                pack(eventOff, pFrame->EventMap.At(r, c));
            }
        }
    }

    return msg;
}

std::unique_ptr<sensor_msgs::msg::PointCloud2> pointsToRosMsg(const pho::api::PFrame& pFrame) {
    const auto resolution = pFrame->GetResolution();

    const bool hasRgb = !pFrame->TextureRGB.Empty();
    const bool hasIntensity = !hasRgb && !pFrame->Texture.Empty();

    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->height = resolution.Height;
    msg->width = resolution.Width;
    msg->is_bigendian = false;
    msg->is_dense = false;

    uint32_t offset = 0;
    addField("x", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    addField("y", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    addField("z", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    if (hasRgb) {
        addField("rgb", sensor_msgs::msg::PointField::UINT32, 4, offset, msg);
    } else if (hasIntensity) {
        addField("intensity", sensor_msgs::msg::PointField::FLOAT32, 4, offset, msg);
    }

    msg->point_step = offset;
    msg->row_step = msg->width * msg->point_step;
    msg->data.resize(msg->height * msg->row_step, 0);

    uint8_t* dataPtr = msg->data.data();
    for (int r = 0; r < resolution.Height; ++r) {
        for (int c = 0; c < resolution.Width; ++c) {
            uint8_t* p = dataPtr + r * msg->row_step + c * msg->point_step;

            auto pt = pFrame->PointCloud.At(r, c);
            pt.x /= 1000.f;
            pt.y /= 1000.f;
            pt.z /= 1000.f;
            uint32_t off = 0;
            packField(p, off, pt.x);
            packField(p, off, pt.y);
            packField(p, off, pt.z);

            if (hasRgb) {
                const auto& color = pFrame->TextureRGB.At(r, c);
                const auto r8 = static_cast<uint8_t>((static_cast<float>(color.r) / 1023.0f) * 255.0f);
                const auto g8 = static_cast<uint8_t>((static_cast<float>(color.g) / 1023.0f) * 255.0f);
                const auto b8 = static_cast<uint8_t>((static_cast<float>(color.b) / 1023.0f) * 255.0f);
                uint32_t packed = (static_cast<uint32_t>(r8) << 16) | (static_cast<uint32_t>(g8) << 8) | static_cast<uint32_t>(b8);
                packField(p, off, packed);
            } else if (hasIntensity) {
                packField(p, off, pFrame->Texture.At(r, c));
            }
        }
    }

    return msg;
}

std::unique_ptr<sensor_msgs::msg::Image> rgb16MatToRosMsg(const pho::api::TextureRGB16& img) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->height = static_cast<uint32_t>(img.Size.Height);
    msg->width = static_cast<uint32_t>(img.Size.Width);
    msg->encoding = "rgb16";
    msg->is_bigendian = false;
    msg->step = msg->width * 3 * sizeof(uint16_t);
    msg->data.resize(msg->height * msg->step);

    uint8_t* dataPtr = msg->data.data();
    for (int r = 0; r < img.Size.Height; ++r) {
        for (int c = 0; c < img.Size.Width; ++c) {
            const auto& pixel = img.At(r, c);
            uint8_t* dst = dataPtr + r * msg->step + c * 3 * sizeof(uint16_t);
            uint32_t off = 0;
            packField(dst, off, pixel.r);
            packField(dst, off, pixel.g);
            packField(dst, off, pixel.b);
        }
    }
    return msg;
}

template <typename ScalarMat>
std::unique_ptr<sensor_msgs::msg::Image> scalarMapToRosMsg(const ScalarMat& mat) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->height = static_cast<uint32_t>(mat.Size.Height);
    msg->width = static_cast<uint32_t>(mat.Size.Width);
    msg->encoding = "32FC1";
    msg->is_bigendian = false;
    msg->step = msg->width * static_cast<uint32_t>(sizeof(float));
    msg->data.resize(msg->height * msg->step);

    uint8_t* dataPtr = msg->data.data();
    for (int r = 0; r < mat.Size.Height; ++r) {
        for (int c = 0; c < mat.Size.Width; ++c) {
            uint32_t off = 0;
            packField(dataPtr + r * msg->step + c * sizeof(float), off, mat.At(r, c));
        }
    }
    return msg;
}

std::unique_ptr<sensor_msgs::msg::Image> normalMapToRosMsg(const pho::api::PFrame& pFrame) {
    const auto& mat = pFrame->NormalMap;
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->height = static_cast<uint32_t>(mat.Size.Height);
    msg->width = static_cast<uint32_t>(mat.Size.Width);
    msg->encoding = "32FC3";
    msg->is_bigendian = false;
    msg->step = msg->width * 3 * static_cast<uint32_t>(sizeof(float));
    msg->data.resize(msg->height * msg->step);

    uint8_t* dataPtr = msg->data.data();
    for (int r = 0; r < mat.Size.Height; ++r) {
        for (int c = 0; c < mat.Size.Width; ++c) {
            const auto& n = mat.At(r, c);
            uint8_t* dst = dataPtr + r * msg->step + c * 3 * sizeof(float);
            uint32_t off = 0;
            packField(dst, off, n.x);
            packField(dst, off, n.y);
            packField(dst, off, n.z);
        }
    }
    return msg;
}

std::unique_ptr<sensor_msgs::msg::Image> depthMapToRosMsg(const pho::api::PFrame& pFrame) {
    return scalarMapToRosMsg(pFrame->DepthMap);
}

std::unique_ptr<sensor_msgs::msg::Image> confidenceMapToRosMsg(const pho::api::PFrame& pFrame) {
    return scalarMapToRosMsg(pFrame->ConfidenceMap);
}

std::unique_ptr<sensor_msgs::msg::Image> eventMapToRosMsg(const pho::api::PFrame& pFrame) {
    return scalarMapToRosMsg(pFrame->EventMap);
}

std::unique_ptr<sensor_msgs::msg::Image> textureToRosMsg(const pho::api::PFrame& pFrame) {
    return scalarMapToRosMsg(pFrame->Texture);
}

std::unique_ptr<sensor_msgs::msg::Image> textureRgbToRosMsg(const pho::api::PFrame& pFrame) {
    return rgb16MatToRosMsg(pFrame->TextureRGB);
}

std::unique_ptr<sensor_msgs::msg::Image> colorCameraImageToRosMsg(const pho::api::PFrame& pFrame) {
    return rgb16MatToRosMsg(pFrame->ColorCameraImage);
}

}