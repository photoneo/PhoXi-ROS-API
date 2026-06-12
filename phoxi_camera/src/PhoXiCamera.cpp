#include "phoxi_camera/PhoXiCamera.h"

#include <array>
#include <chrono>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/RosConversions.h"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace phoxi_camera {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static constexpr char PARAM_SEP = '.';
static constexpr std::string_view DEVICE_SETTINGS_PREFIX = "deviceSettings";
static constexpr std::string_view DEVICE_INFO_PREFIX = "deviceInfo";
static constexpr std::string_view FRAME_SETTINGS_PREFIX = "frameSettings";
static constexpr std::array<std::string_view, 7> COMPONENTS = {
    "PointCloud", "NormalMap", "DepthMap", "Texture", "ConfidenceMap", "ColorCameraImage", "EventMap"
};

static std::string connectionStatusToString(PhoXiDeviceInformation::PhoXiConnectionStatus status) {
    switch (status) {
        case PhoXiDeviceInformation::Ready:
            return "Ready";
        case PhoXiDeviceInformation::Occupied:
            return "Occupied";
        case PhoXiDeviceInformation::Connected:
            return "Connected";
        default:
            return "Undefined";
    }
}

static std::string deviceKeyToParamName(const std::string& key) {
    std::string name = key;
    std::replace(name.begin(), name.end(), '/', PARAM_SEP);
    return name;
}

// PhoXiReprojectionMap, PhoXiMesh and ProjectionGeometry_64f define operator==
// but not operator!=, so std::variant::operator!= cannot be instantiated.
// Use this helper instead of `!=` on SettingValue.
static bool settingValuesEqual(const SettingValue& a, const SettingValue& b) {
    if (a.index() != b.index()) {
        return false;
    }
    return std::visit(
            [&b](const auto& av) -> bool {
                const auto* bv = std::get_if<std::decay_t<decltype(av)>>(&b);
                return bv && (av == *bv);
            },
            a);
}

PhoXiCamera::PhoXiCamera(std::string deviceId, const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("phoxi_camera", options),
      mPhoXiInterface(std::make_unique<PhoXiInterface>()),
      mDeviceId(std::move(deviceId)) {
    RCLCPP_INFO(get_logger(), "Creating PhoXi Camera node for device: %s", mDeviceId.c_str());
    declareParameters();
}

PhoXiCamera::PhoXiCamera(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("phoxi_camera", options),
      mPhoXiInterface(std::make_unique<PhoXiInterface>()) {
    RCLCPP_INFO(get_logger(), "Creating PhoXi Camera node.");
    declareParameters();
    get_parameter("device_id", mDeviceId);
}

PhoXiCamera::~PhoXiCamera() {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(mShutdownCallbackHandle);

    try {
        if (mPhoXiInterface->isAcquiring()) {
            mPhoXiInterface->stopAcquisition();
        }
    } catch (...) {
    }

    if (mPhoXiInterface->isConnected()) {
        mPhoXiInterface->disconnectCamera();
    }
}

CallbackReturn PhoXiCamera::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Configuring PhoXi Camera node...");

    get_parameter("device_id", mDeviceId);
    get_parameter("frame_id", mFrameId);
    get_parameter("publish_combined", mPublishCombined);
    if (mDeviceId.empty()) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: 'device_id' parameter is empty.");
        return CallbackReturn::FAILURE;
    }

    try {
        mPhoXiInterface->connectCamera(mDeviceId, std::bind(&PhoXiCamera::onFrameCallback, this, std::placeholders::_1));
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    try {
        declareDeviceInfoParameters();
        loadDeviceSettingDescriptors();
        declareDeviceSettingParameters();
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed (device settings): %s", e.what());
        try {
            mPhoXiInterface->disconnectCamera();
        } catch (...) {
        }
        return CallbackReturn::FAILURE;
    }

    try {
        std::vector<std::pair<std::string, bool>> components;
        for (const auto& componentName : COMPONENTS) {
            const auto param = get_parameter(std::string(FRAME_SETTINGS_PREFIX) + PARAM_SEP + std::string(componentName));
            if (param.get_type() != rclcpp::PARAMETER_NOT_SET) {
                components.emplace_back(std::string(componentName), param.as_bool());
            }
        }
        if (!components.empty()) {
            mPhoXiInterface->setFrameOutputSettings(components);
        }
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        try {
            mPhoXiInterface->disconnectCamera();
        } catch (...) {
        }
        return CallbackReturn::FAILURE;
    }

    mFrameInfoPub = create_publisher<phoxi_camera_msgs::msg::FrameInfo>("frameInfo", rclcpp::SystemDefaultsQoS());
    mPrimaryCameraInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>("frameInfo/currentCamera", rclcpp::SystemDefaultsQoS());
    mColorCameraInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>("frameInfo/currentColorCamera", rclcpp::SystemDefaultsQoS());
    mPointCloudPub = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SystemDefaultsQoS());
    mPointsPub = create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SystemDefaultsQoS());
    mNormalMapPub = create_publisher<sensor_msgs::msg::Image>("normals", rclcpp::SystemDefaultsQoS());
    mDepthMapPub = create_publisher<sensor_msgs::msg::Image>("depth", rclcpp::SystemDefaultsQoS());
    mConfidenceMapPub = create_publisher<sensor_msgs::msg::Image>("confidence", rclcpp::SystemDefaultsQoS());
    mEventMapPub = create_publisher<sensor_msgs::msg::Image>("event", rclcpp::SystemDefaultsQoS());
    mTexturePub = create_publisher<sensor_msgs::msg::Image>("intensity", rclcpp::SystemDefaultsQoS());
    mTextureRgbPub = create_publisher<sensor_msgs::msg::Image>("texture", rclcpp::SystemDefaultsQoS());
    mColorCameraImagePub = create_publisher<sensor_msgs::msg::Image>("color_camera_image", rclcpp::SystemDefaultsQoS());

    mConnectService = create_service<phoxi_camera_msgs::srv::Connect>("~/connect", std::bind(&PhoXiCamera::connectCallback, this, std::placeholders::_1, std::placeholders::_2));
    mDisconnectService = create_service<std_srvs::srv::Trigger>("~/disconnect", std::bind(&PhoXiCamera::disconnectCallback, this, std::placeholders::_1, std::placeholders::_2));
    mTriggerFrameService = create_service<phoxi_camera_msgs::srv::TriggerFrame>(
            "~/trigger_frame", std::bind(&PhoXiCamera::triggerFrameCallback, this, std::placeholders::_1, std::placeholders::_2));
    mGetProfileListService = create_service<phoxi_camera_msgs::srv::GetProfileList>(
            "~/profiles/list", std::bind(&PhoXiCamera::getProfileListCallback, this, std::placeholders::_1, std::placeholders::_2));
    mGetActiveProfileService = create_service<phoxi_camera_msgs::srv::GetActiveProfile>(
            "~/profiles/get_active", std::bind(&PhoXiCamera::getActiveProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mSetActiveProfileService = create_service<phoxi_camera_msgs::srv::SetActiveProfile>(
            "~/profiles/set_active", std::bind(&PhoXiCamera::setActiveProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mCreateProfileService = create_service<phoxi_camera_msgs::srv::CreateProfile>(
            "~/profiles/create", std::bind(&PhoXiCamera::createProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mDeleteProfileService = create_service<phoxi_camera_msgs::srv::DeleteProfile>(
            "~/profiles/delete", std::bind(&PhoXiCamera::deleteProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mUpdateProfileService = create_service<phoxi_camera_msgs::srv::UpdateProfile>(
            "~/profiles/update", std::bind(&PhoXiCamera::updateProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mGetStartupProfileService = create_service<phoxi_camera_msgs::srv::GetStartupProfile>(
            "~/profiles/get_startup", std::bind(&PhoXiCamera::getStartupProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mSetStartupProfileService = create_service<phoxi_camera_msgs::srv::SetStartupProfile>(
            "~/profiles/set_startup", std::bind(&PhoXiCamera::setStartupProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mExportProfileService = create_service<phoxi_camera_msgs::srv::ExportProfile>(
            "~/profiles/export", std::bind(&PhoXiCamera::exportProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mImportProfileService = create_service<phoxi_camera_msgs::srv::ImportProfile>(
            "~/profiles/import", std::bind(&PhoXiCamera::importProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mResetActiveProfileService =
            create_service<std_srvs::srv::Trigger>("~/profiles/reset", std::bind(&PhoXiCamera::resetActiveProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Configuration complete. Connected to device: %s", mDeviceId.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhoXiCamera::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Activating PhoXi Camera node...");

    activatePublishers();

    try {
        mPhoXiInterface->startAcquisition();
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Activation failed: %s", e.what());
        deactivatePublishers();
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Activation complete.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhoXiCamera::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Deactivating PhoXi Camera node...");

    try {
        mPhoXiInterface->stopAcquisition();
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_WARN(get_logger(), "Failed to stop acquisition during deactivation: %s", e.what());
    }
    drainFrameCallback();

    deactivatePublishers();

    RCLCPP_INFO(get_logger(), "Deactivation complete.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhoXiCamera::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up PhoXi Camera node...");

    mSettingDescriptors.clear();
    mParamToDescriptor.clear();

    try {
        mPhoXiInterface->disconnectCamera();
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_WARN(get_logger(), "Failed to disconnect during cleanup: %s", e.what());
    }
    drainFrameCallback();

    cleanupResources();
    RCLCPP_INFO(get_logger(), "Cleanup complete.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhoXiCamera::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Shutting down PhoXi Camera node...");

    try {
        if (mPhoXiInterface->isAcquiring()) {
            mPhoXiInterface->stopAcquisition();
        }
    } catch (...) {
    }
    if (mPhoXiInterface->isConnected()) {
        try {
            mPhoXiInterface->disconnectCamera();
        } catch (const PhoXiInterfaceException& e) {
            RCLCPP_WARN(get_logger(), "Failed to disconnect during shutdown: %s", e.what());
        }
    }

    drainFrameCallback();
    cleanupResources();
    RCLCPP_INFO(get_logger(), "Shutdown complete.");
    return CallbackReturn::SUCCESS;
}

void PhoXiCamera::drainFrameCallback() {
    std::lock_guard<std::mutex> lock(mFrameMutex);
}

void PhoXiCamera::declareParameters() {
    declare_parameter<std::string>("device_id", mDeviceId);
    declare_parameter<std::string>("frame_id", "phoxi_camera_sensor");
    declare_parameter<bool>("publish_combined", false);
    rcl_interfaces::msg::ParameterDescriptor dynamicDesc;
    dynamicDesc.dynamic_typing = true;
    for (const auto& componentName : COMPONENTS) {
        declare_parameter(std::string(FRAME_SETTINGS_PREFIX) + PARAM_SEP + std::string(componentName), rclcpp::ParameterValue{}, dynamicDesc);
    }

    mParamCallbackHandle = add_on_set_parameters_callback(std::bind(&PhoXiCamera::onParametersChanged, this, std::placeholders::_1));

    mShutdownCallbackHandle = get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() { rclcpp_lifecycle::LifecycleNode::shutdown(); });
}

void PhoXiCamera::loadDeviceSettingDescriptors() {
    mSettingDescriptors.clear();
    mParamToDescriptor.clear();

    const auto schema = mPhoXiInterface->getSettingInfos();

    for (const auto& info : schema) {
        const size_t idx = mSettingDescriptors.size();
        mSettingDescriptors.push_back({info.key, info.type, info.isSettable});

        const std::string base = std::string(DEVICE_SETTINGS_PREFIX) + PARAM_SEP + deviceKeyToParamName(info.key);

        switch (info.type) {
            case SettingValueType::PHOXI_SIZE:
            case SettingValueType::PHOXI_SIZE_64F:
                mParamToDescriptor[base + PARAM_SEP + "width"] = {idx, "width"};
                mParamToDescriptor[base + PARAM_SEP + "height"] = {idx, "height"};
                break;
            case SettingValueType::PHOXI_2DROI:
                mParamToDescriptor[base + PARAM_SEP + "x_min"] = {idx, "x_min"};
                mParamToDescriptor[base + PARAM_SEP + "x_max"] = {idx, "x_max"};
                mParamToDescriptor[base + PARAM_SEP + "y_min"] = {idx, "y_min"};
                mParamToDescriptor[base + PARAM_SEP + "y_max"] = {idx, "y_max"};
                break;
            case SettingValueType::AXIS_VOLUME_64F:
                mParamToDescriptor[base + PARAM_SEP + "x_min"] = {idx, "x_min"};
                mParamToDescriptor[base + PARAM_SEP + "x_max"] = {idx, "x_max"};
                mParamToDescriptor[base + PARAM_SEP + "y_min"] = {idx, "y_min"};
                mParamToDescriptor[base + PARAM_SEP + "y_max"] = {idx, "y_max"};
                mParamToDescriptor[base + PARAM_SEP + "z_min"] = {idx, "z_min"};
                mParamToDescriptor[base + PARAM_SEP + "z_max"] = {idx, "z_max"};
                break;
            case SettingValueType::POINT3_64F:
                mParamToDescriptor[base + PARAM_SEP + "r"] = {idx, "r"};
                mParamToDescriptor[base + PARAM_SEP + "g"] = {idx, "g"};
                mParamToDescriptor[base + PARAM_SEP + "b"] = {idx, "b"};
                break;
            case SettingValueType::SCANNING_VOLUME:
                mParamToDescriptor[base + PARAM_SEP + "origin"] = {idx, "origin"};
                mParamToDescriptor[base + PARAM_SEP + "top_left"] = {idx, "top_left"};
                mParamToDescriptor[base + PARAM_SEP + "top_right"] = {idx, "top_right"};
                mParamToDescriptor[base + PARAM_SEP + "bottom_left"] = {idx, "bottom_left"};
                mParamToDescriptor[base + PARAM_SEP + "bottom_right"] = {idx, "bottom_right"};
                mParamToDescriptor[base + PARAM_SEP + "top_contour"] = {idx, "top_contour"};
                mParamToDescriptor[base + PARAM_SEP + "bottom_contour"] = {idx, "bottom_contour"};
                break;
            case SettingValueType::SCANNING_VOLUME_MESH:
                mParamToDescriptor[base + PARAM_SEP + "points_per_section"] = {idx, "points_per_section"};
                mParamToDescriptor[base + PARAM_SEP + "vertices"] = {idx, "vertices"};
                mParamToDescriptor[base + PARAM_SEP + "indices"] = {idx, "indices"};
                break;
            case SettingValueType::REPROJECTION_MAP:
                mParamToDescriptor[base + PARAM_SEP + "width"] = {idx, "width"};
                mParamToDescriptor[base + PARAM_SEP + "height"] = {idx, "height"};
                mParamToDescriptor[base + PARAM_SEP + "cv_type"] = {idx, "cv_type"};
                break;
            default:
                mParamToDescriptor[base] = {idx, ""};
                break;
        }
    }
}

void PhoXiCamera::declareSettingParams(const SettingDescriptor& desc, const std::string& base, const SettingValue& devVal) {
    auto declP = [this](const std::string& name, rclcpp::ParameterValue v) {
        declare_parameter(name, std::move(v));
    };

    switch (desc.type) {
        case SettingValueType::BOOL:
            declP(base, rclcpp::ParameterValue(std::get<bool>(devVal)));
            break;
        case SettingValueType::INT:
            declP(base, rclcpp::ParameterValue(std::get<int64_t>(devVal)));
            break;
        case SettingValueType::DOUBLE:
            declP(base, rclcpp::ParameterValue(std::get<double>(devVal)));
            break;
        case SettingValueType::STRING:
            declP(base, rclcpp::ParameterValue(std::get<std::string>(devVal)));
            break;
        case SettingValueType::DOUBLE_ARRAY:
        case SettingValueType::CUTTING_PLANES: {
            std::vector<double> arr;
            if (desc.type == SettingValueType::CUTTING_PLANES) {
                const auto& planes = std::get<std::vector<pho::api::Plane_64f>>(devVal);
                for (const auto& p : planes) {
                    arr.push_back(static_cast<double>(p.normal.x));
                    arr.push_back(static_cast<double>(p.normal.y));
                    arr.push_back(static_cast<double>(p.normal.z));
                    arr.push_back(static_cast<double>(p.d));
                }
            } else {
                arr = std::get<std::vector<double>>(devVal);
            }
            declP(base, rclcpp::ParameterValue(arr));
            break;
        }
        case SettingValueType::PHOXI_SIZE: {
            const auto& s = std::get<pho::api::PhoXiSize>(devVal);
            declP(base + PARAM_SEP + "width", rclcpp::ParameterValue((int64_t)s.Width));
            declP(base + PARAM_SEP + "height", rclcpp::ParameterValue((int64_t)s.Height));
            break;
        }
        case SettingValueType::PHOXI_SIZE_64F: {
            const auto& s = std::get<pho::api::PhoXiSize_64f>(devVal);
            declP(base + PARAM_SEP + "width", rclcpp::ParameterValue((double)s.Width));
            declP(base + PARAM_SEP + "height", rclcpp::ParameterValue((double)s.Height));
            break;
        }
        case SettingValueType::PHOXI_2DROI: {
            const auto& roi = std::get<pho::api::PhoXi2DROI>(devVal);
            declP(base + PARAM_SEP + "x_min", rclcpp::ParameterValue((int64_t)(int32_t)roi.Min.x));
            declP(base + PARAM_SEP + "y_min", rclcpp::ParameterValue((int64_t)(int32_t)roi.Min.y));
            declP(base + PARAM_SEP + "x_max", rclcpp::ParameterValue((int64_t)(int32_t)roi.Max.x));
            declP(base + PARAM_SEP + "y_max", rclcpp::ParameterValue((int64_t)(int32_t)roi.Max.y));
            break;
        }
        case SettingValueType::AXIS_VOLUME_64F: {
            const auto& vol = std::get<pho::api::AxisVolume_64f>(devVal);
            declP(base + PARAM_SEP + "x_min", rclcpp::ParameterValue((double)vol.min.x));
            declP(base + PARAM_SEP + "y_min", rclcpp::ParameterValue((double)vol.min.y));
            declP(base + PARAM_SEP + "z_min", rclcpp::ParameterValue((double)vol.min.z));
            declP(base + PARAM_SEP + "x_max", rclcpp::ParameterValue((double)vol.max.x));
            declP(base + PARAM_SEP + "y_max", rclcpp::ParameterValue((double)vol.max.y));
            declP(base + PARAM_SEP + "z_max", rclcpp::ParameterValue((double)vol.max.z));
            break;
        }
        case SettingValueType::POINT3_64F: {
            const auto& p = std::get<pho::api::Point3_64f>(devVal);
            declP(base + PARAM_SEP + "r", rclcpp::ParameterValue((double)p.x));
            declP(base + PARAM_SEP + "g", rclcpp::ParameterValue((double)p.y));
            declP(base + PARAM_SEP + "b", rclcpp::ParameterValue((double)p.z));
            break;
        }
        case SettingValueType::SCANNING_VOLUME: {
            const auto& geom = std::get<pho::api::ProjectionGeometry_64f>(devVal);
            auto toArr = [](const pho::api::Point3_64f& pt) -> std::vector<double> { return {(double)pt.x, (double)pt.y, (double)pt.z}; };
            auto toContourArr = [&toArr](const std::vector<pho::api::Point3_64f>& pts) {
                std::vector<double> arr;
                for (const auto& pt : pts) {
                    arr.push_back((double)pt.x);
                    arr.push_back((double)pt.y);
                    arr.push_back((double)pt.z);
                }
                return arr;
            };
            declP(base + PARAM_SEP + "origin", rclcpp::ParameterValue(toArr(geom.Origin)));
            declP(base + PARAM_SEP + "top_left", rclcpp::ParameterValue(toArr(geom.TopLeftTangentialVector)));
            declP(base + PARAM_SEP + "top_right", rclcpp::ParameterValue(toArr(geom.TopRightTangentialVector)));
            declP(base + PARAM_SEP + "bottom_left", rclcpp::ParameterValue(toArr(geom.BottomLeftTangentialVector)));
            declP(base + PARAM_SEP + "bottom_right", rclcpp::ParameterValue(toArr(geom.BottomRightTangentialVector)));
            declP(base + PARAM_SEP + "top_contour", rclcpp::ParameterValue(toContourArr(geom.TopContourPoints)));
            declP(base + PARAM_SEP + "bottom_contour", rclcpp::ParameterValue(toContourArr(geom.BottomContourPoints)));
            break;
        }
        case SettingValueType::SCANNING_VOLUME_MESH: {
            const auto& mesh = std::get<pho::api::PhoXiMesh>(devVal);
            std::vector<double> verts;
            for (const auto& pt : mesh.Vertices) {
                verts.push_back((double)pt.x);
                verts.push_back((double)pt.y);
                verts.push_back((double)pt.z);
            }
            std::vector<int64_t> idxs;
            for (unsigned int i : mesh.Indices) {
                idxs.push_back((int64_t)i);
            }
            declP(base + PARAM_SEP + "points_per_section", rclcpp::ParameterValue((int64_t)mesh.PointsPerSection));
            declP(base + PARAM_SEP + "vertices", rclcpp::ParameterValue(verts));
            declP(base + PARAM_SEP + "indices", rclcpp::ParameterValue(idxs));
            break;
        }
        case SettingValueType::REPROJECTION_MAP: {
            const auto& reproj = std::get<pho::api::PhoXiReprojectionMap>(devVal);
            declP(base + PARAM_SEP + "width", rclcpp::ParameterValue((int64_t)reproj.Map.Size.Width));
            declP(base + PARAM_SEP + "height", rclcpp::ParameterValue((int64_t)reproj.Map.Size.Height));
            declP(base + PARAM_SEP + "cv_type", rclcpp::ParameterValue((int64_t)0));
            break;
        }
    }
}

void PhoXiCamera::declareDeviceInfoParameters() {
    const auto info = mPhoXiInterface->getDeviceInfo();
    const std::string prefix = std::string(DEVICE_INFO_PREFIX) + PARAM_SEP;

    rcl_interfaces::msg::ParameterDescriptor readOnly;
    readOnly.read_only = true;

    auto declare = [&](const std::string& name, const auto& value) {
        try {
            declare_parameter(prefix + name, value, readOnly);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
        }
    };

    declare("name", info.name);
    declare("type", static_cast<std::string>(info.type));
    declare("deviceId", info.hwIdentification);
    declare("ipAddress", info.ipAddress);
    declare("status", connectionStatusToString(info.status));
    declare("firmwareVersion", info.firmwareVersion);
    declare("variant", info.variant);
    declare("isAlpha", info.isAlpha);
    declare("isBlue", info.isBlue);
    declare("isColor", info.isColor);
    declare("isFileCam", info.isFileCam);
}

void PhoXiCamera::declareDeviceSettingParameters() {
    if (mSettingDescriptors.empty()) {
        return;
    }

    std::vector<std::string> allKeys;
    allKeys.reserve(mSettingDescriptors.size());
    for (const auto& desc : mSettingDescriptors) {
        allKeys.push_back(desc.deviceKey);
    }
    const auto deviceValues = mPhoXiInterface->getSettings(allKeys);

    mDeclaringDeviceSettings = true;

    for (size_t i = 0; i < mSettingDescriptors.size(); ++i) {
        const auto& desc = mSettingDescriptors[i];
        const auto it = deviceValues.find(desc.deviceKey);
        if (it == deviceValues.end()) {
            continue;
        }

        const std::string base = std::string(DEVICE_SETTINGS_PREFIX) + PARAM_SEP + deviceKeyToParamName(desc.deviceKey);
        try {
            declareSettingParams(desc, base, it->second);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Could not declare param for '%s': %s", desc.deviceKey.c_str(), e.what());
        }
    }

    mDeclaringDeviceSettings = false;

    std::vector<std::pair<std::string, SettingValue>> overrides;
    for (size_t i = 0; i < mSettingDescriptors.size(); ++i) {
        const auto& desc = mSettingDescriptors[i];
        if (!desc.isSettable) {
            continue;
        }

        const auto it = deviceValues.find(desc.deviceKey);
        if (it == deviceValues.end()) {
            continue;
        }

        const std::string base = std::string(DEVICE_SETTINGS_PREFIX) + PARAM_SEP + deviceKeyToParamName(desc.deviceKey);
        try {
            const auto currentVal = reconstructSettingValue(desc, base);
            if (!settingValuesEqual(currentVal, it->second)) {
                overrides.push_back({desc.deviceKey, currentVal});
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Could not compare setting '%s': %s", desc.deviceKey.c_str(), e.what());
        }
    }

    if (!overrides.empty()) {
        mPhoXiInterface->setSettings(overrides);
    }
}

SettingValue PhoXiCamera::reconstructSettingValue(const SettingDescriptor& desc, const std::string& base) const {
    switch (desc.type) {
        case SettingValueType::BOOL:
            return get_parameter(base).as_bool();
        case SettingValueType::INT:
            return get_parameter(base).as_int();
        case SettingValueType::DOUBLE:
            return get_parameter(base).as_double();
        case SettingValueType::STRING:
            return get_parameter(base).as_string();
        case SettingValueType::DOUBLE_ARRAY:
            return get_parameter(base).as_double_array();
        case SettingValueType::PHOXI_SIZE: {
            pho::api::PhoXiSize s;
            s.Width = (int32_t)get_parameter(base + PARAM_SEP + "width").as_int();
            s.Height = (int32_t)get_parameter(base + PARAM_SEP + "height").as_int();
            return s;
        }
        case SettingValueType::PHOXI_SIZE_64F: {
            pho::api::PhoXiSize_64f s;
            s.Width = get_parameter(base + PARAM_SEP + "width").as_double();
            s.Height = get_parameter(base + PARAM_SEP + "height").as_double();
            return s;
        }
        case SettingValueType::PHOXI_2DROI: {
            pho::api::PhoXi2DROI roi;
            roi.Min.x = (int32_t)get_parameter(base + PARAM_SEP + "x_min").as_int();
            roi.Min.y = (int32_t)get_parameter(base + PARAM_SEP + "y_min").as_int();
            roi.Max.x = (int32_t)get_parameter(base + PARAM_SEP + "x_max").as_int();
            roi.Max.y = (int32_t)get_parameter(base + PARAM_SEP + "y_max").as_int();
            return roi;
        }
        case SettingValueType::AXIS_VOLUME_64F: {
            pho::api::AxisVolume_64f vol;
            vol.min.x = get_parameter(base + PARAM_SEP + "x_min").as_double();
            vol.min.y = get_parameter(base + PARAM_SEP + "y_min").as_double();
            vol.min.z = get_parameter(base + PARAM_SEP + "z_min").as_double();
            vol.max.x = get_parameter(base + PARAM_SEP + "x_max").as_double();
            vol.max.y = get_parameter(base + PARAM_SEP + "y_max").as_double();
            vol.max.z = get_parameter(base + PARAM_SEP + "z_max").as_double();
            return vol;
        }
        case SettingValueType::POINT3_64F: {
            pho::api::Point3_64f p;
            p.x = get_parameter(base + PARAM_SEP + "r").as_double();
            p.y = get_parameter(base + PARAM_SEP + "g").as_double();
            p.z = get_parameter(base + PARAM_SEP + "b").as_double();
            return p;
        }
        default:
            throw PhoXiInterfaceException("reconstructSettingValue: unhandled type for '" + base + "'");
    }
}

SettingValue PhoXiCamera::paramToSettingValue(SettingValueType type, const rclcpp::ParameterValue& pv) const {
    switch (type) {
        case SettingValueType::BOOL:
            return pv.get<bool>();
        case SettingValueType::INT:
            return pv.get<int64_t>();
        case SettingValueType::DOUBLE:
            return pv.get<double>();
        case SettingValueType::STRING:
            return pv.get<std::string>();
        case SettingValueType::DOUBLE_ARRAY:
            return pv.get<std::vector<double>>();
        default:
            throw PhoXiInterfaceException("paramToSettingValue: not a simple type");
    }
}

SettingValue PhoXiCamera::applyFieldUpdate(const SettingValue& current, SettingValueType type, const std::string& field, const rclcpp::Parameter& param) const {
    switch (type) {
        case SettingValueType::PHOXI_SIZE: {
            auto s = std::get<pho::api::PhoXiSize>(current);
            if (field == "width") {
                s.Width = (int32_t)param.as_int();
            }
            if (field == "height") {
                s.Height = (int32_t)param.as_int();
            }
            return s;
        }
        case SettingValueType::PHOXI_SIZE_64F: {
            auto s = std::get<pho::api::PhoXiSize_64f>(current);
            if (field == "width") {
                s.Width = param.as_double();
            }
            if (field == "height") {
                s.Height = param.as_double();
            }
            return s;
        }
        case SettingValueType::PHOXI_2DROI: {
            auto roi = std::get<pho::api::PhoXi2DROI>(current);
            if (field == "x_min") {
                roi.Min.x = (int32_t)param.as_int();
            }
            if (field == "y_min") {
                roi.Min.y = (int32_t)param.as_int();
            }
            if (field == "x_max") {
                roi.Max.x = (int32_t)param.as_int();
            }
            if (field == "y_max") {
                roi.Max.y = (int32_t)param.as_int();
            }
            return roi;
        }
        case SettingValueType::AXIS_VOLUME_64F: {
            auto vol = std::get<pho::api::AxisVolume_64f>(current);
            if (field == "x_min") {
                vol.min.x = param.as_double();
            }
            if (field == "y_min") {
                vol.min.y = param.as_double();
            }
            if (field == "z_min") {
                vol.min.z = param.as_double();
            }
            if (field == "x_max") {
                vol.max.x = param.as_double();
            }
            if (field == "y_max") {
                vol.max.y = param.as_double();
            }
            if (field == "z_max") {
                vol.max.z = param.as_double();
            }
            return vol;
        }
        case SettingValueType::POINT3_64F: {
            auto p = std::get<pho::api::Point3_64f>(current);
            if (field == "r") {
                p.x = param.as_double();
            }
            if (field == "g") {
                p.y = param.as_double();
            }
            if (field == "b") {
                p.z = param.as_double();
            }
            return p;
        }
        default:
            return current;
    }
}

rcl_interfaces::msg::SetParametersResult PhoXiCamera::onParametersChanged(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    if (mDeclaringDeviceSettings) {
        return result;
    }

    const std::string framePfx = std::string(FRAME_SETTINGS_PREFIX) + PARAM_SEP;
    const std::string devPfx = std::string(DEVICE_SETTINGS_PREFIX) + PARAM_SEP;

    std::vector<std::pair<std::string, bool>> components;
    for (const auto& p : params) {
        const auto& name = p.get_name();
        if (name.rfind(framePfx, 0) == 0) {
            components.emplace_back(name.substr(framePfx.size()), p.as_bool());
        }
    }

    if (!components.empty() && mPhoXiInterface->isConnected()) {
        try {
            mPhoXiInterface->setFrameOutputSettings(components);
        } catch (const PhoXiInterfaceException& e) {
            result.successful = false;
            result.reason = e.what();
            return result;
        }
    }

    std::map<size_t, std::vector<const rclcpp::Parameter*>> deviceChanges;
    for (const auto& p : params) {
        const auto& name = p.get_name();
        if (name.rfind(devPfx, 0) != 0) {
            continue;
        }

        const auto it = mParamToDescriptor.find(name);
        if (it == mParamToDescriptor.end()) {
            result.successful = false;
            result.reason = "Unknown device setting parameter: '" + name + "'";
            return result;
        }
        deviceChanges[it->second.first].push_back(&p);
    }

    if (deviceChanges.empty()) {
        return result;
    }

    for (const auto& [descIdx, _] : deviceChanges) {
        const auto& desc = mSettingDescriptors[descIdx];
        if (!desc.isSettable) {
            result.successful = false;
            result.reason = "'" + desc.deviceKey + "' is a read-only device setting";
            return result;
        }
    }

    if (!mPhoXiInterface->isConnected()) {
        return result; // params updated in ROS2 but no device call
    }

    std::vector<std::pair<std::string, SettingValue>> toSet;
    for (const auto& [descIdx, paramPtrs] : deviceChanges) {
        const auto& desc = mSettingDescriptors[descIdx];
        const auto& firstParam = *paramPtrs[0];
        const auto& fieldName = mParamToDescriptor.at(firstParam.get_name()).second;

        try {
            if (fieldName.empty()) {
                toSet.push_back({desc.deviceKey, paramToSettingValue(desc.type, firstParam.get_parameter_value())});
            } else {
                auto current = mPhoXiInterface->getSetting(desc.deviceKey);
                for (const auto* pPtr : paramPtrs) {
                    const auto& f = mParamToDescriptor.at(pPtr->get_name()).second;
                    current = applyFieldUpdate(current, desc.type, f, *pPtr);
                }
                toSet.push_back({desc.deviceKey, current});
            }
        } catch (const std::exception& e) {
            result.successful = false;
            result.reason = std::string("Failed to prepare setting '") + desc.deviceKey + "': " + e.what();
            return result;
        }
    }

    try {
        mPhoXiInterface->setSettings(toSet);
    } catch (const PhoXiInterfaceException& e) {
        result.successful = false;
        result.reason = e.what();
    }

    return result;
}

void PhoXiCamera::activatePublishers() {
    mFrameInfoPub->on_activate();
    mPrimaryCameraInfoPub->on_activate();
    mColorCameraInfoPub->on_activate();
    mColorCameraImagePub->on_activate();
    if (mPublishCombined) {
        mPointCloudPub->on_activate();
    } else {
        mPointsPub->on_activate();
        mNormalMapPub->on_activate();
        mDepthMapPub->on_activate();
        mConfidenceMapPub->on_activate();
        mEventMapPub->on_activate();
        mTexturePub->on_activate();
        mTextureRgbPub->on_activate();
    }
}

void PhoXiCamera::deactivatePublishers() {
    mFrameInfoPub->on_deactivate();
    mPrimaryCameraInfoPub->on_deactivate();
    mColorCameraInfoPub->on_deactivate();
    mColorCameraImagePub->on_deactivate();
    if (mPublishCombined) {
        mPointCloudPub->on_deactivate();
    } else {
        mPointsPub->on_deactivate();
        mNormalMapPub->on_deactivate();
        mDepthMapPub->on_deactivate();
        mConfidenceMapPub->on_deactivate();
        mEventMapPub->on_deactivate();
        mTexturePub->on_deactivate();
        mTextureRgbPub->on_deactivate();
    }
}

void PhoXiCamera::cleanupResources() {
    mFrameInfoPub.reset();
    mPrimaryCameraInfoPub.reset();
    mColorCameraInfoPub.reset();
    mPointCloudPub.reset();
    mPointsPub.reset();
    mNormalMapPub.reset();
    mDepthMapPub.reset();
    mConfidenceMapPub.reset();
    mEventMapPub.reset();
    mTexturePub.reset();
    mTextureRgbPub.reset();
    mColorCameraImagePub.reset();

    mConnectService.reset();
    mDisconnectService.reset();
    mTriggerFrameService.reset();
    mGetProfileListService.reset();
    mGetActiveProfileService.reset();
    mSetActiveProfileService.reset();
    mCreateProfileService.reset();
    mDeleteProfileService.reset();
    mUpdateProfileService.reset();
    mGetStartupProfileService.reset();
    mSetStartupProfileService.reset();
    mExportProfileService.reset();
    mImportProfileService.reset();
    mResetActiveProfileService.reset();
}

void PhoXiCamera::onFrameCallback(const PhoXiFrame& frame) {
    std::lock_guard<std::mutex> lock(mFrameMutex);
    try {
        const rclcpp::Time rosStamp = get_clock()->now();

        auto setHeader = [&](sensor_msgs::msg::Image& img) {
            img.header.frame_id = mFrameId;
            img.header.stamp = rosStamp;
        };

        auto shouldPublish = [](const auto& pub) { return pub->is_activated() && pub->get_subscription_count() > 0; };

        if (frame.pointCloud && shouldPublish(mPointCloudPub)) {
            auto msg = phoXiFrameToRosMsg(frame);
            msg->header.frame_id = mFrameId;
            msg->header.stamp = rosStamp;
            mPointCloudPub->publish(std::move(msg));
        }

        if (frame.pointCloud && shouldPublish(mPointsPub)) {
            auto msg = pointsToRosMsg(frame);
            msg->header.frame_id = mFrameId;
            msg->header.stamp = rosStamp;
            mPointsPub->publish(std::move(msg));
        }

        if (frame.normalMap && shouldPublish(mNormalMapPub)) {
            auto img = normalMapToRosMsg(*frame.normalMap);
            setHeader(*img);
            mNormalMapPub->publish(std::move(img));
        }

        if (frame.depthMap && shouldPublish(mDepthMapPub)) {
            auto img = depthMapToRosMsg(*frame.depthMap);
            setHeader(*img);
            mDepthMapPub->publish(std::move(img));
        }

        if (frame.confidenceMap && shouldPublish(mConfidenceMapPub)) {
            auto img = confidenceMapToRosMsg(*frame.confidenceMap);
            setHeader(*img);
            mConfidenceMapPub->publish(std::move(img));
        }

        if (frame.eventMap && shouldPublish(mEventMapPub)) {
            auto img = eventMapToRosMsg(*frame.eventMap);
            setHeader(*img);
            mEventMapPub->publish(std::move(img));
        }

        if (frame.texture && shouldPublish(mTexturePub)) {
            auto img = textureToRosMsg(*frame.texture);
            setHeader(*img);
            mTexturePub->publish(std::move(img));
        }

        if (frame.textureRgb && shouldPublish(mTextureRgbPub)) {
            auto img = textureRgbToRosMsg(*frame.textureRgb);
            setHeader(*img);
            mTextureRgbPub->publish(std::move(img));
        }

        if (frame.colorCamera && shouldPublish(mColorCameraImagePub)) {
            auto img = colorCameraImageToRosMsg(*frame.colorCamera);
            setHeader(*img);
            mColorCameraImagePub->publish(std::move(img));
        }

        if (frame.frameInfo) {
            const bool pubFrameInfo = shouldPublish(mFrameInfoPub);
            const bool pubPrimaryCamera = shouldPublish(mPrimaryCameraInfoPub);
            const bool pubColorCamera = shouldPublish(mColorCameraInfoPub);
            if (pubFrameInfo || pubPrimaryCamera || pubColorCamera) {
                auto parsed = parseFrameInfo(*frame.frameInfo);
                if (parsed.frameInfo && pubFrameInfo) {
                    parsed.frameInfo->header.frame_id = mFrameId;
                    parsed.frameInfo->header.stamp = rosStamp;
                    mFrameInfoPub->publish(std::move(parsed.frameInfo));
                }
                if (parsed.currentCamera && pubPrimaryCamera) {
                    parsed.currentCamera->header.frame_id = mFrameId;
                    parsed.currentCamera->header.stamp = rosStamp;
                    mPrimaryCameraInfoPub->publish(std::move(parsed.currentCamera));
                }
                if (parsed.currentColorCamera && pubColorCamera) {
                    parsed.currentColorCamera->header.frame_id = mFrameId;
                    parsed.currentColorCamera->header.stamp = rosStamp;
                    mColorCameraInfoPub->publish(std::move(parsed.currentColorCamera));
                }
            }
        }

        RCLCPP_INFO(get_logger(), "Frame published successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to publish frame: %s", e.what());
    }
}

void PhoXiCamera::connectCallback(
        const std::shared_ptr<const phoxi_camera_msgs::srv::Connect::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::Connect::Response>& response) {
    auto stateId = get_current_state().id();
    if (stateId != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE && stateId != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_WARN(get_logger(), "Node must be in active or inactive state to connect.");
        response->success = false;
        response->message = "Node is not in active or inactive state.";
        return;
    }

    std::string snToUse = request->sn.empty() ? mDeviceId : request->sn;
    if (snToUse.empty()) {
        RCLCPP_ERROR(get_logger(), "Connection failed: No device ID provided.");
        response->success = false;
        response->message = "No device ID provided.";
        return;
    }

    RCLCPP_INFO(get_logger(), "Service call to connect to device: %s", snToUse.c_str());

    bool isActive = (stateId == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    if (isActive) {
        try {
            mPhoXiInterface->stopAcquisition();
        } catch (...) {
        }
        drainFrameCallback();
    }
    try {
        mPhoXiInterface->connectCamera(snToUse, std::bind(&PhoXiCamera::onFrameCallback, this, std::placeholders::_1));
        if (isActive) {
            mPhoXiInterface->startAcquisition();
        }
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Connection failed: %s", e.what());
        response->success = false;
        response->message = e.what();
        return;
    }

    RCLCPP_INFO(get_logger(), "Successfully connected to device: %s", snToUse.c_str());
    response->success = true;
}

void PhoXiCamera::disconnectCallback(const std::shared_ptr<const std_srvs::srv::Trigger::Request>& /*request*/, const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    auto stateId = get_current_state().id();
    if (stateId != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE && stateId != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_WARN(get_logger(), "Node must be in active or inactive state to disconnect.");
        response->success = false;
        response->message = "Node is not in active or inactive state.";
        return;
    }

    RCLCPP_INFO(get_logger(), "Service call to disconnect from device.");
    try {
        mPhoXiInterface->disconnectCamera();
    } catch (const PhoXiInterfaceException&) {
        // device already disconnected
    }
    RCLCPP_INFO(get_logger(), "Device disconnected.");
    response->success = true;
}

void PhoXiCamera::triggerFrameCallback(
        const std::shared_ptr<const phoxi_camera_msgs::srv::TriggerFrame::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::TriggerFrame::Response>& response) {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(get_logger(), "Node is not active. Cannot trigger frame.");
        response->success = false;
        response->message = "Node is not in the active state.";
        return;
    }

    RCLCPP_INFO(get_logger(), "Triggering frame (wait_grabbing_end=%s).", request->wait_grabbing_end ? "true" : "false");
    try {
        mPhoXiInterface->triggerFrame(request->wait_grabbing_end);
        RCLCPP_INFO(get_logger(), "Frame triggered successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to trigger frame: %s", e.what());
        response->success = false;
        response->message = e.what();
        return;
    }
    response->success = true;
}

void PhoXiCamera::getProfileListCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::GetProfileList::Request>& /*request*/,
        const std::shared_ptr<phoxi_camera_msgs::srv::GetProfileList::Response>& response) {
    RCLCPP_INFO(get_logger(), "Getting profile list.");
    try {
        auto profiles = mPhoXiInterface->getProfileList();
        for (const auto& p : profiles) {
            response->names.push_back(p.Name);
            response->is_factory.push_back(p.IsFactory);
        }
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to get profile list: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::getActiveProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::GetActiveProfile::Request>& /*request*/,
        const std::shared_ptr<phoxi_camera_msgs::srv::GetActiveProfile::Response>& response) {
    RCLCPP_INFO(get_logger(), "Getting active profile.");
    try {
        response->name = mPhoXiInterface->getActiveProfile();
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to get active profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::setActiveProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::SetActiveProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::SetActiveProfile::Response>& response) {
    RCLCPP_INFO(get_logger(), "Setting active profile to '%s'.", request->name.c_str());
    try {
        mPhoXiInterface->setActiveProfile(request->name);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to set active profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::createProfileCallback(
        const std::shared_ptr<const phoxi_camera_msgs::srv::CreateProfile::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::CreateProfile::Response>& response) {
    RCLCPP_INFO(get_logger(), "Creating profile '%s'.", request->name.c_str());
    try {
        mPhoXiInterface->createProfile(request->name);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to create profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::deleteProfileCallback(
        const std::shared_ptr<const phoxi_camera_msgs::srv::DeleteProfile::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::DeleteProfile::Response>& response) {
    RCLCPP_INFO(get_logger(), "Deleting profile '%s'.", request->name.c_str());
    try {
        mPhoXiInterface->deleteProfile(request->name);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to delete profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::updateProfileCallback(
        const std::shared_ptr<const phoxi_camera_msgs::srv::UpdateProfile::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::UpdateProfile::Response>& response) {
    RCLCPP_INFO(get_logger(), "Updating profile '%s'.", request->name.c_str());
    try {
        mPhoXiInterface->updateProfile(request->name);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to update profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::getStartupProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::GetStartupProfile::Request>& /*request*/,
        const std::shared_ptr<phoxi_camera_msgs::srv::GetStartupProfile::Response>& response) {
    RCLCPP_INFO(get_logger(), "Getting startup profile.");
    try {
        response->name = mPhoXiInterface->getStartupProfile();
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to get startup profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::setStartupProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::SetStartupProfile::Request>& request,
        const std::shared_ptr<phoxi_camera_msgs::srv::SetStartupProfile::Response>& response) {
    RCLCPP_INFO(get_logger(), "Setting startup profile to '%s'.", request->name.c_str());
    try {
        mPhoXiInterface->setStartupProfile(request->name);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to set startup profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::exportProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::ExportProfile::Request>& /*request*/,
        const std::shared_ptr<phoxi_camera_msgs::srv::ExportProfile::Response>& response) {
    RCLCPP_INFO(get_logger(), "Exporting active profile.");
    try {
        auto content = mPhoXiInterface->exportProfile();
        response->name = content.Name;
        response->content.assign(content.Content.begin(), content.Content.end());
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to export profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::importProfileCallback(
        const std::shared_ptr<const phoxi_camera_msgs::srv::ImportProfile::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::ImportProfile::Response>& response) {
    RCLCPP_INFO(get_logger(), "Importing profile '%s'.", request->name.c_str());
    try {
        pho::api::PhoXiProfileContent content;
        content.Name = request->name;
        content.Content.assign(request->content.begin(), request->content.end());
        mPhoXiInterface->importProfile(content);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to import profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::resetActiveProfileCallback(
        const std::shared_ptr<const std_srvs::srv::Trigger::Request>& /*request*/, const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    RCLCPP_INFO(get_logger(), "Resetting active profile to factory defaults.");
    try {
        mPhoXiInterface->resetActiveProfile();
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Failed to reset active profile: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}
} // namespace phoxi_camera

RCLCPP_COMPONENTS_REGISTER_NODE(phoxi_camera::PhoXiCamera)