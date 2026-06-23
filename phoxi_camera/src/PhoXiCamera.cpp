#include "phoxi_camera/PhoXiCamera.h"

#include <array>
#include <chrono>
#include <optional>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/RosConversions.h"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace phoxi_camera {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static constexpr char PARAM_SEP = '.';
static constexpr std::string_view DEVICE_SETTINGS_PREFIX = "device_settings";
static constexpr std::string_view DEVICE_INFO_PREFIX = "device_info";
static constexpr std::string_view FRAME_SETTINGS_PREFIX = "frame_settings";

static pho::api::PhoXiTriggerMode parseTriggerMode(const std::string& modeStr) {
    if (modeStr == "Software") {
        return pho::api::PhoXiTriggerMode::Software;
    }
    if (modeStr == "Freerun") {
        return pho::api::PhoXiTriggerMode::Freerun;
    }
    throw InvalidTriggerMode("Invalid trigger_mode '" + modeStr + "'. Expected 'Software' or 'Freerun'.");
}

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
    get_parameter("logout_on_exit", mLogoutOnExit);
    get_parameter("stop_acquisition_on_exit", mStopAcquisitionOnExit);
}

PhoXiCamera::PhoXiCamera(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("phoxi_camera", options), mPhoXiInterface(std::make_unique<PhoXiInterface>()) {
    RCLCPP_INFO(get_logger(), "Creating PhoXi Camera node.");
    declareParameters();
    get_parameter("device_id", mDeviceId);
    get_parameter("logout_on_exit", mLogoutOnExit);
    get_parameter("stop_acquisition_on_exit", mStopAcquisitionOnExit);
}

PhoXiCamera::~PhoXiCamera() {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(mShutdownCallbackHandle);
    if (mPhoXiInterface->isConnected()) {
        mPhoXiInterface->disconnectCamera(mLogoutOnExit, mStopAcquisitionOnExit);
    }
}

CallbackReturn PhoXiCamera::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Configuring PhoXi Camera node...");

    {
        std::lock_guard<std::mutex> lock(mDeviceIdMutex);
        get_parameter("device_id", mDeviceId);
    }
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

    get_parameter("trigger_mode", mTriggerMode);
    try {
        mPhoXiInterface->setTriggerMode(parseTriggerMode(mTriggerMode));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed (trigger_mode): %s", e.what());
        try {
            mPhoXiInterface->disconnectCamera(mLogoutOnExit, mStopAcquisitionOnExit);
        } catch (...) {
        }
        return CallbackReturn::FAILURE;
    }

    try {
        declareDeviceInfoParameters();
        loadDeviceSettingDescriptors();
        declareDeviceSettingParameters();
        declareFrameSettingParameters();
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        try {
            mPhoXiInterface->disconnectCamera(mLogoutOnExit, mStopAcquisitionOnExit);
        } catch (...) {
        }
        return CallbackReturn::FAILURE;
    }

    mFrameErrorPub = create_publisher<phoxi_camera_msgs::msg::FrameError>("frame_error", rclcpp::SystemDefaultsQoS());
    mFrameInfoPub = create_publisher<phoxi_camera_msgs::msg::FrameInfo>("frame_info", rclcpp::SystemDefaultsQoS());
    mPrimaryCameraInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>("frame_info/current_camera", rclcpp::SystemDefaultsQoS());
    mColorCameraInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>("frame_info/current_color_camera", rclcpp::SystemDefaultsQoS());
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
        mPhoXiInterface->disconnectCamera(mLogoutOnExit, mStopAcquisitionOnExit);
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
            mPhoXiInterface->disconnectCamera(mLogoutOnExit, mStopAcquisitionOnExit);
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
    declare_parameter<std::string>("trigger_mode", mTriggerMode);
    declare_parameter<bool>("logout_on_exit", mLogoutOnExit);
    declare_parameter<bool>("stop_acquisition_on_exit", mStopAcquisitionOnExit);

    mParamCallbackHandle = add_on_set_parameters_callback(std::bind(&PhoXiCamera::onParametersChanged, this, std::placeholders::_1));

    mShutdownCallbackHandle = get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() {
        rclcpp_lifecycle::LifecycleNode::shutdown();
    });

    mRebootService = create_service<std_srvs::srv::Trigger>("~/reboot", std::bind(&PhoXiCamera::rebootCallback, this, std::placeholders::_1, std::placeholders::_2));
    mShutdownService = create_service<std_srvs::srv::Trigger>("~/shutdown", std::bind(&PhoXiCamera::shutdownCallback, this, std::placeholders::_1, std::placeholders::_2));
    mFactoryResetService =
            create_service<std_srvs::srv::Trigger>("~/factory_reset", std::bind(&PhoXiCamera::factoryResetCallback, this, std::placeholders::_1, std::placeholders::_2));
    mLogDownloadService =
            create_service<phoxi_camera_msgs::srv::LogDownload>("~/log_download", std::bind(&PhoXiCamera::logDownloadCallback, this, std::placeholders::_1, std::placeholders::_2));
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

bool PhoXiCamera::declareSettingParam(const SettingDescriptor& desc, const std::string& base, const SettingValue& devVal) {
    auto declP = [this](const std::string& name, const rclcpp::ParameterValue& v) {
        const auto& a = declare_parameter(name, v);
        return a == v;
    };

    bool r = true;
    switch (desc.type) {
        case SettingValueType::BOOL:
            return declP(base, rclcpp::ParameterValue(std::get<bool>(devVal)));
        case SettingValueType::INT:
            return declP(base, rclcpp::ParameterValue(std::get<int64_t>(devVal)));
        case SettingValueType::DOUBLE:
            return declP(base, rclcpp::ParameterValue(std::get<double>(devVal)));
        case SettingValueType::STRING:
            return declP(base, rclcpp::ParameterValue(std::get<std::string>(devVal)));
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
            return declP(base, rclcpp::ParameterValue(arr));
        }
        case SettingValueType::PHOXI_SIZE: {
            const auto& s = std::get<pho::api::PhoXiSize>(devVal);
            r &= declP(base + PARAM_SEP + "width", rclcpp::ParameterValue(static_cast<int64_t>(s.Width)));
            r &= declP(base + PARAM_SEP + "height", rclcpp::ParameterValue(static_cast<int64_t>(s.Height)));
            return r;
        }
        case SettingValueType::PHOXI_SIZE_64F: {
            const auto& s = std::get<pho::api::PhoXiSize_64f>(devVal);
            r &= declP(base + PARAM_SEP + "width", rclcpp::ParameterValue(s.Width));
            r &= declP(base + PARAM_SEP + "height", rclcpp::ParameterValue(s.Height));
            return r;
        }
        case SettingValueType::PHOXI_2DROI: {
            const auto& roi = std::get<pho::api::PhoXi2DROI>(devVal);
            r &= declP(base + PARAM_SEP + "x_min", rclcpp::ParameterValue(static_cast<int64_t>(roi.Min.x)));
            r &= declP(base + PARAM_SEP + "y_min", rclcpp::ParameterValue(static_cast<int64_t>(roi.Min.y)));
            r &= declP(base + PARAM_SEP + "x_max", rclcpp::ParameterValue(static_cast<int64_t>(roi.Max.x)));
            r &= declP(base + PARAM_SEP + "y_max", rclcpp::ParameterValue(static_cast<int64_t>(roi.Max.y)));
            return r;
        }
        case SettingValueType::AXIS_VOLUME_64F: {
            const auto& vol = std::get<pho::api::AxisVolume_64f>(devVal);
            r &= declP(base + PARAM_SEP + "x_min", rclcpp::ParameterValue(vol.min.x));
            r &= declP(base + PARAM_SEP + "y_min", rclcpp::ParameterValue(vol.min.y));
            r &= declP(base + PARAM_SEP + "z_min", rclcpp::ParameterValue(vol.min.z));
            r &= declP(base + PARAM_SEP + "x_max", rclcpp::ParameterValue(vol.max.x));
            r &= declP(base + PARAM_SEP + "y_max", rclcpp::ParameterValue(vol.max.y));
            r &= declP(base + PARAM_SEP + "z_max", rclcpp::ParameterValue(vol.max.z));
            return r;
        }
        case SettingValueType::POINT3_64F: {
            const auto& p = std::get<pho::api::Point3_64f>(devVal);
            r &= declP(base + PARAM_SEP + "r", rclcpp::ParameterValue(p.x));
            r &= declP(base + PARAM_SEP + "g", rclcpp::ParameterValue(p.y));
            r &= declP(base + PARAM_SEP + "b", rclcpp::ParameterValue(p.z));
            return r;
        }
        case SettingValueType::SCANNING_VOLUME: {
            const auto& geom = std::get<pho::api::ProjectionGeometry_64f>(devVal);
            auto toArr = [](const pho::api::Point3_64f& pt) -> std::vector<double> {
                return {pt.x, pt.y, pt.z};
            };
            auto toContourArr = [&](const std::vector<pho::api::Point3_64f>& pts) {
                std::vector<double> arr;
                for (const auto& pt : pts) {
                    arr.push_back(pt.x);
                    arr.push_back(pt.y);
                    arr.push_back(pt.z);
                }
                return arr;
            };
            r &= declP(base + PARAM_SEP + "origin", rclcpp::ParameterValue(toArr(geom.Origin)));
            r &= declP(base + PARAM_SEP + "top_left", rclcpp::ParameterValue(toArr(geom.TopLeftTangentialVector)));
            r &= declP(base + PARAM_SEP + "top_right", rclcpp::ParameterValue(toArr(geom.TopRightTangentialVector)));
            r &= declP(base + PARAM_SEP + "bottom_left", rclcpp::ParameterValue(toArr(geom.BottomLeftTangentialVector)));
            r &= declP(base + PARAM_SEP + "bottom_right", rclcpp::ParameterValue(toArr(geom.BottomRightTangentialVector)));
            r &= declP(base + PARAM_SEP + "top_contour", rclcpp::ParameterValue(toContourArr(geom.TopContourPoints)));
            r &= declP(base + PARAM_SEP + "bottom_contour", rclcpp::ParameterValue(toContourArr(geom.BottomContourPoints)));
            return r;
        }
        case SettingValueType::SCANNING_VOLUME_MESH: {
            const auto& mesh = std::get<pho::api::PhoXiMesh>(devVal);
            std::vector<double> vertices;
            std::vector<int64_t> indices;
            vertices.reserve(mesh.Vertices.size() * 3);
            indices.reserve(mesh.Indices.size());
            for (const auto& pt : mesh.Vertices) {
                vertices.push_back(pt.x);
                vertices.push_back(pt.y);
                vertices.push_back(pt.z);
            }
            for (unsigned int i : mesh.Indices) {
                indices.push_back(i);
            }
            r &= declP(base + PARAM_SEP + "points_per_section", rclcpp::ParameterValue(static_cast<int64_t>(mesh.PointsPerSection)));
            r &= declP(base + PARAM_SEP + "vertices", rclcpp::ParameterValue(vertices));
            r &= declP(base + PARAM_SEP + "indices", rclcpp::ParameterValue(indices));
            return r;
        }
        case SettingValueType::REPROJECTION_MAP: {
            const auto& reproj = std::get<pho::api::PhoXiReprojectionMap>(devVal);
            r &= declP(base + PARAM_SEP + "width", rclcpp::ParameterValue(static_cast<int64_t>(reproj.Map.Size.Width)));
            r &= declP(base + PARAM_SEP + "height", rclcpp::ParameterValue(static_cast<int64_t>(reproj.Map.Size.Height)));
            r &= declP(base + PARAM_SEP + "cv_type", rclcpp::ParameterValue(0));
            return r;
        }
    }

    return r;
}

void PhoXiCamera::declareFrameSettingParameters() {
    const auto deviceValues = mPhoXiInterface->getFrameOutputSettings(mFrameOutputComponents);
    mFrameOutputComponents.clear();
    mFrameOutputComponents.reserve(deviceValues.size());
    for (const auto& [component, _] : deviceValues) {
        mFrameOutputComponents.emplace_back(component);
    }

    std::vector<std::pair<std::string, bool>> overrides;

    mDeclaringDeviceSettings = true;

    for (const auto& component : mFrameOutputComponents) {
        const auto deviceValueIt = deviceValues.find(component);
        if (deviceValueIt == deviceValues.end()) {
            continue;
        }

        const std::string paramName = std::string(FRAME_SETTINGS_PREFIX) + PARAM_SEP + component;
        try {
            auto value = declare_parameter(paramName, deviceValueIt->second);
            if (value != deviceValueIt->second) {
                overrides.emplace_back(component, value);
            }
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
            // Already declared from a previous configure — keep the stored value.
            try {
                const bool value = get_parameter(paramName).as_bool();
                if (value != deviceValueIt->second) {
                    overrides.emplace_back(component, value);
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Could not compare frame setting '%s': %s", component.c_str(), e.what());
            }
        }
    }

    mDeclaringDeviceSettings = false;

    if (!overrides.empty()) {
        mPhoXiInterface->setFrameOutputSettings(overrides);
    }
}

void PhoXiCamera::declareDeviceInfoParameters() {
    const auto info = mPhoXiInterface->getDeviceInfo();
    const std::string prefix = std::string(DEVICE_INFO_PREFIX) + PARAM_SEP;

    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;

    auto declare = [&](const std::string& name, const auto& value) {
        try {
            declare_parameter(prefix + name, value, desc);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
        }
    };

    declare("name", info.name);
    declare("type", static_cast<std::string>(info.type));
    declare("deviceId", info.hwIdentification);
    declare("ipAddress", info.ipAddress);
    declare("ipv6Address", info.ipv6Address);
    declare("hostname", info.hostname);
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
    std::vector<std::pair<std::string, SettingValue>> overrides;
    mDeclaringDeviceSettings = true;

    for (const auto& desc : mSettingDescriptors) {
        const auto deviceValueIt = deviceValues.find(desc.deviceKey);
        if (deviceValueIt == deviceValues.end()) {
            continue;
        }

        const std::string base = std::string(DEVICE_SETTINGS_PREFIX) + PARAM_SEP + deviceKeyToParamName(desc.deviceKey);
        try {
            if (!declareSettingParam(desc, base, deviceValueIt->second) && desc.isSettable) {
                const auto value = reconstructSettingValue(desc, base);
                overrides.emplace_back(desc.deviceKey, value);
            }
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
            // Already declared from a previous configure — keep the stored value.
            try {
                if (desc.isSettable) {
                    const auto value = reconstructSettingValue(desc, base);
                    if (!settingValuesEqual(value, deviceValueIt->second)) {
                        overrides.emplace_back(desc.deviceKey, value);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Could not compare setting '%s': %s", desc.deviceKey.c_str(), e.what());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Could not declare param for '%s': %s", desc.deviceKey.c_str(), e.what());
        }
    }

    mDeclaringDeviceSettings = false;

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
            s.Width = static_cast<int32_t>(get_parameter(base + PARAM_SEP + "width").as_int());
            s.Height = static_cast<int32_t>(get_parameter(base + PARAM_SEP + "height").as_int());
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
            roi.Min.x = static_cast<int32_t>(get_parameter(base + PARAM_SEP + "x_min").as_int());
            roi.Min.y = static_cast<int32_t>(get_parameter(base + PARAM_SEP + "y_min").as_int());
            roi.Max.x = static_cast<int32_t>(get_parameter(base + PARAM_SEP + "x_max").as_int());
            roi.Max.y = static_cast<int32_t>(get_parameter(base + PARAM_SEP + "y_max").as_int());
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
                s.Width = static_cast<int32_t>(param.as_int());
            }
            if (field == "height") {
                s.Height = static_cast<int32_t>(param.as_int());
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
                roi.Min.x = static_cast<int32_t>(param.as_int());
            }
            if (field == "y_min") {
                roi.Min.y = static_cast<int32_t>(param.as_int());
            }
            if (field == "x_max") {
                roi.Max.x = static_cast<int32_t>(param.as_int());
            }
            if (field == "y_max") {
                roi.Max.y = static_cast<int32_t>(param.as_int());
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

std::optional<rcl_interfaces::msg::SetParametersResult> PhoXiCamera::handleNodeParameter(const rclcpp::Parameter& param) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    const auto& name = param.get_name();
    if (name == "trigger_mode") {
        const auto& modeStr = param.as_string();
        try {
            mPhoXiInterface->setTriggerMode(parseTriggerMode(modeStr));
        } catch (const std::exception& e) {
            result.successful = false;
            result.reason = e.what();
            return result;
        }
        mTriggerMode = modeStr;
    } else if (name == "logout_on_exit") {
        mLogoutOnExit = param.as_bool();
    } else if (name == "stop_acquisition_on_exit") {
        mStopAcquisitionOnExit = param.as_bool();
    } else {
        return {};
    }

    return result;
}

rcl_interfaces::msg::SetParametersResult PhoXiCamera::onParametersChanged(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    if (mDeclaringDeviceSettings) {
        return result;
    }

    static const std::string FRAME_SETTINGS_PREFIX_P = std::string(FRAME_SETTINGS_PREFIX) + PARAM_SEP;
    static const std::string DEVICE_SETTINGS_PREFIX_P = std::string(DEVICE_SETTINGS_PREFIX) + PARAM_SEP;
    std::vector<std::pair<std::string, bool>> components;
    std::map<size_t, std::vector<const rclcpp::Parameter*>> deviceChanges;

    for (auto& p : params) {
        const auto& name = p.get_name();
        if (const auto nodeResult = handleNodeParameter(p)) {
            if (!nodeResult->successful) {
                if (params.size() == 1) {
                    return *nodeResult;
                }
                RCLCPP_WARN(get_logger(), "%s", nodeResult->reason.c_str());
            }
        } else if (name.rfind(FRAME_SETTINGS_PREFIX_P, 0) == 0) {
            components.emplace_back(name.substr(FRAME_SETTINGS_PREFIX_P.size()), p.as_bool());
        } else if (name.rfind(DEVICE_SETTINGS_PREFIX_P, 0) == 0) {
            const auto it = mParamToDescriptor.find(name);
            if (it == mParamToDescriptor.end()) {
                auto reason = "Unknown device setting parameter: '" + name + "'";
                if (params.size() == 1) {
                    result.successful = false;
                    result.reason = std::move(reason);
                    return result;
                }
                RCLCPP_WARN(get_logger(), reason.c_str());
                continue;
            }
            auto idx = it->second.first;
            const auto& desc = mSettingDescriptors[idx];
            if (!desc.isSettable) {
                auto reason = "'" + desc.deviceKey + "' is a read-only device setting";
                if (params.size() == 1) {
                    result.successful = false;
                    result.reason = std::move(reason);
                    return result;
                }
                RCLCPP_WARN(get_logger(), reason.c_str());
                continue;
            }
            deviceChanges[idx].push_back(&p);
        }
    }

    if (!components.empty()) {
        try {
            mPhoXiInterface->setFrameOutputSettings(components);
        } catch (const PhoXiInterfaceException& e) {
            result.successful = false;
            result.reason = e.what();
            return result;
        }
    }

    if (deviceChanges.empty()) {
        return result;
    }

    std::vector<std::pair<std::string, SettingValue>> toSet;
    for (const auto& [descIdx, paramPtrs] : deviceChanges) {
        const auto& desc = mSettingDescriptors[descIdx];
        const auto& firstParam = *paramPtrs[0];
        const auto& fieldName = mParamToDescriptor.at(firstParam.get_name()).second;

        try {
            if (fieldName.empty()) {
                toSet.emplace_back(desc.deviceKey, paramToSettingValue(desc.type, firstParam.get_parameter_value()));
            } else {
                auto current = mPhoXiInterface->getSetting(desc.deviceKey);
                for (const auto* pPtr : paramPtrs) {
                    const auto& f = mParamToDescriptor.at(pPtr->get_name()).second;
                    current = applyFieldUpdate(current, desc.type, f, *pPtr);
                }
                toSet.emplace_back(desc.deviceKey, current);
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
    mFrameErrorPub->on_activate();
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
    mFrameErrorPub->on_deactivate();
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
    mFrameErrorPub.reset();
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

        auto setHeader = [&](auto& message) {
            message.header.frame_id = mFrameId;
            message.header.stamp = rosStamp;
        };

        auto shouldPublish = [](const auto& pub) {
            return pub->is_activated() && pub->get_subscription_count() > 0;
        };

        std::optional<ParsedFrameInfo> parsedFrameInfo;
        if (frame.frameInfo) {
            try {
                parsedFrameInfo = parseFrameInfo(*frame.frameInfo);
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Failed to parse frameInfo: %s", e.what());
            }
        }

        if (parsedFrameInfo && !parsedFrameInfo->successful) {
            if (parsedFrameInfo->frameError && shouldPublish(mFrameErrorPub)) {
                setHeader(*parsedFrameInfo->frameError);
                mFrameErrorPub->publish(std::move(parsedFrameInfo->frameError));
            }
            RCLCPP_WARN(get_logger(), "Frame not successful, skipping data publish.");
            return;
        }

        if (frame.pointCloud && shouldPublish(mPointCloudPub)) {
            auto msg = phoXiFrameToRosMsg(frame);
            setHeader(*msg);
            mPointCloudPub->publish(std::move(msg));
        }

        if (frame.pointCloud && shouldPublish(mPointsPub)) {
            auto msg = pointsToRosMsg(frame);
            setHeader(*msg);
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

        if (parsedFrameInfo) {
            const bool pubFrameInfo = shouldPublish(mFrameInfoPub);
            const bool pubPrimaryCamera = shouldPublish(mPrimaryCameraInfoPub);
            const bool pubColorCamera = shouldPublish(mColorCameraInfoPub);
            if (parsedFrameInfo->frameInfo && pubFrameInfo) {
                setHeader(*parsedFrameInfo->frameInfo);
                mFrameInfoPub->publish(std::move(parsedFrameInfo->frameInfo));
            }
            if (parsedFrameInfo->currentCamera && pubPrimaryCamera) {
                setHeader(*parsedFrameInfo->currentCamera);
                mPrimaryCameraInfoPub->publish(std::move(parsedFrameInfo->currentCamera));
            }
            if (parsedFrameInfo->currentColorCamera && pubColorCamera) {
                setHeader(*parsedFrameInfo->currentColorCamera);
                mColorCameraInfoPub->publish(std::move(parsedFrameInfo->currentColorCamera));
            }
        }

        RCLCPP_INFO(get_logger(), "Frame published successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to publish frame: %s", e.what());
    }
}

void PhoXiCamera::rebootCallback(const std::shared_ptr<const std_srvs::srv::Trigger::Request>& /*request*/, const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    const std::string deviceId = [this] {
        std::lock_guard<std::mutex> lock(mDeviceIdMutex);
        return mDeviceId;
    }();
    if (deviceId.empty()) {
        response->success = false;
        response->message = "No device_id configured.";
        return;
    }
    RCLCPP_INFO(get_logger(), "Rebooting device '%s'.", deviceId.c_str());
    try {
        mPhoXiInterface->rebootDevice(deviceId);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Reboot failed: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::shutdownCallback(const std::shared_ptr<const std_srvs::srv::Trigger::Request>& /*request*/, const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    const std::string deviceId = [this] {
        std::lock_guard<std::mutex> lock(mDeviceIdMutex);
        return mDeviceId;
    }();
    if (deviceId.empty()) {
        response->success = false;
        response->message = "No device_id configured.";
        return;
    }
    RCLCPP_INFO(get_logger(), "Shutting down device '%s'.", deviceId.c_str());
    try {
        mPhoXiInterface->shutdownDevice(deviceId);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Shutdown failed: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::factoryResetCallback(
        const std::shared_ptr<const std_srvs::srv::Trigger::Request>& /*request*/, const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    const std::string deviceId = [this] {
        std::lock_guard<std::mutex> lock(mDeviceIdMutex);
        return mDeviceId;
    }();
    if (deviceId.empty()) {
        response->success = false;
        response->message = "No device_id configured.";
        return;
    }
    RCLCPP_INFO(get_logger(), "Factory resetting device '%s'.", deviceId.c_str());
    try {
        mPhoXiInterface->factoryResetDevice(deviceId);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Factory reset failed: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void PhoXiCamera::logDownloadCallback(
        const std::shared_ptr<const phoxi_camera_msgs::srv::LogDownload::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::LogDownload::Response>& response) {
    const std::string deviceId = [this] {
        std::lock_guard<std::mutex> lock(mDeviceIdMutex);
        return mDeviceId;
    }();
    if (deviceId.empty()) {
        response->success = false;
        response->message = "No device_id configured.";
        return;
    }
    RCLCPP_INFO(get_logger(), "Downloading log from device '%s' to '%s'.", deviceId.c_str(), request->logfile_path.c_str());
    try {
        mPhoXiInterface->downloadDeviceLog(deviceId, request->logfile_path, request->overwrite);
        response->success = true;
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Log download failed: %s", e.what());
        response->success = false;
        response->message = e.what();
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

    const std::string& snToUse = request->sn.empty() ? mDeviceId : request->sn;
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
}  // namespace phoxi_camera

RCLCPP_COMPONENTS_REGISTER_NODE(phoxi_camera::PhoXiCamera)