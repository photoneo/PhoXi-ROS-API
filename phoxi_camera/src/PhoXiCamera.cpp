#include "phoxi_camera/PhoXiCamera.h"

#include <array>
#include <chrono>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/RosConversions.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace phoxi_camera
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static constexpr std::string_view kFrameSettingsPrefix = "frameSettings/";

static const std::array<std::string_view, 7> kComponents = {
    "PointCloud", "NormalMap", "DepthMap", "Texture", "ConfidenceMap", "ColorCameraImage", "EventMap"
};

PhoXiCamera::PhoXiCamera(std::string deviceId, const rclcpp::NodeOptions& options) :
    rclcpp_lifecycle::LifecycleNode("phoxi_camera", options),
    mPhoXiInterface(std::make_unique<PhoXiInterface>()),
    mDeviceId(std::move(deviceId)) {
    RCLCPP_INFO(get_logger(), "Creating PhoXi Camera node for device: %s", mDeviceId.c_str());
    declareParameters();
}

PhoXiCamera::PhoXiCamera(const rclcpp::NodeOptions& options) :
    rclcpp_lifecycle::LifecycleNode("phoxi_camera", options),
    mPhoXiInterface(std::make_unique<PhoXiInterface>()) {
    RCLCPP_INFO(get_logger(), "Creating PhoXi Camera node.");
    declareParameters();
    get_parameter("device_id", mDeviceId);
}

PhoXiCamera::~PhoXiCamera() {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(
        mShutdownCallbackHandle);

    try {
        if (mPhoXiInterface->isAcquiring()) {
            mPhoXiInterface->stopAcquisition();
        }
    } catch (...) {}

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
        mPhoXiInterface->connectCamera(mDeviceId,
            std::bind(&PhoXiCamera::onFrameCallback, this, std::placeholders::_1));
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    try {
        std::vector<std::pair<std::string, bool>> components;
        for (const auto& componentName : kComponents) {
            const auto param = get_parameter(std::string(kFrameSettingsPrefix) + std::string(componentName));
            if (param.get_type() != rclcpp::PARAMETER_NOT_SET) {
                components.emplace_back(std::string(componentName), param.as_bool());
            }
        }
        if (!components.empty()) {
            mPhoXiInterface->setFrameOutputSettings(components);
        }
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        try { mPhoXiInterface->disconnectCamera(); } catch (...) {}
        return CallbackReturn::FAILURE;
    }

    mPointCloudPub = create_publisher<sensor_msgs::msg::PointCloud2>(
    "point_cloud", rclcpp::SystemDefaultsQoS());
    mPointsPub = create_publisher<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SystemDefaultsQoS());
    mNormalMapPub = create_publisher<sensor_msgs::msg::Image>(
        "normals", rclcpp::SystemDefaultsQoS());
    mDepthMapPub = create_publisher<sensor_msgs::msg::Image>(
        "depth", rclcpp::SystemDefaultsQoS());
    mConfidenceMapPub = create_publisher<sensor_msgs::msg::Image>(
        "confidence", rclcpp::SystemDefaultsQoS());
    mEventMapPub = create_publisher<sensor_msgs::msg::Image>(
        "event", rclcpp::SystemDefaultsQoS());
    mTexturePub = create_publisher<sensor_msgs::msg::Image>(
        "intensity", rclcpp::SystemDefaultsQoS());
    mTextureRgbPub = create_publisher<sensor_msgs::msg::Image>(
        "texture", rclcpp::SystemDefaultsQoS());
    mColorCameraImagePub = create_publisher<sensor_msgs::msg::Image>(
        "color_camera_image", rclcpp::SystemDefaultsQoS());

    mConnectService = create_service<phoxi_camera_msgs::srv::Connect>(
        "~/connect",
        std::bind(&PhoXiCamera::connectCallback, this, std::placeholders::_1, std::placeholders::_2));
    mDisconnectService = create_service<std_srvs::srv::Trigger>(
        "~/disconnect",
        std::bind(&PhoXiCamera::disconnectCallback, this, std::placeholders::_1, std::placeholders::_2));
    mTriggerFrameService = create_service<phoxi_camera_msgs::srv::TriggerFrame>(
        "~/trigger_frame",
        std::bind(&PhoXiCamera::triggerFrameCallback, this, std::placeholders::_1, std::placeholders::_2));
    mGetProfileListService = create_service<phoxi_camera_msgs::srv::GetProfileList>(
        "~/profiles/list",
        std::bind(&PhoXiCamera::getProfileListCallback, this, std::placeholders::_1, std::placeholders::_2));
    mGetActiveProfileService = create_service<phoxi_camera_msgs::srv::GetActiveProfile>(
        "~/profiles/get_active",
        std::bind(&PhoXiCamera::getActiveProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mSetActiveProfileService = create_service<phoxi_camera_msgs::srv::SetActiveProfile>(
        "~/profiles/set_active",
        std::bind(&PhoXiCamera::setActiveProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mCreateProfileService = create_service<phoxi_camera_msgs::srv::CreateProfile>(
        "~/profiles/create",
        std::bind(&PhoXiCamera::createProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mDeleteProfileService = create_service<phoxi_camera_msgs::srv::DeleteProfile>(
        "~/profiles/delete",
        std::bind(&PhoXiCamera::deleteProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mUpdateProfileService = create_service<phoxi_camera_msgs::srv::UpdateProfile>(
        "~/profiles/update",
        std::bind(&PhoXiCamera::updateProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mGetStartupProfileService = create_service<phoxi_camera_msgs::srv::GetStartupProfile>(
        "~/profiles/get_startup",
        std::bind(&PhoXiCamera::getStartupProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mSetStartupProfileService = create_service<phoxi_camera_msgs::srv::SetStartupProfile>(
        "~/profiles/set_startup",
        std::bind(&PhoXiCamera::setStartupProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mExportProfileService = create_service<phoxi_camera_msgs::srv::ExportProfile>(
        "~/profiles/export",
        std::bind(&PhoXiCamera::exportProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mImportProfileService = create_service<phoxi_camera_msgs::srv::ImportProfile>(
        "~/profiles/import",
        std::bind(&PhoXiCamera::importProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    mResetActiveProfileService = create_service<std_srvs::srv::Trigger>(
        "~/profiles/reset",
        std::bind(&PhoXiCamera::resetActiveProfileCallback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Configuration complete. Connected to device: %s",
                mDeviceId.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhoXiCamera::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
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
        RCLCPP_WARN(get_logger(), "Failed to stop acquisition during deactivation: %s",
                    e.what());
    }
    drainFrameCallback();

    deactivatePublishers();

    RCLCPP_INFO(get_logger(), "Deactivation complete.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhoXiCamera::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up PhoXi Camera node...");

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
    } catch (...) {}
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

void PhoXiCamera::declareParameters()
{
    declare_parameter<std::string>("device_id", mDeviceId);
    declare_parameter<std::string>("frame_id", "phoxi_camera_sensor");
    declare_parameter<bool>("publish_combined", false);
    rcl_interfaces::msg::ParameterDescriptor dynamicDesc;
    dynamicDesc.dynamic_typing = true;
    for (const auto& componentName : kComponents) {
        declare_parameter(std::string(kFrameSettingsPrefix) + std::string(componentName),
            rclcpp::ParameterValue{}, dynamicDesc);
    }

    mParamCallbackHandle = add_on_set_parameters_callback(
        std::bind(&PhoXiCamera::onParametersChanged, this, std::placeholders::_1));

    mShutdownCallbackHandle =
        get_node_base_interface()->get_context()->add_pre_shutdown_callback(
            [this]() { rclcpp_lifecycle::LifecycleNode::shutdown(); });
}

rcl_interfaces::msg::SetParametersResult PhoXiCamera::onParametersChanged(
    const std::vector<rclcpp::Parameter>& params)
{
    std::vector<std::pair<std::string, bool>> components;
    for (const auto& p : params) {
        const auto& name = p.get_name();
        if (name.rfind(kFrameSettingsPrefix, 0) == 0) {
            components.emplace_back(name.substr(kFrameSettingsPrefix.size()), p.as_bool());
        }
    }

    if (!components.empty() && mPhoXiInterface->isConnected()) {
        try {
            mPhoXiInterface->setFrameOutputSettings(components);
        } catch (const PhoXiInterfaceException& e) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            result.reason = e.what();
            return result;
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void PhoXiCamera::activatePublishers() {
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
        mColorCameraImagePub->on_activate();
    }
}

void PhoXiCamera::deactivatePublishers() {
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
        mColorCameraImagePub->on_deactivate();
    }
}

void PhoXiCamera::cleanupResources() {
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

void PhoXiCamera::onFrameCallback(const PhoXiFrame& frame)
{
    std::lock_guard<std::mutex> lock(mFrameMutex);
    try {
        const rclcpp::Time rosStamp = get_clock()->now();

        auto setHeader = [&](sensor_msgs::msg::Image& img) {
            img.header.frame_id = mFrameId;
            img.header.stamp = rosStamp;
        };

        auto shouldPublish = [](const auto& pub) {
            return pub->is_activated() && pub->get_subscription_count() > 0;
        };

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

        RCLCPP_INFO(get_logger(), "Frame published successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to publish frame: %s", e.what());
    }
}

void PhoXiCamera::connectCallback(
    const std::shared_ptr<const phoxi_camera_msgs::srv::Connect::Request>& request,
    const std::shared_ptr<phoxi_camera_msgs::srv::Connect::Response>& response) {
    auto stateId = get_current_state().id();
    if (stateId != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
        stateId != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
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
        try { mPhoXiInterface->stopAcquisition(); } catch (...) {}
        drainFrameCallback();
    }
    try {
        mPhoXiInterface->connectCamera(snToUse, std::bind(&PhoXiCamera::onFrameCallback,
            this, std::placeholders::_1));
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

void PhoXiCamera::disconnectCallback(
    const std::shared_ptr<const std_srvs::srv::Trigger::Request>& /*request*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    auto stateId = get_current_state().id();
    if (stateId != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
        stateId != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
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
    const std::shared_ptr<const phoxi_camera_msgs::srv::TriggerFrame::Request>& request,
    const std::shared_ptr<phoxi_camera_msgs::srv::TriggerFrame::Response>& response) {
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

void PhoXiCamera::getProfileListCallback(
    const std::shared_ptr<const phoxi_camera_msgs::srv::GetProfileList::Request>& /*request*/,
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

void PhoXiCamera::getActiveProfileCallback(
    const std::shared_ptr<const phoxi_camera_msgs::srv::GetActiveProfile::Request>& /*request*/,
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

void PhoXiCamera::setActiveProfileCallback(
    const std::shared_ptr<const phoxi_camera_msgs::srv::SetActiveProfile::Request>& request,
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
    const std::shared_ptr<const phoxi_camera_msgs::srv::CreateProfile::Request>& request,
    const std::shared_ptr<phoxi_camera_msgs::srv::CreateProfile::Response>& response) {
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
    const std::shared_ptr<const phoxi_camera_msgs::srv::DeleteProfile::Request>& request,
    const std::shared_ptr<phoxi_camera_msgs::srv::DeleteProfile::Response>& response) {
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
    const std::shared_ptr<const phoxi_camera_msgs::srv::UpdateProfile::Request>& request,
    const std::shared_ptr<phoxi_camera_msgs::srv::UpdateProfile::Response>& response) {
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

void PhoXiCamera::getStartupProfileCallback(
    const std::shared_ptr<const phoxi_camera_msgs::srv::GetStartupProfile::Request>& /*request*/,
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

void PhoXiCamera::setStartupProfileCallback(
    const std::shared_ptr<const phoxi_camera_msgs::srv::SetStartupProfile::Request>& request,
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

void PhoXiCamera::exportProfileCallback(
    const std::shared_ptr<const phoxi_camera_msgs::srv::ExportProfile::Request>& /*request*/,
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
    const std::shared_ptr<const phoxi_camera_msgs::srv::ImportProfile::Request>& request,
    const std::shared_ptr<phoxi_camera_msgs::srv::ImportProfile::Response>& response) {
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
    const std::shared_ptr<const std_srvs::srv::Trigger::Request>& /*request*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
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
