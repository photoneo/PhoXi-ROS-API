#include "phoxi_camera/PhoXiCamera.h"

#include <chrono>

#include "lifecycle_msgs/msg/state.hpp"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/RosConversions.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace phoxi_camera
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

PhoXiCamera::PhoXiCamera(const std::string& deviceId, const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("phoxi_camera", options), mDeviceId(deviceId) {
    RCLCPP_INFO(get_logger(), "Creating PhoXi Camera node for device: %s", mDeviceId.c_str());
    declare_parameter<std::string>("device_id", mDeviceId);
    declare_parameter<std::string>("frame_id", "phoxi_camera_sensor");
}

PhoXiCamera::PhoXiCamera(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("phoxi_camera", options) {
    RCLCPP_INFO(get_logger(), "Creating PhoXi Camera node.");
    declare_parameter<std::string>("device_id", "");
    declare_parameter<std::string>("frame_id", "phoxi_camera_sensor");
    get_parameter("device_id", mDeviceId);
}

PhoXiCamera::~PhoXiCamera() {
    if (mPhoXiInterface) {
        try {
            if (mPhoXiInterface->isAcquiring()) {
                mPhoXiInterface->stopAcquisition();
            }
        } catch (...) {}
        if (mPhoXiInterface->isConnected()) {
            mPhoXiInterface->disconnectCamera();
        }
    }
}

CallbackReturn PhoXiCamera::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Configuring PhoXi Camera node...");

    get_parameter("device_id", mDeviceId);
    get_parameter("frame_id", mFrameId);

    if (mDeviceId.empty()) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: 'device_id' parameter is empty.");
        return CallbackReturn::FAILURE;
    }

    if (!mPhoXiInterface) {
        mPhoXiInterface = std::make_unique<PhoXiInterface>();
    }

    mPointCloudPub = create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", rclcpp::SystemDefaultsQoS());
    mPointsPub = create_publisher<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SystemDefaultsQoS());
    mNormalMapPub = create_publisher<sensor_msgs::msg::Image>(
        "normal_map", rclcpp::SystemDefaultsQoS());
    mDepthMapPub = create_publisher<sensor_msgs::msg::Image>(
        "depth", rclcpp::SystemDefaultsQoS());
    mConfidenceMapPub = create_publisher<sensor_msgs::msg::Image>(
        "confidence", rclcpp::SystemDefaultsQoS());
    mEventMapPub = create_publisher<sensor_msgs::msg::Image>(
        "event_map", rclcpp::SystemDefaultsQoS());
    mTexturePub = create_publisher<sensor_msgs::msg::Image>(
        "texture", rclcpp::SystemDefaultsQoS());
    mTextureRgbPub = create_publisher<sensor_msgs::msg::Image>(
        "texture_rgb", rclcpp::SystemDefaultsQoS());
    mColorCameraImagePub = create_publisher<sensor_msgs::msg::Image>(
        "color_camera_image", rclcpp::SystemDefaultsQoS());

    try {
        mPhoXiInterface->connectCamera(mDeviceId, std::bind(&PhoXiCamera::on_frame_cb, this, std::placeholders::_1));
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        mPhoXiInterface.reset();
        mPointCloudPub.reset();
        mPointsPub.reset();
        mNormalMapPub.reset();
        mDepthMapPub.reset();
        mConfidenceMapPub.reset();
        mEventMapPub.reset();
        mTexturePub.reset();
        mTextureRgbPub.reset();
        mColorCameraImagePub.reset();
        return CallbackReturn::FAILURE;
    }

    mConnectService = create_service<phoxi_camera_msgs::srv::Connect>(
        "~/connect",
        std::bind(&PhoXiCamera::connect_cb, this, std::placeholders::_1, std::placeholders::_2));
    mDisconnectService = create_service<std_srvs::srv::Trigger>(
        "~/disconnect",
        std::bind(&PhoXiCamera::disconnect_cb, this, std::placeholders::_1, std::placeholders::_2));
    mTriggerFrameService = create_service<phoxi_camera_msgs::srv::TriggerFrame>(
        "~/trigger_frame",
        std::bind(&PhoXiCamera::trigger_frame_cb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Configuration complete. Connected to device: %s",
                mDeviceId.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhoXiCamera::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Activating PhoXi Camera node...");

    mPointCloudPub->on_activate();
    mPointsPub->on_activate();
    mNormalMapPub->on_activate();
    mDepthMapPub->on_activate();
    mConfidenceMapPub->on_activate();
    mEventMapPub->on_activate();
    mTexturePub->on_activate();
    mTextureRgbPub->on_activate();
    mColorCameraImagePub->on_activate();

    try {
        mPhoXiInterface->startAcquisition();
    } catch (const PhoXiInterfaceException& e) {
        RCLCPP_ERROR(get_logger(), "Activation failed: %s", e.what());
        mPointCloudPub->on_deactivate();
        mPointsPub->on_deactivate();
        mNormalMapPub->on_deactivate();
        mDepthMapPub->on_deactivate();
        mConfidenceMapPub->on_deactivate();
        mEventMapPub->on_deactivate();
        mTexturePub->on_deactivate();
        mTextureRgbPub->on_deactivate();
        mColorCameraImagePub->on_deactivate();
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
    { std::lock_guard<std::mutex> lock(mFrameMutex); }  // drain any in-flight callback

    mPointCloudPub->on_deactivate();
    mPointsPub->on_deactivate();
    mNormalMapPub->on_deactivate();
    mDepthMapPub->on_deactivate();
    mConfidenceMapPub->on_deactivate();
    mEventMapPub->on_deactivate();
    mTexturePub->on_deactivate();
    mTextureRgbPub->on_deactivate();
    mColorCameraImagePub->on_deactivate();

    RCLCPP_INFO(get_logger(), "Deactivation complete.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhoXiCamera::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up PhoXi Camera node...");

    if (mPhoXiInterface) {
        try {
            mPhoXiInterface->disconnectCamera();
        } catch (const PhoXiInterfaceException& e) {
            RCLCPP_WARN(get_logger(), "Failed to disconnect during cleanup: %s", e.what());
        }
        drainFrameCallback();
    }

    mPhoXiInterface.reset();
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
    RCLCPP_INFO(get_logger(), "Cleanup complete.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhoXiCamera::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Shutting down PhoXi Camera node...");

    if (mPhoXiInterface) {
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
    }

    mPhoXiInterface.reset();
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
    RCLCPP_INFO(get_logger(), "Shutdown complete.");
    return CallbackReturn::SUCCESS;
}

void PhoXiCamera::drainFrameCallback() {
    std::lock_guard<std::mutex> lock(mFrameMutex);
}

void PhoXiCamera::connect_cb(
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
        mPhoXiInterface->connectCamera(snToUse,
            std::bind(&PhoXiCamera::on_frame_cb, this, std::placeholders::_1));
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

void PhoXiCamera::disconnect_cb(
    const std::shared_ptr<const std_srvs::srv::Trigger::Request>& /*request*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) const {
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

void PhoXiCamera::trigger_frame_cb(
    const std::shared_ptr<const phoxi_camera_msgs::srv::TriggerFrame::Request>& request,
    const std::shared_ptr<phoxi_camera_msgs::srv::TriggerFrame::Response>& response) const {
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

void PhoXiCamera::on_frame_cb(const pho::api::PFrame& frame) {
    if (!frame) {
        RCLCPP_WARN(get_logger(), "Received null frame, skipping.");
        return;
    }
    std::lock_guard<std::mutex> lock(mFrameMutex);
    try {
        std::chrono::nanoseconds stamp{0};
        if (frame->Info.FrameStartTime.IsValid()) {
            stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                frame->Info.FrameStartTime.Time.time_since_epoch());
        } else if (frame->Info.FrameTimestamp > 0.) {
            stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double, std::milli>(frame->Info.FrameTimestamp));
        }
        const rclcpp::Time rosStamp = (stamp.count() > 0)
            ? rclcpp::Time(stamp.count(), get_clock()->get_clock_type())
            : get_clock()->now();

        auto setHeader = [&](sensor_msgs::msg::Image& img) {
            img.header.frame_id = mFrameId;
            img.header.stamp = rosStamp;
        };

        auto pc_msg = phoXiFrameToRosMsg(frame);
        pc_msg->header.frame_id = mFrameId;
        pc_msg->header.stamp = rosStamp;
        if (mPointCloudPub->is_activated()) {
            mPointCloudPub->publish(std::move(pc_msg));
        }

        if (!frame->PointCloud.Empty() && mPointsPub->is_activated()) {
            auto pts_msg = pointsToRosMsg(frame);
            pts_msg->header.frame_id = mFrameId;
            pts_msg->header.stamp = rosStamp;
            mPointsPub->publish(std::move(pts_msg));
        }

        if (!frame->NormalMap.Empty() && mNormalMapPub->is_activated()) {
            auto img = normalMapToRosMsg(frame);
            setHeader(*img);
            mNormalMapPub->publish(std::move(img));
        }

        if (!frame->DepthMap.Empty() && mDepthMapPub->is_activated()) {
            auto img = depthMapToRosMsg(frame);
            setHeader(*img);
            mDepthMapPub->publish(std::move(img));
        }

        if (!frame->ConfidenceMap.Empty() && mConfidenceMapPub->is_activated()) {
            auto img = confidenceMapToRosMsg(frame);
            setHeader(*img);
            mConfidenceMapPub->publish(std::move(img));
        }

        if (!frame->EventMap.Empty() && mEventMapPub->is_activated()) {
            auto img = eventMapToRosMsg(frame);
            setHeader(*img);
            mEventMapPub->publish(std::move(img));
        }

        if (!frame->Texture.Empty() && mTexturePub->is_activated()) {
            auto img = textureToRosMsg(frame);
            setHeader(*img);
            mTexturePub->publish(std::move(img));
        }

        if (!frame->TextureRGB.Empty() && mTextureRgbPub->is_activated()) {
            auto img = textureRgbToRosMsg(frame);
            setHeader(*img);
            mTextureRgbPub->publish(std::move(img));
        }

        if (!frame->ColorCameraImage.Empty() && mColorCameraImagePub->is_activated()) {
            auto img = colorCameraImageToRosMsg(frame);
            setHeader(*img);
            mColorCameraImagePub->publish(std::move(img));
        }

        RCLCPP_INFO(get_logger(), "Frame published successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to publish frame: %s", e.what());
    }
}
} // namespace phoxi_camera

RCLCPP_COMPONENTS_REGISTER_NODE(phoxi_camera::PhoXiCamera)
