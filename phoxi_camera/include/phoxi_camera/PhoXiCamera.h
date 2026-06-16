/**
 * @file PhoXiCamera.h
 * @brief ROS2 lifecycle node for Photoneo PhoXi 3D sensors.
 *
 * **Lifecycle transitions**
 *
 * - **configure**: Connects to the device; declares device-setting parameters.
 * - **activate**: Starts acquisition; activates publishers.
 * - **deactivate**: Stops acquisition; deactivates publishers.
 * - **cleanup**: Disconnects from the device; destroys publishers and services.
 * - **shutdown**: Stops acquisition and disconnects regardless of current state.
 *
 * **Parameters**
 *
 * Declared on node creation — available in all lifecycle states:
 * - `device_id` (string, default `""`): Hardware identification of the target device.
 * - `frame_id` (string, default `"phoxi_camera_sensor"`): TF frame ID stamped on all published messages.
 * - `publish_combined` (bool, default `false`): Publish only `point_cloud` instead of individual topics.
 * - `frameSettings.<Component>` (bool, unset by default): Enable/disable individual frame output components.
 *   Valid component names: `PointCloud`, `NormalMap`, `DepthMap`, `Texture`, `ConfidenceMap`, `ColorCameraImage`, `EventMap`.
 *   Changes are forwarded to the device immediately when connected.
 *
 * Declared during `configure` — available from the configured state onwards:
 * - `deviceSettings.*` (various types): Device-setting parameters read from the device schema on connect.
 *   Parameters flagged as read-only by the device reject writes.
 * - `deviceInfo.*` (various types): Device identity information (name, type, IP address, firmware version, etc.).
 *   Read-only.
 *
 * **Published topics (active state only)**
 *
 * Always published:
 * - `frameError` (`phoxi_camera_msgs/FrameError`): API-reported frame errors.
 * - `frameInfo` (`phoxi_camera_msgs/FrameInfo`): Per-frame metadata.
 * - `frameInfo/currentCamera` (`sensor_msgs/CameraInfo`): Primary camera intrinsics.
 * - `frameInfo/currentColorCamera` (`sensor_msgs/CameraInfo`): Color camera intrinsics.
 *
 * When `publish_combined` is `true`:
 * - `point_cloud` (`sensor_msgs/PointCloud2`): XYZ + normals + color/intensity + confidence + depth + event.
 *
 * When `publish_combined` is `false` (default):
 * - `points` (`sensor_msgs/PointCloud2`): XYZ-only lean point cloud.
 * - `normals` (`sensor_msgs/Image`, 32FC3): Per-pixel normal vectors.
 * - `depth` (`sensor_msgs/Image`, 32FC1): Orthogonal distance in mm.
 * - `confidence` (`sensor_msgs/Image`, 32FC1): Measurement confidence.
 * - `event` (`sensor_msgs/Image`, 32FC1): Time-of-measurement (MotionCam only).
 * - `intensity` (`sensor_msgs/Image`, 32FC1): Grayscale texture.
 * - `texture` (`sensor_msgs/Image`, rgb8): RGB structured-light texture.
 * - `color_camera_image` (`sensor_msgs/Image`, rgb8): Color camera image.
 *
 * **Services**
 *
 * Available in all lifecycle states:
 * - `~/reboot` (`std_srvs/Trigger`): Reboot the device.
 * - `~/shutdown` (`std_srvs/Trigger`): Shut down the device.
 * - `~/factory_reset` (`std_srvs/Trigger`): Restore factory defaults.
 * - `~/log_download` (`phoxi_camera_msgs/LogDownload`): Download device logs.
 *
 * Available after configure:
 * - `~/connect` (`phoxi_camera_msgs/Connect`): **Deprecated.** Connect to a device by ID.
 *   Kept for backwards compatibility; prefer the lifecycle `configure` transition instead.
 * - `~/disconnect` (`std_srvs/Trigger`): **Deprecated.** Disconnect from the current device.
 *   Kept for backwards compatibility; prefer the lifecycle `cleanup` transition instead.
 * - `~/trigger_frame` (`phoxi_camera_msgs/TriggerFrame`): Manually trigger a frame (active state only).
 * - `~/profiles/list` (`phoxi_camera_msgs/GetProfileList`): List saved profiles.
 * - `~/profiles/get_active` (`phoxi_camera_msgs/GetActiveProfile`): Get the active profile name.
 * - `~/profiles/set_active` (`phoxi_camera_msgs/SetActiveProfile`): Activate a profile by name.
 * - `~/profiles/create` (`phoxi_camera_msgs/CreateProfile`): Save current settings as a new profile.
 * - `~/profiles/delete` (`phoxi_camera_msgs/DeleteProfile`): Delete a profile by name.
 * - `~/profiles/update` (`phoxi_camera_msgs/UpdateProfile`): Overwrite a profile with current settings.
 * - `~/profiles/get_startup` (`phoxi_camera_msgs/GetStartupProfile`): Get the startup profile name.
 * - `~/profiles/set_startup` (`phoxi_camera_msgs/SetStartupProfile`): Set the startup profile.
 * - `~/profiles/export` (`phoxi_camera_msgs/ExportProfile`): Export the active profile as binary.
 * - `~/profiles/import` (`phoxi_camera_msgs/ImportProfile`): Import a profile from binary.
 * - `~/profiles/reset` (`std_srvs/Trigger`): Reset the active profile to factory defaults.
 */
#ifndef PHOXI_CAMERA_ROSINTERFACE_H
#define PHOXI_CAMERA_ROSINTERFACE_H

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera_msgs/msg/frame_error.hpp"
#include "phoxi_camera_msgs/msg/frame_info.hpp"
#include "phoxi_camera_msgs/srv/connect.hpp"
#include "phoxi_camera_msgs/srv/log_download.hpp"
#include "phoxi_camera_msgs/srv/create_profile.hpp"
#include "phoxi_camera_msgs/srv/delete_profile.hpp"
#include "phoxi_camera_msgs/srv/export_profile.hpp"
#include "phoxi_camera_msgs/srv/get_active_profile.hpp"
#include "phoxi_camera_msgs/srv/get_profile_list.hpp"
#include "phoxi_camera_msgs/srv/get_startup_profile.hpp"
#include "phoxi_camera_msgs/srv/import_profile.hpp"
#include "phoxi_camera_msgs/srv/set_active_profile.hpp"
#include "phoxi_camera_msgs/srv/set_startup_profile.hpp"
#include "phoxi_camera_msgs/srv/trigger_frame.hpp"
#include "phoxi_camera_msgs/srv/update_profile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace phoxi_camera {

/**
 * @brief ROS2 lifecycle node providing full access to a Photoneo PhoXi 3D sensor.
 *
 * Manages device connection, acquisition, and publishing of 3D scan data.
 * Device settings are exposed as ROS2 parameters and can be updated at runtime.
 *
 * @see PhoXiInterface for the underlying API abstraction.
 */
class PhoXiCamera : public rclcpp_lifecycle::LifecycleNode {
public:
    /**
     * @brief Construct a node targeting a specific device.
     *
     * @param deviceId Hardware identification string of the PhoXi device.
     * @param options  ROS2 node options.
     */
    explicit PhoXiCamera(std::string deviceId, const rclcpp::NodeOptions& options);

    /**
     * @brief Construct a node; device ID is read from the `device_id` parameter.
     *
     * @param options ROS2 node options; `device_id` must be set before `configure`.
     */
    explicit PhoXiCamera(const rclcpp::NodeOptions& options);

    /** @brief Stops acquisition and disconnects from the device if still connected. */
    ~PhoXiCamera() override;

    /**
     * @brief Connect to the device and create publishers and services.
     *
     * Reads `device_id`, `frame_id`, and `publish_combined` parameters, then
     * connects to the device and declares `deviceSettings.*` and `deviceInfo.*`
     * parameters from the device schema.
     *
     * @param previous_state Lifecycle state before this transition.
     * @return SUCCESS on success; FAILURE if the device is unreachable.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Start acquisition and activate publishers.
     *
     * @param previous_state Lifecycle state before this transition.
     * @return SUCCESS on success; FAILURE if acquisition cannot be started.
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Stop acquisition and deactivate publishers.
     *
     * Acquisition-stop failures are logged as warnings but do not cause FAILURE.
     *
     * @param previous_state Lifecycle state before this transition.
     * @return SUCCESS always.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Disconnect from the device and release all publishers and services.
     *
     * @param previous_state Lifecycle state before this transition.
     * @return SUCCESS always (disconnect failures are logged but not fatal).
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Stop acquisition and disconnect regardless of current state.
     *
     * @param previous_state Lifecycle state before this transition.
     * @return SUCCESS always.
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

protected:
    /** @brief Underlying PhoXi API interface; replaceable in tests via subclassing. */
    std::unique_ptr<PhoXiInterface> mPhoXiInterface;

private:
    /** @brief Describes a single device setting as exposed on the ROS2 parameter server. */
    struct SettingDescriptor {
        std::string deviceKey;  ///< API-side key (slash-separated path, e.g. "Scanning/Mode").
        SettingValueType type;  ///< Strongly-typed variant tag.
        bool isSettable;        ///< False for read-only device settings.
    };

    /** @brief Block until any in-progress frame callback has returned. */
    void drainFrameCallback();
    /** @brief Declare static parameters (device_id, frame_id, frameSettings.*) and register callbacks. */
    void declareParameters();
    /** @brief Populate mSettingDescriptors and mParamToDescriptor from the device schema. */
    void loadDeviceSettingDescriptors();
    /** @brief Declare `deviceSettings.*` parameters from the current device values and apply any overrides. */
    void declareDeviceSettingParameters();
    /** @brief Declare read-only `deviceInfo.*` parameters from the connected device. */
    void declareDeviceInfoParameters();
    /** @brief Activate the publishers selected by the `publish_combined` parameter. */
    void activatePublishers();
    /** @brief Deactivate the publishers selected by the `publish_combined` parameter. */
    void deactivatePublishers();
    /** @brief Reset all publisher and service shared pointers to release resources. */
    void cleanupResources();

    /** @brief Receive a frame from the API and publish all active topics. */
    void onFrameCallback(const PhoXiFrame& frame);

    /**
     * @brief Forward parameter changes to the device or to frameSettings.
     *
     * Called by the ROS2 parameter system for every `set_parameters` call.
     * Changes to `deviceSettings.*` are forwarded to the device via `PhoXiInterface::setSettings`.
     * Changes to `frameSettings.*` are forwarded via `PhoXiInterface::setFrameOutputSettings`.
     *
     * @param params Parameters that were changed.
     * @return Result with `successful=true` on success, or an error reason string.
     */
    rcl_interfaces::msg::SetParametersResult onParametersChanged(const std::vector<rclcpp::Parameter>& params);

    /**
     * @brief Declare one or more ROS2 parameters for a device setting.
     *
     * Object-typed settings (e.g. `PHOXI_2DROI`) are split into sub-parameters.
     *
     * @param desc      Descriptor for the setting.
     * @param baseParam ROS2 parameter name prefix (e.g. `deviceSettings.Scanning.Mode`).
     * @param devVal    Initial value read from the device, used as the default.
     */
    void declareSettingParams(const SettingDescriptor& desc, const std::string& baseParam, const SettingValue& devVal);

    /**
     * @brief Reconstruct a `SettingValue` from the currently declared ROS2 parameters.
     *
     * @param desc      Descriptor identifying the setting type.
     * @param baseParam ROS2 parameter name prefix for this setting.
     * @return The reconstructed `SettingValue`.
     */
    SettingValue reconstructSettingValue(const SettingDescriptor& desc, const std::string& baseParam) const;

    /**
     * @brief Convert a single ROS2 `ParameterValue` to a `SettingValue` for scalar types.
     *
     * @param type Scalar setting type (BOOL, INT, DOUBLE, STRING, or DOUBLE_ARRAY).
     * @param pv   The ROS2 parameter value to convert.
     * @return The corresponding `SettingValue`.
     * @throws PhoXiInterfaceException if `type` is not a scalar type.
     */
    SettingValue paramToSettingValue(SettingValueType type, const rclcpp::ParameterValue& pv) const;

    /**
     * @brief Apply a single field change to an object-typed `SettingValue`.
     *
     * Used to propagate partial parameter updates (e.g. only `x_min` of an ROI).
     *
     * @param current Existing value to update.
     * @param type    Object setting type (e.g. PHOXI_2DROI).
     * @param field   Sub-field name (e.g. `"x_min"`).
     * @param param   New ROS2 parameter value for that field.
     * @return Updated `SettingValue`.
     */
    SettingValue applyFieldUpdate(const SettingValue& current, SettingValueType type, const std::string& field, const rclcpp::Parameter& param) const;

    void rebootCallback(
            const std::shared_ptr<const std_srvs::srv::Trigger::Request>& request,
            const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);
    void shutdownCallback(
            const std::shared_ptr<const std_srvs::srv::Trigger::Request>& request,
            const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);
    void factoryResetCallback(
            const std::shared_ptr<const std_srvs::srv::Trigger::Request>& request,
            const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);
    void logDownloadCallback(
            const std::shared_ptr<const phoxi_camera_msgs::srv::LogDownload::Request>& request,
            const std::shared_ptr<phoxi_camera_msgs::srv::LogDownload::Response>& response);
    /** @deprecated Use the lifecycle `configure` transition instead. */
    void connectCallback(
            const std::shared_ptr<const phoxi_camera_msgs::srv::Connect::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::Connect::Response>& response);
    /** @deprecated Use the lifecycle `cleanup` transition instead. */
    void disconnectCallback(const std::shared_ptr<const std_srvs::srv::Trigger::Request>& request, const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);
    void triggerFrameCallback(
            const std::shared_ptr<const phoxi_camera_msgs::srv::TriggerFrame::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::TriggerFrame::Response>& response);
    void getProfileListCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::GetProfileList::Request>& request,
            const std::shared_ptr<phoxi_camera_msgs::srv::GetProfileList::Response>& response);
    void getActiveProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::GetActiveProfile::Request>& request,
            const std::shared_ptr<phoxi_camera_msgs::srv::GetActiveProfile::Response>& response);
    void setActiveProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::SetActiveProfile::Request>& request,
            const std::shared_ptr<phoxi_camera_msgs::srv::SetActiveProfile::Response>& response);
    void createProfileCallback(
            const std::shared_ptr<const phoxi_camera_msgs::srv::CreateProfile::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::CreateProfile::Response>& response);
    void deleteProfileCallback(
            const std::shared_ptr<const phoxi_camera_msgs::srv::DeleteProfile::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::DeleteProfile::Response>& response);
    void updateProfileCallback(
            const std::shared_ptr<const phoxi_camera_msgs::srv::UpdateProfile::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::UpdateProfile::Response>& response);
    void getStartupProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::GetStartupProfile::Request>& request,
            const std::shared_ptr<phoxi_camera_msgs::srv::GetStartupProfile::Response>& response);
    void setStartupProfileCallback(const std::shared_ptr<const phoxi_camera_msgs::srv::SetStartupProfile::Request>& request,
            const std::shared_ptr<phoxi_camera_msgs::srv::SetStartupProfile::Response>& response);
    void exportProfileCallback(
            const std::shared_ptr<const phoxi_camera_msgs::srv::ExportProfile::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::ExportProfile::Response>& response);
    void importProfileCallback(
            const std::shared_ptr<const phoxi_camera_msgs::srv::ImportProfile::Request>& request, const std::shared_ptr<phoxi_camera_msgs::srv::ImportProfile::Response>& response);
    void resetActiveProfileCallback(const std::shared_ptr<const std_srvs::srv::Trigger::Request>& request, const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);

    // Publishers (topic)
    rclcpp_lifecycle::LifecyclePublisher<phoxi_camera_msgs::msg::FrameError>::SharedPtr mFrameErrorPub;   ///< topic: `frameError`
    rclcpp_lifecycle::LifecyclePublisher<phoxi_camera_msgs::msg::FrameInfo>::SharedPtr mFrameInfoPub;     ///< topic: `frameInfo`
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr mPrimaryCameraInfoPub;  ///< topic: `frameInfo/currentCamera`
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr mColorCameraInfoPub;    ///< topic: `frameInfo/currentColorCamera`
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudPub;        ///< topic: `point_cloud` (combined mode)
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointsPub;            ///< topic: `points` (individual mode)
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mNormalMapPub;               ///< topic: `normals` (32FC3)
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mDepthMapPub;                ///< topic: `depth` (32FC1, mm)
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mConfidenceMapPub;           ///< topic: `confidence` (32FC1)
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mEventMapPub;                ///< topic: `event` (32FC1, MotionCam only)
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mTexturePub;                 ///< topic: `intensity` (32FC1 grayscale)
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mTextureRgbPub;              ///< topic: `texture` (rgb8 structured-light)
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr mColorCameraImagePub;        ///< topic: `color_camera_image` (rgb8)
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mRebootService;             ///< service: `~/reboot`
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mShutdownService;           ///< service: `~/shutdown`
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mFactoryResetService;       ///< service: `~/factory_reset`
    rclcpp::Service<phoxi_camera_msgs::srv::LogDownload>::SharedPtr mLogDownloadService;   ///< service: `~/log_download`
    rclcpp::Service<phoxi_camera_msgs::srv::Connect>::SharedPtr mConnectService;           ///< service: `~/connect` (deprecated — prefer lifecycle configure)
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mDisconnectService;                 ///< service: `~/disconnect` (deprecated — prefer lifecycle cleanup)
    rclcpp::Service<phoxi_camera_msgs::srv::TriggerFrame>::SharedPtr mTriggerFrameService; ///< service: `~/trigger_frame`
    rclcpp::Service<phoxi_camera_msgs::srv::GetProfileList>::SharedPtr mGetProfileListService;     ///< service: `~/profiles/list`
    rclcpp::Service<phoxi_camera_msgs::srv::GetActiveProfile>::SharedPtr mGetActiveProfileService; ///< service: `~/profiles/get_active`
    rclcpp::Service<phoxi_camera_msgs::srv::SetActiveProfile>::SharedPtr mSetActiveProfileService; ///< service: `~/profiles/set_active`
    rclcpp::Service<phoxi_camera_msgs::srv::CreateProfile>::SharedPtr mCreateProfileService;       ///< service: `~/profiles/create`
    rclcpp::Service<phoxi_camera_msgs::srv::DeleteProfile>::SharedPtr mDeleteProfileService;       ///< service: `~/profiles/delete`
    rclcpp::Service<phoxi_camera_msgs::srv::UpdateProfile>::SharedPtr mUpdateProfileService;       ///< service: `~/profiles/update`
    rclcpp::Service<phoxi_camera_msgs::srv::GetStartupProfile>::SharedPtr mGetStartupProfileService; ///< service: `~/profiles/get_startup`
    rclcpp::Service<phoxi_camera_msgs::srv::SetStartupProfile>::SharedPtr mSetStartupProfileService; ///< service: `~/profiles/set_startup`
    rclcpp::Service<phoxi_camera_msgs::srv::ExportProfile>::SharedPtr mExportProfileService;         ///< service: `~/profiles/export`
    rclcpp::Service<phoxi_camera_msgs::srv::ImportProfile>::SharedPtr mImportProfileService;         ///< service: `~/profiles/import`
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mResetActiveProfileService;                   ///< service: `~/profiles/reset`
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr mParamCallbackHandle; ///< Handle for the on-set-parameters callback.
    rclcpp::PreShutdownCallbackHandle mShutdownCallbackHandle; ///< Triggers lifecycle shutdown before the context is destroyed.
    std::mutex mFrameMutex;     ///< Guards onFrameCallback; used by drainFrameCallback to wait for in-flight callbacks.
    std::mutex mDeviceIdMutex;  ///< Guards mDeviceId which may be read from service callbacks.
    std::string mDeviceId;      ///< Hardware identification of the current or target device.
    std::string mFrameId;       ///< TF frame ID stamped on all published messages.
    bool mPublishCombined = false; ///< When true, publish `point_cloud` only; otherwise publish individual topics.

    std::vector<SettingDescriptor> mSettingDescriptors; ///< One entry per device setting exposed as a ROS2 parameter.
    /** @brief Maps each ROS2 parameter name to its setting descriptor index and sub-field name.
     *
     * Sub-field name is empty for scalar types; for object types (e.g. ROI) it names the field
     * (e.g. `"x_min"`).
     */
    std::map<std::string, std::pair<size_t, std::string>> mParamToDescriptor;
    bool mDeclaringDeviceSettings = false; ///< Set to true during declareDeviceSettingParameters to suppress device calls.
};

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROSINTERFACE_H
