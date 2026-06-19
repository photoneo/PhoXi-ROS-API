/**
 * @file PhoXiInterface.h
 * @brief Abstraction layer over the Photoneo PhoXi API for connection, acquisition, and settings.
 *
 * `PhoXiInterface` owns the API device handle and exposes a virtual API so that the
 * `PhoXiCamera` node can be tested with mock implementations.
 *
 * Device settings are accessed through a schema-driven typed API:
 * - `getSettingInfos()` returns all available settings and their types after `connectCamera()`.
 * - `getSetting()` / `setSetting()` and the batch variants operate on `SettingValue` variants.
 */
#ifndef PHOXI_CAMERA_PHOXIINTERFACE_H
#define PHOXI_CAMERA_PHOXIINTERFACE_H

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include "PhoXi.h"
#include "phoxi_camera/PhoXiDeviceInformation.h"
#include "phoxi_camera/PhoXiException.h"
#include "phoxi_camera/PhoXiFrame.h"

namespace phoxi_camera {

/**
 * @brief Discriminator tag for the `SettingValue` variant.
 *
 * Each enumerator maps to the corresponding C++ type held by `SettingValue`.
 */
enum class SettingValueType {
    BOOL,              ///< `bool`
    INT,               ///< `int64_t`
    DOUBLE,            ///< `double`
    STRING,            ///< `std::string`
    DOUBLE_ARRAY,      ///< `std::vector<double>`
    PHOXI_SIZE,        ///< `pho::api::PhoXiSize` — integer {Width, Height}
    PHOXI_SIZE_64F,    ///< `pho::api::PhoXiSize_64f` — floating-point {Width, Height}
    PHOXI_2DROI,       ///< `pho::api::PhoXi2DROI` — 2D region of interest
    AXIS_VOLUME_64F,   ///< `pho::api::AxisVolume_64f` — axis-aligned 3D bounding box
    POINT3_64F,        ///< `pho::api::Point3_64f` — 3D point / RGB triplet
    CUTTING_PLANES,    ///< `std::vector<pho::api::Plane_64f>` — list of cutting planes
    SCANNING_VOLUME,   ///< `pho::api::ProjectionGeometry_64f` — scanning volume geometry
    SCANNING_VOLUME_MESH, ///< `pho::api::PhoXiMesh` — scanning volume as a mesh
    REPROJECTION_MAP,  ///< `pho::api::PhoXiReprojectionMap` — pixel-to-3D reprojection map
};

/**
 * @brief Strongly-typed container for a single device setting value.
 *
 * The active alternative is identified by the matching `SettingValueType` enumerator.
 * Use `std::get<T>()` or `std::visit()` to access the value.
 */
using SettingValue = std::variant<
    bool,
    int64_t,
    double,
    std::string,
    std::vector<double>,
    pho::api::PhoXiSize,
    pho::api::PhoXiSize_64f,
    pho::api::PhoXi2DROI,
    pho::api::AxisVolume_64f,
    pho::api::Point3_64f,
    std::vector<pho::api::Plane_64f>,
    pho::api::ProjectionGeometry_64f,
    pho::api::PhoXiMesh,
    pho::api::PhoXiReprojectionMap
>;

/** @brief Metadata for a single device setting returned by the API schema. */
struct SettingInfo {
    std::string key;       ///< API-side slash-separated path (e.g. `"Scanning/Mode"`).
    SettingValueType type; ///< Value type discriminator.
    bool isSettable;       ///< False if the device reports this setting as read-only.
};

/** @brief Metadata for a single frame output component returned by the API schema. */
struct FrameComponentInfo {
    std::string name;  ///< Component name (e.g. `"PointCloud"`, `"NormalMap"`).
    bool isSettable;   ///< False if the device reports this component as read-only.
};

/**
 * @brief Virtual interface to the Photoneo PhoXi API.
 *
 * Provides device discovery, connection, acquisition control, frame triggering,
 * profile management, and a schema-driven typed settings API.
 *
 * All `virtual` methods can be overridden by mock implementations for unit testing.
 */
class PhoXiInterface {
public:
    /** @brief Callback invoked from the API thread when a new frame arrives. */
    using GetFrameCallback = std::function<void(const PhoXiFrame&)>;

    PhoXiInterface() = default;
    virtual ~PhoXiInterface() = default;

    /**
     * Return all PhoXi Devices ids connected on network with all information about devices.
     *
     * \throw PhoXiControlNotRunning when PhoXi Control is not running
     */
    std::vector<PhoXiDeviceInformation> deviceList();

    /**
     * Return all PhoXi Devices ids connected on network.
     *
     * \throw PhoXiControlNotRunning when PhoXi Control is not running
     */
    std::vector<std::string> cameraList();

    /**
     * Return device information for the currently connected device.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     */
    virtual PhoXiDeviceInformation getDeviceInfo();

    /**
     * Connect to camera.
     *
     * \param deviceId - identification number
     * \param getFrameCallback - asynchronous notifications on arriving frames
     * \throw PhoXiControlNotRunning when PhoXi Control is not running
     * \throw PhoXiDeviceNotFound when PhoXi Device with HWIdentification is not available
     * \throw UnableToStartAcquisition when connection failed
     */
    virtual void connectCamera(const std::string& deviceId, GetFrameCallback&& getFrameCallback);

    /**
     * Disconnect from camera if connected to any.
     *
     * Equivalent to disconnectCamera(true, true)
     */
    virtual void disconnectCamera();

    /**
     * Disconnect from camera if connected to any.
     *
     * \param logoutDevice - logout device from PhoXi Control application on disconnect
     * \param stopDeviceAcquisition - stop device acquisition on device disconnect
     */
    virtual void disconnectCamera(bool logoutDevice, bool stopDeviceAcquisition);

    /**
     * Trigger new image.
     *
     * \param waitGrabbingEnd if true waits for the grabbing to complete before returning
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw UnableToTriggerFrame when triggering fails
     */
    virtual void triggerFrame(bool waitGrabbingEnd);

    /**
     * Test if connection to PhoXi Device is working
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     */
    void isOk() const;

    /**
     * Test if connection to PhoXi Device is working
     */
    virtual bool isConnected();

    /**
     * Test if PhoXi Device is Acquiring
     */
    virtual bool isAcquiring();

    /**
     * Start acquisition
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw UnableToStartAcquisition if acquisition was not started
     */
    virtual void startAcquisition();

    /**
     * Stop acquisition
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw UnableToStopAcquisition if acquisition was not stopped
     */
    virtual void stopAcquisition();

    /**
     * Trigger new Image
     *
     * \return positive id on success, negative number on failure
     * (-1 Trigger not accepted, -2 Device is not running, -3 Communication Error,
     *  -4 WaitForGrabbingEnd is not supported)
     */
    int triggerImage(bool waitForGrab = false);

    /**
     * Get trigger mode
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     */
    pho::api::PhoXiTriggerMode getTriggerMode() const;

    /**
     * Set trigger mode
     *
     * \param mode new trigger mode
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw InvalidTriggerMode when invalid trigger mode is passed
     */
    virtual void setTriggerMode(pho::api::PhoXiTriggerMode mode);

    /**
     * Get the list of saved profiles.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual std::vector<pho::api::PhoXiProfileDescriptor> getProfileList();

    /**
     * Get the name of the currently active profile.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual std::string getActiveProfile();

    /**
     * Activate a profile by name.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void setActiveProfile(const std::string& name);

    /**
     * Get the name of the startup profile.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual std::string getStartupProfile();

    /**
     * Set the startup profile by name.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void setStartupProfile(const std::string& name);

    /**
     * Save the current device settings as a new profile.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void createProfile(const std::string& name);

    /**
     * Delete a profile by name.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void deleteProfile(const std::string& name);

    /**
     * Overwrite a profile with the current device settings.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void updateProfile(const std::string& name);

    /**
     * Export the active profile as binary content.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual pho::api::PhoXiProfileContent exportProfile();

    /**
     * Import a profile from binary content.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void importProfile(const pho::api::PhoXiProfileContent& content);

    /**
     * Reset the active profile to factory defaults.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void resetActiveProfile();

    /**
     * Reboot the device. Does not require an active connection.
     *
     * \throw PhoXiInterfaceException on failure
     */
    virtual void rebootDevice(const std::string& deviceId);

    /**
     * Shut down the device. Does not require an active connection.
     *
     * \throw PhoXiInterfaceException on failure
     */
    virtual void shutdownDevice(const std::string& deviceId);

    /**
     * Reset the device to factory defaults. Does not require an active connection.
     *
     * \throw PhoXiInterfaceException on failure
     */
    virtual void factoryResetDevice(const std::string& deviceId);

    /**
     * Download device logs to a file. Does not require an active connection.
     *
     * \throw PhoXiInterfaceException on failure
     */
    virtual void downloadDeviceLog(const std::string& deviceId, const std::string& logfilePath, bool overwrite);

    /**
     * Set which data components the device should include in each frame.
     * Only the listed components are updated; omitted components retain their current device setting.
     *
     * \param components pairs of (component name, enabled), e.g. {{"PointCloud", true}, {"Texture", false}}
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void setFrameOutputSettings(const std::vector<std::pair<std::string, bool>>& components);

    /**
     * Return the setting descriptors built when connectCamera() succeeded.
     * Empty if the device does not expose a schema or the camera is not connected.
     */
    virtual std::vector<SettingInfo> getSettingInfos() const { return mSettingInfos; }

    /**
     * Get the current enabled/disabled state of the given frame output components.
     *
     * \param componentNames Component names to query (e.g. `"PointCloud"`, `"NormalMap"`).
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual std::map<std::string, bool> getFrameOutputSettings(const std::vector<std::string>& componentNames);

    /**
     * Get a single device setting value.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error or unknown key
     */
    virtual SettingValue getSetting(const std::string& key);

    /**
     * Get multiple device setting values in one round-trip.
     * Keys not returned by the device are omitted from the result.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual std::map<std::string, SettingValue> getSettings(const std::vector<std::string>& keys);

    /**
     * Set a single device setting value.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void setSetting(const std::string& key, const SettingValue& value);

    /**
     * Set multiple device setting values in one round-trip.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on API error
     */
    virtual void setSettings(const std::vector<std::pair<std::string, SettingValue>>& keyValues);

    pho::api::PPhoXi mPhoXiDevice; ///< API device handle; valid after a successful `connectCamera()`.

protected:
    pho::api::PhoXiFactory mPhoXiFactory; ///< API factory used for device discovery and construction.

private:
    static void frameAcceptor(const phoxi_frame_record_t* records, void* userData);
    void loadDeviceSchema();

    const int CONNECTION_TIMEOUT_MS = 60000; ///< Maximum time in ms to wait for a device connection.
    mutable std::mutex mFrameCallbackMutex;  ///< Guards mFrameCallback against concurrent access.
    GetFrameCallback mFrameCallback;         ///< User-supplied callback invoked on each frame.
    std::map<std::string, SettingValueType> mSchemaTypeCache; ///< Cached type mapping from the device schema.
    std::vector<SettingInfo> mSettingInfos;          ///< All settings discovered from the device schema on connect.
};
}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIINTERFACE_H
