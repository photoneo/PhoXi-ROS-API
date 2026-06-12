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

enum class SettingValueType {
    BOOL,
    INT,
    DOUBLE,
    STRING,
    DOUBLE_ARRAY,
    PHOXI_SIZE,
    PHOXI_SIZE_64F,
    PHOXI_2DROI,
    AXIS_VOLUME_64F,
    POINT3_64F,
    CUTTING_PLANES,
    SCANNING_VOLUME,
    SCANNING_VOLUME_MESH,
    REPROJECTION_MAP,
};

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

struct SettingInfo {
    std::string key;
    SettingValueType type;
    bool isSettable;
};

class PhoXiInterface {
public:
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
     */
    virtual void disconnectCamera();

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
     * \param startAcquisition if true Acquisition will be started
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw InvalidTriggerMode when invalid trigger mode is passed
     */
    void setTriggerMode(pho::api::PhoXiTriggerMode mode);

    /**
     * Get the list of saved profiles.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual std::vector<pho::api::PhoXiProfileDescriptor> getProfileList();

    /**
     * Get the name of the currently active profile.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual std::string getActiveProfile();

    /**
     * Activate a profile by name.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual void setActiveProfile(const std::string& name);

    /**
     * Get the name of the startup profile.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual std::string getStartupProfile();

    /**
     * Set the startup profile by name.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual void setStartupProfile(const std::string& name);

    /**
     * Save the current device settings as a new profile.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual void createProfile(const std::string& name);

    /**
     * Delete a profile by name.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual void deleteProfile(const std::string& name);

    /**
     * Overwrite a profile with the current device settings.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual void updateProfile(const std::string& name);

    /**
     * Export the active profile as binary content.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual pho::api::PhoXiProfileContent exportProfile();

    /**
     * Import a profile from binary content.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual void importProfile(const pho::api::PhoXiProfileContent& content);

    /**
     * Reset the active profile to factory defaults.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
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
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual void setFrameOutputSettings(const std::vector<std::pair<std::string, bool>>& components);

    /**
     * Return the setting descriptors built when connectCamera() succeeded.
     * Empty if the device does not expose a schema or the camera is not connected.
     */
    virtual std::vector<SettingInfo> getSettingInfos() const { return mSettingInfos; }

    /**
     * Get a single device setting value.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error or unknown key
     */
    virtual SettingValue getSetting(const std::string& key);

    /**
     * Get multiple device setting values in one round-trip.
     * Keys not returned by the device are omitted from the result.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual std::map<std::string, SettingValue> getSettings(const std::vector<std::string>& keys);

    /**
     * Set a single device setting value.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual void setSetting(const std::string& key, const SettingValue& value);

    /**
     * Set multiple device setting values in one round-trip.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw PhoXiInterfaceException on SDK error
     */
    virtual void setSettings(const std::vector<std::pair<std::string, SettingValue>>& keyValues);

    pho::api::PPhoXi mPhoXiDevice;

protected:
    pho::api::PhoXiFactory mPhoXiFactory;

private:
    static void frameAcceptor(const phoxi_frame_record_t* records, void* userData);
    void loadDeviceSchema();

    const int CONNECTION_TIMEOUT_MS = 60000;
    mutable std::mutex mFrameCallbackMutex;
    GetFrameCallback mFrameCallback;
    std::map<std::string, SettingValueType> mSchemaTypeCache;
    std::vector<SettingInfo> mSettingInfos;
};
}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIINTERFACE_H
