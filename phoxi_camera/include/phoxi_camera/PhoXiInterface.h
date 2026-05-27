#ifndef PHOXI_CAMERA_PHOXIINTERFACE_H
#define PHOXI_CAMERA_PHOXIINTERFACE_H

#include <memory>
#include <string>
#include <vector>

#include "PhoXi.h"
#include "phoxi_camera/PhoXiDeviceInformation.h"
#include "phoxi_camera/PhoXiException.h"

namespace phoxi_camera
{
class PhoXiInterface
{
  public:
    using GetFrameCb = std::function<void(pho::api::PFrame)>;

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
     * Connect to camera.
     *
     * \param deviceId - identification number
     * \param getFrameCallback - asynchronous notifications on arriving frames
     * \param mode - trigger mode to set after connection
     * \param startAcquisition if true Acquisition will be started
     * \throw PhoXiControlNotRunning when PhoXi Control is not running
     * \throw PhoXiDeviceNotFound when PhoXi Device with HWIdentification is not available
     * \throw UnableToStartAcquisition when connection failed
     */
    virtual void connectCamera(const std::string& deviceId, GetFrameCb&& getFrameCallback);

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
    virtual void triggerFrame(bool waitGrabbingEnd = false);

    /**
     * Test if connection to PhoXi Device is working
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     */
    void isOk();

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
     * Set trigger mode
     *
     * \param mode new trigger mode
     * \param startAcquisition if true Acquisition will be started
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw InvalidTriggerMode when invalid trigger mode is passed
     */
    void setTriggerMode(pho::api::PhoXiTriggerMode mode);

    /**
     * Get trigger mode
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     */
    pho::api::PhoXiTriggerMode getTriggerMode();

    pho::api::PPhoXi mPhoXiDevice;

  protected:
    pho::api::PhoXiFactory mPhoXiFactory;

  private:
    const int CONNECTION_TIMEOUT_MS = 60000;
};
}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIINTERFACE_H
