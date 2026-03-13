#ifndef PHOXI_CAMERA_PHOXIINTERFACE_H
#define PHOXI_CAMERA_PHOXIINTERFACE_H

#include <memory>
#include <string>
#include <vector>

#include "PhoXi.h"
#include "phoxi_camera/PhoXiDeviceInformation.h"
#include "phoxi_camera/PhoXiException.h"

// * PhoXiInterface
/**
 * Wrapper to PhoXi Device api to make interface easier
 *
 */
namespace phoxi_camera
{
class PhoXiInterface
{
  public:
    using GetFrameCb = std::function<void(pho::api::PFrame)>;

    /**
     * Default constructor.
     */
    PhoXiInterface() = default;

    /**
     * Return all PhoXi Devices ids connected on network with all information about devices.
     *
     * \throw PhoXiControlNotRunning when PhoXi Control is not running
     */
    std::vector<PhoXiDeviceInformation> deviceList();

    /**
     * Return all PhoXi Devices ids connected on network.
     *
     * \note returned id can be passed to connectCamera method
     * \throw PhoXiControlNotRunning when PhoXi Control is not running
     */
    std::vector<std::string> cameraList();

    /**
     * Connect to camera.
     *
     * \param HWIdentification - identification number
     * \param getFrameCallback - asynchronous notifications on arriving frames
     * \param mode - trigger mode to set after connection
     * \param startAcquisition if true Acquisition will be started
     * \throw PhoXiControlNotRunning when PhoXi Control is not running
     * \throw PhoXiDeviceNotFound when PhoXi Device with HWIdentification is not available on
     * network \throw UnableToStartAcquisition when connection failed
     */
    virtual void connectCamera(
        std::string HWIdentification, GetFrameCb&& getFrameCallback,
        pho::api::PhoXiTriggerMode mode = pho::api::PhoXiTriggerMode::Software,
        bool startAcquisition = true);

    /**
     * Disconnect from camera if connected to any.
     */
    virtual void disconnectCamera();

    /**
     * Trigger new image and return PFrame.
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     */
    virtual void triggerFrame();

    /**
     * Test if connection to PhoXi Device is working
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     */
    void isOk();

    /**
     * Test if connection to PhoXi Device is working
     */
    bool isConnected();

    /**
     * Test if PhoXi Device is Acquiring
     */
    bool isAcquiring();

    /**
     * Start acquisition
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw UnableToStartAcquisition if acquisition was not started
     */
    void startAcquisition();

    /**
     * Stop acquisition
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw UnableToStartAcquisition if acquisition was not stopped
     */
    void stopAcquisition();

    /**
     * Trigger new Image
     *
     * \return @return positive id on success, negative number on failure (-1 Trigger not
     * accepted,
     * -2 Device is not running, -3 Communication Error, -4 WaitForGrabbingEnd is not supported)
     * \note id can be passed to getPFrame method
     */
    int triggerImage(bool waitForGrab = false);

    /**
     * Set trigger mode
     *
     * \param mode new trigger mode
     * \param startAcquisition if true Acquisition will be started
     * \note if mode is Freerun new PFrames will be triggered immediately after acquisition is
     * started
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     * \throw InvalidTriggerMode when invalid trigger mode is passed
     */
    void setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition = false);

    /**
     * Get trigger mode
     *
     * \throw PhoXiDeviceNotConnected when no device is connected
     */
    pho::api::PhoXiTriggerMode getTriggerMode();

    pho::api::PPhoXi phoXiDevice;

  protected:
    pho::api::PhoXiFactory phoXiFactory;

  private:
    const int CONNECTION_TIMEOUT_MS = 60000;
};
}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIINTERFACE_H
