#include "phoxi_camera/PhoXiInterface.h"

#include "phoxi_camera/PhoXiConversions.h"

namespace phoxi_camera
{
std::vector<PhoXiDeviceInformation> PhoXiInterface::deviceList() {
    if (!mPhoXiFactory.isPhoXiControlRunning()) {
        mPhoXiDevice.Reset();
        throw PhoXiControlNotRunning("PhoXi Control is not running");
    }
    std::vector<PhoXiDeviceInformation> deviceInfo;
    auto dl = mPhoXiFactory.GetDeviceList();
    toPhoXiCameraDeviceInforamtion(dl, deviceInfo);
    return deviceInfo;
}

std::vector<std::string> PhoXiInterface::cameraList() {
    auto dl = deviceList();
    std::vector<std::string> hwIdentificationList;
    for (const auto& device : dl) {
        hwIdentificationList.push_back(device);
    }
    return hwIdentificationList;
}

void PhoXiInterface::connectCamera(const std::string& deviceId, GetFrameCb&& getFrameCallback) {
    if (isConnected()) {
        if (mPhoXiDevice->HardwareIdentification == deviceId) {
            setTriggerMode(pho::api::PhoXiTriggerMode::Software);
            mPhoXiDevice->EnableAsyncGetFrame(std::move(getFrameCallback));
            return;
        }
    }

    mPhoXiDevice = mPhoXiFactory.CreateAndConnect(deviceId, CONNECTION_TIMEOUT_MS);
    if (!mPhoXiDevice) {
        throw UnableToStartAcquisition("Device was not able to connect. Disconnected.");
    }
    
    setTriggerMode(pho::api::PhoXiTriggerMode::Software);
    mPhoXiDevice->EnableAsyncGetFrame(std::move(getFrameCallback));
}

void PhoXiInterface::disconnectCamera() {
    if (mPhoXiDevice) {
        mPhoXiDevice->Disconnect(true, true);
    }
    mPhoXiDevice.Reset();
}

void PhoXiInterface::triggerFrame(bool waitGrabbingEnd) {
    isOk();
    try {
        triggerImage(waitGrabbingEnd);
    } catch (UnableToTriggerFrame&) {
        throw;
    }
}

void PhoXiInterface::isOk() {
    if (!mPhoXiDevice || !mPhoXiDevice->isConnected()) {
        throw PhoXiDeviceNotConnected("No device connected");
    }
}

bool PhoXiInterface::isConnected() {
    return (mPhoXiDevice && mPhoXiDevice->isConnected());
}

bool PhoXiInterface::isAcquiring() {
    return (mPhoXiDevice && mPhoXiDevice->isAcquiring());
}

void PhoXiInterface::startAcquisition() {
    isOk();
    if (mPhoXiDevice->isAcquiring()) {
        return;
    }
    mPhoXiDevice->StartAcquisition();
    if (!mPhoXiDevice->isAcquiring()) {
        throw UnableToStartAcquisition("Unable to start acquisition.");
    }
}

void PhoXiInterface::stopAcquisition() {
    isOk();
    if (!mPhoXiDevice->isAcquiring()) {
        return;
    }
    mPhoXiDevice->StopAcquisition();
    if (mPhoXiDevice->isAcquiring()) {
        throw UnableToStopAcquisition("Unable to stop acquisition.");
    }
}

int PhoXiInterface::triggerImage(bool waitForGrab) {
    setTriggerMode(pho::api::PhoXiTriggerMode::Software);
    int frameId = mPhoXiDevice->TriggerFrame(true, waitForGrab);

    if (frameId < 0) {
        switch (frameId) {
            case -1:
                throw UnableToTriggerFrame("Trigger not accepted.");
            case -2:
                throw UnableToTriggerFrame("Device is not running.");
            case -3:
                throw UnableToTriggerFrame("Communication Error.");
            case -4:
                throw UnableToTriggerFrame("WaitForGrabbingEnd is not supported.");
            case -5:
                throw UnableToTriggerFrame("Timeout.");
            case -6:
                throw UnableToTriggerFrame("Device is not connected.");
            case -7:
                throw UnableToTriggerFrame("Device is not acquiring.");
            default:
                throw UnableToTriggerFrame("Unknown error.");
        }
    }
    return frameId;
}

void PhoXiInterface::setTriggerMode(pho::api::PhoXiTriggerMode mode) {
    if (!((mode == pho::api::PhoXiTriggerMode::Software) ||
          (mode == pho::api::PhoXiTriggerMode::Freerun))) {
        throw InvalidTriggerMode("Invalid trigger mode " + std::to_string(mode) + ".");
    }
    isOk();
    if (mode == mPhoXiDevice->TriggerMode.GetValue()) {
        return;
    }
    stopAcquisition();
    mPhoXiDevice->TriggerMode = mode;
}

pho::api::PhoXiTriggerMode PhoXiInterface::getTriggerMode() {
    isOk();
    return mPhoXiDevice->TriggerMode;
}

}  // namespace phoxi_camera
