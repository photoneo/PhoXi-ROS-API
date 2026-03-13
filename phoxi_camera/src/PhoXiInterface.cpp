#include "phoxi_camera/PhoXiInterface.h"

#include "phoxi_camera/PhoXiConversions.h"

namespace phoxi_camera
{
std::vector<PhoXiDeviceInformation> PhoXiInterface::deviceList() {
    if (!phoXiFactory.isPhoXiControlRunning()) {
        phoXiDevice.Reset();
        throw PhoXiControlNotRunning("PhoXi Control is not running");
    }
    std::vector<PhoXiDeviceInformation> deviceInfo;
    auto dl = phoXiFactory.GetDeviceList();
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

void PhoXiInterface::connectCamera(std::string HWIdentification, GetFrameCb&& getFrameCallback,
                                   pho::api::PhoXiTriggerMode mode, bool startAcquisition) {
    if (this->isConnected()) {
        if (phoXiDevice->HardwareIdentification == HWIdentification) {
            this->setTriggerMode(mode, startAcquisition);
            return;
        }
    }
    std::vector<std::string> cl = cameraList();
    auto it = std::find(cl.begin(), cl.end(), HWIdentification);
    if (it == cl.end()) {
        throw PhoXiDeviceNotFound("Device not found");
    }
    disconnectCamera();
    phoXiDevice.Reset();  // This is needed before assign new instance
    if (!(phoXiDevice = phoXiFactory.CreateAndConnect(*it, CONNECTION_TIMEOUT_MS))) {
        disconnectCamera();
        throw UnableToStartAcquisition("Device was not able to connect. Disconnected.");
    }
    this->setTriggerMode(mode, startAcquisition);
    phoXiDevice->EnableAsyncGetFrame(std::move(getFrameCallback));
}

void PhoXiInterface::disconnectCamera() {
    if (phoXiDevice && phoXiDevice->isConnected()) {
        phoXiDevice->Disconnect(true, true);
    }
    phoXiDevice.Reset();  // Device instance is not usable after disconnect, call destructor
}

void PhoXiInterface::triggerFrame() {
    this->isOk();

    int id;
    try {
        id = this->triggerImage(true);
    } catch (UnableToTriggerFrame& e) {
        throw;
    }

    return;
}

void PhoXiInterface::isOk() {
    if (!phoXiDevice || !phoXiDevice->isConnected()) {
        throw PhoXiDeviceNotConnected("No device connected");
    }
}

bool PhoXiInterface::isConnected() { return (phoXiDevice && phoXiDevice->isConnected()); }

bool PhoXiInterface::isAcquiring() { return (phoXiDevice && phoXiDevice->isAcquiring()); }

void PhoXiInterface::startAcquisition() {
    this->isOk();
    if (phoXiDevice->isAcquiring()) {
        return;
    }
    phoXiDevice->StartAcquisition();
    if (!phoXiDevice->isAcquiring()) {
        throw UnableToStartAcquisition("Unable to start acquisition.");
    }
}

void PhoXiInterface::stopAcquisition() {
    this->isOk();
    if (!phoXiDevice->isAcquiring()) {
        return;
    }
    phoXiDevice->StopAcquisition();
    if (phoXiDevice->isAcquiring()) {
        throw UnableToStopAcquisition("Unable to stop acquisition.");
    }
}

int PhoXiInterface::triggerImage(bool waitForGrab) {
    this->setTriggerMode(pho::api::PhoXiTriggerMode::Software, true);
    int frame_id = phoXiDevice->TriggerFrame(true, waitForGrab);

    if (frame_id < 0) {
        switch (frame_id) {
            case -1:
                throw UnableToTriggerFrame("Trigger not accepted.");
            case -2:
                throw UnableToTriggerFrame("Device is not running.");
            case -3:
                throw UnableToTriggerFrame("Communication Error.");
            case -4:
                throw UnableToTriggerFrame("WaitForGrabbingEnd is not supported.");
            default:
                throw UnableToTriggerFrame("Unknown error.");
        }
    }
    return frame_id;
}

void PhoXiInterface::setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition) {
    if (!((mode == pho::api::PhoXiTriggerMode::Software) ||
          (mode == pho::api::PhoXiTriggerMode::Hardware) ||
          (mode == pho::api::PhoXiTriggerMode::Freerun) ||
          (mode == pho::api::PhoXiTriggerMode::NoValue))) {
        throw InvalidTriggerMode("Invalid trigger mode " + std::to_string(mode) + ".");
    }
    this->isOk();
    if (mode == phoXiDevice->TriggerMode.GetValue()) {
        if (startAcquisition) {
            this->startAcquisition();
        } else {
            this->stopAcquisition();
        }
        return;
    }
    this->stopAcquisition();
    phoXiDevice->TriggerMode = mode;
    if (startAcquisition) {
        this->startAcquisition();
    }
}

pho::api::PhoXiTriggerMode PhoXiInterface::getTriggerMode() {
    this->isOk();
    return phoXiDevice->TriggerMode;
}

}  // namespace phoxi_camera
