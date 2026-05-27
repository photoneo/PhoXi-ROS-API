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

std::vector<pho::api::PhoXiProfileDescriptor> PhoXiInterface::getProfileList() {
    isOk();
    std::vector<pho::api::PhoXiProfileDescriptor> list = mPhoXiDevice->Profiles;
    if (!mPhoXiDevice->Profiles.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to get profile list: " +
                                      mPhoXiDevice->Profiles.GetLastErrorMessage());
    }
    return list;
}

std::string PhoXiInterface::getActiveProfile() {
    isOk();
    std::string name = mPhoXiDevice->ActiveProfile;
    if (!mPhoXiDevice->ActiveProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to get active profile: " +
                                      mPhoXiDevice->ActiveProfile.GetLastErrorMessage());
    }
    return name;
}

void PhoXiInterface::setActiveProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->ActiveProfile = name;
    if (!mPhoXiDevice->ActiveProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to set active profile: " +
                                      mPhoXiDevice->ActiveProfile.GetLastErrorMessage());
    }
}

std::string PhoXiInterface::getStartupProfile() {
    isOk();
    std::string name = mPhoXiDevice->StartupProfile;
    if (!mPhoXiDevice->StartupProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to get startup profile: " +
                                      mPhoXiDevice->StartupProfile.GetLastErrorMessage());
    }
    return name;
}

void PhoXiInterface::setStartupProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->StartupProfile = name;
    if (!mPhoXiDevice->StartupProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to set startup profile: " +
                                      mPhoXiDevice->StartupProfile.GetLastErrorMessage());
    }
}

void PhoXiInterface::createProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->CreateProfile = name;
    if (!mPhoXiDevice->CreateProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to create profile: " +
                                      mPhoXiDevice->CreateProfile.GetLastErrorMessage());
    }
}

void PhoXiInterface::deleteProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->DeleteProfile = name;
    if (!mPhoXiDevice->DeleteProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to delete profile: " +
                                      mPhoXiDevice->DeleteProfile.GetLastErrorMessage());
    }
}

void PhoXiInterface::updateProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->UpdateProfile = name;
    if (!mPhoXiDevice->UpdateProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to update profile: " +
                                      mPhoXiDevice->UpdateProfile.GetLastErrorMessage());
    }
}

pho::api::PhoXiProfileContent PhoXiInterface::exportProfile() {
    isOk();
    pho::api::PhoXiProfileContent content = mPhoXiDevice->ExportProfile;
    if (!mPhoXiDevice->ExportProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to export profile: " +
                                      mPhoXiDevice->ExportProfile.GetLastErrorMessage());
    }
    return content;
}

void PhoXiInterface::importProfile(const pho::api::PhoXiProfileContent& content) {
    isOk();
    mPhoXiDevice->ImportProfile = content;
    if (!mPhoXiDevice->ImportProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to import profile: " +
                                      mPhoXiDevice->ImportProfile.GetLastErrorMessage());
    }
}

void PhoXiInterface::resetActiveProfile() {
    isOk();
    if (!mPhoXiDevice->ResetActivePreset()) {
        throw PhoXiInterfaceException("Failed to reset active profile.");
    }
}

}  // namespace phoxi_camera
