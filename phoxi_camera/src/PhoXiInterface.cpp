#include "phoxi_camera/PhoXiInterface.h"

#include <string>

#include "phoxi/details/PhoXiConstants.h"
#include "phoxi/details/jsoncons/json.hpp"
#include "phoxi_camera/PhoXiConversions.h"

namespace
{
pho_jsoncons::json createCommand(const std::string& command, const std::string& operation = "") {
    pho_jsoncons::json req(pho_jsoncons::json_object_arg);
    req.insert_or_assign("command", command);
    if (!operation.empty()) {
        req.insert_or_assign("operation", operation);
    }
    req.insert_or_assign("params", pho_jsoncons::json(pho_jsoncons::json_object_arg));
    return req;
}

pho_jsoncons::json createDeviceCommand(const std::string& command, const std::string& deviceId, const std::string& operation = "") {
    auto req = createCommand(command, operation);
    req["params"].insert_or_assign("device_id", deviceId);
    return req;
}

pho_jsoncons::json executeCommand(const pho_jsoncons::json& request) {
    std::string reqStr;
    request.dump(reqStr);

    const std::string commandName = request.contains("command") ? request["command"].as<std::string>() : "unknown";
    std::string response;
    const int err = phoxi_command_execute(
        reqStr.c_str(), reqStr.size(),
        pho::api::details::InMillis(pho::api::DEFAULT_COMMAND_TIMEOUT),
        &pho::api::details::CommandStringConverter,
        &response);

    switch (err) {
        case PHOXI_COMMAND_OK:
            break;
        case PHOXI_COMMAND_TIMEOUT:
            throw phoxi_camera::PhoXiInterfaceException("'" + commandName + "' command timed out.");
        default: {
            std::string errMsg = "'" + commandName + "' command failed";
            const char* lastError = nullptr;
            phoxi_error_last(&lastError);
            if (lastError) {
                errMsg += ": ";
                errMsg += lastError;
            }
            throw phoxi_camera::PhoXiInterfaceException(errMsg);
        }
    }

    if (response.empty()) {
        throw phoxi_camera::PhoXiInterfaceException("'" + commandName + "' command returned empty response.");
    }

    try {
        const auto resp = pho_jsoncons::json::parse(response);

        // Legacy "answer" format
        if (resp.contains("answer")) {
            const auto& answer = resp["answer"];
            const std::string error = answer.contains("error") ? answer["error"].as<std::string>() : "";
            const std::string message = answer.contains("message") ? answer["message"].as<std::string>() : "";
            if (!error.empty() && error != "no error") {
                std::string errMsg = "'" + commandName + "' error: " + error;
                if (!message.empty()) { errMsg += ", " + message; }
                throw phoxi_camera::PhoXiInterfaceException(errMsg);
            }
            return resp;
        }

        // Modern "response" format
        if (resp.contains("response")) {
            const auto& r = resp["response"];
            const std::string status = r.contains("status") ? r["status"].as<std::string>() : "";
            const std::string message = r.contains("message") ? r["message"].as<std::string>() : "";
            if (status == "ok") {
                return resp;
            }
            std::string errMsg = "'" + commandName + "' error";
            if (!message.empty()) { errMsg += ": " + message; }
            throw phoxi_camera::PhoXiInterfaceException(errMsg);
        }

        throw phoxi_camera::PhoXiInterfaceException("'" + commandName + "' invalid response.");
    } catch (const phoxi_camera::PhoXiInterfaceException&) {
        throw;
    } catch (const std::exception& e) {
        throw phoxi_camera::PhoXiInterfaceException(
            "'" + commandName + "' failed to parse response: " + e.what());
    }
}

} // namespace

namespace phoxi_camera
{

std::vector<PhoXiDeviceInformation> PhoXiInterface::deviceList() {
    if (!mPhoXiFactory.isPhoXiControlRunning()) {
        mPhoXiDevice.Reset();
        throw PhoXiControlNotRunning("PhoXi Control is not running");
    }
    std::vector<PhoXiDeviceInformation> deviceInfo;
    toPhoXiCameraDeviceInforamtion(mPhoXiFactory.GetDeviceList(), deviceInfo);
    return deviceInfo;
}

std::vector<std::string> PhoXiInterface::cameraList() {
    std::vector<std::string> deviceIdList;
    for (const auto& device : deviceList()) {
        deviceIdList.push_back(device);
    }
    return deviceIdList;
}

void PhoXiInterface::connectCamera(const std::string& deviceId, GetFrameCallback&& getFrameCallback) {
    if (deviceId.empty()) {
        throw PhoXiInterfaceException("Device ID can not be empty.");
    }

    if (!getFrameCallback) {
        throw PhoXiInterfaceException("Frame callback must not be null.");
    }

    if (mPhoXiDevice && isConnected()) {
        if (mPhoXiDevice->HardwareIdentification.GetStoredValue() != deviceId) {
            throw UnableToConnect("Device still connected, disconnect first.");
        }
        std::lock_guard<std::mutex> lock(mFrameCallbackMutex);
        mFrameCallback = std::move(getFrameCallback);
        return;
    }

    mPhoXiDevice = mPhoXiFactory.CreateAndConnect(deviceId, CONNECTION_TIMEOUT_MS);
    if (!mPhoXiDevice) {
        throw UnableToConnect("Device was not able to connect. Disconnected.");
    }

    setTriggerMode(pho::api::PhoXiTriggerMode::Software);

    if (phoxi_aframe_enable(deviceId.c_str(), &PhoXiInterface::frameAcceptor, this) != PHOXI_OK) {
        const char* errMsg = nullptr;
        phoxi_error_last(&errMsg);
        disconnectCamera();
        throw UnableToStartAcquisition(errMsg ? errMsg : "Failed to enable async frame receiving.");
    }

    {
        std::lock_guard<std::mutex> lock(mFrameCallbackMutex);
        mFrameCallback = std::move(getFrameCallback);
    }
}

void PhoXiInterface::disconnectCamera() {
    if (mPhoXiDevice) {
        phoxi_aframe_disable(mPhoXiDevice->HardwareIdentification.GetStoredValue().c_str());
        mPhoXiDevice->Disconnect(true, true);
    }

    mPhoXiDevice.Reset();

    {
        std::lock_guard<std::mutex> lock(mFrameCallbackMutex);
        mFrameCallback = nullptr;
    }
}

void PhoXiInterface::triggerFrame(bool waitGrabbingEnd) {
    isOk();
    triggerImage(waitGrabbingEnd);
}

void PhoXiInterface::isOk() const {
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

pho::api::PhoXiTriggerMode PhoXiInterface::getTriggerMode() const {
    isOk();
    return mPhoXiDevice->TriggerMode;
}

void PhoXiInterface::setTriggerMode(pho::api::PhoXiTriggerMode mode) {
    switch (mode) {
        case pho::api::PhoXiTriggerMode::Software:
        case pho::api::PhoXiTriggerMode::Freerun:
            break;
        default:
            throw InvalidTriggerMode("Unsupported trigger mode " + std::to_string(mode) + ".");
    }

    isOk();
    if (mode == mPhoXiDevice->TriggerMode.GetValue()) {
        return;
    }

    const auto isAcquiring = mPhoXiDevice->isAcquiring();
    if (isAcquiring) {
        stopAcquisition();
    }

    mPhoXiDevice->TriggerMode = mode;
    if (!mPhoXiDevice->TriggerMode.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to set trigger mode: " + std::to_string(mode) + ".");
    }

    if (isAcquiring) {
        startAcquisition();
    }
}

std::vector<pho::api::PhoXiProfileDescriptor> PhoXiInterface::getProfileList() {
    isOk();
    std::vector<pho::api::PhoXiProfileDescriptor> list = mPhoXiDevice->Profiles;
    if (!mPhoXiDevice->Profiles.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to get profile list: " + mPhoXiDevice->Profiles.GetLastErrorMessage());
    }
    return list;
}

std::string PhoXiInterface::getActiveProfile() {
    isOk();
    std::string name = mPhoXiDevice->ActiveProfile;
    if (!mPhoXiDevice->ActiveProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to get active profile: " + mPhoXiDevice->ActiveProfile.GetLastErrorMessage());
    }
    return name;
}

void PhoXiInterface::setActiveProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->ActiveProfile = name;
    if (!mPhoXiDevice->ActiveProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to set active profile: " + mPhoXiDevice->ActiveProfile.GetLastErrorMessage());
    }
}

std::string PhoXiInterface::getStartupProfile() {
    isOk();
    std::string name = mPhoXiDevice->StartupProfile;
    if (!mPhoXiDevice->StartupProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to get startup profile: " + mPhoXiDevice->StartupProfile.GetLastErrorMessage());
    }
    return name;
}

void PhoXiInterface::setStartupProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->StartupProfile = name;
    if (!mPhoXiDevice->StartupProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to set startup profile: " + mPhoXiDevice->StartupProfile.GetLastErrorMessage());
    }
}

void PhoXiInterface::createProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->CreateProfile = name;
    if (!mPhoXiDevice->CreateProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to create profile: " + mPhoXiDevice->CreateProfile.GetLastErrorMessage());
    }
}

void PhoXiInterface::deleteProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->DeleteProfile = name;
    if (!mPhoXiDevice->DeleteProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to delete profile: " + mPhoXiDevice->DeleteProfile.GetLastErrorMessage());
    }
}

void PhoXiInterface::updateProfile(const std::string& name) {
    isOk();
    mPhoXiDevice->UpdateProfile = name;
    if (!mPhoXiDevice->UpdateProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to update profile: " + mPhoXiDevice->UpdateProfile.GetLastErrorMessage());
    }
}

pho::api::PhoXiProfileContent PhoXiInterface::exportProfile() {
    isOk();
    pho::api::PhoXiProfileContent content = mPhoXiDevice->ExportProfile;
    if (!mPhoXiDevice->ExportProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to export profile: " + mPhoXiDevice->ExportProfile.GetLastErrorMessage());
    }
    return content;
}

void PhoXiInterface::importProfile(const pho::api::PhoXiProfileContent& content) {
    isOk();
    mPhoXiDevice->ImportProfile = content;
    if (!mPhoXiDevice->ImportProfile.isLastOperationSuccessful()) {
        throw PhoXiInterfaceException("Failed to import profile: " + mPhoXiDevice->ImportProfile.GetLastErrorMessage());
    }
}

void PhoXiInterface::resetActiveProfile() {
    isOk();
    if (!mPhoXiDevice->ResetActivePreset()) {
        throw PhoXiInterfaceException("Failed to reset active profile.");
    }
}

void PhoXiInterface::setFrameOutputSettings(const std::vector<std::pair<std::string, bool>>& components) {
    isOk();
    const std::string deviceId = mPhoXiDevice->HardwareIdentification.GetStoredValue();

    auto req = createDeviceCommand("frame_settings", deviceId, "set");
    pho_jsoncons::json componentsJson(pho_jsoncons::json_object_arg);
    for (const auto& [name, enabled] : components) {
        componentsJson.insert_or_assign(name, enabled);
    }
    req["params"].insert_or_assign("components", std::move(componentsJson));

    executeCommand(req);
}

void PhoXiInterface::frameAcceptor(const phoxi_frame_record_t* records, void* userData) {
    auto* self = static_cast<PhoXiInterface*>(userData);

    std::lock_guard<std::mutex> lock(self->mFrameCallbackMutex);

    if (!self->mFrameCallback || !records || records->type == PHOXI_FRAME_TYPE_EMPTY) {
        return;
    }

    PhoXiFrame frame;
    for (const auto* r = records; r->type != PHOXI_FRAME_TYPE_EMPTY; ++r) {
        if (!r->data) {
            continue;
        }
        switch (r->type) {
            case PHOXI_FRAME_TYPE_POINTCLOUD: frame.pointCloud = r; break;
            case PHOXI_FRAME_TYPE_NORMALMAP: frame.normalMap = r; break;
            case PHOXI_FRAME_TYPE_DEPTHMAP: frame.depthMap = r; break;
            case PHOXI_FRAME_TYPE_CONFIDENCEMAP: frame.confidenceMap = r; break;
            case PHOXI_FRAME_TYPE_EVENTMAP: frame.eventMap = r; break;
            case PHOXI_FRAME_TYPE_TEXTURE:
                if (r->format == PHOXI_FRAME_FORMAT_RGB_16) {
                    frame.textureRgb = r;
                } else {
                    frame.texture = r;
                }
                break;
            case PHOXI_FRAME_TYPE_COLORCAMERAIMAGE: frame.colorCamera = r; break;
            default: break;
        }
    }

    self->mFrameCallback(frame);
}

}  // namespace phoxi_camera
