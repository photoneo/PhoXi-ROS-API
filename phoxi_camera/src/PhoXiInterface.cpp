#include "phoxi_camera/PhoXiInterface.h"

#include <set>
#include <string>

#include "phoxi/details/PhoXiConstants.h"
#include "phoxi/details/jsoncons/json.hpp"
#include "phoxi_camera/PhoXiConversions.h"

namespace {
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
            reqStr.c_str(), reqStr.size(), pho::api::details::InMillis(pho::api::DEFAULT_COMMAND_TIMEOUT), &pho::api::details::CommandStringConverter, &response);

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

        if (resp.contains("answer")) {
            const auto& answer = resp["answer"];
            const std::string error = answer.contains("error") ? answer["error"].as<std::string>() : "";
            const std::string message = answer.contains("message") ? answer["message"].as<std::string>() : "";
            if (!error.empty() && error != "no error") {
                std::string errMsg = "'" + commandName + "' error: " + error;
                if (!message.empty()) {
                    errMsg += ", " + message;
                }
                throw phoxi_camera::PhoXiInterfaceException(errMsg);
            }
            return resp;
        }

        if (resp.contains("response")) {
            const auto& r = resp["response"];
            const std::string status = r.contains("status") ? r["status"].as<std::string>() : "";
            const std::string message = r.contains("message") ? r["message"].as<std::string>() : "";
            if (status == "ok") {
                return resp;
            }
            std::string errMsg = "'" + commandName + "' error";
            if (!message.empty()) {
                errMsg += ": " + message;
            }
            throw phoxi_camera::PhoXiInterfaceException(errMsg);
        }

        throw phoxi_camera::PhoXiInterfaceException("'" + commandName + "' invalid response.");
    } catch (const phoxi_camera::PhoXiInterfaceException&) {
        throw;
    } catch (const std::exception& e) {
        throw phoxi_camera::PhoXiInterfaceException("'" + commandName + "' failed to parse response: " + e.what());
    }
}

phoxi_camera::SettingValueType classifySettingType(const pho_jsoncons::json& node) {
    using phoxi_camera::SettingValueType;

    if (!node.is_object()) {
        return SettingValueType::DOUBLE_ARRAY;
    }

    // oneOf: all variants have {width, height} → PHOXI_SIZE (e.g. CapturingResolution)
    if (node.contains("oneOf") && node["oneOf"].is_array()) {
        bool allResolution = !node["oneOf"].empty();
        for (const auto& variant : node["oneOf"].array_range()) {
            if (!variant.contains("required")) {
                allResolution = false;
                break;
            }
            bool hasW = false, hasH = false;
            for (const auto& r : variant["required"].array_range()) {
                const auto s = r.as<std::string>();
                if (s == "width")
                    hasW = true;
                if (s == "height")
                    hasH = true;
            }
            if (!hasW || !hasH) {
                allResolution = false;
                break;
            }
        }
        return allResolution ? SettingValueType::PHOXI_SIZE : SettingValueType::DOUBLE_ARRAY;
    }

    const std::string type = node.contains("type") ? node["type"].as<std::string>() : "";

    if (type == "boolean")
        return SettingValueType::BOOL;
    if (type == "integer")
        return SettingValueType::INT;
    if (type == "number")
        return SettingValueType::DOUBLE;
    if (type == "string" || node.contains("enum"))
        return SettingValueType::STRING;

    if (type == "array") {
        if (!node.contains("items") || !node["items"].is_object()) {
            return SettingValueType::DOUBLE_ARRAY;
        }
        const auto& items = node["items"];
        const std::string itemsType = items.contains("type") ? items["type"].as<std::string>() : "";
        if (itemsType == "number" || itemsType == "integer")
            return SettingValueType::DOUBLE_ARRAY;
        if (itemsType == "object" && items.contains("properties")) {
            const auto& p = items["properties"];
            if (p.contains("d") && p.contains("x") && p.contains("y") && p.contains("z")) {
                return SettingValueType::CUTTING_PLANES;
            }
        }
        return SettingValueType::DOUBLE_ARRAY;
    }

    if (type == "object") {
        if (!node.contains("properties"))
            return SettingValueType::DOUBLE_ARRAY;
        const auto& props = node["properties"];

        if (props.contains("data") && props["data"].is_object()) {
            const auto& dataProp = props["data"];
            if (dataProp.contains("contentEncoding") && dataProp["contentEncoding"].as<std::string>() == "base64") {
                return SettingValueType::REPROJECTION_MAP;
            }
        }

        if (props.contains("vertices") && props.contains("indices") && props.contains("points_per_section")) {
            return SettingValueType::SCANNING_VOLUME_MESH;
        }

        if (props.contains("origin") || props.contains("top_left")) {
            return SettingValueType::SCANNING_VOLUME;
        }

        const auto propsCount = props.size();
        auto isInt = [&](const char* k) { return props[k].contains("type") && props[k]["type"].as<std::string>() == "integer"; };

        if (propsCount == 2 && props.contains("height") && props.contains("width")) {
            return isInt("height") ? SettingValueType::PHOXI_SIZE : SettingValueType::PHOXI_SIZE_64F;
        }

        if (propsCount == 4 && props.contains("x_min") && props.contains("x_max") && props.contains("y_min") && props.contains("y_max")) {
            return isInt("x_min") ? SettingValueType::PHOXI_2DROI : SettingValueType::AXIS_VOLUME_64F;
        }

        if (propsCount == 6 && props.contains("x_min") && props.contains("x_max") && props.contains("y_min") && props.contains("y_max") && props.contains("z_min") &&
                props.contains("z_max")) {
            return SettingValueType::AXIS_VOLUME_64F;
        }

        if (propsCount == 3 && props.contains("b") && props.contains("g") && props.contains("r")) {
            return SettingValueType::POINT3_64F;
        }
    }

    return SettingValueType::DOUBLE_ARRAY;
}

phoxi_camera::SettingValue jsonToSettingValue(phoxi_camera::SettingValueType type, const pho_jsoncons::json& j) {
    using phoxi_camera::SettingValue;
    using phoxi_camera::SettingValueType;

    auto readPoint3 = [](const pho_jsoncons::json& p) -> pho::api::Point3_64f { return {p["x"].as<double>(), p["y"].as<double>(), p["z"].as<double>()}; };

    switch (type) {
        case SettingValueType::BOOL:
            return j.as<bool>();
        case SettingValueType::INT:
            return j.as<int64_t>();
        case SettingValueType::DOUBLE:
            return j.as<double>();
        case SettingValueType::STRING:
            return j.as<std::string>();
        case SettingValueType::DOUBLE_ARRAY: {
            std::vector<double> arr;
            if (j.is_array()) {
                for (const auto& e : j.array_range()) {
                    arr.push_back(e.as<double>());
                }
            }
            return arr;
        }
        case SettingValueType::PHOXI_SIZE:
            return pho::api::PhoXiSize{j["width"].as<int32_t>(), j["height"].as<int32_t>()};
        case SettingValueType::PHOXI_SIZE_64F:
            return pho::api::PhoXiSize_64f{j["width"].as<double>(), j["height"].as<double>()};
        case SettingValueType::PHOXI_2DROI:
            return pho::api::PhoXi2DROI{j["x_min"].as<int32_t>(), j["y_min"].as<int32_t>(), j["x_max"].as<int32_t>(), j["y_max"].as<int32_t>()};
        case SettingValueType::AXIS_VOLUME_64F: {
            pho::api::AxisVolume_64f vol;
            vol.min.x = j["x_min"].as<double>();
            vol.min.y = j["y_min"].as<double>();
            vol.min.z = j["z_min"].as<double>();
            vol.max.x = j["x_max"].as<double>();
            vol.max.y = j["y_max"].as<double>();
            vol.max.z = j["z_max"].as<double>();
            return vol;
        }
        case SettingValueType::POINT3_64F:
            return pho::api::Point3_64f{j["r"].as<double>(), j["g"].as<double>(), j["b"].as<double>()};
        case SettingValueType::CUTTING_PLANES: {
            std::vector<pho::api::Plane_64f> planes;
            for (const auto& e : j.array_range()) {
                planes.emplace_back(pho::api::Point3_64f{e["x"].as<double>(), e["y"].as<double>(), e["z"].as<double>()}, e["d"].as<double>());
            }
            return planes;
        }
        case SettingValueType::SCANNING_VOLUME_MESH: {
            pho::api::PhoXiMesh mesh;
            mesh.PointsPerSection = j["points_per_section"].as<unsigned int>();
            for (const auto& v : j["vertices"].array_range()) {
                mesh.Vertices.push_back(readPoint3(v));
            }
            for (const auto& idx : j["indices"].array_range()) {
                mesh.Indices.push_back(idx.as<unsigned int>());
            }
            return mesh;
        }
        case SettingValueType::SCANNING_VOLUME: {
            pho::api::ProjectionGeometry_64f geom;
            geom.Origin = readPoint3(j["origin"]);
            geom.TopLeftTangentialVector = readPoint3(j["top_left"]);
            geom.TopRightTangentialVector = readPoint3(j["top_right"]);
            geom.BottomLeftTangentialVector = readPoint3(j["bottom_left"]);
            geom.BottomRightTangentialVector = readPoint3(j["bottom_right"]);
            for (const auto& p : j["top_contour"].array_range()) {
                geom.TopContourPoints.push_back(readPoint3(p));
            }
            for (const auto& p : j["bottom_contour"].array_range()) {
                geom.BottomContourPoints.push_back(readPoint3(p));
            }
            return geom;
        }
        case SettingValueType::REPROJECTION_MAP: {
            pho::api::PhoXiReprojectionMap reproj;
            const int h = j.contains("height") ? j["height"].as<int>() : 0;
            const int w = j.contains("width") ? j["width"].as<int>() : 0;
            if (h > 0 && w > 0) {
                reproj.Map.Resize(pho::api::PhoXiSize{w, h});
            }
            return reproj;
        }
    }
    return std::vector<double>{};
}

pho_jsoncons::json settingValueToJson(const phoxi_camera::SettingValue& val) {
    // MatType<T> members (x,y,z of Point3; x,y of Point2) need explicit casts
    // because jsoncons doesn't have a to_json specialisation for MatType<T>.
    auto makePoint3Json = [](const pho::api::Point3_64f& pt) {
        pho_jsoncons::json p(pho_jsoncons::json_object_arg);
        p.insert_or_assign("x", static_cast<double>(pt.x));
        p.insert_or_assign("y", static_cast<double>(pt.y));
        p.insert_or_assign("z", static_cast<double>(pt.z));
        return p;
    };

    return std::visit(
            [&makePoint3Json](const auto& v) -> pho_jsoncons::json {
                using T = std::decay_t<decltype(v)>;

                if constexpr (std::is_same_v<T, bool> || std::is_same_v<T, int64_t> || std::is_same_v<T, double> || std::is_same_v<T, std::string>) {
                    return pho_jsoncons::json(v);
                } else if constexpr (std::is_same_v<T, std::vector<double>>) {
                    pho_jsoncons::json arr(pho_jsoncons::json_array_arg);
                    for (double d : v) {
                        arr.push_back(d);
                    }
                    return arr;
                } else if constexpr (std::is_same_v<T, pho::api::PhoXiSize>) {
                    pho_jsoncons::json o(pho_jsoncons::json_object_arg);
                    o.insert_or_assign("width", static_cast<int32_t>(v.Width));
                    o.insert_or_assign("height", static_cast<int32_t>(v.Height));
                    return o;
                } else if constexpr (std::is_same_v<T, pho::api::PhoXiSize_64f>) {
                    pho_jsoncons::json o(pho_jsoncons::json_object_arg);
                    o.insert_or_assign("width", static_cast<double>(v.Width));
                    o.insert_or_assign("height", static_cast<double>(v.Height));
                    return o;
                } else if constexpr (std::is_same_v<T, pho::api::PhoXi2DROI>) {
                    pho_jsoncons::json o(pho_jsoncons::json_object_arg);
                    o.insert_or_assign("x_min", static_cast<int32_t>(v.Min.x));
                    o.insert_or_assign("y_min", static_cast<int32_t>(v.Min.y));
                    o.insert_or_assign("x_max", static_cast<int32_t>(v.Max.x));
                    o.insert_or_assign("y_max", static_cast<int32_t>(v.Max.y));
                    return o;
                } else if constexpr (std::is_same_v<T, pho::api::AxisVolume_64f>) {
                    pho_jsoncons::json o(pho_jsoncons::json_object_arg);
                    o.insert_or_assign("x_min", static_cast<double>(v.min.x));
                    o.insert_or_assign("y_min", static_cast<double>(v.min.y));
                    o.insert_or_assign("z_min", static_cast<double>(v.min.z));
                    o.insert_or_assign("x_max", static_cast<double>(v.max.x));
                    o.insert_or_assign("y_max", static_cast<double>(v.max.y));
                    o.insert_or_assign("z_max", static_cast<double>(v.max.z));
                    return o;
                } else if constexpr (std::is_same_v<T, pho::api::Point3_64f>) {
                    // WhiteBalance/Components: stored as r=x, g=y, b=z
                    pho_jsoncons::json o(pho_jsoncons::json_object_arg);
                    o.insert_or_assign("r", static_cast<double>(v.x));
                    o.insert_or_assign("g", static_cast<double>(v.y));
                    o.insert_or_assign("b", static_cast<double>(v.z));
                    return o;
                } else if constexpr (std::is_same_v<T, std::vector<pho::api::Plane_64f>>) {
                    pho_jsoncons::json arr(pho_jsoncons::json_array_arg);
                    for (const auto& plane : v) {
                        pho_jsoncons::json p(pho_jsoncons::json_object_arg);
                        p.insert_or_assign("x", static_cast<double>(plane.normal.x));
                        p.insert_or_assign("y", static_cast<double>(plane.normal.y));
                        p.insert_or_assign("z", static_cast<double>(plane.normal.z));
                        p.insert_or_assign("d", static_cast<double>(plane.d));
                        arr.push_back(p);
                    }
                    return arr;
                } else if constexpr (std::is_same_v<T, pho::api::PhoXiMesh>) {
                    pho_jsoncons::json o(pho_jsoncons::json_object_arg);
                    o.insert_or_assign("points_per_section", v.PointsPerSection);
                    pho_jsoncons::json vertices(pho_jsoncons::json_array_arg);
                    for (const auto& pt : v.Vertices) {
                        vertices.push_back(makePoint3Json(pt));
                    }
                    o.insert_or_assign("vertices", vertices);
                    pho_jsoncons::json idxArr(pho_jsoncons::json_array_arg);
                    for (unsigned int idx : v.Indices) {
                        idxArr.push_back(idx);
                    }
                    o.insert_or_assign("indices", idxArr);
                    return o;
                } else if constexpr (std::is_same_v<T, pho::api::ProjectionGeometry_64f>) {
                    pho_jsoncons::json o(pho_jsoncons::json_object_arg);
                    o.insert_or_assign("origin", makePoint3Json(v.Origin));
                    o.insert_or_assign("top_left", makePoint3Json(v.TopLeftTangentialVector));
                    o.insert_or_assign("top_right", makePoint3Json(v.TopRightTangentialVector));
                    o.insert_or_assign("bottom_left", makePoint3Json(v.BottomLeftTangentialVector));
                    o.insert_or_assign("bottom_right", makePoint3Json(v.BottomRightTangentialVector));
                    pho_jsoncons::json tc(pho_jsoncons::json_array_arg);
                    for (const auto& pt : v.TopContourPoints) {
                        tc.push_back(makePoint3Json(pt));
                    }
                    o.insert_or_assign("top_contour", tc);
                    pho_jsoncons::json bc(pho_jsoncons::json_array_arg);
                    for (const auto& pt : v.BottomContourPoints) {
                        bc.push_back(makePoint3Json(pt));
                    }
                    o.insert_or_assign("bottom_contour", bc);
                    return o;
                } else {
                    // PhoXiReprojectionMap — read-only, serialization not required
                    return pho_jsoncons::json(pho_jsoncons::json_object_arg);
                }
            },
            val);
}

}  // anonymous namespace

namespace phoxi_camera {

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

    loadDeviceSchema();
}

void PhoXiInterface::disconnectCamera() {
    if (mPhoXiDevice) {
        phoxi_aframe_disable(mPhoXiDevice->HardwareIdentification.GetStoredValue().c_str());
        mPhoXiDevice->Disconnect(true, true);
    }

    mPhoXiDevice.Reset();
    mSchemaTypeCache.clear();
    mSettingInfos.clear();

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

void PhoXiInterface::loadDeviceSchema() {
    mSchemaTypeCache.clear();
    mSettingInfos.clear();

    auto req = createCommand("schema");
    std::string reqStr;
    req.dump(reqStr);

    std::string responseStr;
    const int err = phoxi_command_execute(
            reqStr.c_str(), reqStr.size(), pho::api::details::InMillis(pho::api::DEFAULT_COMMAND_TIMEOUT), &pho::api::details::CommandStringConverter, &responseStr);

    if (err != PHOXI_OK || responseStr.empty()) {
        return;  // schema unavailable — settings will be empty, non-fatal
    }

    pho_jsoncons::json schema;
    try {
        schema = pho_jsoncons::json::parse(responseStr);
    } catch (const std::exception&) {
        return;
    }

    if (!schema.contains("device_settings")) {
        return;
    }

    const auto& ds = schema["device_settings"];

    std::map<std::string, pho_jsoncons::json> getSchemaProps;
    if (ds.contains("get")) {
        const auto& getSchema = ds["get"];
        if (getSchema.contains("response")) {
            const auto& resp = getSchema["response"];
            if (resp.contains("$defs") && resp["$defs"].contains("settings") && resp["$defs"]["settings"].contains("properties")) {
                for (const auto& kv : resp["$defs"]["settings"]["properties"].object_range()) {
                    getSchemaProps[kv.key()] = kv.value();
                }
            }
        }
    }

    std::set<std::string> settableKeys;
    if (ds.contains("set")) {
        const auto& setSchema = ds["set"];
        if (setSchema.contains("request")) {
            const auto& req2 = setSchema["request"];
            if (req2.contains("$defs") && req2["$defs"].contains("settings") && req2["$defs"]["settings"].contains("properties")) {
                for (const auto& kv : req2["$defs"]["settings"]["properties"].object_range()) {
                    settableKeys.insert(kv.key());
                }
            }
        }
    }

    for (const auto& [key, schemaNode] : getSchemaProps) {
        const auto type = classifySettingType(schemaNode);
        mSchemaTypeCache[key] = type;
        mSettingInfos.push_back({key, type, settableKeys.count(key) > 0});
    }
}

std::map<std::string, SettingValue> PhoXiInterface::getSettings(const std::vector<std::string>& keys) {
    isOk();
    if (keys.empty()) {
        return {};
    }

    const std::string deviceId = mPhoXiDevice->HardwareIdentification.GetStoredValue();
    auto req = createDeviceCommand("device_settings", deviceId, "get");
    pho_jsoncons::json settingsArr(pho_jsoncons::json_array_arg);
    for (const auto& key : keys) {
        settingsArr.push_back(key);
    }
    req["params"].insert_or_assign("settings", std::move(settingsArr));

    const auto resp = executeCommand(req);

    if (!resp.contains("settings")) {
        throw PhoXiInterfaceException("'device_settings' get response missing 'settings' field");
    }

    std::map<std::string, SettingValue> result;
    const auto& settings = resp["settings"];
    for (const auto& key : keys) {
        if (!settings.contains(key)) {
            continue;
        }
        auto typeIt = mSchemaTypeCache.find(key);
        if (typeIt == mSchemaTypeCache.end()) {
            continue;
        }
        try {
            result[key] = jsonToSettingValue(typeIt->second, settings[key]);
        } catch (const std::exception&) {
        }
    }
    return result;
}

SettingValue PhoXiInterface::getSetting(const std::string& key) {
    auto result = getSettings({key});
    auto it = result.find(key);
    if (it == result.end()) {
        throw PhoXiInterfaceException("'device_settings' get: no value returned for '" + key + "'");
    }
    return it->second;
}

void PhoXiInterface::setSettings(const std::vector<std::pair<std::string, SettingValue>>& keyValues) {
    isOk();
    if (keyValues.empty()) {
        return;
    }

    const std::string deviceId = mPhoXiDevice->HardwareIdentification.GetStoredValue();
    auto req = createDeviceCommand("device_settings", deviceId, "set");
    pho_jsoncons::json settingsObj(pho_jsoncons::json_object_arg);
    for (const auto& [key, value] : keyValues) {
        settingsObj.insert_or_assign(key, settingValueToJson(value));
    }
    req["params"].insert_or_assign("settings", std::move(settingsObj));
    executeCommand(req);
}

void PhoXiInterface::setSetting(const std::string& key, const SettingValue& value) {
    setSettings({{key, value}});
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
            case PHOXI_FRAME_TYPE_POINTCLOUD:
                frame.pointCloud = r;
                break;
            case PHOXI_FRAME_TYPE_NORMALMAP:
                frame.normalMap = r;
                break;
            case PHOXI_FRAME_TYPE_DEPTHMAP:
                frame.depthMap = r;
                break;
            case PHOXI_FRAME_TYPE_CONFIDENCEMAP:
                frame.confidenceMap = r;
                break;
            case PHOXI_FRAME_TYPE_EVENTMAP:
                frame.eventMap = r;
                break;
            case PHOXI_FRAME_TYPE_TEXTURE:
                if (r->format == PHOXI_FRAME_FORMAT_RGB_16) {
                    frame.textureRgb = r;
                } else {
                    frame.texture = r;
                }
                break;
            case PHOXI_FRAME_TYPE_COLORCAMERAIMAGE:
                frame.colorCamera = r;
                break;
            default:
                break;
        }
    }

    self->mFrameCallback(frame);
}

}  // namespace phoxi_camera
