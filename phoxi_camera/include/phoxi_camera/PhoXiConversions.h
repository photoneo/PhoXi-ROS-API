#ifndef PHOXI_CAMERA_PHOXICONVERSIONS_H
#define PHOXI_CAMERA_PHOXICONVERSIONS_H

#include <vector>

#include "PhoXi.h"
#include "phoxi_camera/PhoXiDeviceInformation.h"

inline void toPhoXiCameraDeviceInformation(const pho::api::PhoXiDeviceInformation& phoXiDeviceInformation,
    phoxi_camera::PhoXiDeviceInformation& phoXiCameraDeviceInformation) {
    phoXiCameraDeviceInformation.name = phoXiDeviceInformation.Name;
    phoXiCameraDeviceInformation.type = phoXiDeviceInformation.Type;
    phoXiCameraDeviceInformation.hwIdentification = phoXiDeviceInformation.HWIdentification;
    phoXiCameraDeviceInformation.ipAddress = phoXiDeviceInformation.IPv4;
    if (phoXiCameraDeviceInformation.ipAddress.empty()) {
        phoXiCameraDeviceInformation.ipAddress = "unknown";
    }
    if (phoXiDeviceInformation.Status.Ready) {
        if (phoXiDeviceInformation.Status.Attached) {
            phoXiCameraDeviceInformation.status = phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::Connected;
        } else {
            phoXiCameraDeviceInformation.status = phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::Ready;
        }
    } else {
        if (phoXiDeviceInformation.Status.Attached) {
            phoXiCameraDeviceInformation.status = phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::Undefined;
        } else {
            phoXiCameraDeviceInformation.status = phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::Occupied;
        }
    }

    phoXiCameraDeviceInformation.isFileCam = phoXiDeviceInformation.IsFileCamera;
    if (phoXiCameraDeviceInformation.isFileCam) {
        phoXiCameraDeviceInformation.ipAddress = "N/A (FileCam)";
    }
    phoXiCameraDeviceInformation.firmwareVersion = phoXiDeviceInformation.FirmwareVersion;
    phoXiCameraDeviceInformation.variant = phoXiDeviceInformation.Variant;

    phoXiCameraDeviceInformation.isAlpha = false;
    phoXiCameraDeviceInformation.isColor = false;
    phoXiCameraDeviceInformation.isBlue = false;

    if (phoXiDeviceInformation.CheckFeature("Alpha")) {
        phoXiCameraDeviceInformation.isAlpha = true;
    }
    if (phoXiDeviceInformation.CheckFeature("Color")) {
        phoXiCameraDeviceInformation.isColor = true;
    }
    if (phoXiDeviceInformation.CheckFeature("Blue")) {
        phoXiCameraDeviceInformation.isBlue = true;
    }
}

inline void toPhoXiCameraDeviceInformation(const std::vector<pho::api::PhoXiDeviceInformation>& phoXiDeviceInformation,
    std::vector<phoxi_camera::PhoXiDeviceInformation>& phoXiCameraDeviceInformation) {
    phoXiCameraDeviceInformation.clear();
    phoXiCameraDeviceInformation.resize(phoXiDeviceInformation.size());
    for (int i = 0; i < phoXiDeviceInformation.size(); ++i) {
        toPhoXiCameraDeviceInformation(phoXiDeviceInformation[i], phoXiCameraDeviceInformation[i]);
    }
}

#endif  // PHOXI_CAMERA_PHOXICONVERSIONS_H
