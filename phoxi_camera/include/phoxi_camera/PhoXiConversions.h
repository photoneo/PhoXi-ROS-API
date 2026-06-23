/**
 * @file PhoXiConversions.h
 * @brief Conversion helpers from `pho::api::PhoXiDeviceInformation` to the ROS-layer type.
 */
#ifndef PHOXI_CAMERA_PHOXICONVERSIONS_H
#define PHOXI_CAMERA_PHOXICONVERSIONS_H

#include <vector>

#include "PhoXi.h"
#include "phoxi_camera/PhoXiDeviceInformation.h"

/**
 * @brief Convert a single API device-information record to the ROS-layer type.
 *
 * Maps fields, derives the connection status from the API `Status` flags, and
 * queries optional device features (Alpha, Color, Blue).
 *
 * @param phoXiDeviceInformation     Source: API device information.
 * @param phoXiCameraDeviceInformation Destination: ROS-layer device information (overwritten).
 */
inline void toPhoXiCameraDeviceInformation(const pho::api::PhoXiDeviceInformation& phoXiDeviceInformation, phoxi_camera::PhoXiDeviceInformation& phoXiCameraDeviceInformation) {
    phoXiCameraDeviceInformation.name = phoXiDeviceInformation.Name;
    phoXiCameraDeviceInformation.type = phoXiDeviceInformation.Type;
    phoXiCameraDeviceInformation.hwIdentification = phoXiDeviceInformation.HWIdentification;
    phoXiCameraDeviceInformation.ipAddress = phoXiDeviceInformation.IPv4;
    phoXiCameraDeviceInformation.ipv6Address = phoXiDeviceInformation.IPv6;
    phoXiCameraDeviceInformation.hostname = phoXiDeviceInformation.Hostname;
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
        phoXiCameraDeviceInformation.ipAddress = "N/A";
        phoXiCameraDeviceInformation.ipv6Address = "N/A";
        phoXiCameraDeviceInformation.hostname = "N/A";
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

/**
 * @brief Convert a list of API device-information records to the ROS-layer type.
 *
 * The destination vector is cleared and resized before conversion.
 *
 * @param phoXiDeviceInformation      Source list of API device information records.
 * @param phoXiCameraDeviceInformation Destination list (overwritten).
 */
inline void toPhoXiCameraDeviceInformation(
        const std::vector<pho::api::PhoXiDeviceInformation>& phoXiDeviceInformation, std::vector<phoxi_camera::PhoXiDeviceInformation>& phoXiCameraDeviceInformation) {
    phoXiCameraDeviceInformation.clear();
    phoXiCameraDeviceInformation.resize(phoXiDeviceInformation.size());
    for (int i = 0; i < phoXiDeviceInformation.size(); ++i) {
        toPhoXiCameraDeviceInformation(phoXiDeviceInformation[i], phoXiCameraDeviceInformation[i]);
    }
}

#endif  // PHOXI_CAMERA_PHOXICONVERSIONS_H
