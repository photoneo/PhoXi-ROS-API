/**
 * @file PhoXiDeviceInformation.h
 * @brief Device identity and connection-status data for a discovered PhoXi device.
 */
#ifndef PHOXI_CAMERA_PHOXIDEVICEINFORMATION_H
#define PHOXI_CAMERA_PHOXIDEVICEINFORMATION_H

#include <string>

#include "PhoXi.h"

namespace phoxi_camera {
class PhoXiInterface;

/**
 * @brief Identity and connection status of a PhoXi device on the network.
 *
 * Populated by `PhoXiInterface::deviceList()` and `PhoXiInterface::getDeviceInfo()`.
 * The hardware identification string is used as the unique device key throughout the API.
 */
class PhoXiDeviceInformation {
public:
    friend PhoXiInterface;

    /** @brief Connection state as reported by the PhoXi Control service. */
    enum PhoXiConnectionStatus {
        Undefined = 0,  ///< Device is attached but not ready (e.g. still initializing).
        Ready = 1,      ///< Device is reachable and no client is connected.
        Occupied = 2,   ///< Device is reachable but another client already has it.
        Connected = 3   ///< This process is connected to the device.
    };

    /** @brief Implicit conversion to the hardware identification string. */
    operator std::string() const { return hwIdentification; }
    /** @brief Equality by hardware identification. */
    bool operator==(const PhoXiDeviceInformation& other) const { return hwIdentification == other.hwIdentification; }
    /** @brief Equality against a hardware identification string. */
    bool operator==(const std::string& hwIdentification) const { return this->hwIdentification == hwIdentification; }

    std::string name;                ///< Human-readable device name.
    pho::api::PhoXiDeviceType type;  ///< API device-type enum (scanner model family).
    std::string hwIdentification;    ///< Unique hardware serial / identification string.
    std::string ipAddress;           ///< IPv4 address, or `"N/A (FileCam)"` for file cameras.
    PhoXiConnectionStatus status;    ///< Current reachability and ownership state.
    std::string firmwareVersion;     ///< Firmware version string reported by the device.
    std::string variant;             ///< Device variant string (e.g. hardware configuration tier).
    bool isAlpha;                    ///< True if the device has the Alpha feature.
    bool isBlue;                     ///< True if the device has the Blue feature.
    bool isColor;                    ///< True if the device has an integrated color camera.
    bool isFileCam;                  ///< True if this is a file-based virtual camera.
};
}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIDEVICEINFORMATION_H
