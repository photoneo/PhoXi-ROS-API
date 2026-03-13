#ifndef PHOXI_CAMERA_PHOXIDEVICEINFORMATION_H
#define PHOXI_CAMERA_PHOXIDEVICEINFORMATION_H

#include <string>

#include "PhoXi.h"

namespace phoxi_camera
{
class PhoXiInterface;

class PhoXiDeviceInformation
{
  public:
    friend PhoXiInterface;
    enum PhoXiConnectionStatus
    {
        Undefined = 0,
        Ready = 1,
        Occupied = 2,
        Connected = 3
    };

    operator std::string() const { return hwIdentification; }

    bool operator==(const PhoXiDeviceInformation& other) {
        return hwIdentification == other.hwIdentification;
    }

    bool operator==(const std::string& hwIdentification) {
        return this->hwIdentification == hwIdentification;
    }

    std::string name;
    pho::api::PhoXiDeviceType type;
    std::string hwIdentification;
    std::string IPaddress;
    PhoXiConnectionStatus status;
    std::string firmwareVersion;
    std::string variant;
    bool isAlpha;
    bool isBlue;
    bool isColor;
    bool isFileCam;
};
}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIDEVICEINFORMATION_H
