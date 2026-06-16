/**
 * @file PhoXiException.h
 * @brief Exception hierarchy for the PhoXi camera interface.
 *
 * All exceptions derive from `PhoXiInterfaceException`, which itself inherits
 * `std::exception`, so callers may catch at any level of the hierarchy.
 */
#ifndef PHOXI_CAMERA_PHOXIEXCEPTION_H
#define PHOXI_CAMERA_PHOXIEXCEPTION_H

#include <exception>
#include <string>

namespace phoxi_camera {

/** @brief Base class for all PhoXi interface exceptions. */
class PhoXiInterfaceException : public std::exception {
public:
    explicit PhoXiInterfaceException(std::string message) : message(message) {}

    virtual const char* what() const throw() { return message.c_str(); }

private:
    std::string message;
};

/** @brief PhoXi Control software is not running or unreachable. */
class PhoXiControlNotRunning : public PhoXiInterfaceException {
public:
    explicit PhoXiControlNotRunning(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief No device with the requested hardware identification was found. */
class PhoXiDeviceNotFound : public PhoXiInterfaceException {
public:
    explicit PhoXiDeviceNotFound(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief Connection to the device could not be established. */
class UnableToConnect : public PhoXiInterfaceException {
public:
    explicit UnableToConnect(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief Acquisition could not be started on the connected device. */
class UnableToStartAcquisition : public PhoXiInterfaceException {
public:
    explicit UnableToStartAcquisition(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief Acquisition could not be stopped on the connected device. */
class UnableToStopAcquisition : public PhoXiInterfaceException {
public:
    explicit UnableToStopAcquisition(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief A received frame contains invalid or incomplete data. */
class CorruptedFrame : public PhoXiInterfaceException {
public:
    explicit CorruptedFrame(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief Frame could not be triggered (device busy, communication error, etc.). */
class UnableToTriggerFrame : public PhoXiInterfaceException {
public:
    explicit UnableToTriggerFrame(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief An operation was attempted without an active device connection. */
class PhoXiDeviceNotConnected : public PhoXiInterfaceException {
public:
    explicit PhoXiDeviceNotConnected(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief The requested coordinate space is not supported by this device. */
class CoordinateSpaceNotSupported : public PhoXiInterfaceException {
public:
    explicit CoordinateSpaceNotSupported(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief The provided transformation matrix is invalid. */
class InvalidTransformationMatrix : public PhoXiInterfaceException {
public:
    explicit InvalidTransformationMatrix(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief The provided trigger mode is not valid for this device. */
class InvalidTriggerMode : public PhoXiInterfaceException {
public:
    explicit InvalidTriggerMode(std::string message) : PhoXiInterfaceException(message) {}
};

/** @brief The requested resolution is not supported by this device. */
class InvalidResolution : public PhoXiInterfaceException {
public:
    explicit InvalidResolution(std::string message) : PhoXiInterfaceException(message) {}
};
}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIEXCEPTION_H
