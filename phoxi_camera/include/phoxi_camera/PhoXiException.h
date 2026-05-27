#ifndef PHOXI_CAMERA_PHOXIEXCEPTION_H
#define PHOXI_CAMERA_PHOXIEXCEPTION_H

#include <exception>
#include <string>

namespace phoxi_camera {
class PhoXiInterfaceException : public std::exception {
public:
    explicit PhoXiInterfaceException(std::string message) : message(message) {}

    virtual const char* what() const throw() { return message.c_str(); }

private:
    std::string message;
};

class PhoXiControlNotRunning : public PhoXiInterfaceException {
public:
    explicit PhoXiControlNotRunning(std::string message) : PhoXiInterfaceException(message) {}
};

class PhoXiDeviceNotFound : public PhoXiInterfaceException {
public:
    explicit PhoXiDeviceNotFound(std::string message) : PhoXiInterfaceException(message) {}
};

class UnableToConnect : public PhoXiInterfaceException {
public:
    explicit UnableToConnect(std::string message) : PhoXiInterfaceException(message) {}
};

class UnableToStartAcquisition : public PhoXiInterfaceException {
public:
    explicit UnableToStartAcquisition(std::string message) : PhoXiInterfaceException(message) {}
};

class UnableToStopAcquisition : public PhoXiInterfaceException {
public:
    explicit UnableToStopAcquisition(std::string message) : PhoXiInterfaceException(message) {}
};

class CorruptedFrame : public PhoXiInterfaceException {
public:
    explicit CorruptedFrame(std::string message) : PhoXiInterfaceException(message) {}
};

class UnableToTriggerFrame : public PhoXiInterfaceException {
public:
    explicit UnableToTriggerFrame(std::string message) : PhoXiInterfaceException(message) {}
};

class PhoXiDeviceNotConnected : public PhoXiInterfaceException {
public:
    explicit PhoXiDeviceNotConnected(std::string message) : PhoXiInterfaceException(message) {}
};

class CoordinateSpaceNotSupported : public PhoXiInterfaceException {
public:
    explicit CoordinateSpaceNotSupported(std::string message) : PhoXiInterfaceException(message) {}
};

class InvalidTransformationMatrix : public PhoXiInterfaceException {
public:
    explicit InvalidTransformationMatrix(std::string message) : PhoXiInterfaceException(message) {}
};

class InvalidTriggerMode : public PhoXiInterfaceException {
public:
    explicit InvalidTriggerMode(std::string message) : PhoXiInterfaceException(message) {}
};

class InvalidResolution : public PhoXiInterfaceException {
public:
    explicit InvalidResolution(std::string message) : PhoXiInterfaceException(message) {}
};
}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIEXCEPTION_H
