# Published when a scan fails and no frame data is available.
# Check the messages array for device-reported error codes and descriptions.

std_msgs/Header header      # Timestamp and frame ID matching the failed scan.
FrameMessage[] messages     # One or more error messages reported by the device.
