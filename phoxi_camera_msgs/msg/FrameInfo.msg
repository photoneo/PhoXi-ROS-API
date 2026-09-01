# Per-frame metadata published alongside the scan data.
# All timing values are in milliseconds unless noted otherwise.

std_msgs/Header header  # ROS timestamp and frame ID matching the scan.

string hw_id            # Hardware identifier of the device that produced this frame.
int32 index             # Sequential frame index since acquisition started.
int32 total_scan_count  # Total number of scans performed by the device since power-on.

float64 timestamp             # Device-side timestamp of the frame in milliseconds.
float64 duration              # Total frame acquisition duration in milliseconds.
float64 duration_computation  # Computation time within the frame in milliseconds.
float64 duration_transfer     # Transfer time of the frame data in milliseconds.

bool is_early_transfer_frame  # True when the frame was transferred before full computation finished.

float64[3] sensor_position  # Sensor origin in the configured coordinate space [x, y, z] in mm.
float64[3] sensor_x_axis    # Sensor X-axis unit vector in the configured coordinate space.
float64[3] sensor_y_axis    # Sensor Y-axis unit vector in the configured coordinate space.
float64[3] sensor_z_axis    # Sensor Z-axis unit vector in the configured coordinate space.

float64[3] balance_rgb  # White balance coefficients [R, G, B] applied during capture (color cameras only).

int32[2] camera_binning         # Camera binning applied to the image sensor [height_bins, width_bins].
float64[2] camera_binning_factor  # Effective resolution scaling factor due to binning [height, width].

float64[] temperature  # Device temperature readings in degrees Celsius, one value per thermal sensor.

string frame_start_grand_master_identity  # PTP grandmaster clock identity string at frame start.
string frame_start_port_state             # PTP port synchronization state at frame start.
int64 frame_start_time_ns                 # PTP-synchronized frame start time in nanoseconds since epoch.

int32 marker_dots_status    # Status code from the marker dot detection algorithm.
string marker_dots_message  # Diagnostic message from the marker dot detection algorithm.

FrameMessage[] messages  # Informational or warning messages associated with this frame.
