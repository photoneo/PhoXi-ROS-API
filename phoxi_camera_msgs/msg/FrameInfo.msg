std_msgs/Header header

string hw_id
int32 index
int32 total_scan_count

float64 timestamp
float64 duration
float64 duration_computation
float64 duration_transfer

bool is_early_transfer_frame

float64[3] sensor_position
float64[3] sensor_x_axis
float64[3] sensor_y_axis
float64[3] sensor_z_axis

float64[3] balance_rgb

int32[2] camera_binning
float64[2] camera_binning_factor

float64[] temperature

string frame_start_grand_master_identity
string frame_start_port_state
int64 frame_start_time_ns

int32 marker_dots_status
string marker_dots_message
