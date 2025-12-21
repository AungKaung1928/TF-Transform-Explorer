# Custom message for TF health diagnostics

std_msgs/Header header

# Frame being monitored
string parent_frame
string child_frame

# Health status
bool is_healthy
string status_message

# Timing info
float64 transform_age_seconds
float64 max_allowed_age_seconds

# Position jump detection
float64 position_jump_meters
float64 max_allowed_jump_meters
bool jump_detected