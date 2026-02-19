std_msgs/Header header

# Current active mode
# 0=NORMAL  1=CAUTION  2=RESTRICTED  3=EMERGENCY_STOP
uint8 mode

# Previous mode (for transition logging)
uint8 previous_mode

# How long the robot has been in current mode (seconds)
float32 time_in_mode

# Whether an E-STOP has been latched (requires manual reset)
bool estop_latched

# Short status string
string status_message
