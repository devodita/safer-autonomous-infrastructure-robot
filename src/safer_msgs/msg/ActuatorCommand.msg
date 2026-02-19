std_msgs/Header header

# Desired linear velocity (m/s)
float32 linear_velocity

# Desired angular velocity (rad/s)
float32 angular_velocity

# Hard cap on angular velocity for this command (rad/s)
float32 max_angular_velocity

# Alert level to apply alongside motion command
# 0=off  1=visual  2=visual+beep  3=strobe+horn
uint8 alert_level

# Source of this command
# 0=navigation  1=operator  2=decision_override
uint8 command_source
