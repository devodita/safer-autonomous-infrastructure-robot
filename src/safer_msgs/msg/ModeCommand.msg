std_msgs/Header header

# Target mode
# 0=NORMAL  1=CAUTION  2=RESTRICTED  3=EMERGENCY_STOP
uint8 mode

# Who/what issued this command
string issuer

# Optional human-readable reason
string reason
