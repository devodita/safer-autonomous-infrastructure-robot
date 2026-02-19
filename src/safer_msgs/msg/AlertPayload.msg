std_msgs/Header header

# Alert severity [0-3]
uint8 severity

# Hazard class that triggered this alert
uint8 hazard_class
string hazard_label

# Robot pose at time of alert
geometry_msgs/Point robot_position

# Risk score at time of alert
float32 risk_score

# Operational mode at time of alert
uint8 mode

# Free-text alert message
string message

# Unique alert ID for deduplication
string alert_id
