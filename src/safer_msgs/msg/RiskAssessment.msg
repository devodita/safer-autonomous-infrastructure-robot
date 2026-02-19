std_msgs/Header header

# Composite risk score [0.0, 1.0]
float32 risk_score

# Breakdown per contributing factor [0.0, 1.0]
float32 proximity_risk
float32 density_risk
float32 velocity_risk
float32 severity_risk

# Number of active hazards contributing to this assessment
uint8 active_hazard_count

# Recommended operational mode
# 0=NORMAL  1=CAUTION  2=RESTRICTED  3=EMERGENCY_STOP
uint8 recommended_mode

# Human-readable reasoning string
string reasoning
