std_msgs/Header header

# Hazard class ID — see hazard_classes.yaml
uint8 hazard_class

# Human-readable label
string hazard_label

# Detection confidence [0.0, 1.0]
float32 confidence

# Estimated position of the hazard in the robot's base_link frame
geometry_msgs/Point position

# Approximate bounding box dimensions (metres)
float32 width
float32 depth

# Sensor that produced this detection
# 0=depth_camera  1=ultrasonic  2=lidar  3=fusion
uint8 source_sensor

# Severity level [0-3] mirroring hazard_classes.yaml
uint8 severity
