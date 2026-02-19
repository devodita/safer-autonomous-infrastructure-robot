# safer_perception

Sensor fusion and hazard detection package for the SAFER robot.

## Nodes

| Node | Language | Role |
|---|---|---|
| `hazard_detector_node` | C++ | Depth drop detection + ultrasonic obstacle detection |
| `hazard_classifier` | Python | ML-based hazard class refinement (ONNX) |
| `pedestrian_density_estimator` | Python | Background subtraction pedestrian count |

## Topics Published

| Topic | Type | Description |
|---|---|---|
| `/safer/hazard_detections` | `safer_msgs/HazardDetection` | Raw detections |
| `/safer/hazard_detections/classified` | `safer_msgs/HazardDetection` | ML-refined detections |
| `/safer/pedestrian_count` | `std_msgs/UInt16` | Current pedestrian estimate |

## Configuration

Edit `config/perception_params.yaml` to tune thresholds and camera intrinsics.
