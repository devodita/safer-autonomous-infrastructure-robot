# safer-autonomous-infrastructure-robot

SAFER: **S**ensor-fused **A**utonomous **F**ield **E**mergency **R**esponder

A ROS (Melodic/Noetic) workspace for a safety-aware autonomous ground robot capable of
detecting and mitigating urban hazards (open manholes, obstacles, pedestrian density).

## Package Overview

| Package | Role |
|---|---|
| `safer_msgs` | Custom ROS messages, services, actions |
| `safer_perception` | Hazard detection — depth, ultrasonic, ML classifier |
| `safer_decision` | Risk scoring + finite state machine |
| `safer_navigation` | Patrol, MPC trajectory tracking, costmap layer |
| `safer_actuation` | Motor, brake, and alert peripheral control |
| `safer_comms` | MQTT bridge + REST dashboard publisher |
| `safer_bringup` | Master launch files and robot URDF |

## Directory Structure

```
safer-autonomous-infrastructure-robot/
├── src/                    # All ROS packages
├── firmware/               # Arduino + STM32 embedded code
├── simulation/             # Gazebo scenario scripts
├── scripts/                # Dev/ops utilities
├── LICENSE
└── README.md
```

## Build

```bash
cd safer-autonomous-infrastructure-robot
catkin_make
source devel/setup.bash
```

## Launch (Hardware)

```bash
roslaunch safer_bringup safer_robot.launch
```

## Launch (Simulation)

```bash
roslaunch safer_bringup safer_simulation.launch
```

## Author

Devodita Chakravarty

IIT Kharagpur

## Contributing
Feel free to contribute to this repository! If you have any suggestions or improvements, open an issue or submit a pull request to make this repo more informative.

Happy coding!
