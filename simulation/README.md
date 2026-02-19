# SAFER Simulation

Gazebo-based scenario scripts for testing the SAFER robot's hazard response pipeline.

## Scenarios

### open_manhole_scenario.py
Spawns an open manhole model directly in the robot's patrol path and verifies:
1. The perception stack detects the manhole (`hazard_class=1`)
2. The decision stack escalates to `EMERGENCY_STOP` within 10 seconds of detection

**Run:**
```bash
# Terminal 1 — start simulation
roslaunch safer_bringup safer_simulation.launch

# Terminal 2 — run scenario
rosrun safer_simulation open_manhole_scenario.py
```

## Adding New Scenarios

Create a new Python script in `scenarios/`. Follow the pattern:
- Subscribe to `/safer/mode_status` and `/safer/hazard_detections`
- Use the Gazebo `/gazebo/spawn_sdf_model` service to place hazard objects
- Assert expected system behaviour within a deadline
- Clean up spawned models on exit
