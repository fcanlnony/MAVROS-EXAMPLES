# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MAVROS-EXAMPLES is a collection of Python-based ROS nodes demonstrating autonomous drone control using MAVROS (MAVLink + ROS) with PX4 autopilot. The repository contains example implementations for flying predetermined trajectories in OFFBOARD mode.

**Environment:**
- OS: Ubuntu 20.04
- ROS: Noetic
- PX4: v1.17.0-alpha1
- Gazebo: 11.15.1
- Python: >= 3.8

## Architecture

### Core Control Pattern

All trajectory nodes follow a common control architecture:

1. **State Management**: Subscribe to `mavros/state` to monitor connection, flight mode, and arming status
2. **Position Feedback**: Subscribe to `mavros/local_position/pose` for current drone position in local ENU coordinates
3. **Setpoint Publishing**: Publish `PoseStamped` messages to `mavros/setpoint_position/local` at 20Hz
4. **Service Clients**: Use `mavros/cmd/arming` and `mavros/set_mode` for mode changes

### PX4 OFFBOARD Mode Requirements

**Critical Safety Protocol**: Before switching to OFFBOARD mode, PX4 requires a stream of setpoints to already be running. All nodes send 100 dummy setpoints (at the first waypoint) before attempting mode switch to satisfy this "heartbeat" requirement and prevent immediate failsafe triggering.

### Waypoint Navigation Logic

1. Calculate Euclidean distance between current position and target waypoint
2. If distance ≤ TOLERANCE, increment to next waypoint
3. Loop back to first waypoint when sequence completes
4. Publish target pose at control loop frequency (20Hz)

### State Machine

The main control loop handles:
- **Connection Wait**: Block until `current_state.connected` is true
- **Mode Management**: Request OFFBOARD mode every 5 seconds if not active
- **Arming**: Request ARM every 5 seconds after OFFBOARD is set
- **Navigation**: Check distance and update target waypoint

## Project Structure

```
src/
├── circle/          # Circular trajectory (100 waypoints, R=5.0m)
│   ├── circle.py    # Main control node
│   └── launch/
│       └── circle.launch
└── rectangle/       # Rectangular trajectory (4 corners, 2x2m)
    ├── rectangle.py # Main control node
    └── launch/
        └── rectangle.launch
```

## Running Examples

### Circle Trajectory
```bash
chmod +x src/circle/circle.py
roslaunch src/circle/launch/circle.launch
```

**Parameters:**
- Radius: 5.0m
- Waypoints: 100 (pre-calculated using NumPy)
- Altitude: 2.0m
- Tolerance: 0.3m
- Control Rate: 20Hz

### Rectangle Trajectory
```bash
chmod +x src/rectangle/rectangle.py
roslaunch src/rectangle/launch/rectangle.launch
```

**Parameters:**
- Corners: (0,0,2), (2,0,2), (2,2,2), (0,2,2)
- Waypoints: 4
- Tolerance: 0.05m
- Control Rate: 20Hz

## Key Implementation Details

### Trajectory Generation

**Circle** (src/circle/circle.py:15-20): Pre-calculates waypoints using:
```python
theta = np.linspace(0, 2 * np.pi, num_points)
x = R * np.cos(theta) + xc
y = R * np.sin(theta) + yc
z = np.full(num_points, 2.0)
```

**Rectangle** (src/rectangle/rectangle.py:10): Hardcoded waypoint tuple.

### Tolerance Values

Different tolerances reflect trajectory characteristics:
- Circle: 0.3m (smoother path, larger acceptance radius)
- Rectangle: 0.05m (sharp corners, tighter precision)

### Launch Files

Both launch files follow the same pattern:
1. Include PX4 SITL + Gazebo via `$(find px4)/launch/mavros_posix_sitl.launch`
2. Launch the trajectory control node with `pkg="offboard_py"`

Note: The package name in launch files is `offboard_py` but the actual directory structure uses `circle` and `rectangle` as folder names.

## Dependencies

**ROS Packages:**
- mavros
- mavros_msgs
- geometry_msgs

**Python Libraries:**
- rospy
- numpy
- math

## Development Notes

When creating new trajectory examples:
1. Follow the established control pattern (100 initial setpoints, 5-second retry intervals)
2. Set control rate to 20Hz for consistency with PX4 expectations
3. Adjust TOLERANCE based on trajectory smoothness (sharp corners need tighter tolerance)
4. Pre-calculate complex trajectories (circles, spirals) vs. hardcode simple waypoints (rectangles, squares)
5. Always implement continuous looping by resetting waypoint index to 0
