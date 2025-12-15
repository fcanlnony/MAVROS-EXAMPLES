## MAVROS EXAMPLES

[简体中文](doc/README_CN.md)

---

## Environment

OS: Ubuntu 20.04

ROS Version: Noetic

PX4 Version: v1.17.0-alpha1

Gazebo Version: 11.15.1

Python Version: >= 3.8

---

## Trajectory Examples

### Basic Trajectories

These examples use position-only control with waypoint-based navigation:

- **Circle**: [Circle](src/circle/README.md) - Circular trajectory with constant radius
- **Rectangle**: [Rectangle](src/rectangle/README.md) - Rectangular trajectory with sharp corners
- **Cylinder**: [Cylinder](src/cylinder/README.md) - Cylindrical spiral ascending trajectory
- **Figure-8**: [Figure-8](src/figure8/README.md) - Figure-8 (infinity symbol) trajectory using Lissajous curves
- **Helix**: [Helix](src/helix/README.md) - Helical spiral with adjustable radius (cylindrical/conical)

### Wind-Resistant Trajectories

These examples use position + velocity feedforward control for improved tracking accuracy in windy conditions:

- **Circle (Wind-Resistant)**: [Circle Wind-Resist](src/circle_wind_resist/README.md) - Time-based circular trajectory with velocity feedforward
- **Rectangle (Wind-Resistant)**: [Rectangle Wind-Resist](src/rectangle_wind_resist/README.md) - Rectangular trajectory with velocity feedforward and dynamic yaw
- **Cylinder (Wind-Resistant)**: [Cylinder Wind-Resist](src/cylinder_wind_resist/README.md) - Time-based cylindrical spiral with velocity feedforward
- **Figure-8 (Wind-Resistant)**: [Figure-8 Wind-Resist](src/figure8_wind_resist/README.md) - Time-based figure-8 trajectory with velocity feedforward
- **Helix (Wind-Resistant)**: [Helix Wind-Resist](src/helix_wind_resist/README.md) - Time-based helical spiral with velocity feedforward

---

## Key Differences

| Feature              | Basic Trajectories       | Wind-Resistant Trajectories |
| :------------------- | :----------------------- | :-------------------------- |
| **Control Method**   | Position only            | Position + Velocity         |
| **Topic**            | `setpoint_position/local` | `setpoint_raw/local`        |
| **Message Type**     | `PoseStamped`            | `PositionTarget`            |
| **Trajectory Logic** | Waypoint-based           | Time-based continuous       |
| **Wind Performance** | Moderate lag             | Reduced lag, better tracking|

---

## Quick Start

1. **Build the workspace:**
   ```bash
   catkin_make
   source devel/setup.bash
   ```

2. **Launch a trajectory example:**
   ```bash
   # Basic trajectory
   roslaunch src/circle/launch/circle.launch

   # Wind-resistant trajectory
   roslaunch src/circle_wind_resist/launch/circle_wind_resist.launch
   ```

3. **The drone will automatically:**
   - Connect to PX4 SITL
   - Switch to OFFBOARD mode
   - Arm and execute the trajectory

---

## Documentation

For detailed implementation information, please refer to:
- [CLAUDE.md](CLAUDE.md) - Project architecture and development guidelines
- Individual trajectory README files linked above

---

## License

This project is for educational and research purposes.
