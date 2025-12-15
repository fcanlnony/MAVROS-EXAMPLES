# MAVROS Offboard Circle Trajectory Control (Wind-Resistant)

This ROS node implements a wind-resistant circular flight path for PX4-based drones using the MAVROS package. Unlike the basic circle trajectory, this version uses velocity feedforward control to maintain trajectory accuracy in windy conditions.

[简体中文](README_CN.md)

## Functionality
* **Velocity Feedforward:** Publishes both position and velocity setpoints using `PositionTarget` messages for improved trajectory tracking.
* **Time-Based Generation:** Calculates trajectory setpoints in real-time based on elapsed time rather than pre-calculated waypoints.
* **Auto-Engagement:** Automatically attempts to switch the flight controller to `OFFBOARD` mode and `ARM` the vehicle once the connection is established.
* **Enhanced Control:** Uses `mavros/setpoint_raw/local` topic for simultaneous position and velocity commands.
* **Continuous Flight:** Infinite circular trajectory with automatic loop completion tracking.

## Implementation Logic

### 1. Real-Time Trajectory Generation
Position and velocity are calculated dynamically using parametric equations:

**Position:**
$$x = x_c + R \cdot \cos(\omega t)$$
$$y = y_c + R \cdot \sin(\omega t)$$
$$z = h$$

**Velocity (position derivative):**
$$v_x = -R \omega \cdot \sin(\omega t)$$
$$v_y = R \omega \cdot \cos(\omega t)$$
$$v_z = 0$$

**Yaw (tangent direction):**
$$\psi = \omega t + \frac{\pi}{2}$$

Where:
- $R=2.0m$ (radius)
- $\omega=0.3 \text{ rad/s}$ (angular velocity)
- $h=2.0m$ (altitude)

### 2. Why Velocity Feedforward?
In windy conditions, position-only control exhibits lag because the controller must first detect position error before reacting. By providing the expected velocity, the flight controller can:
- Anticipate required thrust adjustments
- Reduce tracking error and overshoot
- Maintain smoother trajectory execution

### 3. Safety Handshake
PX4 requires a stream of setpoints *before* switching to Offboard mode. The script sends 100 initial setpoints to satisfy this requirement.

### 4. State Machine & Control Loop
The main loop (20Hz):
1. **Mode/Arming Check:** Attempts mode switch and arming every 5 seconds if needed.
2. **Trajectory Update:** Calculates current position and velocity based on elapsed time.
3. **Publishing:** Sends `PositionTarget` message with both position and velocity fields.

## Dependencies
* **ROS 1** (Noetic recommended)
* **MAVROS** (with `mavros_msgs` and `geometry_msgs`)
* **Python Libraries:** `rospy`, `math`

## Key Parameters

| Parameter | Value | Description                       |
| :-------- | :---- | :-------------------------------- |
| `R`       | 2.0   | Circle radius (meters)            |
| `HEIGHT`  | 2.0   | Flight altitude (meters)          |
| `OMEGA`   | 0.3   | Angular velocity (rad/s)          |
| `RATE_HZ` | 20.0  | Control loop frequency            |

## Usage

Run the control node:

```bash
chmod +x circle_wind_resist.py
roslaunch launch/circle_wind_resist.launch
```

## Comparison with Basic Version

| Feature              | Basic Version            | Wind-Resistant Version          |
| :------------------- | :----------------------- | :------------------------------ |
| **Setpoint Type**    | Position only            | Position + Velocity             |
| **Topic**            | `setpoint_position/local` | `setpoint_raw/local`            |
| **Message Type**     | `PoseStamped`            | `PositionTarget`                |
| **Trajectory Logic** | Waypoint-based           | Time-based continuous           |
| **Wind Performance** | Moderate lag             | Reduced lag, better tracking    |
