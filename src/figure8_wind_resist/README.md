# MAVROS Offboard Figure-8 Trajectory Control (Wind-Resistant)

This ROS node implements a wind-resistant figure-8 (infinity symbol) flight path for PX4-based drones using the MAVROS package. Unlike the waypoint-based basic version, this implementation uses real-time velocity feedforward control for superior tracking accuracy in windy conditions.

[简体中文](README_CN.md)

## Functionality
* **Velocity Feedforward:** Publishes both position and velocity setpoints using `PositionTarget` messages for improved trajectory tracking.
* **Time-Based Generation:** Calculates trajectory setpoints in real-time based on elapsed time using Lissajous curve mathematics.
* **Auto-Engagement:** Automatically attempts to switch the flight controller to `OFFBOARD` mode and `ARM` the vehicle once the connection is established.
* **Enhanced Control:** Uses `mavros/setpoint_raw/local` topic for simultaneous position and velocity commands.
* **Dynamic Yaw:** Continuously updates yaw angle to align with velocity direction (tangent to curve).
* **Continuous Flight:** Infinite figure-8 trajectory with automatic loop completion tracking.

## Implementation Logic

### 1. Real-Time Trajectory Generation
Position and velocity are calculated dynamically using Lissajous curve equations:

**Position:**
$$x = x_c + A_x \cdot \sin(\omega t)$$
$$y = y_c + \frac{A_y}{2} \cdot \sin(2\omega t)$$
$$z = h$$

**Velocity (position derivative):**
$$v_x = A_x \omega \cdot \cos(\omega t)$$
$$v_y = A_y \omega \cdot \cos(2\omega t)$$
$$v_z = 0$$

**Yaw (tangent direction):**
$$\psi = \text{atan2}(v_y, v_x)$$

Where:
- $A_x=2.0m$ (X-axis amplitude)
- $A_y=1.0m$ (Y-axis amplitude)
- $\omega=0.3 \text{ rad/s}$ (angular velocity)
- $h=2.5m$ (altitude)
- Frequency ratio 1:2 creates the figure-8 shape

### 2. Why Velocity Feedforward?
In windy conditions, position-only control exhibits lag because the controller must first detect position error before reacting. By providing the expected velocity:
- The flight controller anticipates required thrust adjustments
- Tracking error and overshoot are reduced
- Figure-8 shape remains precise despite wind disturbances

### 3. Safety Handshake
PX4 requires a stream of setpoints *before* switching to Offboard mode. The script sends 100 initial setpoints to satisfy this requirement.

### 4. State Machine & Control Loop
The main loop (20Hz):
1. **Mode/Arming Check:** Attempts mode switch and arming every 5 seconds if needed.
2. **Trajectory Update:** Calculates current position and velocity based on elapsed time.
3. **Publishing:** Sends `PositionTarget` message with position, velocity, and yaw fields.

## Dependencies
* **ROS 1** (Noetic recommended)
* **MAVROS** (with `mavros_msgs` and `geometry_msgs`)
* **Python Libraries:** `rospy`, `math`

## Key Parameters

| Parameter       | Value | Description                       |
| :-------------- | :---- | :-------------------------------- |
| `AMPLITUDE_X`   | 2.0   | X-direction amplitude (meters)    |
| `AMPLITUDE_Y`   | 1.0   | Y-direction amplitude (meters)    |
| `HEIGHT`        | 2.5   | Flight altitude (meters)          |
| `OMEGA`         | 0.3   | Angular velocity (rad/s)          |
| `RATE_HZ`       | 20.0  | Control loop frequency            |

## Usage

Run the control node:

```bash
chmod +x figure8_wind_resist.py
roslaunch launch/figure8_wind_resist.launch
```

## Comparison with Basic Version

| Feature              | Basic Version            | Wind-Resistant Version          |
| :------------------- | :----------------------- | :------------------------------ |
| **Setpoint Type**    | Position only            | Position + Velocity             |
| **Topic**            | `setpoint_position/local` | `setpoint_raw/local`            |
| **Message Type**     | `PoseStamped`            | `PositionTarget`                |
| **Trajectory Logic** | Waypoint-based (100 pts) | Time-based continuous           |
| **Yaw Control**      | Not specified            | Dynamic (follows velocity)      |
| **Wind Performance** | Moderate lag             | Reduced lag, better tracking    |
