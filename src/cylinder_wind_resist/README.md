# MAVROS Offboard Cylindrical Spiral Trajectory Control (Wind-Resistant)

This ROS node implements a wind-resistant cylindrical spiral flight path for PX4-based drones using the MAVROS package. Unlike the waypoint-based basic version, this implementation uses real-time velocity feedforward control for superior tracking accuracy in windy conditions.

[简体中文](README_CN.md)

## Functionality
* **Velocity Feedforward:** Publishes both position and velocity setpoints using `PositionTarget` messages for improved trajectory tracking.
* **Time-Based Generation:** Calculates trajectory setpoints in real-time based on elapsed time rather than discrete waypoints.
* **Auto-Engagement:** Automatically attempts to switch the flight controller to `OFFBOARD` mode and `ARM` the vehicle once the connection is established.
* **Enhanced Control:** Uses `mavros/setpoint_raw/local` topic for simultaneous position and velocity commands.
* **Post-Spiral Loiter:** After completing the spiral ascent, continues circling at the top altitude.

## Implementation Logic

### 1. Real-Time Trajectory Generation
Position and velocity are calculated dynamically using parametric equations:

**Position:**
$$x = x_c + R \cdot \cos(\omega t)$$
$$y = y_c + R \cdot \sin(\omega t)$$
$$z = z_{start} + v_z \cdot t$$

**Velocity (position derivative):**
$$v_x = -R \omega \cdot \sin(\omega t)$$
$$v_y = R \omega \cdot \cos(\omega t)$$
$$v_z = \frac{z_{end} - z_{start}}{T_{total}}$$

**Yaw (tangent direction):**
$$\psi = \omega t + \frac{\pi}{2}$$

Where:
- $R=2.0m$ (cylinder radius)
- $\omega=0.3 \text{ rad/s}$ (angular velocity)
- $T_{total} = \frac{2\pi \cdot n_{spirals}}{\omega}$ (total spiral time)
- Height: $z_{start}=1.5m$ to $z_{end}=4.0m$

### 2. Why Velocity Feedforward?
In windy conditions, position-only control exhibits lag because the controller must first detect position error before reacting. By providing the expected velocity:
- The flight controller anticipates required thrust adjustments
- Tracking error and overshoot are reduced
- Spiral trajectory remains smooth despite wind disturbances

### 3. Post-Spiral Behavior
Once the spiral completes (when $t > T_{total}$), the drone transitions to circular loiter at the top altitude:
- Position: Circular path at $z_{end}$
- Velocity: Tangential only ($v_z=0$)

### 4. Safety Handshake
PX4 requires a stream of setpoints *before* switching to Offboard mode. The script sends 100 initial setpoints to satisfy this requirement.

### 5. State Machine & Control Loop
The main loop (20Hz):
1. **Mode/Arming Check:** Attempts mode switch and arming every 5 seconds if needed.
2. **Trajectory Update:** Calculates current position and velocity based on elapsed time.
3. **Publishing:** Sends `PositionTarget` message with position, velocity, and yaw fields.

## Dependencies
* **ROS 1** (Noetic recommended)
* **MAVROS** (with `mavros_msgs` and `geometry_msgs`)
* **Python Libraries:** `rospy`, `math`

## Key Parameters

| Parameter      | Value | Description                         |
| :------------- | :---- | :---------------------------------- |
| `R`            | 2.0   | Cylinder radius (meters)            |
| `HEIGHT_START` | 1.5   | Starting altitude (meters)          |
| `HEIGHT_END`   | 4.0   | Ending altitude (meters)            |
| `OMEGA`        | 0.3   | Angular velocity (rad/s)            |
| `NUM_SPIRALS`  | 3     | Number of complete rotations        |
| `RATE_HZ`      | 20.0  | Control loop frequency              |

## Usage

Run the control node:

```bash
chmod +x cylinder_wind_resist.py
roslaunch launch/cylinder_wind_resist.launch
```

## Comparison with Basic Version

| Feature              | Basic Version            | Wind-Resistant Version          |
| :------------------- | :----------------------- | :------------------------------ |
| **Setpoint Type**    | Position only            | Position + Velocity             |
| **Topic**            | `setpoint_position/local` | `setpoint_raw/local`            |
| **Message Type**     | `PoseStamped`            | `PositionTarget`                |
| **Trajectory Logic** | Waypoint-based (300 pts) | Time-based continuous           |
| **Wind Performance** | Moderate lag             | Reduced lag, better tracking    |
| **Post-Completion**  | Restart from bottom      | Loiter at top altitude          |
