# MAVROS Offboard Helix Trajectory Control (Wind-Resistant)

This ROS node implements a wind-resistant helical flight path for PX4-based drones using the MAVROS package. Unlike the waypoint-based basic version, this implementation uses real-time velocity feedforward control for superior tracking accuracy in windy conditions, supporting both cylindrical and conical spiral configurations.

[简体中文](README_CN.md)

## Functionality
* **Velocity Feedforward:** Publishes both position and velocity setpoints using `PositionTarget` messages for improved trajectory tracking.
* **Time-Based Generation:** Calculates trajectory setpoints in real-time based on elapsed time rather than discrete waypoints.
* **Auto-Engagement:** Automatically attempts to switch the flight controller to `OFFBOARD` mode and `ARM` the vehicle once the connection is established.
* **Enhanced Control:** Uses `mavros/setpoint_raw/local` topic for simultaneous position and velocity commands.
* **Flexible Design:** Supports both cylindrical helix (`R_START = R_END`) and conical helix (`R_START ≠ R_END`).
* **Post-Helix Loiter:** After completing the spiral ascent, continues circling at the top altitude and radius.

## Implementation Logic

### 1. Real-Time Trajectory Generation
Position and velocity are calculated dynamically using parametric equations:

**Position:**
$$x = x_c + r(t) \cdot \cos(\omega t)$$
$$y = y_c + r(t) \cdot \sin(\omega t)$$
$$z = z_{start} + v_z \cdot t$$

**Velocity (position derivative):**
$$v_x = v_r \cdot \cos(\omega t) - r(t) \omega \cdot \sin(\omega t)$$
$$v_y = v_r \cdot \sin(\omega t) + r(t) \omega \cdot \cos(\omega t)$$
$$v_z = \frac{z_{end} - z_{start}}{T_{total}}$$

**Yaw (tangent direction):**
$$\psi = \text{atan2}(v_y, v_x)$$

Where:
- $r(t) = R_{start} + v_r \cdot t$ (time-varying radius)
- $v_r = \frac{R_{end} - R_{start}}{T_{total}}$ (radial velocity)
- $\omega=0.4 \text{ rad/s}$ (angular velocity)
- $T_{total} = \frac{2\pi \cdot n_{turns}}{\omega}$ (total helix time)
- Height: $z_{start}=1.5m$ to $z_{end}=5.0m$
- Radius: $R_{start}=2.0m$ to $R_{end}=2.0m$ (cylindrical by default)

### 2. Why Velocity Feedforward?
In windy conditions, position-only control exhibits lag because the controller must first detect position error before reacting. By providing the expected velocity:
- The flight controller anticipates required thrust adjustments
- Tracking error and overshoot are reduced
- Helical trajectory remains smooth despite wind disturbances

### 3. Post-Helix Behavior
Once the helix completes (when $t > T_{total}$), the drone transitions to circular loiter at the top:
- Position: Circular path at $z_{end}$ with radius $R_{end}$
- Velocity: Tangential only ($v_z=0$, $v_r=0$)

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

| Parameter      | Value | Description                                        |
| :------------- | :---- | :------------------------------------------------- |
| `R_START`      | 2.0   | Starting radius (meters)                           |
| `R_END`        | 2.0   | Ending radius (meters, ≠ start for conical helix)  |
| `HEIGHT_START` | 1.5   | Starting altitude (meters)                         |
| `HEIGHT_END`   | 5.0   | Ending altitude (meters)                           |
| `OMEGA`        | 0.4   | Angular velocity (rad/s)                           |
| `NUM_TURNS`    | 4     | Number of complete rotations                       |
| `RATE_HZ`      | 20.0  | Control loop frequency                             |

## Usage

Run the control node:

```bash
chmod +x helix_wind_resist.py
roslaunch launch/helix_wind_resist.launch
```

## Comparison with Basic Version

| Feature              | Basic Version            | Wind-Resistant Version          |
| :------------------- | :----------------------- | :------------------------------ |
| **Setpoint Type**    | Position only            | Position + Velocity             |
| **Topic**            | `setpoint_position/local` | `setpoint_raw/local`            |
| **Message Type**     | `PoseStamped`            | `PositionTarget`                |
| **Trajectory Logic** | Waypoint-based (400 pts) | Time-based continuous           |
| **Yaw Control**      | Not specified            | Dynamic (follows velocity)      |
| **Wind Performance** | Moderate lag             | Reduced lag, better tracking    |
| **Post-Completion**  | Restart from bottom      | Loiter at top altitude/radius   |
