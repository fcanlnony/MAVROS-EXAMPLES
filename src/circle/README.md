# MAVROS Offboard Circle Trajectory Control

This ROS node implements an autonomous circular flight path for PX4-based drones using the MAVROS package. It handles state management (Arming/Offboard mode) and executes a waypoint-based navigation logic using local position feedback.

[简体中文](README_CN.md)

## Functionality
* **Trajectory Generation:** Pre-calculates a circular path defined by 100 discrete waypoints using NumPy.
* **Auto-Engagement:** Automatically attempts to switch the flight controller to `OFFBOARD` mode and `ARM` the vehicle once the connection is established.
* **Position Control:** Publishes `PoseStamped` messages to `mavros/setpoint_position/local` at 20Hz.
* **Waypoint Verification:** Uses Euclidean distance checks to validate if a waypoint is reached before proceeding to the next one.
* **Continuous Looping:** Once the circle completes, the trajectory index resets, creating an infinite flight loop.

## Implementation Logic

### 1. Trajectory Pre-calculation
The circle is discretized into an array of coordinates $(x, y, z)$ before the control loop starts:
$$x = R \cdot \cos(\theta) + x_c$$
$$y = R \cdot \sin(\theta) + y_c$$
Where $R=5.0m$, Altitude fixed at $2.0m$.

### 2. Safety Handshake
PX4 requires a stream of setpoints *before* switching to Offboard mode to prevent immediate failsafe triggering. The script sends 100 dummy setpoints (targeting the first waypoint) to satisfy this "heartbeat" requirement.

### 3. State Machine & Control Loop
The main loop (20Hz) performs three tasks:
1.  **Mode/Arming Check:** Periodically checks (every 5s) if the drone is in `OFFBOARD` mode and `ARMED`. If not, it calls the respective MAVROS services.
2.  **Navigation Logic:**
    * Calculates the error vector magnitude between current local position and target setpoint.
    * **Threshold:** If distance < `TOLERANCE` (0.3m), the target index increments.
3.  **Publishing:** Sends the current target pose to the flight controller.

## Dependencies
* **ROS 1** (Noetic recommended)
* **MAVROS** (with `mavros_msgs` and `geometry_msgs`)
* **Python Libraries:** `rospy`, `numpy`, `math`

## Key Parameters

| Parameter    | Value | Description                                        |
| :----------- | :---- | :------------------------------------------------- |
| `R`          | 5.0   | Radius of the circle (meters)                      |
| `num_points` | 100   | Granularity of the trajectory                      |
| `RATE_HZ`    | 20.0  | Control loop frequency                             |
| `TOLERANCE`  | 0.3   | Acceptance radius for reaching a waypoint (meters) |

## Usage

Run the control node:

```bash
chmod +x circle.py
roslaunch launch/circle.launch
```