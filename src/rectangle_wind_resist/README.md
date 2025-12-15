# MAVROS Offboard Rectangle Trajectory Control (Wind-Resistant)

This ROS node implements a wind-resistant rectangular flight path for PX4-based drones using the MAVROS package. Unlike the basic rectangle trajectory, this version uses velocity feedforward control to maintain trajectory accuracy and reduce position lag in windy conditions.

[简体中文](README_CN.md)

## Functional Description

* **Velocity Feedforward:** Calculates and publishes velocity vectors pointing toward target corners for improved trajectory tracking.
* **Auto-Configuration:** Automatically attempts to switch the flight mode to `OFFBOARD` and arm the vehicle once connected.
* **Waypoint Navigation:** Navigates through 4 corner coordinates defining a 4x2 meter rectangle at 2 meters altitude.
* **Continuous Looping:** Upon completing the sequence, automatically loops back to the first corner.
* **Enhanced Control:** Uses `mavros/setpoint_raw/local` topic for simultaneous position and velocity commands.
* **Dynamic Yaw:** Continuously updates yaw angle to face the target corner.

## Implementation Logic

### 1. Initialization & Communication
* **Node**: Initializes `rectangle_trajectory_wind_resist_node`.
* **Subscribers**:
    * `mavros/state`: Monitors connection status, flight mode, and arming state.
    * `mavros/local_position/pose`: Retrieves the drone's current local ENU coordinates.
* **Publishers**:
    * `mavros/setpoint_raw/local`: Sends `PositionTarget` messages with position, velocity, and yaw.
* **Services**: Clients for `cmd/arming` and `set_mode`.

### 2. Velocity Feedforward Calculation
For each control cycle, the velocity vector is computed:
$$\vec{v} = \frac{\vec{d}}{|\vec{d}|} \cdot v_{desired}$$

Where:
- $\vec{d}$ = vector from current position to target corner
- $v_{desired}=0.5 \text{ m/s}$ (constant speed)

This feedforward term helps the controller anticipate required thrust, reducing lag when flying through wind.

### 3. Pre-flight Setup (Safety Requirement)
Before switching to `OFFBOARD` mode, the script sends 100 initial setpoints at 20Hz to satisfy PX4's safety requirements.

### 4. State Machine Control
* If not in `OFFBOARD` mode: Requests mode switch every 5 seconds.
* If not `ARMED`: Requests arming every 5 seconds.

### 5. Navigation Logic
* **Distance Check**: Calculates Euclidean distance to target corner.
* **Corner Transition**: When distance < `TOLERANCE` (0.2m), advances to next corner.
* **Yaw Control**: Continuously updates yaw to face target using $\text{atan2}(dy, dx)$.

## Dependencies

* **ROS 1** (tested on Noetic/Melodic)
* **MAVROS**: `mavros`, `mavros_msgs`, `geometry_msgs`
* **PX4 Autopilot**: SITL or compatible hardware
* **Python Libraries**: `math`

## Key Parameters

| Parameter   | Value | Description                                        |
| :---------- | :---- | :------------------------------------------------- |
| `LENGTH`    | 4.0   | Rectangle length (meters)                          |
| `WIDTH`     | 2.0   | Rectangle width (meters)                           |
| `HEIGHT`    | 2.0   | Flight altitude (meters)                           |
| `SPEED`     | 0.5   | Flight speed (m/s)                                 |
| `RATE_HZ`   | 20.0  | Control loop frequency                             |
| `TOLERANCE` | 0.2   | Acceptance radius for reaching a corner (meters)   |

## Usage

Run the control node:

```bash
chmod +x rectangle_wind_resist.py
roslaunch launch/rectangle_wind_resist.launch
```

## Comparison with Basic Version

| Feature              | Basic Version            | Wind-Resistant Version          |
| :------------------- | :----------------------- | :------------------------------ |
| **Setpoint Type**    | Position only            | Position + Velocity             |
| **Topic**            | `setpoint_position/local` | `setpoint_raw/local`            |
| **Message Type**     | `PoseStamped`            | `PositionTarget`                |
| **Yaw Control**      | Static                   | Dynamic (faces target)          |
| **Wind Performance** | Position lag             | Reduced lag with feedforward    |
| **Tolerance**        | 0.05m (tighter)          | 0.2m (relaxed for smoother flow)|
