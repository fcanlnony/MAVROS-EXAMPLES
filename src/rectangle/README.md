# MAVROS Offboard Rectangle Trajectory Control

This project implements a ROS node using `rospy` to control a PX4-based drone via MAVROS. It automates the process of arming the vehicle, switching to `OFFBOARD` mode, and navigating through a predefined rectangular waypoint sequence in a loop.

[简体中文](README_CN.md)

## Functional Description

* **Auto-Configuration**: Automatically attempts to switch the flight mode to `OFFBOARD` and arm the vehicle once the connection is established.
* **Waypoint Navigation**: Navigates through a set of 4 coordinates defining a 2x2 meter square at an altitude of 2 meters.
* **Continuous Looping**: Upon completing the sequence, the drone automatically loops back to the first point, flying the rectangle indefinitely.
* **Position Feedback**: Uses Euclidean distance calculation to determine if a waypoint has been reached (Tolerance: 0.05m).

## Dependencies

* **ROS 1** (tested on Noetic/Melodic)
* **MAVROS**: `mavros`, `mavros_msgs`, `geometry_msgs`
* **PX4 Autopilot**: SITL or compatible hardware
* **Python Libraries**: `numpy`, `math`

## Implementation Logic

### 1. Initialization & Communication
* **Node**: Initializes `offb_node_py`.
* **Subscribers**:
    * `mavros/state`: Monitors connection status, flight mode, and arming state.
    * `mavros/local_position/pose`: Retrieves the drone's current local ENU coordinates.
* **Publishers**:
    * `mavros/setpoint_position/local`: Sends target coordinates ($x, y, z$) to the flight controller.
* **Services**: Clients for `cmd/arming` and `set_mode`.

### 2. Pre-flight Setup (Safety Requirement)
Before switching to `OFFBOARD` mode, the PX4 flight stack requires a stream of setpoints to be already running. The script sends the initial waypoint 100 times at 20Hz to satisfy this safety condition.

### 3. State Machine Control
Inside the main loop, the script checks the `current_state`:
* If not in `OFFBOARD` mode: Requests mode switch every 5 seconds.
* If not `ARMED`: Requests arming every 5 seconds (only after mode switch is attempted/successful).

### 4. Navigation Logic
* **Distance Calculation**: Calculates the Euclidean distance between `current_local_pose` and the target waypoint.
    $$d = \sqrt{(x_{cur}-x_{tgt})^2 + (y_{cur}-y_{tgt})^2 + (z_{cur}-z_{tgt})^2}$$
* **Waypoint Switching**:
    * Defined Threshold (`TOLERANCE`): **0.05 meters**.
    * When $d \le TOLERANCE$, the index increments.
    * If the index exceeds the list length, it resets to 0.

## Usage

Run the control node:

```bash
chmod +x rectangle.py
roslaunch launch/rectangle.launch
```