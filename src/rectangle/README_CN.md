# MAVROS Offboard 矩形轨迹巡航

本项目实现了一个基于 `rospy` 的 ROS 节点，通过 MAVROS 控制 PX4 无人机。该脚本实现了无人机的自动解锁、切换 `OFFBOARD` 模式，并按预设的矩形航点序列进行循环飞行。

[ENGLISH](README.md)

## 功能描述

* **自动配置**: 建立连接后，自动尝试切换飞行模式至 `OFFBOARD` 并解锁（Arm）飞机。
* **航点导航**: 按照预设的 4 个坐标点飞行，轨迹为一个 2x2 米的正方形，飞行高度保持在 2 米。
* **无限循环**: 完成一轮航点后，自动重置目标至起始点，持续循环飞行。
* **位置反馈**: 使用欧几里得距离判断是否到达航点（判定阈值：0.05米）。

## 依赖项

* **ROS 1** (已在 Noetic/Melodic 测试)
* **MAVROS**: `mavros`, `mavros_msgs`, `geometry_msgs`
* **PX4 Autopilot**: SITL 仿真或兼容硬件
* **Python 库**: `numpy`, `math`

## 实现思路

### 1. 初始化与通信
* **节点**: 初始化 `offb_node_py`。
* **订阅者 (Subscribers)**:
    * `mavros/state`: 监控连接状态、飞行模式和解锁状态。
    * `mavros/local_position/pose`: 获取无人机当前的本地 ENU 坐标。
* **发布者 (Publishers)**:
    * `mavros/setpoint_position/local`: 向飞控发送目标坐标 ($x, y, z$)。
* **服务 (Services)**: 实例化 `cmd/arming` 和 `set_mode` 客户端。

### 2. 起飞前准备 (安全机制)
PX4 飞控要求在切换到 `OFFBOARD` 模式之前，必须已经接收到一定数量的设定点（Setpoints）数据流。本脚本在主循环开始前，以 20Hz 的频率预发送 100 次初始航点数据，以满足此安全条件。

### 3. 状态机控制
在主循环中，脚本持续检查 `current_state`:
* 若当前非 `OFFBOARD` 模式：每隔 5 秒请求一次模式切换。
* 若当前未解锁 (`ARMED`)：在尝试模式切换后，每隔 5 秒请求一次解锁。

### 4. 导航逻辑
* **距离计算**: 计算当前位置 `current_local_pose` 与目标航点的欧几里得距离。
    $$d = \sqrt{(x_{cur}-x_{tgt})^2 + (y_{cur}-y_{tgt})^2 + (z_{cur}-z_{tgt})^2}$$
    
* **航点切换**:
    * 设定阈值 (`TOLERANCE`): **0.05 米**。
    
    * 当 $d \le TOLERANCE$ 时，索引自增。
    
    * 若索引超出航点列表长度，重置为 0，实现循环飞行。
    
## 使用方法

运行控制节点：

```bash
chmod +x rectangle.py
roslaunch launch/rectangle.launch
```