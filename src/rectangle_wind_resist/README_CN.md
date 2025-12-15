# MAVROS Offboard 矩形轨迹巡航 (抗风版)

本 ROS 节点通过 MAVROS 功能包实现 PX4 飞控的抗风矩形航线飞行。与基础矩形轨迹不同，此版本使用速度前馈控制以在风力条件下保持轨迹精度并减少位置滞后。

[ENGLISH](README.md)

## 功能描述

* **速度前馈**：计算并发布指向目标角点的速度向量，提高轨迹跟踪性能。
* **自动配置**: 建立连接后，自动尝试切换飞行模式至 `OFFBOARD` 并解锁(Arm)飞机。
* **航点导航**: 按照预设的 4 个角点坐标飞行，轨迹为 4x2 米矩形，飞行高度保持在 2 米。
* **无限循环**: 完成一轮航点后，自动重置目标至起始点，持续循环飞行。
* **增强控制**：使用 `mavros/setpoint_raw/local` 话题同时发送位置和速度指令。
* **动态偏航**：持续更新偏航角以面向目标角点。

## 实现思路

### 1. 初始化与通信
* **节点**: 初始化 `rectangle_trajectory_wind_resist_node`。
* **订阅者 (Subscribers)**:
    * `mavros/state`: 监控连接状态、飞行模式和解锁状态。
    * `mavros/local_position/pose`: 获取无人机当前的本地 ENU 坐标。
* **发布者 (Publishers)**:
    * `mavros/setpoint_raw/local`: 发送包含位置、速度和偏航角的 `PositionTarget` 消息。
* **服务 (Services)**: 实例化 `cmd/arming` 和 `set_mode` 客户端。

### 2. 速度前馈计算
在每个控制周期，计算速度向量:
$$\vec{v} = \frac{\vec{d}}{|\vec{d}|} \cdot v_{desired}$$

其中:
- $\vec{d}$ = 从当前位置到目标角点的向量
- $v_{desired}=0.5 \text{ m/s}$ (恒定速度)

此前馈项帮助控制器预测所需推力，减少迎风飞行时的滞后。

### 3. 起飞前准备 (安全机制)
在切换到 `OFFBOARD` 模式之前，脚本以 20Hz 频率预发送 100 个初始设定点以满足 PX4 安全要求。

### 4. 状态机控制
* 若当前非 `OFFBOARD` 模式：每隔 5 秒请求一次模式切换。
* 若当前未解锁 (`ARMED`)：每隔 5 秒请求一次解锁。

### 5. 导航逻辑
* **距离检查**: 计算到目标角点的欧几里得距离。
* **角点切换**: 当距离 < `TOLERANCE` (0.2m) 时，切换到下一角点。
* **偏航控制**: 使用 $\text{atan2}(dy, dx)$ 持续更新偏航角以面向目标。

## 依赖项

* **ROS 1** (已在 Noetic/Melodic 测试)
* **MAVROS**: `mavros`, `mavros_msgs`, `geometry_msgs`
* **PX4 Autopilot**: SITL 仿真或兼容硬件
* **Python 库**: `math`

## 关键参数

| 参数        | 值   | 说明                       |
| :---------- | :--- | :------------------------- |
| `LENGTH`    | 4.0  | 矩形长度 (米)              |
| `WIDTH`     | 2.0  | 矩形宽度 (米)              |
| `HEIGHT`    | 2.0  | 飞行高度 (米)              |
| `SPEED`     | 0.5  | 飞行速度 (m/s)             |
| `RATE_HZ`   | 20.0 | 控制频率                   |
| `TOLERANCE` | 0.2  | 角点到达判定半径 (米)      |

## 使用方法

运行控制节点：

```bash
chmod +x rectangle_wind_resist.py
roslaunch launch/rectangle_wind_resist.launch
```

## 与基础版本对比

| 特性         | 基础版本                 | 抗风版本                       |
| :----------- | :----------------------- | :----------------------------- |
| **设定点类型** | 仅位置                   | 位置 + 速度                    |
| **话题**     | `setpoint_position/local` | `setpoint_raw/local`           |
| **消息类型** | `PoseStamped`            | `PositionTarget`               |
| **偏航控制** | 静态                     | 动态(面向目标)                 |
| **抗风性能** | 有位置滞后               | 前馈减少滞后                   |
| **判定阈值** | 0.05m (更严格)           | 0.2m (放宽以实现更流畅飞行)    |
