# MAVROS Offboard 螺旋线轨迹控制 (抗风版)

本 ROS 节点通过 MAVROS 功能包实现 PX4 飞控的抗风螺旋线航线飞行。与基于航点的基础版本不同，此实现使用实时速度前馈控制，在风力条件下具有更优的跟踪精度，支持圆柱形和圆锥形螺旋配置。

[ENGLISH](README.md)

## 功能描述

* **速度前馈**：使用 `PositionTarget` 消息同时发布位置和速度设定点，提高轨迹跟踪性能。
* **基于时间生成**：根据经过的时间实时计算轨迹设定点，而非离散航点。
* **自动介入**：建立连接后，自动尝试将飞控切换至 `OFFBOARD` 模式并执行 `ARM`(解锁)操作。
* **增强控制**：使用 `mavros/setpoint_raw/local` 话题同时发送位置和速度指令。
* **灵活设计**：支持圆柱螺旋(`R_START = R_END`)和圆锥螺旋(`R_START ≠ R_END`)。
* **螺旋后悬停**：完成螺旋上升后，在顶部高度和半径处继续绕圈飞行。

## 实现思路

### 1. 实时轨迹生成
使用参数方程动态计算位置和速度:

**位置:**
$$x = x_c + r(t) \cdot \cos(\omega t)$$
$$y = y_c + r(t) \cdot \sin(\omega t)$$
$$z = z_{start} + v_z \cdot t$$

**速度 (位置的导数):**
$$v_x = v_r \cdot \cos(\omega t) - r(t) \omega \cdot \sin(\omega t)$$
$$v_y = v_r \cdot \sin(\omega t) + r(t) \omega \cdot \cos(\omega t)$$
$$v_z = \frac{z_{end} - z_{start}}{T_{total}}$$

**偏航角 (切线方向):**
$$\psi = \text{atan2}(v_y, v_x)$$

其中:
- $r(t) = R_{start} + v_r \cdot t$ (时变半径)
- $v_r = \frac{R_{end} - R_{start}}{T_{total}}$ (径向速度)
- $\omega=0.4 \text{ rad/s}$ (角速度)
- $T_{total} = \frac{2\pi \cdot n_{turns}}{\omega}$ (总螺旋时间)
- 高度: $z_{start}=1.5m$ 到 $z_{end}=5.0m$
- 半径: $R_{start}=2.0m$ 到 $R_{end}=2.0m$ (默认圆柱形)

### 2. 为什么使用速度前馈?
在有风条件下，纯位置控制会表现出滞后，因为控制器必须先检测到位置误差才能做出反应。通过提供预期速度:
- 飞控可以预测所需的推力调整
- 减少跟踪误差和过冲
- 即使有风扰，螺旋轨迹仍保持平滑

### 3. 螺旋后行为
螺旋完成后(当 $t > T_{total}$)，无人机转为在顶部绕圈悬停:
- 位置: $z_{end}$ 处半径为 $R_{end}$ 的圆形路径
- 速度: 仅切向 ($v_z=0$, $v_r=0$)

### 4. 安全握手 (Safety Handshake)
PX4 飞控要求在切换到 Offboard 模式**之前**接收到设定点流。脚本预先发送 100 个初始设定点以满足此要求。

### 5. 状态机与控制环
主循环(20Hz):
1. **状态监控**：如需要，每隔 5 秒尝试一次模式切换和解锁。
2. **轨迹更新**：根据经过时间计算当前位置和速度。
3. **指令发布**：发送包含位置、速度和偏航角字段的 `PositionTarget` 消息。

## 依赖项
* **ROS 1** (推荐 Noetic)
* **MAVROS** (需包含 `mavros_msgs` 和 `geometry_msgs`)
* **Python 库**: `rospy`, `math`

## 关键参数
| 参数           | 值   | 说明                               |
| :------------- | :--- | :--------------------------------- |
| `R_START`      | 2.0  | 起始半径 (米)                      |
| `R_END`        | 2.0  | 结束半径 (米，不等于起始则为圆锥) |
| `HEIGHT_START` | 1.5  | 起始高度 (米)                      |
| `HEIGHT_END`   | 5.0  | 结束高度 (米)                      |
| `OMEGA`        | 0.4  | 角速度 (rad/s)                     |
| `NUM_TURNS`    | 4    | 螺旋圈数                           |
| `RATE_HZ`      | 20.0 | 控制频率                           |

## 使用方法

运行控制节点：

```bash
chmod +x helix_wind_resist.py
roslaunch launch/helix_wind_resist.launch
```

## 与基础版本对比

| 特性         | 基础版本                 | 抗风版本                       |
| :----------- | :----------------------- | :----------------------------- |
| **设定点类型** | 仅位置                   | 位置 + 速度                    |
| **话题**     | `setpoint_position/local` | `setpoint_raw/local`           |
| **消息类型** | `PoseStamped`            | `PositionTarget`               |
| **轨迹逻辑** | 基于航点 (400点)         | 基于时间连续                   |
| **偏航控制** | 未指定                   | 动态(跟随速度)                 |
| **抗风性能** | 有一定滞后               | 滞后减少，跟踪性能更好         |
| **完成后**   | 从底部重新开始           | 在顶部高度/半径处悬停          |
