# MAVROS Offboard 圆形轨迹控制

本 ROS 节点通过 MAVROS 功能包实现 PX4 飞控的自主圆形航线飞行。节点内部集成了状态机管理（自动解锁/切模式）以及基于位置反馈的航点导航逻辑。

[ENGLISH](README.md)

## 功能描述

* **轨迹生成**：使用 NumPy 预计算包含 100 个离散航点的圆形路径。
* **自动介入**：建立连接后，自动尝试将飞控切换至 `OFFBOARD` 模式并执行 `ARM`（解锁）操作。
* **位置控制**：以 20Hz 的频率向 `mavros/setpoint_position/local` 话题发布 `PoseStamped` 消息。
* **航点判定**：使用欧几里得距离校验当前位置与目标点的偏差，并在误差范围内自动切换至下一航点。
* **持续循环**：完成一圈飞行后，索引自动重置，实现无限循环飞行。

## 实现思路

### 1. 轨迹预计算 (Pre-calculation)
在进入控制循环前，利用极坐标公式将圆形离散化为 $(x, y, z)$ 坐标数组：
$$x = R \cdot \cos(\theta) + x_c$$
$$y = R \cdot \sin(\theta) + y_c$$
其中半径 $R=5.0m$，飞行高度固定为 $2.0m$。

### 2. 安全握手 (Safety Handshake)
PX4 飞控要求在切换到 Offboard 模式**之前**，必须已经持续接收到设定点（Setpoints）。本脚本在进入主循环前预先发送 100 个目标点数据，就像是“热机”信号，防止切模式瞬间因无数据触发 Failsafe。

### 3. 状态机与控制环
主循环（20Hz）并行处理以下任务：
1.  **状态监控**：每隔 5 秒检查一次当前模式是否为 `OFFBOARD` 及是否已 `ARMED`。若否，则调用相应 MAVROS 服务进行请求。
2.  **导航逻辑**：
    * 计算当前本地位置 (Local Position) 与目标设定点的空间距离向量模长。
    * **阈值判定**：若距离 < `TOLERANCE` (0.3m)，则视为到达，目标索引（Index）自增。
3.  **指令发布**：持续发布当前索引对应的位姿数据。

## 依赖项
* **ROS 1** (推荐 Noetic)
* **MAVROS** (需包含 `mavros_msgs` 和 `geometry_msgs`)
* **Python 库**: `rospy`, `numpy`, `math`

## 关键参数
| 参数         | 值   | 说明                          |
| :----------- | :--- | :---------------------------- |
| `R`          | 5.0  | 飞行半径 (米)                 |
| `num_points` | 100  | 轨迹采样点数 (决定圆的平滑度) |
| `RATE_HZ`    | 20.0 | 控制频率                      |
| `TOLERANCE`  | 0.3  | 航点到达判定半径 (米)         |

## Usage / 使用方法

运行控制节点：

```bash
chmod +x circle.py
roslaunch launch/circle.launch
```