## MAVROS 例程

[ENGLISH](../README.md)

---

## 环境配置

操作系统: Ubuntu 20.04

ROS版本: Noetic

PX4版本: v1.17.0-alpha1

Gazebo版本: 11.15.1

Python版本: >= 3.8

---

## 轨迹示例

### 基础轨迹

这些示例使用纯位置控制，基于航点导航：

- **圆形轨迹**: [圆形](../src/circle/README_CN.md) - 恒定半径的圆形轨迹
- **矩形轨迹**: [矩形](../src/rectangle/README_CN.md) - 带尖锐转角的矩形轨迹
- **圆柱螺旋**: [圆柱螺旋](../src/cylinder/README_CN.md) - 圆柱形螺旋上升轨迹
- **8字形轨迹**: [8字形](../src/figure8/README_CN.md) - 使用李萨如曲线的8字形(∞)轨迹
- **螺旋线轨迹**: [螺旋线](../src/helix/README_CN.md) - 可调半径的螺旋线(圆柱形/圆锥形)

### 抗风轨迹

这些示例使用位置+速度前馈控制，在风力条件下具有更好的跟踪精度：

- **圆形轨迹(抗风版)**: [圆形抗风](../src/circle_wind_resist/README_CN.md) - 基于时间的圆形轨迹，带速度前馈
- **矩形轨迹(抗风版)**: [矩形抗风](../src/rectangle_wind_resist/README_CN.md) - 带速度前馈和动态偏航的矩形轨迹
- **圆柱螺旋(抗风版)**: [圆柱螺旋抗风](../src/cylinder_wind_resist/README_CN.md) - 基于时间的圆柱螺旋，带速度前馈
- **8字形轨迹(抗风版)**: [8字形抗风](../src/figure8_wind_resist/README_CN.md) - 基于时间的8字形轨迹，带速度前馈
- **螺旋线轨迹(抗风版)**: [螺旋线抗风](../src/helix_wind_resist/README_CN.md) - 基于时间的螺旋线，带速度前馈

---

## 主要区别

| 特性         | 基础轨迹                 | 抗风轨迹                   |
| :----------- | :----------------------- | :------------------------- |
| **控制方法** | 仅位置                   | 位置 + 速度                |
| **话题**     | `setpoint_position/local` | `setpoint_raw/local`       |
| **消息类型** | `PoseStamped`            | `PositionTarget`           |
| **轨迹逻辑** | 基于航点                 | 基于时间连续               |
| **抗风性能** | 有一定滞后               | 滞后减少，跟踪性能更好     |

---

## 快速开始

1. **编译工作空间:**
   ```bash
   catkin_make
   source devel/setup.bash
   ```

2. **启动轨迹示例:**
   ```bash
   # 基础轨迹
   roslaunch src/circle/launch/circle.launch

   # 抗风轨迹
   roslaunch src/circle_wind_resist/launch/circle_wind_resist.launch
   ```

3. **无人机将自动:**
   - 连接到 PX4 SITL
   - 切换到 OFFBOARD 模式
   - 解锁并执行轨迹

---

## 文档说明

详细实现信息请参考：
- [CLAUDE.md](../CLAUDE.md) - 项目架构和开发指南
- 上方链接的各个轨迹 README 文件

---

## 许可证

本项目用于教育和研究目的。
