#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
圆柱螺旋轨迹飞行 (抗风版 - 位置+速度前馈)
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math

# 轨迹参数
R = 2.0              # 圆柱半径 (m)
xc = 0.0             # 中心X
yc = 0.0             # 中心Y
HEIGHT_START = 1.5   # 起始高度 (m)
HEIGHT_END = 4.0     # 结束高度 (m)
OMEGA = 0.3          # 角速度 (rad/s)
NUM_SPIRALS = 3      # 螺旋圈数

# 计算总时间和垂直速度
TOTAL_ANGLE = 2 * math.pi * NUM_SPIRALS
TOTAL_TIME = TOTAL_ANGLE / OMEGA
VZ = (HEIGHT_END - HEIGHT_START) / TOTAL_TIME

current_state = State()
current_local_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def local_pose_cb(msg):
    global current_local_pose
    current_local_pose = msg

def generate_spiral_setpoint(t):
    """生成螺旋轨迹的位置和速度"""
    angle = OMEGA * t

    # 位置
    x = xc + R * math.cos(angle)
    y = yc + R * math.sin(angle)
    z = HEIGHT_START + VZ * t

    # 速度 (位置的导数)
    vx = -R * OMEGA * math.sin(angle)
    vy = R * OMEGA * math.cos(angle)
    vz = VZ

    # 偏航角 (沿切线方向)
    yaw = angle + math.pi / 2

    return x, y, z, vx, vy, vz, yaw

if __name__ == "__main__":
    rospy.init_node("cylinder_trajectory_wind_resist_node")
    RATE_HZ = 20.0

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber(
        "mavros/local_position/pose", PoseStamped, callback=local_pose_cb
    )

    setpoint_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(RATE_HZ)

    # 等待FCU连接
    rospy.loginfo("等待FCU连接...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("FCU已连接")

    # 初始设定点
    target = PositionTarget()
    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    target.type_mask = (
        PositionTarget.IGNORE_AFX |
        PositionTarget.IGNORE_AFY |
        PositionTarget.IGNORE_AFZ |
        PositionTarget.IGNORE_YAW_RATE
    )

    x, y, z, vx, vy, vz, yaw = generate_spiral_setpoint(0)
    target.position.x = x
    target.position.y = y
    target.position.z = z
    target.velocity.x = vx
    target.velocity.y = vy
    target.velocity.z = vz
    target.yaw = yaw

    # 预发送setpoint
    rospy.loginfo("预发送setpoint...")
    for i in range(100):
        if rospy.is_shutdown():
            break
        target.header.stamp = rospy.Time.now()
        setpoint_pub.publish(target)
        rate.sleep()

    offb_set_mode = SetModeRequest(custom_mode='OFFBOARD')
    arm_cmd = CommandBoolRequest(value=True)
    last_req = rospy.Time.now()

    start_time = rospy.Time.now()
    rospy.loginfo("开始圆柱螺旋轨迹飞行 (抗风模式)...")

    while not rospy.is_shutdown():
        # 尝试切换OFFBOARD模式
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD模式已启用")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("飞机已解锁")
                last_req = rospy.Time.now()

        # 计算当前时间对应的轨迹点
        t = (rospy.Time.now() - start_time).to_sec()

        # 限制在总时间内，完成后保持在顶部绕圈
        if t > TOTAL_TIME:
            # 到达顶部后在顶部绕圈
            t_circle = t - TOTAL_TIME
            angle = OMEGA * t_circle
            x = xc + R * math.cos(angle)
            y = yc + R * math.sin(angle)
            z = HEIGHT_END
            vx = -R * OMEGA * math.sin(angle)
            vy = R * OMEGA * math.cos(angle)
            vz = 0
            yaw = angle + math.pi / 2
        else:
            x, y, z, vx, vy, vz, yaw = generate_spiral_setpoint(t)

        # 更新设定点
        target.header.stamp = rospy.Time.now()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.velocity.x = vx
        target.velocity.y = vy
        target.velocity.z = vz
        target.yaw = yaw

        setpoint_pub.publish(target)

        # 进度输出
        if t <= TOTAL_TIME:
            progress = (t / TOTAL_TIME) * 100
            if int(progress) % 20 == 0:
                rospy.loginfo_throttle(3, f"螺旋进度: {progress:.0f}%, 高度: {z:.2f}m")

        rate.sleep()
