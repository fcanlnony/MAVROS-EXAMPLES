#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
圆形轨迹飞行 (抗风版 - 位置+速度前馈)
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
import numpy as np

# 轨迹参数
R = 2.0          # 圆半径 (m)
xc = 0.0         # 圆心X
yc = 0.0         # 圆心Y
HEIGHT = 2.0     # 飞行高度 (m)
OMEGA = 0.3      # 角速度 (rad/s)

current_state = State()
current_local_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def local_pose_cb(msg):
    global current_local_pose
    current_local_pose = msg

def generate_circle_setpoint(t):
    """生成圆形轨迹的位置和速度"""
    # 位置
    x = xc + R * math.cos(OMEGA * t)
    y = yc + R * math.sin(OMEGA * t)
    z = HEIGHT

    # 速度 (位置的导数)
    vx = -R * OMEGA * math.sin(OMEGA * t)
    vy = R * OMEGA * math.cos(OMEGA * t)
    vz = 0.0

    # 偏航角 (沿切线方向)
    yaw = OMEGA * t + math.pi / 2

    return x, y, z, vx, vy, vz, yaw

if __name__ == "__main__":
    rospy.init_node("circle_trajectory_wind_resist_node")
    RATE_HZ = 20.0

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber(
        "mavros/local_position/pose", PoseStamped, callback=local_pose_cb
    )

    # 使用 setpoint_raw/local 发送位置+速度
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

    x, y, z, vx, vy, vz, yaw = generate_circle_setpoint(0)
    target.position.x = x
    target.position.y = y
    target.position.z = z
    target.velocity.x = vx
    target.velocity.y = vy
    target.velocity.z = vz
    target.yaw = yaw

    # 预发送setpoint (OFFBOARD模式需要)
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
    rospy.loginfo("开始圆形轨迹飞行 (抗风模式)...")

    while not rospy.is_shutdown():
        # 尝试切换OFFBOARD模式
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD模式已启用")
            last_req = rospy.Time.now()
        else:
            # 尝试解锁
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("飞机已解锁")
                last_req = rospy.Time.now()

        # 计算当前时间对应的轨迹点
        t = (rospy.Time.now() - start_time).to_sec()
        x, y, z, vx, vy, vz, yaw = generate_circle_setpoint(t)

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

        # 每圈输出一次
        loops = int(t * OMEGA / (2 * math.pi))
        if loops > 0 and abs(t * OMEGA % (2 * math.pi)) < 0.1:
            rospy.loginfo_throttle(5, f"已完成 {loops} 圈")

        rate.sleep()
