#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
矩形轨迹飞行 (抗风版 - 位置+速度前馈)
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
import numpy as np

# 轨迹参数
LENGTH = 4.0     # 矩形长度 (m)
WIDTH = 2.0      # 矩形宽度 (m)
HEIGHT = 2.0     # 飞行高度 (m)
xc = 0.0         # 中心X
yc = 0.0         # 中心Y
SPEED = 0.5      # 飞行速度 (m/s)

# 生成矩形轨迹点 (4个角点)
corners = [
    (xc - LENGTH/2, yc - WIDTH/2, HEIGHT),  # 左下
    (xc + LENGTH/2, yc - WIDTH/2, HEIGHT),  # 右下
    (xc + LENGTH/2, yc + WIDTH/2, HEIGHT),  # 右上
    (xc - LENGTH/2, yc + WIDTH/2, HEIGHT),  # 左上
]

current_state = State()
current_local_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def local_pose_cb(msg):
    global current_local_pose
    current_local_pose = msg

def get_velocity_to_target(current, target, speed):
    """计算从当前点到目标点的速度向量"""
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    dz = target[2] - current[2]
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist < 0.01:
        return 0, 0, 0
    return dx/dist*speed, dy/dist*speed, dz/dist*speed

if __name__ == "__main__":
    rospy.init_node("rectangle_trajectory_wind_resist_node")
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

    current_corner_index = 0
    target.position.x = corners[current_corner_index][0]
    target.position.y = corners[current_corner_index][1]
    target.position.z = corners[current_corner_index][2]
    target.velocity.x = 0
    target.velocity.y = 0
    target.velocity.z = 0
    target.yaw = 0

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

    TOLERANCE = 0.2

    rospy.loginfo("开始矩形轨迹飞行 (抗风模式)...")
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

        # 获取当前位置
        current_x = current_local_pose.pose.position.x
        current_y = current_local_pose.pose.position.y
        current_z = current_local_pose.pose.position.z
        current_pos = (current_x, current_y, current_z)

        aim = corners[current_corner_index]

        # 计算距离
        distance = math.sqrt((current_x - aim[0])**2 + (current_y - aim[1])**2 + (current_z - aim[2])**2)

        # 计算速度前馈
        vx, vy, vz = get_velocity_to_target(current_pos, aim, SPEED)

        # 到达当前角点后切换到下一个
        if distance < TOLERANCE:
            rospy.loginfo(f"到达角点 {current_corner_index + 1}: ({aim[0]:.2f}, {aim[1]:.2f}, {aim[2]:.2f})")

            current_corner_index += 1
            if current_corner_index >= len(corners):
                rospy.loginfo("矩形轨迹完成，循环继续...")
                current_corner_index = 0

            aim = corners[current_corner_index]
            rospy.loginfo(f"飞往角点 {current_corner_index + 1}: ({aim[0]:.2f}, {aim[1]:.2f}, {aim[2]:.2f})")

        # 计算偏航角 (朝向目标点)
        yaw = math.atan2(aim[1] - current_y, aim[0] - current_x)

        # 更新设定点
        target.header.stamp = rospy.Time.now()
        target.position.x = aim[0]
        target.position.y = aim[1]
        target.position.z = aim[2]
        target.velocity.x = vx
        target.velocity.y = vy
        target.velocity.z = vz
        target.yaw = yaw

        setpoint_pub.publish(target)
        rate.sleep()
