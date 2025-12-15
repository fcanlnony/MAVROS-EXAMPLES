#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
圆柱螺旋轨迹飞行
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
import numpy as np

# 轨迹参数
R = 2.0              # 圆柱半径 (m)
xc = 0.0             # 中心X
yc = 0.0             # 中心Y
HEIGHT_START = 1.5   # 起始高度 (m)
HEIGHT_END = 4.0     # 结束高度 (m)
num_spirals = 3      # 螺旋圈数
num_points = 100     # 每圈点数

# 生成螺旋轨迹点
total_points = num_spirals * num_points
theta = np.linspace(0, 2 * np.pi * num_spirals, total_points)
heights = np.linspace(HEIGHT_START, HEIGHT_END, total_points)

x_coords = R * np.cos(theta) + xc
y_coords = R * np.sin(theta) + yc
z_coords = heights
aim_points = np.column_stack((x_coords, y_coords, z_coords))

current_state = State()
current_local_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def local_pose_cb(msg):
    global current_local_pose
    current_local_pose = msg

if __name__ == "__main__":
    rospy.init_node("cylinder_trajectory_node")
    RATE_HZ = 20.0

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber(
        "mavros/local_position/pose", PoseStamped, callback=local_pose_cb
    )

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

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

    pose = PoseStamped()
    current_aim_index = 0

    pose.pose.position.x = aim_points[current_aim_index][0]
    pose.pose.position.y = aim_points[current_aim_index][1]
    pose.pose.position.z = aim_points[current_aim_index][2]

    # 预发送setpoint (OFFBOARD模式需要)
    rospy.loginfo("预发送setpoint...")
    for i in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest(custom_mode='OFFBOARD')
    arm_cmd = CommandBoolRequest(value=True)
    last_req = rospy.Time.now()

    TOLERANCE = 0.3  # 到达判断阈值

    rospy.loginfo("开始圆柱螺旋轨迹飞行...")
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

        # 获取当前位置
        current_x = current_local_pose.pose.position.x
        current_y = current_local_pose.pose.position.y
        current_z = current_local_pose.pose.position.z

        aim_x = aim_points[current_aim_index][0]
        aim_y = aim_points[current_aim_index][1]
        aim_z = aim_points[current_aim_index][2]

        # 计算距离
        distance = math.sqrt((current_x - aim_x)**2 + (current_y - aim_y)**2 + (current_z - aim_z)**2)

        # 到达当前点后切换到下一个点
        if distance < TOLERANCE:
            current_aim_index += 1
            if current_aim_index >= len(aim_points):
                rospy.loginfo("圆柱螺旋轨迹完成，循环继续...")
                current_aim_index = 0

            next_aim = aim_points[current_aim_index]
            pose.pose.position.x = next_aim[0]
            pose.pose.position.y = next_aim[1]
            pose.pose.position.z = next_aim[2]

            # 每10%进度输出一次
            progress = (current_aim_index / len(aim_points)) * 100
            if int(progress) % 10 == 0:
                rospy.loginfo(f"进度: {progress:.0f}%, 高度: {next_aim[2]:.2f}m")

        local_pos_pub.publish(pose)
        rate.sleep()
