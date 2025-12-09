#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
import numpy as np

aim_points = ((0, 0, 2), (2, 0, 2), (2, 2, 2), (0, 2, 2))

current_state = State()
current_local_pose = PoseStamped() 

def state_cb(msg):
    global current_state
    current_state = msg

def local_pose_cb(msg): 
    global current_local_pose
    current_local_pose = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    RATE_HZ = 20.0 # 控制频率


    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sub = rospy.Subscriber( 
        "mavros/local_position/pose", PoseStamped, callback = local_pose_cb
    )

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    rate = rospy.Rate(RATE_HZ)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
        
    pose = PoseStamped()
    current_aim_index = 0
    
    pose.pose.position.x = aim_points[current_aim_index][0] 
    pose.pose.position.y = aim_points[current_aim_index][1]
    pose.pose.position.z = aim_points[current_aim_index][2]

    for i in range(100):
        if(rospy.is_shutdown()): break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest(custom_mode = 'OFFBOARD')
    arm_cmd = CommandBoolRequest(value = True)
    last_req = rospy.Time.now()

    TOLERANCE = 0.05 
    
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent):
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()
        
        current_x = current_local_pose.pose.position.x
        current_y = current_local_pose.pose.position.y
        current_z = current_local_pose.pose.position.z
        
        aim_x = aim_points[current_aim_index][0]
        aim_y = aim_points[current_aim_index][1]
        aim_z = aim_points[current_aim_index][2]

        distance = math.sqrt((current_x - aim_x)**2 + (current_y - aim_y)**2 + (current_z - aim_z)**2)
        
        if distance <= TOLERANCE:
            rospy.loginfo(f"Reached Point: ({aim_x:.2f}, {aim_y:.2f}, {aim_z:.2f})")
            
            # 切换到下一个目标点
            current_aim_index += 1
            if current_aim_index >= len(aim_points):
                rospy.loginfo("Waypoint sequence complete. Looping.")
                current_aim_index = 0 # 循环航线

            next_aim = aim_points[current_aim_index]
            pose.pose.position.x = next_aim[0]
            pose.pose.position.y = next_aim[1]
            pose.pose.position.z = next_aim[2]
            rospy.loginfo(f"Moving to next Point: ({next_aim[0]:.2f}, {next_aim[1]:.2f}, {next_aim[2]:.2f})")


        local_pos_pub.publish(pose)
        rate.sleep()