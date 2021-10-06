#!/usr/bin/env python

# Author: Isaac Ng

# References
# ----------
# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html


import sys
import copy
import rospy
import moveit_commander
from movo_action_clients.gripper_action_client import GripperActionClient

from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

import geometry_msgs.msg

_upper_body_joints = ["right_shoulder_pan_joint",
                      "right_shoulder_lift_joint",
                      "right_arm_half_joint",
                      "right_elbow_joint",
                      "right_wrist_spherical_1_joint",
                      "right_wrist_spherical_2_joint",
                      "right_wrist_3_joint",
                      "left_shoulder_pan_joint",
                      "left_shoulder_lift_joint",
                      "left_arm_half_joint",
                      "left_elbow_joint",
                      "left_wrist_spherical_1_joint",
                      "left_wrist_spherical_2_joint",
                      "left_wrist_3_joint",
                      "linear_joint",
                      "pan_joint",
                      "tilt_joint"]
                      
# Head looking straight
# [right-arm
#  left-arm,
#  spine, pan, tilt]


default_pose_tucked = [ 0.9, -1.1,  0.9484, -2.27, -0.839,  1.65, -1.7,
                       -0.9,  1.1, -0.9484,  2.27,  0.839, -1.65,  1.7, 
                       0.35, 0, -0.67]
gear_box_pregrasp = [ 0.9, -1.1,  0.9484, -2.27, -2.3,  1.5, 1.3,
                       -0.9,  1.1, -0.9484,  2.27,  0.839, -1.65,  1.7, 
                       0.45, 0, -0.3]
inverse_gear_box_pregrasp = [ 0.9, -1.1,  0.9484, -2.27, -0.839,  1.6, 2.85,
                       -0.9,  1.1, -0.9484,  2.27,  0.839, -1.6,  -2.85, 
                       0.45, 0, -0.7] #left as parallel gripper
# First try
# default_pose_tucked = [ 0.6, -2.0, -0.1, -1.75, -0.0,  0.6, -1.7, 
#                        -0.8994102201926122, 1.1019249789655081, -0.9481396765544474, 2.268680886243469, 0.8389133202381545, -1.6517979692463107, 1.7103904427052532, 
#                        0.35, 0, -0.67]



# [spine,
#  left_arm,
#  pan, tilt,
#  right_arm]
pregrasp_pose = [0.1377732902765274, 
                 -0.6242859721953593, 1.9617776695905196, 0.04507687683133055, 1.7542401890017347, 
                 0.0015246839109828159, -0.593679859683734, 1.6965739629699215, 
                 0.001532116555608809, 0.0, 
                 0.6003267768694354, -2.020244175529901, -0.10757466380822578, -1.6310970849350703, 
                 -0.0015398397422838883, 0.6810874698212247, -1.6766415337912237]


if __name__=="__main__":
    rospy.init_node('movo_moveit_test',
                    anonymous=False)

    moveit_commander.roscpp_initialize(sys.argv)

    scene = moveit_commander.PlanningSceneInterface()

    lgripper = GripperActionClient('left')
    rgripper = GripperActionClient('right')
    gripper_closed = 0.00
    gripper_open = 0.165

    larm_group = moveit_commander.MoveGroupCommander("left_arm")
    rarm_group = moveit_commander.MoveGroupCommander("right_arm")
    upper_body = moveit_commander.MoveGroupCommander("upper_body")

    move_group = MoveGroupInterface("upper_body", "base_link")
    lmove_group = MoveGroupInterface("left_arm", "base_link")
    rmove_group = MoveGroupInterface("right_arm", "base_link")



    print("Done spinning up MoveIt!")

    # lgripper.command(gripper_closed,block=True)
    # rgripper.command(gripper_closed, block=True)
    
    # upper_body.go(joints=[])
    # larm_group.
    

    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, inverse_gear_box_pregrasp, 0.005, wait=True)


        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break
    

    # lgripper.command(gripper_open,block=True)
    # rgripper.command(gripper_open, block=True)

    print("successful")
