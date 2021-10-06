#!/usr/bin/env python
import rospy
import sys
import copy
# import tf
from hrclib_client_v6 import odyssey_Interface
import geometry_msgs.msg 
import math
# import tf
# from tf.transformations import *
# import moveit_commander
# from movo_action_clients.gripper_action_client import GripperActionClient
# from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
# from moveit_python import MoveGroupInterface, PlanningSceneInterface
import time


class Euler_Cal():
    def __init__(self):
        pass
    def quat_to_euler(self,rot):
        print(rot)
        q0 = rot.w
        q1 = rot.x
        q2 = rot.y
        q3 = rot.z
        alpha = math.atan2(2 * (q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
        beta = math.asin(2* (q0*q2 - q1*q3))
        gama = math.atan2(2* (q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
        print('alpha:',alpha,'beta',beta,'gama',gama)
        return alpha, beta, gama

if __name__=="__main__":
    rospy.init_node("gear_auto",
                    anonymous=False)
    right_tf = rospy.wait_for_message("/right_ee_gear_pick_pose",geometry_msgs.msg.Transform)
    left_tf = rospy.wait_for_message("/left_ee_gear_pick_pose",geometry_msgs.msg.Transform)
    r_trans = right_tf.translation
    l_trans = left_tf.translation
    print(r_trans)
    print(l_trans)
    r_rot = right_tf.rotation
    l_rot = left_tf.rotation
    # print (r_rot == l_rot)
    r_cal = Euler_Cal()
    l_cal = Euler_Cal()
    r_angle_x, r_angle_y, r_angle_z = r_cal.quat_to_euler(r_rot)
    l_angle_x, l_angle_y, l_angle_z = l_cal.quat_to_euler(l_rot)
    # print(r_angle_x)
    # print(l_angle_x)
    ods = odyssey_Interface()
    ods._L0_dual_set_gripper(0)

    # input()
    # # raw_input()
    # # -pi/2 --> rotate relatively to x axis of base_link
    ods._L0_single_task_move_safe("right",[0.55, -0.091, 1.197],
                                        [r_angle_x, 0, r_angle_z],
                                        [20 for i in range(6)])
    ods._L0_single_task_move_safe("left",[0.532, 0.093, 1.197],
                                        [l_angle_x, 0, l_angle_z],
                                        [20 for i in range(6)])
    #input()
    ods._L0_dual_jp_move_safe_relate(jp_r=[0,0,0,0,0,0,-math.pi + 0.05],rmaxforce=[10,10,10,10,10,10],jp_l=[0,0,0,0,0,0, 0.05],lmaxforce=[10,10,10,10,10,10],duration=5)
    ods._L0_dual_task_move_safe_relate(
        rmove=[0, -0.16, 0], rmaxforce=[30 for i in range(6)],
        lmove=[0, 0.16, 0], lmaxforce=[30 for i in range(6)],
        time=3)
    #input()
    ods._L0_dual_task_move_safe_relate(
        rmove=[0, 0, -0.05], rmaxforce=[30 for i in range(6)],
        lmove=[0, 0, -0.05], lmaxforce=[30 for i in range(6)],
        time=3)
    #input()
    ods._L0_dual_task_move_safe_relate(
        rmove=[0.15, 0, 0], rmaxforce=[30 for i in range(6)],
        lmove=[0.15, 0, 0], lmaxforce=[30 for i in range(6)],
        time=3)
    #input()
    right_ee_tf = rospy.wait_for_message("/right_ee_state",geometry_msgs.msg.Transform)
    left_ee_tf = rospy.wait_for_message("/left_ee_state",geometry_msgs.msg.Transform)
    
    r_ee_trans=right_ee_tf.translation
    l_ee_trans = left_ee_tf.translation
    r_diff = [r_trans.x - r_ee_trans.x, r_trans.y - r_ee_trans.y, r_trans.z - r_ee_trans.z]
    l_diff = [l_trans.x - l_ee_trans.x, l_trans.y - l_ee_trans.y, l_trans.z - l_ee_trans.z]
    print('r ee trans is',r_ee_trans)
    print('l ee trans is',l_ee_trans)
    print('r trans is',r_trans)
    print('r trans is',l_trans)
    print('r diff is',r_diff)
    print('l diff is',l_diff)
    # r_diff = r_trans- right_ee_tf.translation
    # l_diff = l_trans - left_ee_tf.translation
    print("message get")
    # input()
    ods._L0_dual_set_gripper(1)

    ods._L0_dual_task_move_safe_relate(
        rmove=[r_diff[0]*1+0.015, r_diff[1]*0.5, 0], rmaxforce=[30 for i in range(6)],
        lmove=[l_diff[0]*1, l_diff[1]*0.5, 0], lmaxforce=[30 for i in range(6)],
        time=6)
    #input()
    ods._L0_dual_task_move_safe_relate(
        rmove=[0, 0, r_diff[2]-0.035], rmaxforce=[30 for i in range(6)],
        lmove=[0, 0, l_diff[2]-0.01], lmaxforce=[30 for i in range(6)],
        time=6)
    #input()
    ods._L0_dual_task_move_safe_relate(
        rmove=[r_diff[0]*0, r_diff[1]*0.43, 0], rmaxforce=[30 for i in range(6)],
        lmove=[l_diff[0]*0, l_diff[1]*0.43, 0], lmaxforce=[30 for i in range(6)],
        time=6)
    ods._L0_dual_set_gripper(0)
    time.sleep(15)
    # input()
    ods._L0_dual_task_move_safe_relate(
        rmove=[0.0, 0, 0.25], rmaxforce=[40 for i in range(6)],
        lmove=[0.0, 0, 0.25], lmaxforce=[40 for i in range(6)],
        time=3)
    # input()
    ods._L0_dual_task_move_safe_relate(
        rmove=[0.0, 0, -0.16], rmaxforce=[40 for i in range(6)],
        lmove=[0.0, 0, -0.16], lmaxforce=[40 for i in range(6)],
        time=3)
    ods._L0_dual_set_gripper(1)
    ods._L0_dual_task_move_safe_relate(
        rmove=[r_diff[0]*0, -0.1, 0], rmaxforce=[30 for i in range(6)],
        lmove=[l_diff[0]*0, 0.1, 0], lmaxforce=[30 for i in range(6)],
        time=6)