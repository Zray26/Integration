#!/usr/bin/env python

import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import numpy as np
import geometry_msgs.msg

## R arm_to_base = R ar_to_base * R arm_to_ar
# right arm to ar R arm_to_ar
r_trans = np.array([
        [0.9350,    0.3148,   -0.1636,   -0.1030],
        [-0.1757,    0.0102,   -0.9844,   -0.0810],
        [-0.3082,    0.9491,    0.0649,   -0.0480],
        [0,         0,         0,    1.0000],
])

# left arm to ar
l_trans = np.array([
        [-0.9601,    0.2754,   -0.0486,    0.1210],
        [-0.0438,    0.0233,    0.9988,   -0.0910],
        [0.2762,    0.9610,   -0.0103,   -0.1320],
        [0,         0,         0,    1.0000]
])
# import turtlesim.srv
if __name__ == '__main__':
    rospy.init_node('gear_right_tf')
    listener = tf.TransformListener()
    ee_gear = rospy.Publisher('right_ee_gear_pick_pose', geometry_msgs.msg.Transform,queue_size=1)
    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link','/ar_marker_3', rospy.Time(0))
            # (trans,rot) = listener.lookupTransform('/base_link','/ar_marker_8', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # print(trans,rot)

        marker_trans = geometry_msgs.msg.Transform()
        x= trans[0]
        y= trans[1]
        z= trans[2]
        rx=rot[0]
        ry=rot[1]
        rz=rot[2]
        rw=rot[3]
        print(rot)
        q0 = rw
        q1 = rx
        q2 = ry
        q3 = rz

        # Calculate the translation matrix
        R = np.array([
                    [1-2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
                    [2*(q1*q2 + q0*q3), 1-2*(q1**2+q3**2),2*(q2*q3 - q0*q1)],
                    [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1- 2*(q1**2 + q2 **2)]
        ])
        T = np.column_stack((R, np.transpose(trans)))
        T = np.row_stack((T, [0.0, 0.0, 0.0, 1.0]))
        T[3][3] = 1.0
        # print('after',r_trans)
        ww = 0.5 * math.sqrt(1 + R[0][0] + R[1][1]+R[2][2])
        xx = (R[2][1] - R[1][2])/(4 * ww)
        yy = (R[0][2] - R[2][0])/(4 * ww)
        zz = (R[1][0] - R[0][1])/(4 * ww)
        rot_2 = np.array([xx, yy, zz, ww])
        # print('after',np.shape(T))

        arm_final = np. matmul(T,r_trans)
        final_rot = arm_final[0:3][0:3]


        trans_pub = geometry_msgs.msg.Transform()
        trans_pub.translation.x = arm_final[0][3]
        trans_pub.translation.y = arm_final[1][3]
        trans_pub.translation.z = arm_final[2][3]
        lw = 0.5*math.sqrt(1 + final_rot[0][0] + final_rot[1][1] + final_rot[2][2])
        trans_pub.rotation.x=(final_rot[2][1] -final_rot[1][2])/(4 * lw)
        trans_pub.rotation.y=(final_rot[0][2] -final_rot[2][0])/(4 * lw)
        trans_pub.rotation.z=(final_rot[1][0] -final_rot[0][1])/(4 * lw)
        trans_pub.rotation.w=lw
        ee_gear.publish(trans_pub)

        rate.sleep()
