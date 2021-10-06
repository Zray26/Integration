#!/usr/bin/env python

import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
# import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('ee_tf')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    right_transform_info = rospy.Publisher('right_ee_state', geometry_msgs.msg.Transform,queue_size=1)

    left_transform_info = rospy.Publisher('left_ee_state', geometry_msgs.msg.Transform,queue_size=1)

    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        try:
            (trans_r,rot_r) = listener.lookupTransform('/base_link','/right_ee_link', rospy.Time(0))
            (trans_l,rot_l) = listener.lookupTransform('/base_link','/left_ee_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # print(trans,rot)

        left_trans = geometry_msgs.msg.Transform()
        left_trans.translation.x= trans_l[0]
        left_trans.translation.y= trans_l[1]
        left_trans.translation.z= trans_l[2]
        left_trans.rotation.x=rot_l[0]
        left_trans.rotation.y=rot_l[1]
        left_trans.rotation.z=rot_l[2]
        left_trans.rotation.w=rot_l[3]
        # print(left_trans)

        right_trans = geometry_msgs.msg.Transform()
        right_trans.translation.x= trans_r[0]
        right_trans.translation.y= trans_r[1]
        right_trans.translation.z= trans_r[2]
        right_trans.rotation.x=rot_r[0]
        right_trans.rotation.y=rot_r[1]
        right_trans.rotation.z=rot_r[2]
        right_trans.rotation.w=rot_r[3]
        # print(right_trans)
        left_transform_info.publish(left_trans)
        right_transform_info.publish(right_trans)
        rate.sleep()
