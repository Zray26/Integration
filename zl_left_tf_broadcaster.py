#!/usr/bin/env python

import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
# import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('left_tf_listener')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    marker_transform_info = rospy.Publisher('left_ee_distance', geometry_msgs.msg.Transform,queue_size=1)

    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link','/left_ee_link', rospy.Time(0))
            # (trans,rot) = listener.lookupTransform('/base_link','/ar_marker_8', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # print(trans,rot)

        marker_trans = geometry_msgs.msg.Transform()
        marker_trans.translation.x= trans[0]
        marker_trans.translation.y= trans[1]
        marker_trans.translation.z= trans[2]
        marker_trans.rotation.x=rot[0]
        marker_trans.rotation.y=rot[1]
        marker_trans.rotation.z=rot[2]
        marker_trans.rotation.w=rot[3]
        print(marker_trans)
        marker_transform_info.publish(marker_trans)

        rate.sleep()
